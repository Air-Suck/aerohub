#include <uav_gcopter/core.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <uav_planner_interfaces/msg/timed_path.hpp>
#include <algorithm>
#include <set>
#include <ompl/util/RandomNumbers.h>

// This Function produces plans only. A separate consumer owns Behavior execution.
class GCopterFunction : public rclcpp::Node {
  using Trigger = std_srvs::srv::Trigger;
  using Path = uav_planner_interfaces::msg::TimedPath;
  uav_gcopter::Config config_;
  std::string frame_, vehicle_;
  std::vector<double> goal_;
  nav_msgs::msg::Odometry::SharedPtr state_;
  std::vector<Eigen::Vector3d> cloud_;
  rclcpp::Time map_stamp_{0, 0, RCL_ROS_TIME};
  bool map_ready_ = false;
  uint64_t epoch_ = 0, revision_ = 0;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr map_sub_;
  rclcpp::Publisher<Path>::SharedPtr plans_;
  rclcpp::Service<Trigger>::SharedPtr plan_, reset_, health_;
  OnSetParametersCallbackHandle::SharedPtr parameter_guard_;
  bool fresh(const rclcpp::Time &stamp, double limit) {
    double age = (now() - stamp).seconds();
    return stamp.nanoseconds() > 0 && age >= 0 && age <= limit;
  }
public:
  GCopterFunction() : Node("gcopter_function") {
    const int seed = declare_parameter("search_seed", 0);
    if (seed < 0) throw std::invalid_argument("search_seed must be nonnegative");
    if (seed) ompl::RNG::setSeed(seed);
    RCLCPP_INFO(get_logger(), "OMPL seed=%lu", static_cast<unsigned long>(ompl::RNG::getSeed()));
    frame_ = declare_parameter<std::string>("frame_id", "drone0/odom");
    vehicle_ = declare_parameter<std::string>("vehicle_id", "drone0");
    goal_ = declare_parameter<std::vector<double>>("goal", {3., 0., 2.});
    auto low = declare_parameter<std::vector<double>>("map_low", {-25., -25., 0.});
    auto high = declare_parameter<std::vector<double>>("map_high", {25., 25., 5.});
    if (low.size() != 3 || high.size() != 3) throw std::invalid_argument("map bounds require XYZ");
    config_.low = Eigen::Map<Eigen::Vector3d>(low.data());
    config_.high = Eigen::Map<Eigen::Vector3d>(high.data());
    config_.resolution = declare_parameter("resolution", 0.25);
    config_.inflation = declare_parameter("inflation", 0.5);
    config_.rrt_timeout = declare_parameter("rrt_timeout", 0.02);
    auto bounds = declare_parameter<std::vector<double>>("magnitude_bounds", {4., 2.1, 1.05, 2., 12.});
    auto weights = declare_parameter<std::vector<double>>("penalty_weights", {1e4, 1e4, 1e4, 1e4, 1e5});
    auto physics = declare_parameter<std::vector<double>>("physical_parameters", {0.61, 9.8, 0.7, 0.8, 0.01, 1e-4});
    if (bounds.size() != 5 || weights.size() != 5 || physics.size() != 6)
      throw std::invalid_argument("bounds/weights/physics require 5/5/6 entries");
    config_.bounds = Eigen::Map<Eigen::Matrix<double, 5, 1>>(bounds.data());
    config_.weights = Eigen::Map<Eigen::Matrix<double, 5, 1>>(weights.data());
    config_.physics = Eigen::Map<Eigen::Matrix<double, 6, 1>>(physics.data());
    config_.weight_time = declare_parameter("weight_time", 20.);
    config_.smoothing = declare_parameter("smoothing", 0.01);
    config_.tolerance = declare_parameter("relative_cost_tolerance", 1e-5);
    config_.quadrature = declare_parameter("quadrature_intervals", 16);
    declare_parameter("state_max_age", 0.2);
    declare_parameter("map_max_age", 2.0);
    declare_parameter("sample_period", 0.02);
    declare_parameter("yaw", 0.0);
    if (goal_.size() != 3 || !Eigen::Map<Eigen::Vector3d>(goal_.data()).allFinite())
      throw std::invalid_argument("goal must be finite XYZ");
    plans_ = create_publisher<Path>("gcopter/plan", 1);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("sensor_measurements/odom", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::SharedPtr msg) { state_ = msg; });
    map_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>("gcopter/map", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        map_ready_ = false;
        if (msg->header.frame_id != frame_ || !fresh(rclcpp::Time(msg->header.stamp), get_parameter("map_max_age").as_double())) return;
        try {
          if (msg->is_bigendian || msg->point_step == 0 || msg->row_step < msg->width * msg->point_step ||
              msg->data.size() != static_cast<size_t>(msg->row_step) * msg->height ||
              msg->row_step != msg->width * msg->point_step) return;
          for (const auto *name : {"x", "y", "z"}) {
            auto field = std::find_if(msg->fields.begin(), msg->fields.end(),
              [name](const sensor_msgs::msg::PointField &f) { return f.name == name; });
            if (field == msg->fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
                field->count != 1 || field->offset + 4 > msg->point_step) return;
          }
          std::vector<Eigen::Vector3d> points;
          sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x"), y(*msg, "y"), z(*msg, "z");
          for (; x != x.end(); ++x, ++y, ++z) {
            Eigen::Vector3d p(*x, *y, *z);
            if (!p.allFinite()) return;
            points.push_back(p);
          }
          cloud_ = std::move(points); map_stamp_ = rclcpp::Time(msg->header.stamp); map_ready_ = true; ++epoch_;
        } catch (const std::exception &e) { RCLCPP_WARN(get_logger(), "map rejected: %s", e.what()); }
      });
    reset_ = create_service<Trigger>("gcopter/reset", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      cloud_.clear(); map_ready_ = false; state_.reset(); ++epoch_;
      Path invalidation; plans_->publish(invalidation);
      res->success = true; res->message = "cleared; consumer must confirm execution stopped";
    });
    health_ = create_service<Trigger>("gcopter/health", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      res->success = map_ready_ && state_ && state_->header.frame_id == frame_ &&
        fresh(rclcpp::Time(state_->header.stamp), get_parameter("state_max_age").as_double()) &&
        fresh(map_stamp_, get_parameter("map_max_age").as_double());
      res->message = res->success ? "inputs_fresh" : "inputs_not_ready";
    });
    plan_ = create_service<Trigger>("gcopter/plan_once", [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
      res->success = false;
      if (!map_ready_ || !state_ || state_->header.frame_id != frame_ ||
          !fresh(rclcpp::Time(state_->header.stamp), get_parameter("state_max_age").as_double()) ||
          !fresh(map_stamp_, get_parameter("map_max_age").as_double())) { res->message = "inputs_not_ready"; return; }
      // First slice explicitly supports stationary boundaries only, not fabricated measured acceleration.
      const auto &v = state_->twist.twist.linear;
      if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) ||
          Eigen::Vector3d(v.x, v.y, v.z).norm() > 0.05) { res->message = "stationary_start_required"; return; }
      Eigen::Matrix3d start = Eigen::Matrix3d::Zero(), goal = start;
      const auto &p = state_->pose.pose.position;
      start.col(0) = Eigen::Vector3d(p.x, p.y, p.z);
      goal.col(0) = Eigen::Map<Eigen::Vector3d>(goal_.data());
      auto result = uav_gcopter::plan(config_, start, goal, cloud_);
      if (!result) { res->message = result.error; return; }
      if (!fresh(map_stamp_, get_parameter("map_max_age").as_double())) {
        res->message = "map_expired_during_solve"; return;
      }
      // Do not publish executable timing: the consumer must revalidate held start state.
      Path path;
      path.interface_version = 1; path.vehicle_id = vehicle_; path.frame_id = frame_;
      path.task_id = "gcopter-" + std::to_string(epoch_) + "-" + std::to_string(++revision_);
      path.revision = revision_; path.generated_at = now(); path.valid_from = path.generated_at;
      path.is_final = true; path.valid_fields = 7; path.max_speed = config_.bounds[0];
      const double period = get_parameter("sample_period").as_double();
      const double yaw = get_parameter("yaw").as_double();
      if (!std::isfinite(period) || period < 0.001 || !std::isfinite(yaw)) { res->message = "invalid_sampling"; return; }
      const double duration = result.trajectory.getTotalDuration();
      if (!std::isfinite(duration) || duration <= 0 || duration > 600) { res->message = "duration_limit"; return; }
      std::set<int64_t> times{0, static_cast<int64_t>(std::llround(duration * 1e9))};
      double boundary = 0;
      for (const auto &piece : result.trajectory) { boundary += piece.getDuration(); times.insert(std::llround(boundary*1e9)); }
      for (double t = period; t < duration; t += period) times.insert(std::llround(t*1e9));
      for (const auto ns : times) {
        uav_planner_interfaces::msg::TimedTrajectoryPoint point;
        point.time_from_start.sec = ns / 1000000000; point.time_from_start.nanosec = ns % 1000000000;
        double t = ns / 1e9;
        auto pos = result.trajectory.getPos(t), vel = result.trajectory.getVel(t), acc = result.trajectory.getAcc(t);
        if (!pos.allFinite() || !vel.allFinite() || !acc.allFinite() || vel.norm() > config_.bounds[0]*1.01) {
          res->message = "sample_constraint_failed"; return;
        }
        point.position.x = pos.x(); point.position.y = pos.y(); point.position.z = pos.z();
        point.velocity.x = vel.x(); point.velocity.y = vel.y(); point.velocity.z = vel.z();
        point.acceleration.x = acc.x(); point.acceleration.y = acc.y(); point.acceleration.z = acc.z();
        point.yaw = yaw; path.points.push_back(point);
      }
      plans_->publish(path); res->success = true; res->message = path.task_id + ":plan_only_requires_consumer_validation";
    });
    parameter_guard_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "configuration frozen; stop execution and restart with reviewed parameters";
      return result;
    });
  }
};
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GCopterFunction>());
  rclcpp::shutdown();
}
