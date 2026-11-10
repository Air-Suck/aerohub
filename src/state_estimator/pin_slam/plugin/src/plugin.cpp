#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <as2_state_estimator/plugin_base.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace pin_slam {
class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase {
  struct LocalPose {
    int64_t stamp_ns;
    tf2::Transform odom_base;
  };

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pin_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr local_subscription_;
  tf2::Transform earth_map_;
  tf2::Transform sensor_base_;
  tf2::Transform map_odom_ = tf2::Transform::getIdentity();
  std::deque<LocalPose> local_poses_;
  std::mutex mutex_;
  std::string input_world_;
  std::string input_body_;
  std::string local_odom_topic_;
  double pin_max_age_ = 0.5;
  double local_max_age_ = 0.2;
  double pairing_tolerance_ = 0.05;
  double buffer_duration_ = 2.0;
  std::atomic<int64_t> last_pin_stamp_{-1};
  std::atomic<int64_t> last_local_stamp_{-1};
#ifdef PIN_SLAM_AS2_STATE_INTERFACE
  bool frames_initialized_ = false;

  const std::string & get_earth_frame() {return state_estimator_interface_->getEarthFrame();}
  const std::string & get_map_frame() {return state_estimator_interface_->getMapFrame();}
  const std::string & get_odom_frame() {return state_estimator_interface_->getOdomFrame();}
  const std::string & get_base_frame() {return state_estimator_interface_->getBaseFrame();}
#endif

  tf2::Transform transform_parameter(const std::string & name) {
    const auto values = node_ptr_->getParameter<std::vector<double>>(
      name, std::vector<double>{0., 0., 0., 0., 0., 0., 1.});
    if (values.size() != 7) {
      throw std::runtime_error(name + " must be xyz plus quaternion xyzw");
    }
    for (const double value : values) {
      if (!std::isfinite(value)) {throw std::runtime_error(name + " is not finite");}
    }
    tf2::Quaternion rotation(values[3], values[4], values[5], values[6]);
    if (std::abs(rotation.length2() - 1.0) > 0.01) {
      throw std::runtime_error(name + " quaternion is not unit length");
    }
    rotation.normalize();
    return tf2::Transform(rotation, tf2::Vector3(values[0], values[1], values[2]));
  }

  static bool finite_pose(const geometry_msgs::msg::Pose & pose) {
    for (const double value : {
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w})
    {
      if (!std::isfinite(value)) {return false;}
    }
    const double norm = pose.orientation.x * pose.orientation.x +
      pose.orientation.y * pose.orientation.y + pose.orientation.z * pose.orientation.z +
      pose.orientation.w * pose.orientation.w;
    return std::abs(norm - 1.0) <= 0.01;
  }

  static bool finite_twist(const geometry_msgs::msg::Twist & twist) {
    for (const double value : {
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z})
    {
      if (!std::isfinite(value)) {return false;}
    }
    return true;
  }

  static tf2::Transform pose_transform(const geometry_msgs::msg::Pose & pose) {
    tf2::Quaternion rotation(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    rotation.normalize();
    return tf2::Transform(
      rotation, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
  }

  bool fresh(const builtin_interfaces::msg::Time & header_stamp, double max_age) const {
    const rclcpp::Time stamp(header_stamp, node_ptr_->get_clock()->get_clock_type());
    const double age = (node_ptr_->now() - stamp).seconds();
    return age <= max_age && age >= -0.05;
  }

  void warn_reject(const char * source) {
    RCLCPP_WARN_THROTTLE(
      node_ptr_->get_logger(), *node_ptr_->get_clock(), 3000,
      "Reject %s state: frame, numeric, timestamp, freshness, or pairing contract violated",
      source);
  }

public:
#ifdef PIN_SLAM_AS2_STATE_INTERFACE
  void onSetup() override {setup_common();}
  std::vector<as2_state_estimator::TransformInformatonType>
  getTransformationTypesAvailable() const override
  {
    using namespace as2_state_estimator;
    return {EARTH_TO_MAP, MAP_TO_ODOM, ODOM_TO_BASE, TWIST_IN_BASE};
  }
#else
  void on_setup() override {setup_common();}
  bool get_earth_to_map_transform(geometry_msgs::msg::TransformStamped & output) override
  {
    output.header.stamp = node_ptr_->now();
    output.header.frame_id = get_earth_frame();
    output.child_frame_id = get_map_frame();
    output.transform = tf2::toMsg(earth_map_);
    return true;
  }
#endif

private:
  void setup_common()
  {
    input_world_ = node_ptr_->getParameter<std::string>("input_world_frame", "pin_slam_map");
    input_body_ = node_ptr_->getParameter<std::string>("input_body_frame", "pin_slam_lidar");
    local_odom_topic_ = node_ptr_->getParameter<std::string>(
      "local_odometry_topic", "sensor_measurements/odom");
    pin_max_age_ = node_ptr_->getParameter<double>("max_age", 0.5);
    local_max_age_ = node_ptr_->getParameter<double>("local_max_age", 0.2);
    pairing_tolerance_ = node_ptr_->getParameter<double>("pairing_tolerance", 0.05);
    buffer_duration_ = node_ptr_->getParameter<double>("local_buffer_duration", 2.0);
    for (const auto & setting : {
        std::pair<const char *, double>{"max_age", pin_max_age_},
        {"local_max_age", local_max_age_},
        {"pairing_tolerance", pairing_tolerance_},
        {"local_buffer_duration", buffer_duration_}})
    {
      if (!std::isfinite(setting.second) || setting.second <= 0.0) {
        throw std::runtime_error(std::string(setting.first) + " must be positive and finite");
      }
    }
    earth_map_ = transform_parameter("earth_from_pin_world");
    sensor_base_ = transform_parameter("sensor_from_base");
#ifdef PIN_SLAM_AS2_STATE_INTERFACE
    if (node_ptr_->getParameter<double>("publish_hz", 0.0) > 0.0) {
      throw std::runtime_error("PIN-SLAM requires publish_hz=0 to avoid stale state republication");
    }
#else
    const auto stamp = node_ptr_->now();
    geometry_msgs::msg::TransformStamped earth_map;
    earth_map.header.stamp = stamp;
    earth_map.header.frame_id = get_earth_frame();
    earth_map.child_frame_id = get_map_frame();
    earth_map.transform = tf2::toMsg(earth_map_);
    publish_static_transform(earth_map);
#endif
    pin_subscription_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      "pin_slam/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {receive_pin(*message);});
    local_subscription_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      local_odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {receive_local(*message);});
    RCLCPP_INFO(
      node_ptr_->get_logger(),
      "PIN-SLAM global correction uses %s for high-rate local odometry",
      local_odom_topic_.c_str());
  }

  void initialize_frames(const builtin_interfaces::msg::Time & stamp)
  {
#ifdef PIN_SLAM_AS2_STATE_INTERFACE
    if (!frames_initialized_) {
      state_estimator_interface_->setEarthToMap(earth_map_, stamp, true);
      state_estimator_interface_->setMapToOdomPose(map_odom_, stamp, false);
      frames_initialized_ = true;
    }
#else
    (void)stamp;
#endif
  }

  void receive_local(const nav_msgs::msg::Odometry & message)
  {
    const rclcpp::Time stamp(message.header.stamp, node_ptr_->get_clock()->get_clock_type());
    const int64_t previous_stamp = last_local_stamp_.load();
    if (stamp.nanoseconds() == previous_stamp) {
      return;
    }
    if (message.header.frame_id != get_odom_frame() ||
      message.child_frame_id != get_base_frame() || !finite_pose(message.pose.pose) ||
      !finite_twist(message.twist.twist) ||
      stamp.nanoseconds() < previous_stamp || !fresh(message.header.stamp, local_max_age_))
    {
      warn_reject("local odometry");
      return;
    }

    const tf2::Transform odom_base = pose_transform(message.pose.pose);
    tf2::Transform map_odom;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      local_poses_.push_back({stamp.nanoseconds(), odom_base});
      const int64_t oldest = stamp.nanoseconds() -
        static_cast<int64_t>(buffer_duration_ * 1.0e9);
      while (!local_poses_.empty() && local_poses_.front().stamp_ns < oldest) {
        local_poses_.pop_front();
      }
      last_local_stamp_.store(stamp.nanoseconds());
      map_odom = map_odom_;
    }

    initialize_frames(message.header.stamp);
#ifdef PIN_SLAM_AS2_STATE_INTERFACE
    // map->odom is a piecewise-constant global correction. Restamping the
    // current correction with each new local sample keeps the complete TF
    // chain queryable while odom->base carries the high-rate motion.
    state_estimator_interface_->setMapToOdomPose(map_odom, message.header.stamp, false);
    state_estimator_interface_->setOdomToBaseLinkPose(message.pose, message.header.stamp);
    state_estimator_interface_->setTwistInBaseFrame(message.twist, message.header.stamp);
#else
    const tf2::Transform earth_base = earth_map_ * map_odom * odom_base;
    geometry_msgs::msg::PoseStamped output_pose;
    output_pose.header = message.header;
    output_pose.header.frame_id = get_earth_frame();
    tf2::toMsg(earth_base, output_pose.pose);
    geometry_msgs::msg::TwistStamped output_twist;
    output_twist.header = message.header;
    output_twist.header.frame_id = get_base_frame();
    output_twist.twist = message.twist.twist;
    publish_pose(output_pose);
    publish_twist(output_twist);
#endif
  }

  void receive_pin(const nav_msgs::msg::Odometry & message)
  {
    const rclcpp::Time stamp(message.header.stamp, node_ptr_->get_clock()->get_clock_type());
    if (message.header.frame_id != input_world_ || message.child_frame_id != input_body_ ||
      !finite_pose(message.pose.pose) || stamp.nanoseconds() <= last_pin_stamp_.load() ||
      !fresh(message.header.stamp, pin_max_age_))
    {
      warn_reject("PIN-SLAM");
      return;
    }

    LocalPose nearest{0, tf2::Transform::getIdentity()};
    int64_t nearest_error = std::numeric_limits<int64_t>::max();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & candidate : local_poses_) {
        const int64_t error = std::llabs(candidate.stamp_ns - stamp.nanoseconds());
        if (error < nearest_error) {
          nearest = candidate;
          nearest_error = error;
        }
      }
    }
    if (nearest_error > static_cast<int64_t>(pairing_tolerance_ * 1.0e9)) {
      warn_reject("PIN-SLAM");
      return;
    }

    const tf2::Transform map_sensor = pose_transform(message.pose.pose);
    const tf2::Transform map_base = map_sensor * sensor_base_;
    const tf2::Transform correction = map_base * nearest.odom_base.inverse();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      map_odom_ = correction;
      last_pin_stamp_.store(stamp.nanoseconds());
    }
    initialize_frames(message.header.stamp);
#ifdef PIN_SLAM_AS2_STATE_INTERFACE
    state_estimator_interface_->setMapToOdomPose(correction, message.header.stamp, false);
#endif
  }
};
}  // namespace pin_slam

PLUGINLIB_EXPORT_CLASS(
  pin_slam::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
