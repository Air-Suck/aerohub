// Copyright 2026 AeroHub contributors. Licensed under Apache-2.0.
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <deque>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <rog_map/rog_map.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <uav_rog_map_interfaces/msg/map_status.hpp>
#include <uav_rog_map_interfaces/srv/check_line.hpp>
#include <uav_rog_map_interfaces/srv/query_voxel.hpp>

namespace uav_rog_map
{
using Cloud = sensor_msgs::msg::PointCloud2;
using CheckLine = uav_rog_map_interfaces::srv::CheckLine;
using QueryVoxel = uav_rog_map_interfaces::srv::QueryVoxel;
using MapStatus = uav_rog_map_interfaces::msg::MapStatus;
using SteadyClock = std::chrono::steady_clock;

class MapCore final : public rog_map::ROGMap
{
public:
  MapCore(const std::string & config_path, std::function<double()> now_seconds)
  : now_seconds_(std::move(now_seconds))
  {
    cfg_ = rog_map::Config(config_path);
    if (cfg_.ros_callback_en) {
      throw std::invalid_argument("ROG-Map core ros_callback must be disabled for the AeroHub Adapter");
    }
    std::filesystem::create_directories("/tmp/stage3-super");
    init();
  }

  bool ready() const {return !map_empty_;}
  bool esdf_enabled() const {return cfg_.esdf_en;}
  double esdf_distance(const rog_map::Vec3f & point) const
  {
    return cfg_.esdf_en ? esdf_map_->getDistance(point) : std::numeric_limits<double>::quiet_NaN();
  }
  void esdf_slice(
    const rog_map::Vec3f & minimum, const rog_map::Vec3f & maximum, double height,
    pcl::PointCloud<pcl::PointXYZI> & output) const
  {
    output.clear();
    if (cfg_.esdf_en) {
      esdf_map_->getPositiveESDFPointCloud(minimum, maximum, height, output);
    }
  }

private:
  const double getSystemWalltimeNow() override {return now_seconds_();}
  std::function<double()> now_seconds_;
};

struct PendingCloud
{
  Cloud::ConstSharedPtr message;
  SteadyClock::time_point received;
};

class ROGMapNode final : public rclcpp::Node
{
public:
  ROGMapNode()
  : Node("rog_map"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    config_path_ = declare_parameter<std::string>("config_path", "");
    global_frame_ = declare_parameter<std::string>("global_frame", "drone0/odom");
    sensor_frame_ = declare_parameter<std::string>("sensor_frame", "drone0/Lidar");
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "navigation/points");
    transform_wait_ = declare_parameter("transform_wait", 0.25);
    max_input_age_ = declare_parameter("max_input_age", 1.0);
    stale_timeout_ = declare_parameter("stale_timeout", 1.0);
    publish_rate_ = declare_parameter("publish_rate", 1.0);
    queue_size_ = static_cast<size_t>(declare_parameter("queue_size", 8));
    const auto range = declare_parameter<std::vector<double>>(
      "publication_range", {20.0, 20.0, 6.0});

    if (config_path_.empty() || !std::filesystem::is_regular_file(config_path_)) {
      throw std::invalid_argument("config_path must name a readable ROG-Map YAML file");
    }
    if (global_frame_.empty() || sensor_frame_.empty() || cloud_topic_.empty() ||
      !finite_positive(transform_wait_) || !finite_positive(max_input_age_) ||
      !finite_positive(stale_timeout_) || !finite_positive(publish_rate_) || queue_size_ == 0 ||
      range.size() != 3 || !std::all_of(range.begin(), range.end(), finite_positive))
    {
      throw std::invalid_argument("ROG-Map frames, topics, queue and time/range limits are invalid");
    }
    publication_range_ = rog_map::Vec3f(range[0], range[1], range[2]);
    map_ = std::make_unique<MapCore>(config_path_, [this]() {return now().seconds();});

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(queue_size_);
    const auto state_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    cloud_sub_ = create_subscription<Cloud>(
      cloud_topic_, sensor_qos,
      [this](Cloud::ConstSharedPtr message) {enqueue(std::move(message));});
    occupancy_pub_ = create_publisher<Cloud>("map/occupied", state_qos);
    inflated_pub_ = create_publisher<Cloud>("map/inflated_occupied", state_qos);
    esdf_pub_ = create_publisher<Cloud>("map/esdf_slice", state_qos);
    status_pub_ = create_publisher<MapStatus>("map/status", state_qos);
    query_service_ = create_service<QueryVoxel>(
      "map/query_voxel",
      [this](const QueryVoxel::Request::SharedPtr request, QueryVoxel::Response::SharedPtr response) {
        query_voxel(*request, *response);
      });
    line_service_ = create_service<CheckLine>(
      "map/check_line",
      [this](const CheckLine::Request::SharedPtr request, CheckLine::Response::SharedPtr response) {
        check_line(*request, *response);
      });
    input_timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {process_pending();});
    status_timer_ = create_wall_timer(std::chrono::milliseconds(100), [this]() {publish_status();});
    const auto publication_period = std::chrono::duration<double>(1.0 / publish_rate_);
    publication_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(publication_period),
      [this]() {publish_maps();});

    RCLCPP_INFO(
      get_logger(), "ROG-Map awaiting registered XYZ on %s in %s with origin %s",
      cloud_topic_.c_str(), global_frame_.c_str(), sensor_frame_.c_str());
  }

private:
  static bool finite_positive(double value) {return std::isfinite(value) && value > 0.0;}

  void enqueue(Cloud::ConstSharedPtr message)
  {
    if (!epoch_valid_) {
      ++rejected_time_;
      return;
    }
    if (!valid_layout(*message)) {
      ++rejected_malformed_;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Dropping PointCloud2 without packed little-endian scalar FLOAT32 XYZ in the global frame");
      return;
    }
    if (pending_.size() >= queue_size_) {
      pending_.pop_front();
      ++rejected_tf_;
    }
    pending_.push_back({std::move(message), SteadyClock::now()});
  }

  bool valid_layout(const Cloud & message) const
  {
    if (message.header.frame_id != global_frame_ ||
      rclcpp::Time(message.header.stamp).nanoseconds() <= 0 || message.width == 0 ||
      message.height == 0 || message.is_bigendian || message.point_step < 12 ||
      message.row_step != message.width * message.point_step ||
      message.data.size() != static_cast<size_t>(message.row_step) * message.height)
    {
      return false;
    }
    size_t expected_offset = 0;
    for (const auto * field_name : {"x", "y", "z"}) {
      const auto field = std::find_if(
        message.fields.begin(), message.fields.end(),
        [field_name](const auto & candidate) {return candidate.name == field_name;});
      if (field == message.fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
        field->count != 1 || field->offset != expected_offset ||
        field->offset + sizeof(float) > message.point_step)
      {
        return false;
      }
      expected_offset += sizeof(float);
    }
    return true;
  }

  void process_pending()
  {
    const auto current = now();
    if (last_now_ns_ >= 0 && current.nanoseconds() < last_now_ns_) {
      pending_.clear();
      epoch_valid_ = false;
      RCLCPP_ERROR(get_logger(), "ROS time moved backwards; map epoch invalid until process restart");
    }
    last_now_ns_ = current.nanoseconds();
    if (!epoch_valid_) {
      return;
    }

    for (auto it = pending_.begin(); it != pending_.end();) {
      const auto stamp = rclcpp::Time(it->message->header.stamp);
      const auto age = (current - stamp).seconds();
      const auto waited = std::chrono::duration<double>(SteadyClock::now() - it->received).count();
      if (stamp.nanoseconds() <= last_update_.nanoseconds() || age > max_input_age_) {
        ++rejected_time_;
        it = pending_.erase(it);
        continue;
      }
      if (waited > transform_wait_) {
        age < 0.0 ? ++rejected_time_ : ++rejected_tf_;
        it = pending_.erase(it);
        continue;
      }
      if (age < 0.0) {
        ++it;
        continue;
      }
      try {
        const auto transform = tf_buffer_.lookupTransform(global_frame_, sensor_frame_, stamp);
        pcl::PointCloud<pcl::PointXYZ> xyz;
        pcl::fromROSMsg(*it->message, xyz);
        rog_map::PointCloud cloud;
        cloud.reserve(xyz.size());
        for (const auto & point : xyz) {
          if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            continue;
          }
          rog_map::PointType converted;
          converted.x = point.x;
          converted.y = point.y;
          converted.z = point.z;
          converted.intensity = 0.0F;
          cloud.push_back(converted);
        }
        if (cloud.empty()) {
          ++rejected_malformed_;
          it = pending_.erase(it);
          continue;
        }
        const auto & translation = transform.transform.translation;
        const auto & rotation = transform.transform.rotation;
        const rog_map::Pose sensor_pose{
          rog_map::Vec3f(translation.x, translation.y, translation.z),
          super_utils::Quatf(rotation.w, rotation.x, rotation.y, rotation.z)};
        map_->updateMap(cloud, sensor_pose);
        last_update_ = stamp;
        last_point_count_ = cloud.size();
        ++accepted_clouds_;
        it = pending_.erase(it);
      } catch (const tf2::TransformException &) {
        ++it;
      } catch (const std::exception & error) {
        ++rejected_malformed_;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
        it = pending_.erase(it);
      }
    }
  }

  bool ready() const
  {
    if (!epoch_valid_ || !map_->ready() || last_update_.nanoseconds() <= 0) {
      return false;
    }
    const auto age = (now() - last_update_).seconds();
    return age >= 0.0 && age <= stale_timeout_;
  }

  void publish_status()
  {
    MapStatus status;
    status.header.stamp = now();
    status.header.frame_id = global_frame_;
    status.last_update = last_update_;
    status.ready = ready();
    status.epoch_valid = epoch_valid_;
    status.accepted_clouds = accepted_clouds_;
    status.rejected_malformed = rejected_malformed_;
    status.rejected_time = rejected_time_;
    status.rejected_tf = rejected_tf_;
    status.last_point_count = last_point_count_;
    status.input_age = last_update_.nanoseconds() > 0 ? (now() - last_update_).seconds() : -1.0;
    status.resolution = map_->getResolution();
    status.inflation_resolution = map_->getInfResolution();
    status_pub_->publish(status);
  }

  template<typename PointCloudT>
  Cloud to_message(const PointCloudT & points) const
  {
    Cloud message;
    pcl::toROSMsg(points, message);
    message.header.frame_id = global_frame_;
    message.header.stamp = last_update_;
    return message;
  }

  void publish_maps()
  {
    if (!ready()) {
      return;
    }
    const auto state = map_->getRobotState();
    rog_map::Vec3f minimum = state.p - publication_range_ / 2.0;
    rog_map::Vec3f maximum = state.p + publication_range_ / 2.0;
    map_->boundBoxByLocalMap(minimum, maximum);
    if ((maximum - minimum).minCoeff() <= 0.0) {
      return;
    }

    super_utils::vec_E<rog_map::Vec3f> occupied;
    super_utils::vec_E<rog_map::Vec3f> inflated;
    map_->boxSearch(minimum, maximum, super_utils::OCCUPIED, occupied);
    map_->boxSearchInflate(minimum, maximum, super_utils::OCCUPIED, inflated);
    pcl::PointCloud<pcl::PointXYZ> occupied_cloud;
    pcl::PointCloud<pcl::PointXYZ> inflated_cloud;
    occupied_cloud.reserve(occupied.size());
    inflated_cloud.reserve(inflated.size());
    for (const auto & point : occupied) {
      occupied_cloud.emplace_back(point.x(), point.y(), point.z());
    }
    for (const auto & point : inflated) {
      inflated_cloud.emplace_back(point.x(), point.y(), point.z());
    }
    occupancy_pub_->publish(to_message(occupied_cloud));
    inflated_pub_->publish(to_message(inflated_cloud));

    pcl::PointCloud<pcl::PointXYZI> esdf;
    map_->esdf_slice(minimum, maximum, state.p.z(), esdf);
    esdf_pub_->publish(to_message(esdf));
  }

  static rog_map::Vec3f vector_from_point(const geometry_msgs::msg::Point & point)
  {
    return {point.x, point.y, point.z};
  }

  void query_voxel(const QueryVoxel::Request & request, QueryVoxel::Response & response)
  {
    response.ready = ready();
    response.map_stamp = last_update_;
    response.frame_id = global_frame_;
    const auto point = vector_from_point(request.point);
    if (!point.array().isFinite().all()) {
      response.message = "query point is not finite";
      return;
    }
    if (!response.ready) {
      response.message = "map is not ready or input is stale";
      return;
    }
    response.occupancy = static_cast<uint8_t>(map_->getGridType(point));
    response.inflated_occupancy = static_cast<uint8_t>(map_->getInfGridType(point));
    response.has_esdf = map_->esdf_enabled() && response.occupancy != super_utils::OUT_OF_MAP;
    if (response.has_esdf) {
      response.esdf_distance = map_->esdf_distance(point);
    }
    response.message = "ok";
  }

  void check_line(const CheckLine::Request & request, CheckLine::Response & response)
  {
    response.ready = ready();
    response.map_stamp = last_update_;
    response.frame_id = global_frame_;
    const auto start = vector_from_point(request.start);
    const auto end = vector_from_point(request.end);
    if (!start.array().isFinite().all() || !end.array().isFinite().all()) {
      response.message = "line endpoints are not finite";
      return;
    }
    if (!response.ready) {
      response.message = "map is not ready or input is stale";
      return;
    }
    response.clear = map_->isLineFree(
      start, end, request.use_inflated, request.unknown_as_occupied);
    response.message = "ok";
  }

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<MapCore> map_;
  std::string config_path_, global_frame_, sensor_frame_, cloud_topic_;
  double transform_wait_{}, max_input_age_{}, stale_timeout_{}, publish_rate_{};
  size_t queue_size_{};
  rog_map::Vec3f publication_range_{};
  std::deque<PendingCloud> pending_;
  rclcpp::Time last_update_{0, 0, RCL_ROS_TIME};
  int64_t last_now_ns_{-1};
  bool epoch_valid_{true};
  uint64_t accepted_clouds_{0}, rejected_malformed_{0}, rejected_time_{0}, rejected_tf_{0};
  uint64_t last_point_count_{0};
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::Publisher<Cloud>::SharedPtr occupancy_pub_, inflated_pub_, esdf_pub_;
  rclcpp::Publisher<MapStatus>::SharedPtr status_pub_;
  rclcpp::Service<QueryVoxel>::SharedPtr query_service_;
  rclcpp::Service<CheckLine>::SharedPtr line_service_;
  rclcpp::TimerBase::SharedPtr input_timer_, status_timer_, publication_timer_;
};
}  // namespace uav_rog_map

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<uav_rog_map::ROGMapNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("uav_rog_map"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
