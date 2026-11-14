#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/utils.h>

namespace {

std::int64_t nanoseconds(const builtin_interfaces::msg::Time &stamp) {
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

bool finite_quaternion(const geometry_msgs::msg::Quaternion &value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z) && std::isfinite(value.w);
}

class OutputAdapter final : public rclcpp::Node {
 public:
  OutputAdapter() : Node("fast_livo2_output_adapter") {
    const auto upstream_odom = declare_parameter<std::string>(
      "upstream_odom_topic", "/LIVO2/imu_propagate");
    const auto upstream_cloud = declare_parameter<std::string>(
      "upstream_cloud_topic", "/cloud_registered");
    const auto imu_topic = declare_parameter<std::string>(
      "imu_topic", "/drone0/sensor_measurements/imu/data");
    const auto output_odom = declare_parameter<std::string>(
      "output_odom_topic", "/drone0/fast_livo2/odometry");
    const auto output_cloud = declare_parameter<std::string>(
      "output_cloud_topic", "/drone0/perception/fast_livo2/registered_points");
    world_frame_ = declare_parameter<std::string>("world_frame", "camera_init");
    body_frame_ = declare_parameter<std::string>("body_frame", "fast_livo_imu");
    cloud_frame_ = declare_parameter<std::string>("cloud_frame", "drone0/livo_world");
    max_imu_delta_ns_ = static_cast<std::int64_t>(
      declare_parameter<double>("max_imu_delta_s", 0.02) * 1000000000.0);
    if (max_imu_delta_ns_ <= 0) {
      throw std::invalid_argument("max_imu_delta_s must be positive");
    }

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      output_odom, rclcpp::SensorDataQoS().keep_last(20));
    cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      output_cloud, rclcpp::SensorDataQoS().keep_last(2));
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_topic, rclcpp::SensorDataQoS().keep_last(20),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) { latest_imu_ = message; });
    odom_subscription_ = create_subscription<nav_msgs::msg::Odometry>(
      upstream_odom, rclcpp::SensorDataQoS().keep_last(20),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { convert_odometry(*message); });
    cloud_subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      upstream_cloud, rclcpp::SensorDataQoS().keep_last(2),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { convert_cloud(*message); });
  }

 private:
  void convert_odometry(const nav_msgs::msg::Odometry &source) {
    const auto source_stamp = nanoseconds(source.header.stamp);
    if (source_stamp <= last_odom_stamp_ns_ || !latest_imu_) {
      return;
    }
    const auto imu_stamp = nanoseconds(latest_imu_->header.stamp);
    if (std::llabs(source_stamp - imu_stamp) > max_imu_delta_ns_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejecting odometry without paired IMU sample");
      return;
    }
    if (!finite_quaternion(source.pose.pose.orientation)) {
      return;
    }
    const auto &orientation = source.pose.pose.orientation;
    tf2::Quaternion world_from_body(
      orientation.x, orientation.y, orientation.z, orientation.w);
    if (std::abs(world_from_body.length2() - 1.0) > 0.01) {
      return;
    }
    world_from_body.normalize();
    const auto &world_linear = source.twist.twist.linear;
    const tf2::Vector3 linear_body = tf2::quatRotate(
      world_from_body.inverse(),
      tf2::Vector3(world_linear.x, world_linear.y, world_linear.z));

    nav_msgs::msg::Odometry target = source;
    target.header.frame_id = world_frame_;
    target.child_frame_id = body_frame_;
    target.twist.twist.linear.x = linear_body.x();
    target.twist.twist.linear.y = linear_body.y();
    target.twist.twist.linear.z = linear_body.z();
    target.twist.twist.angular = latest_imu_->angular_velocity;
    odom_publisher_->publish(target);
    last_odom_stamp_ns_ = source_stamp;
    last_odom_stamp_ = source.header.stamp;
  }

  void convert_cloud(const sensor_msgs::msg::PointCloud2 &source) {
    if (last_odom_stamp_ns_ <= last_cloud_stamp_ns_ ||
        source.width == 0 || source.height == 0) {
      return;
    }
    sensor_msgs::msg::PointCloud2 target = source;
    target.header.stamp = last_odom_stamp_;
    target.header.frame_id = cloud_frame_;
    cloud_publisher_->publish(target);
    last_cloud_stamp_ns_ = last_odom_stamp_ns_;
  }

  std::string world_frame_;
  std::string body_frame_;
  std::string cloud_frame_;
  std::int64_t max_imu_delta_ns_{20000000};
  std::int64_t last_odom_stamp_ns_{-1};
  std::int64_t last_cloud_stamp_ns_{-1};
  builtin_interfaces::msg::Time last_odom_stamp_;
  sensor_msgs::msg::Imu::ConstSharedPtr latest_imu_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_subscription_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OutputAdapter>());
  rclcpp::shutdown();
  return 0;
}
