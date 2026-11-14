#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

namespace {

bool has_float32_field(const sensor_msgs::msg::PointCloud2 &message, const std::string &name) {
  for (const auto &field : message.fields) {
    if (field.name == name) {
      return field.datatype == sensor_msgs::msg::PointField::FLOAT32 && field.count == 1;
    }
  }
  return false;
}

class PointCloudToLivox final : public rclcpp::Node {
 public:
  PointCloudToLivox() : Node("fast_livo2_pointcloud_adapter") {
    const auto input = declare_parameter<std::string>(
      "input_topic", "/drone0/sensor_measurements/Lidar/points");
    const auto output = declare_parameter<std::string>(
      "output_topic", "/fast_livo2/airsim_lidar");
    expected_frame_ = declare_parameter<std::string>(
      "expected_frame", "drone0/Lidar");
    output_frame_ = declare_parameter<std::string>(
      "output_frame", "fast_livo_lidar");
    publisher_ = create_publisher<livox_ros_driver2::msg::CustomMsg>(
      output, rclcpp::QoS(rclcpp::KeepLast(2)).reliable());
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input, rclcpp::SensorDataQoS().keep_last(2),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { convert(*message); });
  }

 private:
  void convert(const sensor_msgs::msg::PointCloud2 &source) {
    if (source.header.frame_id != expected_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejecting point cloud with unexpected frame");
      return;
    }
    if (!has_float32_field(source, "x") || !has_float32_field(source, "y") ||
        !has_float32_field(source, "z")) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "Rejecting point cloud without float32 XYZ");
      return;
    }
    const std::size_t count = static_cast<std::size_t>(source.width) * source.height;
    if (count == 0 || count > UINT32_MAX) {
      return;
    }

    livox_ros_driver2::msg::CustomMsg target;
    target.header = source.header;
    target.header.frame_id = output_frame_;
    target.timebase =
      static_cast<std::uint64_t>(source.header.stamp.sec) * 1000000000ULL +
      source.header.stamp.nanosec;
    target.point_num = static_cast<std::uint32_t>(count);
    target.lidar_id = 0;
    target.rsvd = std::array<std::uint8_t, 3>{0, 0, 0};
    target.points.reserve(count);

    sensor_msgs::PointCloud2ConstIterator<float> x(source, "x");
    sensor_msgs::PointCloud2ConstIterator<float> y(source, "y");
    sensor_msgs::PointCloud2ConstIterator<float> z(source, "z");
    for (std::size_t index = 0; index < count; ++index, ++x, ++y, ++z) {
      livox_ros_driver2::msg::CustomPoint point;
      point.offset_time = 0;
      point.x = *x;
      point.y = *y;
      point.z = *z;
      point.reflectivity = 0;
      point.tag = 0x10;
      point.line = 0;
      target.points.push_back(point);
    }
    publisher_->publish(target);
  }

  std::string expected_frame_;
  std::string output_frame_;
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PointCloudToLivox>());
  rclcpp::shutdown();
  return 0;
}
