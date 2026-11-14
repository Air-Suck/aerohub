#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace {

class SensorQosAdapter final : public rclcpp::Node {
 public:
  SensorQosAdapter() : Node("fast_livo2_sensor_qos_adapter") {
    const auto imu_input = declare_parameter<std::string>(
      "imu_input_topic", "/drone0/sensor_measurements/imu/data");
    const auto imu_output = declare_parameter<std::string>(
      "imu_output_topic", "/fast_livo2/imu");
    const auto image_input = declare_parameter<std::string>(
      "image_input_topic", "/drone0/sensor_measurements/FrontCamera/rgb/image");
    const auto image_output = declare_parameter<std::string>(
      "image_output_topic", "/fast_livo2/image");

    const auto reliable = rclcpp::QoS(rclcpp::KeepLast(20)).reliable();
    imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>(imu_output, reliable);
    image_publisher_ = create_publisher<sensor_msgs::msg::Image>(image_output, reliable);
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
      imu_input, rclcpp::SensorDataQoS().keep_last(20),
      [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
        imu_publisher_->publish(*message);
      });
    image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
      image_input, rclcpp::SensorDataQoS().keep_last(2),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        image_publisher_->publish(*message);
      });
  }

 private:
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
};

}  // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorQosAdapter>());
  rclcpp::shutdown();
  return 0;
}
