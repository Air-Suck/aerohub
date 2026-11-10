#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <as2_state_estimator/plugin_base.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace pin_slam {
class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase {
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  tf2::Transform earth_world_;
  tf2::Transform sensor_base_;
  std::string input_world_;
  std::string input_body_;
  double max_age_ = 0.5;
  int64_t last_stamp_ = -1;
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

  geometry_msgs::msg::TransformStamped transform_message(
    const tf2::Transform & transform, const std::string & parent,
    const std::string & child, const builtin_interfaces::msg::Time & stamp)
  {
    geometry_msgs::msg::TransformStamped message;
    message.header.stamp = stamp;
    message.header.frame_id = parent;
    message.child_frame_id = child;
    message.transform = tf2::toMsg(transform);
    return message;
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
    output = transform_message(
      earth_world_, get_earth_frame(), get_map_frame(), node_ptr_->now());
    return true;
  }
#endif

private:
  void setup_common()
  {
    input_world_ = node_ptr_->getParameter<std::string>("input_world_frame", "pin_slam_map");
    input_body_ = node_ptr_->getParameter<std::string>("input_body_frame", "pin_slam_lidar");
    max_age_ = node_ptr_->getParameter<double>("max_age", 0.5);
    if (!std::isfinite(max_age_) || max_age_ <= 0.0) {
      throw std::runtime_error("max_age must be positive and finite");
    }
    earth_world_ = transform_parameter("earth_from_pin_world");
    sensor_base_ = transform_parameter("sensor_from_base");
#ifndef PIN_SLAM_AS2_STATE_INTERFACE
    const auto stamp = node_ptr_->now();
    publish_static_transform(transform_message(
      earth_world_, get_earth_frame(), get_map_frame(), stamp));
    publish_static_transform(transform_message(
      tf2::Transform::getIdentity(), get_map_frame(), get_odom_frame(), stamp));
#else
    if (node_ptr_->getParameter<double>("publish_hz", 0.0) > 0.0) {
      throw std::runtime_error("PIN-SLAM requires publish_hz=0 to avoid stale state republication");
    }
#endif
    subscription_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      "pin_slam/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) {receive(*message);});
  }

  void receive(const nav_msgs::msg::Odometry & message)
  {
    auto reject = [this]() {
        RCLCPP_WARN_THROTTLE(
          node_ptr_->get_logger(), *node_ptr_->get_clock(), 3000,
          "Reject PIN-SLAM state: frame, numeric, timestamp, or freshness contract violated");
      };
    if (message.header.frame_id != input_world_ || message.child_frame_id != input_body_) {
      reject();
      return;
    }
    const auto & pose = message.pose.pose;
    const auto & twist = message.twist.twist;
    for (const double value : {
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
        twist.linear.x, twist.linear.y, twist.linear.z,
        twist.angular.x, twist.angular.y, twist.angular.z})
    {
      if (!std::isfinite(value)) {reject(); return;}
    }
    const rclcpp::Time stamp(message.header.stamp, node_ptr_->get_clock()->get_clock_type());
    const double age = (node_ptr_->now() - stamp).seconds();
    if (stamp.nanoseconds() <= last_stamp_ || age > max_age_ || age < -0.05) {
      reject();
      return;
    }
    tf2::Quaternion rotation(
      pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
    if (std::abs(rotation.length2() - 1.0) > 0.01) {reject(); return;}
    rotation.normalize();
    const tf2::Transform world_sensor(
      rotation, tf2::Vector3(pose.position.x, pose.position.y, pose.position.z));
    const tf2::Transform world_base = world_sensor * sensor_base_;
    const tf2::Transform earth_base = earth_world_ * world_base;

    geometry_msgs::msg::PoseStamped output_pose;
    output_pose.header = message.header;
    output_pose.header.frame_id = get_earth_frame();
    output_pose.pose.position.x = earth_base.getOrigin().x();
    output_pose.pose.position.y = earth_base.getOrigin().y();
    output_pose.pose.position.z = earth_base.getOrigin().z();
    output_pose.pose.orientation = tf2::toMsg(earth_base.getRotation());

    const tf2::Vector3 omega(twist.angular.x, twist.angular.y, twist.angular.z);
    const tf2::Vector3 linear(twist.linear.x, twist.linear.y, twist.linear.z);
    const auto base_rotation = sensor_base_.getBasis().transpose();
    const auto base_linear = base_rotation * (linear + omega.cross(sensor_base_.getOrigin()));
    const auto base_omega = base_rotation * omega;
    geometry_msgs::msg::TwistStamped output_twist;
    output_twist.header = message.header;
    output_twist.header.frame_id = get_base_frame();
    output_twist.twist.linear.x = base_linear.x();
    output_twist.twist.linear.y = base_linear.y();
    output_twist.twist.linear.z = base_linear.z();
    output_twist.twist.angular.x = base_omega.x();
    output_twist.twist.angular.y = base_omega.y();
    output_twist.twist.angular.z = base_omega.z();
    last_stamp_ = stamp.nanoseconds();

#ifdef PIN_SLAM_AS2_STATE_INTERFACE
    if (!frames_initialized_) {
      state_estimator_interface_->setEarthToMap(earth_world_, message.header.stamp, true);
      state_estimator_interface_->setMapToOdomPose(
        tf2::Transform::getIdentity(), message.header.stamp, true);
      frames_initialized_ = true;
    }
    state_estimator_interface_->setOdomToBaseLinkPose(world_base, message.header.stamp);
    geometry_msgs::msg::TwistWithCovariance body_twist;
    body_twist.twist = output_twist.twist;
    state_estimator_interface_->setTwistInBaseFrame(body_twist, message.header.stamp);
#else
    publish_transform(transform_message(
      world_base, get_odom_frame(), get_base_frame(), message.header.stamp));
    publish_pose(output_pose);
    publish_twist(output_twist);
#endif
  }
};
}  // namespace pin_slam

PLUGINLIB_EXPORT_CLASS(
  pin_slam::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
