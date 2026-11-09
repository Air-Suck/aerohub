#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <pluginlib/class_list_macros.hpp>
#include <as2_state_estimator/plugin_base.hpp>

namespace fast_livo2 {
class Plugin : public as2_state_estimator_plugin_base::StateEstimatorBase {
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  tf2::Transform earth_world_, imu_base_;
  std::string input_world_, input_body_;
  double max_age_;
  int64_t last_stamp_ = -1;

  tf2::Transform transform_parameter(const std::string &name) {
    auto v = node_ptr_->getParameter<std::vector<double>>(
      name, std::vector<double>{0., 0., 0., 0., 0., 0., 1.});
    if (v.size() != 7) throw std::runtime_error(name + " must be xyz + xyzw");
    for (double x : v) if (!std::isfinite(x)) throw std::runtime_error(name + " is not finite");
    tf2::Quaternion q(v[3], v[4], v[5], v[6]);
    if (std::abs(q.length2() - 1.) > 0.01) throw std::runtime_error(name + " quaternion is not unit");
    q.normalize();
    return tf2::Transform(q, tf2::Vector3(v[0], v[1], v[2]));
  }
  geometry_msgs::msg::TransformStamped tf_message(
    const tf2::Transform &t, const std::string &parent, const std::string &child,
    const builtin_interfaces::msg::Time &stamp) {
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = stamp; out.header.frame_id = parent; out.child_frame_id = child;
    out.transform = tf2::toMsg(t); return out;
  }
public:
  void on_setup() override {
    input_world_ = node_ptr_->getParameter<std::string>("input_world_frame", "camera_init");
    input_body_ = node_ptr_->getParameter<std::string>("input_body_frame", "fast_livo_imu");
    max_age_ = node_ptr_->getParameter<double>("max_age", 0.5);
    if (!(max_age_ > 0.) || !std::isfinite(max_age_)) throw std::runtime_error("invalid max_age");
    earth_world_ = transform_parameter("earth_from_livo_world");
    imu_base_ = transform_parameter("imu_from_base");
    auto stamp = node_ptr_->now();
    publish_static_transform(tf_message(earth_world_, get_earth_frame(), get_map_frame(), stamp));
    publish_static_transform(tf_message(tf2::Transform::getIdentity(), get_map_frame(), get_odom_frame(), stamp));
    // The bridge's namespaced livo_world is an alias for the upstream camera_init chart.
    auto cloud_frame = node_ptr_->getParameter<std::string>("cloud_frame", "drone0/livo_world");
    publish_static_transform(tf_message(tf2::Transform::getIdentity(), get_odom_frame(), cloud_frame, stamp));
    sub_ = node_ptr_->create_subscription<nav_msgs::msg::Odometry>(
      "fast_livo2/odometry", rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) { receive(*msg); });
  }
  bool get_earth_to_map_transform(geometry_msgs::msg::TransformStamped &out) override {
    out = tf_message(earth_world_, get_earth_frame(), get_map_frame(), node_ptr_->now());
    return true;
  }
private:
  void receive(const nav_msgs::msg::Odometry &m) {
    auto reject = [this]() { RCLCPP_WARN_THROTTLE(node_ptr_->get_logger(), *node_ptr_->get_clock(),
      3000, "Reject FAST-LIVO2 state: frame, numeric, timestamp or freshness contract violated"); };
    if (m.header.frame_id != input_world_ || m.child_frame_id != input_body_) { reject(); return; }
    const auto &p = m.pose.pose; const auto &v = m.twist.twist;
    for (double x : {p.position.x,p.position.y,p.position.z,p.orientation.x,p.orientation.y,
      p.orientation.z,p.orientation.w,v.linear.x,v.linear.y,v.linear.z,v.angular.x,v.angular.y,v.angular.z})
      if (!std::isfinite(x)) { reject(); return; }
    rclcpp::Time stamp(m.header.stamp, node_ptr_->get_clock()->get_clock_type());
    double age = (node_ptr_->now() - stamp).seconds();
    // Clock/algorithm resets require restarting the estimator; never silently merge epochs.
    if (stamp.nanoseconds() <= last_stamp_ || age > max_age_ || age < -0.05) { reject(); return; }
    tf2::Quaternion q(p.orientation.x,p.orientation.y,p.orientation.z,p.orientation.w);
    if (std::abs(q.length2() - 1.) > 0.01) { reject(); return; }
    q.normalize();
    tf2::Transform world_imu(q, {p.position.x,p.position.y,p.position.z});
    tf2::Transform world_base = world_imu * imu_base_;
    tf2::Transform earth_base = earth_world_ * world_base;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = m.header; pose.header.frame_id = get_earth_frame();
    pose.pose.position.x = earth_base.getOrigin().x();
    pose.pose.position.y = earth_base.getOrigin().y();
    pose.pose.position.z = earth_base.getOrigin().z();
    pose.pose.orientation = tf2::toMsg(earth_base.getRotation());
    // Input twist is at IMU origin, expressed in IMU axes. Account for lever arm.
    tf2::Vector3 omega(v.angular.x,v.angular.y,v.angular.z);
    tf2::Vector3 linear(v.linear.x,v.linear.y,v.linear.z);
    auto base_rotation = imu_base_.getBasis().transpose();
    auto base_linear = base_rotation * (linear + omega.cross(imu_base_.getOrigin()));
    auto base_omega = base_rotation * omega;
    geometry_msgs::msg::TwistStamped twist;
    twist.header = m.header; twist.header.frame_id = get_base_frame();
    twist.twist.linear.x=base_linear.x(); twist.twist.linear.y=base_linear.y(); twist.twist.linear.z=base_linear.z();
    twist.twist.angular.x=base_omega.x(); twist.twist.angular.y=base_omega.y(); twist.twist.angular.z=base_omega.z();
    last_stamp_ = stamp.nanoseconds();
    publish_transform(tf_message(world_base, get_odom_frame(), get_base_frame(), m.header.stamp));
    publish_pose(pose); publish_twist(twist);
  }
};
}
PLUGINLIB_EXPORT_CLASS(fast_livo2::Plugin, as2_state_estimator_plugin_base::StateEstimatorBase)
