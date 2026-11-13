#pragma once
#include <algorithm>
#include <chrono>
#include <functional>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
namespace ego_ros {
inline rclcpp::Node::SharedPtr node;
inline rclcpp::Time now() { return node->now(); }
inline rclcpp::Time from_seconds(double seconds) {
  return rclcpp::Time(static_cast<int64_t>(seconds * 1e9), RCL_ROS_TIME);
}
inline std::string topic(const std::string &name) {
  return !name.empty() && name.front() == '/' ? name : "~/" + name;
}
template<class T>
void param(const rclcpp::Node::SharedPtr &nh, std::string name, T &value, const T &fallback) {
  std::replace(name.begin(), name.end(), '/', '.');
  if (!nh->has_parameter(name)) nh->declare_parameter<T>(name, fallback);
  nh->get_parameter(name, value);
}
}
