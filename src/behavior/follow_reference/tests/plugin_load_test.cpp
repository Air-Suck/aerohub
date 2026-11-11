#include <follow_reference_behavior/follow_reference_base.hpp>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <iostream>
int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  pluginlib::ClassLoader<follow_reference_base::FollowReferenceBase> loader(
    "as2_behaviors_motion", "follow_reference_base::FollowReferenceBase");
  auto plugin = loader.createSharedInstance("follow_reference_plugin_external_acro::Plugin");
  std::cout << "FollowReference plugin loaded\n";
  plugin.reset();
  rclcpp::shutdown();
}
