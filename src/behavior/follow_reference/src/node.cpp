#include <follow_reference_behavior/follow_reference_behavior.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FollowReferenceBehavior>();
  // Lifecycle/state/control callbacks stay serialized in the default group.
  // Mode feedback has its own group, so TF waits cannot starve its watchdog.
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);executor.spin();
  rclcpp::shutdown();
}
