#include <plan_manage/ego_replan_fsm.h>
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ego_planner_node");
  ego_ros::node = node;
  {
    ego_planner::EGOReplanFSM fsm;
    fsm.init(node);
    rclcpp::spin(node);
  }
  ego_ros::node.reset();
  rclcpp::shutdown();
  return 0;
}
