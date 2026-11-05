// Inspect the real GridMap implementation; this executable never sends flight commands.
#include <swarm_graph/ros2.hpp>
#include <plan_env/grid_map.h>
#include <std_srvs/srv/trigger.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>

int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp::Node>("swarm_map_probe");
  swarm_ros::node=node;
  {
    GridMap map;map.initMap(node);
    sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud;
    nav_msgs::msg::Odometry::ConstSharedPtr odom;
    auto cloud_sub=node->create_subscription<sensor_msgs::msg::PointCloud2>("~/grid_map/cloud",rclcpp::SensorDataQoS(),
      [&cloud](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg){cloud=msg;});
    auto odom_sub=node->create_subscription<nav_msgs::msg::Odometry>("~/grid_map/odom",rclcpp::SensorDataQoS(),
      [&odom](nav_msgs::msg::Odometry::ConstSharedPtr msg){odom=msg;});
    auto service=node->create_service<std_srvs::srv::Trigger>("~/inspect",
      [&](const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response) {
        int considered=0,occupied=0,near_surface=0;
        if (cloud && odom) {
          pcl::PointCloud<pcl::PointXYZ> points;pcl::fromROSMsg(*cloud,points);
          const auto &p=odom->pose.pose.position;
          const Eigen::Vector3d camera(p.x,p.y,p.z);
          const double rx=node->get_parameter("grid_map.local_update_range_x").as_double();
          const double ry=node->get_parameter("grid_map.local_update_range_y").as_double();
          const double rz=node->get_parameter("grid_map.local_update_range_z").as_double();
          for (size_t i=0;i<points.size();i+=std::max<size_t>(1,points.size()/1000)) {
            const auto &v=points[i];Eigen::Vector3d pos(v.x,v.y,v.z);
            if (!pos.allFinite() || !map.isInMap(pos)) continue;
            const auto d=pos-camera;
            if (std::abs(d.x())>=rx || std::abs(d.y())>=ry || std::abs(d.z())>=rz) continue;
            ++considered;
            if (map.getInflateOccupancy(pos)==1) ++occupied;
            const auto distance=map.getDistance(pos);
            if (std::isfinite(distance) && std::abs(distance)<.3) ++near_surface;
          }
        }
        response->success=map.odomValid() && !map.getOdomDepthTimeout();
        std::ostringstream text;
        text<<"{\"healthy\":"<<(response->success?"true":"false")
            <<",\"sampled_input_points\":"<<considered<<",\"occupied_matches\":"<<occupied
            <<",\"esdf_surface_matches\":"<<near_surface<<"}";
        response->message=text.str();
      });
    rclcpp::spin(node);
  }
  swarm_ros::node.reset();node.reset();rclcpp::shutdown();
}
