#include "pixhawk_platform.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <chrono>
#include <limits>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include <thread>
class Harness:public as2_platform_pixhawk::PixhawkPlatform {
public:
 explicit Harness(const rclcpp::NodeOptions& options):PixhawkPlatform(options){}
 void mode(){
   as2_msgs::msg::ControlMode m;m.control_mode=m.BODY_RATES;m.yaw_mode=m.YAW_SPEED;
   platform_info_msg_.current_control_mode=m;
   if(!ownSetPlatformControlMode(m))throw std::runtime_error("mode rejected");
 }
 void attitude(){
   as2_msgs::msg::ControlMode m;m.control_mode=m.ATTITUDE;m.yaw_mode=m.YAW_ANGLE;
   platform_info_msg_.current_control_mode=m;
   if(!ownSetPlatformControlMode(m))throw std::runtime_error("attitude mode rejected");
 }
 void posePair(const geometry_msgs::msg::PoseStamped& pose,const as2_msgs::msg::Thrust& thrust){command_pose_msg_=pose;command_thrust_msg_=thrust;}
 void pair(const geometry_msgs::msg::TwistStamped& rates,const as2_msgs::msg::Thrust& thrust){command_twist_msg_=rates;command_thrust_msg_=thrust;}
};
int main(int argc,char**argv){
 rclcpp::init(argc,argv);
 const auto share=ament_index_cpp::get_package_share_directory("as2_platform_pixhawk");
 rclcpp::NodeOptions options;options.arguments({"--ros-args","-r","__ns:=/ipc_pairing_test","-p","control_modes_file:="+share+"/config/control_modes.yaml","--params-file",share+"/config/platform_config_file.yaml","-p","fmu_prefix:=/ipc_pairing"});
 auto platform=std::make_shared<Harness>(options);platform->mode();
 geometry_msgs::msg::TwistStamped rate;as2_msgs::msg::Thrust thrust;
 rate.header.frame_id=thrust.header.frame_id=platform->getBaseFrameId();rate.header.stamp=thrust.header.stamp=platform->now();
 rate.twist.angular.x=.2;rate.twist.angular.y=-.3;rate.twist.angular.z=.4;thrust.thrust=7.5;
 platform->pair(rate,thrust);if(!platform->ownSendCommand())return 1;
 // Rate arrives first: the older thrust must not be combined with it.
 auto next=rate;next.header.stamp=rclcpp::Time(rate.header.stamp)+rclcpp::Duration::from_seconds(.001);
 platform->pair(next,thrust);if(platform->ownSendCommand())return 2;
 // Thrust arrives first, same invariant in the other order.
 auto next_thrust=thrust;next_thrust.header.stamp=next.header.stamp;
 platform->pair(rate,next_thrust);if(platform->ownSendCommand())return 3;
 auto bad=thrust;bad.thrust=std::numeric_limits<float>::quiet_NaN();
 platform->pair(rate,bad);if(platform->ownSendCommand())return 4;
 auto stale=rate;auto stale_thrust=thrust;
 stale.header.stamp=stale_thrust.header.stamp=platform->now()-rclcpp::Duration::from_seconds(.2);
 platform->pair(stale,stale_thrust);if(platform->ownSendCommand())return 5;
 bad=thrust;bad.header.frame_id="earth";platform->pair(rate,bad);if(platform->ownSendCommand())return 6;
 // A new matched sample is accepted without arming or connecting to any PX4.
 rate.header.stamp=thrust.header.stamp=platform->now();platform->pair(rate,thrust);if(!platform->ownSendCommand())return 7;
 if (argc > 1 && std::string(argv[1]) == "--attitude") {
   platform->attitude();
   geometry_msgs::msg::PoseStamped pose;
   pose.header.frame_id=platform->getOdomFrameId();
   pose.header.stamp=thrust.header.stamp=platform->now();
   pose.pose.orientation.w=1.;platform->posePair(pose,thrust);
   if(!platform->ownSendCommand())return 10;
   auto mixed=pose;mixed.header.stamp=rclcpp::Time(pose.header.stamp)+rclcpp::Duration::from_seconds(.001);
   platform->posePair(mixed,thrust);if(platform->ownSendCommand())return 11;
   auto newer=thrust;newer.header.stamp=mixed.header.stamp;
   platform->posePair(pose,newer);if(platform->ownSendCommand())return 12;
   auto invalid=pose;invalid.pose.orientation.w=2.;platform->posePair(invalid,thrust);if(platform->ownSendCommand())return 13;
   auto stale_pose=pose;auto stale_force=thrust;
   stale_pose.header.stamp=stale_force.header.stamp=platform->now()-rclcpp::Duration::from_seconds(.2);
   platform->posePair(stale_pose,stale_force);if(platform->ownSendCommand())return 14;
   platform->posePair(pose,thrust);if(!platform->ownSendCommand())return 15;
   std::cout<<"PASS: actual ATTITUDE platform fresh pairs, both mixed orders, invalid quaternion, stale rejection\n";
 }
 std::cout<<"PASS: actual platform accepts fresh pairs and rejects both mixed orders, NaN, stale, wrong frame\n";
 rclcpp::shutdown();return 0;
}
