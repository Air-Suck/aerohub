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
 std::cout<<"PASS: actual platform accepts fresh pairs and rejects both mixed orders, NaN, stale, wrong frame\n";
 rclcpp::shutdown();return 0;
}
