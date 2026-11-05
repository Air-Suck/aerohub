// Registered AS2 navigation inputs -> a configured, fixed fleet ENU frame.
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class SwarmNavigationBridge : public rclcpp::Node {
  using Odom = nav_msgs::msg::Odometry;
  using Cloud = sensor_msgs::msg::PointCloud2;
public:
  SwarmNavigationBridge() : Node("swarm_navigation_bridge") {
    source_ = declare_parameter<std::string>("source_frame", "drone0/odom");
    body_ = declare_parameter<std::string>("body_frame", "drone0/base_link");
    target_ = declare_parameter<std::string>("map_frame", "swarm_map");
    timeout_ = declare_parameter("max_input_age", 1.0);
    auto origin = declare_parameter<std::vector<double>>("origin_enu", {0.,0.,0.});
    const double yaw = declare_parameter("origin_yaw", 0.0);
    if (source_.empty() || body_.empty() || target_.empty() || !std::isfinite(timeout_) || timeout_<=0 ||
        origin.size()!=3 || !std::all_of(origin.begin(),origin.end(),[](double x){return std::isfinite(x);}) || !std::isfinite(yaw))
      throw std::invalid_argument("fixed source/map frames and a finite rigid transform are required");
    transform_.header.frame_id=target_;
    transform_.child_frame_id=source_;
    transform_.transform.translation.x=origin[0];
    transform_.transform.translation.y=origin[1];
    transform_.transform.translation.z=origin[2];
    transform_.transform.rotation.z=std::sin(yaw/2);
    transform_.transform.rotation.w=std::cos(yaw/2);
    const auto qos=rclcpp::SensorDataQoS().keep_last(10);
    odom_pub_=create_publisher<Odom>("odom",qos);
    cloud_pub_=create_publisher<Cloud>("points",qos);
    odom_sub_=create_subscription<Odom>(declare_parameter<std::string>("odom_topic","/drone0/navigation/odom"),qos,
      [this](Odom::ConstSharedPtr msg){odom(*msg);});
    cloud_sub_=create_subscription<Cloud>(declare_parameter<std::string>("cloud_topic","/drone0/navigation/points"),qos,
      [this](Cloud::ConstSharedPtr msg){cloud(*msg);});
  }
private:
  bool valid(const std_msgs::msg::Header &header,int64_t &last) {
    const auto current=now().nanoseconds();
    if (last_now_>=0 && current<last_now_) {last_odom_=last_cloud_=0;have_odom_=false;}
    last_now_=current;
    const rclcpp::Time stamp(header.stamp);
    const auto age=(now()-stamp).seconds();
    if (header.frame_id!=source_ || stamp.nanoseconds()<=0 || stamp.nanoseconds()<=last || age<0 || age>timeout_) {
      RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),2000,"Rejecting navigation input with wrong frame or expired/out-of-order time");
      return false;
    }
    return true;
  }
  void odom(const Odom &msg) {
    if (!valid(msg.header,last_odom_) || msg.child_frame_id!=body_) return;
    const auto &p=msg.pose.pose.position;const auto &q=msg.pose.pose.orientation;
    const auto &v=msg.twist.twist.linear;const auto &w=msg.twist.twist.angular;
    for (double value:{p.x,p.y,p.z,q.x,q.y,q.z,q.w,v.x,v.y,v.z,w.x,w.y,w.z})
      if (!std::isfinite(value)) return;
    if (std::abs(q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w-1)>1e-3) return;
    geometry_msgs::msg::PoseWithCovarianceStamped input,output;
    input.header=msg.header;input.pose=msg.pose;
    tf2::doTransform(input,output,transform_);
    Odom result=msg;
    result.header.frame_id=target_;result.pose=output.pose;
    // Twist remains in the original child/body frame, as nav_msgs/Odometry specifies.
    odom_pub_->publish(result);
    last_odom_=rclcpp::Time(msg.header.stamp).nanoseconds();have_odom_=true;
  }
  void cloud(const Cloud &msg) {
    if (!valid(msg.header,last_cloud_) || !have_odom_ ||
        (now()-rclcpp::Time(last_odom_,RCL_ROS_TIME)).seconds()>timeout_) return;
    if (!msg.width || !msg.height || msg.is_bigendian || msg.row_step!=msg.width*msg.point_step ||
        msg.data.size()!=static_cast<size_t>(msg.row_step)*msg.height) return;
    for (const auto *name:{"x","y","z"}) {
      auto field=std::find_if(msg.fields.begin(),msg.fields.end(),[name](const auto &f){return f.name==name;});
      if (field==msg.fields.end() || field->datatype!=sensor_msgs::msg::PointField::FLOAT32 ||
          field->count!=1 || field->offset+4>msg.point_step) return;
    }
    Cloud output;
    tf2::doTransform(msg,output,transform_);
    output.header=msg.header;output.header.frame_id=target_;
    cloud_pub_->publish(output);
    last_cloud_=rclcpp::Time(msg.header.stamp).nanoseconds();
  }
  std::string source_,body_,target_;
  double timeout_;
  int64_t last_now_{-1},last_odom_{0},last_cloud_{0};
  bool have_odom_{false};
  geometry_msgs::msg::TransformStamped transform_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<SwarmNavigationBridge>());rclcpp::shutdown();
}
