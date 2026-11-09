// Copyright 2026 AeroHub contributors. Licensed under Apache-2.0.
#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

class NavigationInputs : public rclcpp::Node
{
  using Steady = std::chrono::steady_clock;
  template<class Message> struct Pending {
    typename Message::ConstSharedPtr message;
    Steady::time_point received;
  };
  using Odom = nav_msgs::msg::Odometry;
  using Cloud = sensor_msgs::msg::PointCloud2;

public:
  NavigationInputs() : Node("navigation_inputs"), buffer_(get_clock()), listener_(buffer_)
  {
    target_ = declare_parameter<std::string>("target_frame", "drone0/odom");
    body_ = declare_parameter<std::string>("body_frame", "drone0/base_link");
    wait_ = declare_parameter("transform_wait", 0.25);
    max_age_ = declare_parameter("max_input_age", 1.0);
    const auto size = declare_parameter("queue_size", 20);
    if (target_.empty() || body_.empty() || !std::isfinite(wait_) || wait_ <= 0 ||
      !std::isfinite(max_age_) || max_age_ <= 0 || size < 1)
    {
      throw std::invalid_argument("Frames must be nonempty and queue/time limits positive");
    }
    queue_size_ = static_cast<size_t>(size);
    const auto qos = rclcpp::SensorDataQoS().keep_last(size);
    odom_pub_ = create_publisher<Odom>("navigation/odom", qos);
    cloud_pub_ = create_publisher<Cloud>("navigation/points", qos);
    odom_sub_ = create_subscription<Odom>(
      declare_parameter<std::string>("odom_topic", "sensor_measurements/odom"), qos,
      [this](Odom::ConstSharedPtr msg) {enqueue(odometry_, msg);});
    cloud_sub_ = create_subscription<Cloud>(
      declare_parameter<std::string>("cloud_topic", "sensor_measurements/Lidar/points"), qos,
      [this](Cloud::ConstSharedPtr msg) {enqueue(clouds_, msg);});
    timer_ = create_wall_timer(std::chrono::milliseconds(5), [this]() {tick();});
  }

private:
  template<class Message>
  void enqueue(std::deque<Pending<Message>> & queue, typename Message::ConstSharedPtr msg)
  {
    if (msg->header.frame_id.empty() || rclcpp::Time(msg->header.stamp).nanoseconds() <= 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Dropping input without frame/time");
      return;
    }
    if (queue.size() >= queue_size_) {
      queue.pop_front();
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Input queue full; dropping oldest");
    }
    queue.push_back({msg, Steady::now()});
  }

  template<class Message>
  void drain(std::deque<Pending<Message>> & queue, int64_t & last_stamp)
  {
    for (auto it = queue.begin(); it != queue.end();) {
      const auto & msg = *it->message;
      const rclcpp::Time stamp(msg.header.stamp);
      const auto age = (now() - stamp).seconds();
      const auto waited = std::chrono::duration<double>(Steady::now() - it->received).count();
      if (stamp.nanoseconds() <= last_stamp || age > max_age_ || waited > wait_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Dropping expired, out-of-order or TF-timeout input");
        it = queue.erase(it);
        continue;
      }
      // Future samples wait for ROS time; there is never a lookup at TimePointZero.
      if (age < 0) {++it; continue;}
      try {
        publish(msg, stamp);
        last_stamp = stamp.nanoseconds();
        it = queue.erase(it);
      } catch (const tf2::TransformException &) {
        ++it;
      } catch (const std::invalid_argument & error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "%s", error.what());
        it = queue.erase(it);
      }
    }
  }

  void publish(const Odom & msg, const rclcpp::Time & stamp)
  {
    if (msg.child_frame_id != body_) {
      throw std::invalid_argument("Odometry child_frame_id differs from configured body_frame");
    }
    const auto &p = msg.pose.pose.position;
    const auto &q = msg.pose.pose.orientation;
    const double norm = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
      !std::isfinite(norm) || std::abs(norm - 1.0) > 1e-3)
    {
      throw std::invalid_argument("Odometry requires finite position and unit quaternion");
    }
    Odom output = msg;
    if (msg.header.frame_id != target_) {
      const auto tf = buffer_.lookupTransform(target_, msg.header.frame_id, stamp);
      geometry_msgs::msg::PoseWithCovarianceStamped input, transformed;
      input.header = msg.header;
      input.pose = msg.pose;
      tf2::doTransform(input, transformed, tf);
      output.pose = transformed.pose;
    }
    output.header.frame_id = target_;
    // Twist and its covariance stay in child_frame_id; preserve measurement time.
    odom_pub_->publish(output);
  }

  void publish(const Cloud & msg, const rclcpp::Time & stamp)
  {
    // Humble's tf2 cloud transform operates on FLOAT32 XYZ. Reject other layouts
    // explicitly rather than interpreting their bytes as float32.
    for (const auto * name : {"x", "y", "z"}) {
      bool found = false;
      for (const auto & field : msg.fields) {
        if (field.name == name) {
          found = field.datatype == sensor_msgs::msg::PointField::FLOAT32 &&
            field.count == 1 && field.offset + 4 <= msg.point_step;
        }
      }
      if (!found) {throw std::invalid_argument("Cloud requires scalar FLOAT32 XYZ fields");}
    }
    if (msg.width == 0 || msg.height == 0 || msg.is_bigendian || msg.row_step != msg.width * msg.point_step ||
      msg.data.size() != static_cast<size_t>(msg.row_step) * msg.height)
    {
      throw std::invalid_argument("Cloud requires packed little-endian rows and valid data length");
    }
    // Require the robot pose at the cloud time as well as the sensor transform.
    buffer_.lookupTransform(target_, body_, stamp);
    Cloud output;
    if (msg.header.frame_id == target_) {
      output = msg;
    } else {
      const auto tf = buffer_.lookupTransform(target_, msg.header.frame_id, stamp);
      tf2::doTransform(msg, output, tf);
    }
    output.header = msg.header;
    output.header.frame_id = target_;
    cloud_pub_->publish(output);
  }

  void tick()
  {
    const auto time = now().nanoseconds();
    if (last_now_ >= 0 && time < last_now_) {
      odometry_.clear(); clouds_.clear(); last_odom_ = last_cloud_ = 0;
      RCLCPP_WARN(get_logger(), "ROS time moved backwards; cleared pending navigation inputs");
    }
    last_now_ = time;
    drain(odometry_, last_odom_);
    drain(clouds_, last_cloud_);
  }

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::string target_, body_;
  double wait_, max_age_;
  size_t queue_size_;
  int64_t last_now_{-1}, last_odom_{0}, last_cloud_{0};
  std::deque<Pending<Odom>> odometry_;
  std::deque<Pending<Cloud>> clouds_;
  rclcpp::Publisher<Odom>::SharedPtr odom_pub_;
  rclcpp::Publisher<Cloud>::SharedPtr cloud_pub_;
  rclcpp::Subscription<Odom>::SharedPtr odom_sub_;
  rclcpp::Subscription<Cloud>::SharedPtr cloud_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<NavigationInputs>());
  rclcpp::shutdown();
  return 0;
}
