#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <follow_reference_behavior/follow_reference_base.hpp>
#include <as2_core/names/topics.hpp>
#include <as2_motion_reference_handlers/acro_motion.hpp>
#include <as2_motion_reference_handlers/hover_motion.hpp>
#include <pluginlib/class_list_macros.hpp>
#include "uav_follow_reference_acro/msg/control_frame.hpp"
#include "uav_follow_reference_acro/msg/control_session.hpp"
#include "uav_follow_reference_acro/stream_gate.hpp"

namespace follow_reference_plugin_external_acro {
using Frame = uav_follow_reference_acro::msg::ControlFrame;
using Session = uav_follow_reference_acro::msg::ControlSession;
using Goal = as2_msgs::action::FollowReference::Goal;
using Status = as2_behavior::ExecutionStatus;
using Mode = as2_msgs::msg::ControlMode;

class AcroHandler : public as2::motionReferenceHandlers::ACROMotion {
public:
  explicit AcroHandler(as2::Node * node) : ACROMotion(node) {
    // Pixhawk's advertised BODY_RATES mode encodes yaw speed.
    desired_control_mode_.yaw_mode = Mode::YAW_SPEED;
  }
  bool prepare() {return checkMode();}
};

class Plugin : public follow_reference_base::FollowReferenceBase {
public:
  void ownInit() override {
    platform_state_ = 0;
    localization_flag_ = false;
    enabled_ = node_ptr_->getParameter<bool>("external_acro.output_enabled", false);
    frame_timeout_ = positive("external_acro.frame_timeout", 0.1);
    solve_timeout_ = positive("external_acro.solve_timeout", 0.1);
    startup_timeout_ = positive("external_acro.startup_timeout", 2.0);
    state_timeout_ = positive("external_acro.state_timeout", 0.2);
    planning_frame_ = node_ptr_->getParameter<std::string>(
      "external_acro.planning_frame", node_ptr_->getEarthFrameId());
    rate_limit_ = positive("external_acro.max_body_rate", 3.0);
    thrust_limit_ = positive("external_acro.max_thrust_newtons", 15.0);
    if (node_ptr_->getParameter<bool>("use_actuator_commands", false)) {
      throw std::invalid_argument("external ACRO requires the controller motion-reference route");
    }
    acro_ = std::make_unique<AcroHandler>(node_ptr_);
    rate_pub_=node_ptr_->create_publisher<geometry_msgs::msg::TwistStamped>(
      as2_names::topics::motion_reference::twist,as2_names::topics::motion_reference::qos);
    thrust_pub_=node_ptr_->create_publisher<as2_msgs::msg::Thrust>(
      as2_names::topics::motion_reference::thrust,as2_names::topics::motion_reference::qos);
    hold_ = std::make_unique<as2::motionReferenceHandlers::HoverMotion>(node_ptr_);
    session_pub_ = node_ptr_->create_publisher<Session>(
      "external_acro/session", rclcpp::QoS(1).reliable().transient_local());
    mode_group_ = node_ptr_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    rclcpp::SubscriptionOptions mode_options;mode_options.callback_group=mode_group_;
    mode_sub_ = node_ptr_->create_subscription<as2_msgs::msg::ControllerInfo>(
      "controller/info", rclcpp::QoS(1), [this](as2_msgs::msg::ControllerInfo::SharedPtr msg) {
        mode_ready_ = msg->input_control_mode.control_mode == Mode::BODY_RATES &&
          msg->input_control_mode.yaw_mode == Mode::YAW_SPEED;
        mode_info_stamp_ = node_ptr_->now().seconds();
      }, mode_options);
    sub_ = node_ptr_->create_subscription<Frame>("external_acro/control",
      rclcpp::SensorDataQoS().keep_last(1), [this](Frame::SharedPtr msg) {
        if (!active_ || !msg->solver_ok ||
            seconds(msg->header.stamp) < seconds(session_.header.stamp) ||
            msg->header.frame_id != node_ptr_->getBaseFrameId() ||
            !finite(msg->body_rates.x) || !finite(msg->body_rates.y) ||
            !finite(msg->body_rates.z) || !finite(msg->thrust_newtons) ||
            std::abs(msg->body_rates.x) > rate_limit_ ||
            std::abs(msg->body_rates.y) > rate_limit_ ||
            std::abs(msg->body_rates.z) > rate_limit_ ||
            msg->thrust_newtons <= 0 || msg->thrust_newtons > thrust_limit_) {return;}
        if (gate_.accept(msg->session_id, msg->revision, msg->sequence,
            seconds(msg->header.stamp), seconds(msg->last_successful_solve),
            node_ptr_->now().seconds(), frame_timeout_, solve_timeout_)) {
          frame_ = *msg;
          last_frame_wall_ = std::chrono::steady_clock::now();
        }
      });
    clock_progress_ = std::chrono::steady_clock::now();
    clock_probe_ = node_ptr_->now().seconds();
    watchdog_ = node_ptr_->create_wall_timer(std::chrono::milliseconds(20), [this]() {
      const auto wall = std::chrono::steady_clock::now();
      const double ros = node_ptr_->now().seconds();
      const bool reversed = ros < clock_probe_;
      if (ros > clock_probe_) {clock_progress_ = wall;}
      clock_probe_ = ros;
      if (active_ && streaming_ && (reversed ||
          std::chrono::duration<double>(wall - clock_progress_).count() > state_timeout_ ||
          std::chrono::duration<double>(wall - last_frame_wall_).count() > frame_timeout_)) {
        fail("wall watchdog: input or ROS clock stopped");
        if (!hold_->sendHover()) {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Watchdog hold request rejected");
        }
      }
    });
  }

  bool own_activate(Goal & goal) override {
    if (!enabled_ || active_ || !supported(goal)) {return false;}
    static std::atomic<std::uint64_t> serial{0};
    session_.session_id = std::to_string(
      std::chrono::steady_clock::now().time_since_epoch().count()) + "-" +
      std::to_string(++serial);
    session_.revision = 0;
    active_ = true;
    result_.follow_reference_success = false;
    begin(goal);
    return true;
  }

  bool own_modify(Goal & goal) override {
    if (!active_ || !supported(goal)) {return false;}
    revoke();
    if (!hold_->sendHover()) {return false;}
    active_ = true;
    begin(goal);
    return true;
  }

  bool own_deactivate(const std::shared_ptr<std::string> & message) override {
    revoke();
    const bool accepted = hold_->sendHover();
    *message = accepted ? "IPC stream revoked; hold mode accepted (motion stop not verified)" :
      "IPC stream revoked; hold mode rejected; recovery required";
    return accepted;
  }

  bool own_pause(const std::shared_ptr<std::string> & message) override {
    revoke();
    const bool accepted = hold_->sendHover();
    *message = "stream revoked; cancel and submit a fresh goal to resume";
    return accepted;
  }

  bool own_resume(const std::shared_ptr<std::string> & message) override {
    *message = "REPLAN_REQUIRED: cancel and submit a fresh goal";
    return false;
  }

  void own_execution_end(const Status &) override {
    revoke();
    if (!hold_->sendHover()) {
      RCLCPP_ERROR(node_ptr_->get_logger(), "External ACRO ended without hold acceptance");
    }
  }

  Status own_run() override {
    const auto wall_now = std::chrono::steady_clock::now();
    const double now = node_ptr_->now().seconds();
    if (!active_) {return Status::FAILURE;}
    if (now < last_ros_time_) {return fail("ROS clock reversed");}
    last_ros_time_ = now;
    if (platform_state_ != as2_msgs::msg::PlatformStatus::FLYING) {
      return fail("platform no longer flying");
    }
    const double state_stamp = seconds(actual_pose_.header.stamp);
    if (state_stamp <= 0 || state_stamp > now || now - state_stamp > state_timeout_) {
      return fail("localization stale");
    }
    if (!gate_.valid(now, frame_timeout_, solve_timeout_)) {
      if (streaming_ || std::chrono::duration<double>(wall_now - started_).count() >
          startup_timeout_) {return fail("control stream absent, stale or solver unhealthy");}
      return Status::RUNNING;
    }
    if (!prepared_) {
      if (!acro_->prepare()) {return fail("BODY_RATES mode rejected");}
      prepared_ = true;
      // prepare may block for 100ms: discard every pre-transition control frame.
      gate_.start(session_.session_id, session_.revision);
      session_.header.stamp = node_ptr_->now();
      session_pub_->publish(session_);
      return Status::RUNNING;
    }
    if (!mode_ready_ || now - mode_info_stamp_ > state_timeout_) {
      if (streaming_ || std::chrono::duration<double>(wall_now - started_).count() >
          startup_timeout_) {
        RCLCPP_ERROR(node_ptr_->get_logger(), "BODY_RATES feedback ready=%d age=%f streaming=%d",mode_ready_.load(),now-mode_info_stamp_.load(),streaming_);
        return fail("BODY_RATES confirmation absent or lost");
      }
      return Status::RUNNING;
    }
    streaming_ = true;
    as2_msgs::msg::Thrust thrust;
    thrust.header = frame_.header;
    thrust.thrust = static_cast<float>(frame_.thrust_newtons);
    // Mode was explicitly prepared and confirmed above. Do not renegotiate on
    // each send through a second, independently scheduled feedback cache.
    geometry_msgs::msg::TwistStamped rates;rates.header=frame_.header;
    rates.twist.angular=frame_.body_rates;
    thrust_pub_->publish(thrust);rate_pub_->publish(rates);
    return Status::RUNNING;  // Continuous follow; caller cancels when its task is complete.
  }

private:
  static bool finite(double x) {return std::isfinite(x);}
  static double seconds(const builtin_interfaces::msg::Time & t) {
    return static_cast<double>(t.sec) + t.nanosec * 1e-9;
  }
  double positive(const std::string & key, double value) {
    const double x = node_ptr_->getParameter<double>(key, value);
    if (!finite(x) || x <= 0) {throw std::invalid_argument(key + " must be positive");}
    return x;
  }
  bool supported(const Goal & goal) const {
    return goal.target_pose.header.frame_id == planning_frame_ && finite(goal.target_pose.point.x) &&
      finite(goal.target_pose.point.y) && finite(goal.target_pose.point.z) &&
      finite(goal.yaw.angle) && goal.yaw.mode == as2_msgs::msg::YawMode::FIXED_YAW &&
      finite(goal.max_speed_x) && goal.max_speed_x > 0 &&
      finite(goal.max_speed_y) && goal.max_speed_y > 0 &&
      finite(goal.max_speed_z) && goal.max_speed_z > 0;
  }
  void begin(const Goal & goal) {
    ++session_.revision;
    session_.header.stamp = node_ptr_->now();
    session_.header.frame_id = goal.target_pose.header.frame_id;
    session_.active = true;
    session_.target = goal.target_pose;
    session_.yaw = goal.yaw;
    session_.max_speed.x = goal.max_speed_x;
    session_.max_speed.y = goal.max_speed_y;
    session_.max_speed.z = goal.max_speed_z;
    gate_.start(session_.session_id, session_.revision);
    frame_ = Frame{};
    streaming_ = false; prepared_ = false;
    last_ros_time_ = node_ptr_->now().seconds();
    started_ = std::chrono::steady_clock::now();
    session_pub_->publish(session_);
  }
  void revoke() {
    active_ = false;
    gate_.stop();
    session_.active = false;
    session_.header.stamp = node_ptr_->now();
    session_pub_->publish(session_);
  }
  Status fail(const char * reason) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "External ACRO: %s", reason);
    revoke();
    return Status::FAILURE;
  }
  uav_follow_reference_acro::StreamGate gate_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr rate_pub_;
  rclcpp::Publisher<as2_msgs::msg::Thrust>::SharedPtr thrust_pub_;
  std::unique_ptr<AcroHandler> acro_;
  std::unique_ptr<as2::motionReferenceHandlers::HoverMotion> hold_;
  rclcpp::Publisher<Session>::SharedPtr session_pub_;
  rclcpp::Subscription<Frame>::SharedPtr sub_;
  rclcpp::Subscription<as2_msgs::msg::ControllerInfo>::SharedPtr mode_sub_;
  Session session_;
  Frame frame_;
  std::chrono::steady_clock::time_point started_;
  std::chrono::steady_clock::time_point last_frame_wall_, clock_progress_;
  rclcpp::TimerBase::SharedPtr watchdog_;
  double clock_probe_{};
  double frame_timeout_{}, solve_timeout_{}, startup_timeout_{}, rate_limit_{}, thrust_limit_{};
  double last_ros_time_{};
  double state_timeout_{};
  rclcpp::CallbackGroup::SharedPtr mode_group_;
  std::atomic<double> mode_info_stamp_{0};
  std::string planning_frame_;
  bool enabled_{false}, active_{false}, prepared_{false}, streaming_{false};
  std::atomic<bool> mode_ready_{false};
};
}
PLUGINLIB_EXPORT_CLASS(follow_reference_plugin_external_acro::Plugin,
  follow_reference_base::FollowReferenceBase)
