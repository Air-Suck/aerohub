// The original FollowPath action and plugins run unchanged through inheritance.
// Only one of the original action and the timed companion may own execution.
#include <memory>
#include <limits>
#include <string>
#include "follow_path_behavior/follow_path_behavior.hpp"
#include "as2_motion_reference_handlers/position_motion.hpp"
#include "as2_motion_reference_handlers/trajectory_motion.hpp"
#include "as2_motion_reference_handlers/hover_motion.hpp"
#include "uav_planner_interfaces/action/follow_timed_path.hpp"
#include "uav_follow_path_execution/timeline.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace uf = uav_follow_path_execution;
using Timed = uav_planner_interfaces::action::FollowTimedPath;
using Handle = rclcpp_action::ServerGoalHandle<Timed>;
using Trigger = std_srvs::srv::Trigger;

class ExtendedFollowPath : public FollowPathBehavior {
public:
  explicit ExtendedFollowPath(const rclcpp::NodeOptions & options)
  : FollowPathBehavior(options), position_(this), trajectory_(this), hover_(this) {
    vehicle_ = getParameter<std::string>("vehicle_id", "drone0");
    frame_ = getParameter<std::string>("timed_frame", vehicle_+"/odom");
    tolerance_ = getParameter<double>("timed_position_tolerance", 0.25);
    continuity_ = getParameter<double>("timed_continuity_tolerance", 0.15);
    velocity_continuity_ = getParameter<double>("timed_velocity_continuity_tolerance", 0.5);
    acceleration_continuity_ = getParameter<double>("timed_acceleration_continuity_tolerance", 0.5);
    yaw_continuity_ = getParameter<double>("timed_yaw_continuity_tolerance", 0.15);
    settle_timeout_ = getParameter<double>("timed_settle_timeout", 10.0);
    pose_timeout_ = getParameter<double>("timed_pose_timeout", 0.5);
    timed_tf_ = std::make_shared<as2::tf::TfHandler>(this);
    info_sub_ = create_subscription<as2_msgs::msg::PlatformInfo>(
      "platform/info", rclcpp::SensorDataQoS(), [this](as2_msgs::msg::PlatformInfo::SharedPtr msg) {
        flying_ = msg->status.state == as2_msgs::msg::PlatformStatus::FLYING;
      });
    controller_sub_ = create_subscription<as2_msgs::msg::ControllerInfo>(
      "controller/info", rclcpp::SensorDataQoS(), [this](as2_msgs::msg::ControllerInfo::SharedPtr msg) {
        const auto mode = msg->input_control_mode.control_mode;
        if (timed_ && trajectory_started_) {
          if (mode == as2_msgs::msg::ControlMode::TRAJECTORY) {trajectory_acquired_ = true;}
          else if (trajectory_acquired_) {mode_lost_ = true;}
        }
      });
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "self_localization/pose", rclcpp::SensorDataQoS(), [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (msg->header.frame_id != frame_ && !timed_tf_->tryConvert(*msg, frame_)) {return;}
        const auto & p = msg->pose.position;
        actual_ = {p.x,p.y,p.z};
        const auto & q = msg->pose.orientation;
        yaw_ = std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
        pose_time_ = rclcpp::Time(msg->header.stamp).seconds();
        has_pose_ = uf::finite(actual_) && std::isfinite(yaw_);
      });
    server_ = rclcpp_action::create_server<Timed>(this, "follow_timed_path",
      [this](const auto &, std::shared_ptr<const Timed::Goal> goal) {
        if (legacy_ || timed_ || !flying_ || !poseFresh()) {return rclcpp_action::GoalResponse::REJECT;}
        try {
          timeline_.start(convert(goal->path), now().seconds());
          if (uf::distance(actual_, timeline_.active.points.front().p) > continuity_) {
            throw std::invalid_argument("initial point is not continuous with vehicle position");
          }
        } catch (const std::exception & e) {
          RCLCPP_WARN(get_logger(), "%s", e.what());
          timeline_.reset();
          return rclcpp_action::GoalResponse::REJECT;
        }
        held_yaw_ = yaw_;
        trajectory_started_ = trajectory_acquired_ = mode_lost_ = false;
        timed_ = true; paused_ = false; needs_replan_ = false;
        last_tick_ = now().seconds();
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const auto &) {
        cancel_requested_ = true;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](auto handle) {handle_ = handle;});
    modify_ = create_service<Timed::Impl::SendGoalService>(
      "follow_timed_path/_behavior/modify", [this](Timed::Impl::SendGoalService::Request::SharedPtr req, Timed::Impl::SendGoalService::Response::SharedPtr res) {
        res->accepted = false;
        if (!timed_ || paused_ || needs_replan_) {return;}
        try {
          timeline_.update(convert(req->goal.path), now().seconds(), continuity_, velocity_continuity_,
            acceleration_continuity_, yaw_continuity_);
          res->accepted = true;
        } catch (const std::exception & e) {RCLCPP_WARN(get_logger(), "%s", e.what());}
      });
    pause_ = create_service<Trigger>("follow_timed_path/_behavior/pause",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        res->success = timed_ && !paused_;
        if (res->success) {paused_ = true; needs_replan_ = true; hover_.sendHover();}
        res->message = "pause invalidates fleet timing; cancel and submit a replanned goal";
      });
    resume_ = create_service<Trigger>("follow_timed_path/_behavior/resume",
      [](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        res->success = false;
        res->message = "REPLAN_REQUIRED: cancel paused goal and submit a future-dated plan";
      });
    stop_ = create_service<Trigger>("follow_timed_path/_behavior/stop",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {res->success = timed_ ? true : hover_.sendHover(); if (timed_) {res->success = finish(false,"STOPPED");}});
    status_pub_ = create_publisher<as2_msgs::msg::BehaviorStatus>(
      "follow_timed_path/_behavior/behavior_status", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this] {tick();});
  }
  bool on_activate(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override {
    if (timed_ || legacy_) {return false;}
    legacy_ = FollowPathBehavior::on_activate(goal);
    return legacy_;
  }
  bool on_modify(std::shared_ptr<const as2_msgs::action::FollowPath::Goal> goal) override {
    return legacy_ && !timed_ && FollowPathBehavior::on_modify(goal);
  }
  bool on_deactivate(const std::shared_ptr<std::string> & message) override {
    return legacy_ && FollowPathBehavior::on_deactivate(message);
  }
  void on_execution_end(const as2_behavior::ExecutionStatus & status) override {
    FollowPathBehavior::on_execution_end(status); legacy_ = false;
  }
private:
  uf::Path convert(const uav_planner_interfaces::msg::TimedPath & m) {
    if (m.interface_version != 1 || m.vehicle_id != vehicle_ || m.frame_id != frame_) {
      throw std::invalid_argument("unsupported interface, vehicle or fixed frame");
    }
    uf::Path p;
    p.task=m.task_id; p.vehicle=m.vehicle_id; p.frame=m.frame_id; p.revision=m.revision;
    p.generated=rclcpp::Time(m.generated_at).seconds(); p.start=rclcpp::Time(m.valid_from).seconds();
    p.fields=m.valid_fields; p.final=m.is_final; p.max_speed=m.max_speed;
    for (const auto & v : m.points) {
      p.points.push_back({rclcpp::Duration(v.time_from_start).seconds(),
        {v.position.x,v.position.y,v.position.z}, {v.velocity.x,v.velocity.y,v.velocity.z},
        {v.acceleration.x,v.acceleration.y,v.acceleration.z},v.yaw});
    }
    return p;
  }
  bool poseFresh() const {
    const double age=now().seconds()-pose_time_;
    return has_pose_ && age>=0 && age<=pose_timeout_;
  }
  bool finish(bool success, const std::string & reason, bool canceled=false) {
    auto result=std::make_shared<Timed::Result>();
    result->success=success; result->reason=reason; result->revision=timeline_.active.revision;
    const bool stopped = hover_.sendHover();
    if (handle_) {
      if (canceled) {handle_->canceled(result);}
      else if (success) {handle_->succeed(result);}
      else {handle_->abort(result);}
    }
    handle_.reset(); timeline_.reset(); timed_=paused_=needs_replan_=cancel_requested_=false;
    return stopped;
  }
  void tick() {
    as2_msgs::msg::BehaviorStatus status;
    status.status = !timed_ ? as2_msgs::msg::BehaviorStatus::IDLE :
      (paused_ ? as2_msgs::msg::BehaviorStatus::PAUSED : as2_msgs::msg::BehaviorStatus::RUNNING);
    status_pub_->publish(status);
    if (!timed_ || !handle_) {return;}
    if (cancel_requested_ && handle_->is_canceling()) {finish(false,"CANCELED",true); return;}
    const auto sample_time=now();
    const double t=sample_time.seconds();
    if (t<last_tick_) {finish(false,"CLOCK_RESET"); return;}
    last_tick_=t;
    if (!poseFresh()) {finish(false,"STALE_LOCALIZATION"); return;}
    if (!flying_) {finish(false,"NOT_FLYING"); return;}
    if (paused_) {return;}
    if (mode_lost_) {finish(false,"CONTROL_MODE_LOST"); return;}
    timeline_.advance(t);
    const auto & p=timeline_.active;
    auto feedback=std::make_shared<Timed::Feedback>();
    feedback->task_id=p.task; feedback->revision=p.revision;
    feedback->elapsed=std::max(0.0,t-p.start); feedback->waiting_for_start=t<p.start;
    const auto s=uf::sample(p,t);
    feedback->tracking_error=uf::distance(actual_,s.p);
    handle_->publish_feedback(feedback);
    if (t<p.start) {
      // The native PID stores trajectory output limits when it receives a
      // POSITION twist. Takeoff can leave horizontal limits at zero. Prepare
      // limits while holding the actual position, never advancing the path.
      if (p.fields & 1) {
        if (!position_.sendPositionCommandWithYawAngle(frame_,actual_[0],actual_[1],actual_[2],
            yaw_,frame_,p.max_speed,p.max_speed,p.max_speed)) {
          finish(false,"MOTION_REFERENCE_REJECTED");
        }
      }
      return;
    }
    if (t>p.end() && !p.final) {finish(false,"TRAJECTORY_EXPIRED"); return;}
    if (t>=p.end() && p.final) {
      if (feedback->tracking_error<=tolerance_) {finish(true,"COMPLETED"); return;}
      if (t>p.end()+settle_timeout_) {finish(false,"TRACKING_TIMEOUT"); return;}
    }
    const double yaw=(p.fields & 4)?s.yaw:held_yaw_;
    bool sent;
    if ((p.fields & 1) && t<p.end()) {
      trajectory_started_ = true;
      const double nan = std::numeric_limits<double>::quiet_NaN();
      as2_msgs::msg::TrajectorySetpoints reference;
      reference.header.frame_id=frame_;
      reference.header.stamp=sample_time;
      as2_msgs::msg::TrajectoryPoint point;
      point.position.x=s.p[0]; point.position.y=s.p[1]; point.position.z=s.p[2];
      point.twist.x=s.v[0]; point.twist.y=s.v[1]; point.twist.z=s.v[2];
      point.acceleration.x=(p.fields & 2)?s.a[0]:nan;
      point.acceleration.y=(p.fields & 2)?s.a[1]:nan;
      point.acceleration.z=(p.fields & 2)?s.a[2]:nan;
      point.yaw_angle=yaw;
      reference.setpoints.push_back(point);
      sent=trajectory_.sendTrajectorySetpoints(reference);
    } else {
      // Final-position settling intentionally releases trajectory mode. The
      // takeover guard still applies while streaming the active trajectory.
      trajectory_started_ = trajectory_acquired_ = false;
      sent=position_.sendPositionCommandWithYawAngle(frame_,s.p[0],s.p[1],s.p[2],yaw,
        frame_,p.max_speed,p.max_speed,p.max_speed);
    }
    if (!sent) {finish(false,"MOTION_REFERENCE_REJECTED");}
  }
  bool legacy_{false},timed_{false},paused_{false},needs_replan_{false},cancel_requested_{false};
  bool trajectory_started_{false},trajectory_acquired_{false},mode_lost_{false};
  double held_yaw_{};
  bool has_pose_{false},flying_{false};
  double pose_time_{},yaw_{},last_tick_{},tolerance_,continuity_,velocity_continuity_;
  double settle_timeout_,pose_timeout_;
  double acceleration_continuity_,yaw_continuity_;
  std::string vehicle_,frame_;
  uf::Vec actual_{};
  uf::Timeline timeline_;
  std::shared_ptr<as2::tf::TfHandler> timed_tf_;
  as2::motionReferenceHandlers::PositionMotion position_;
  as2::motionReferenceHandlers::TrajectoryMotion trajectory_;
  as2::motionReferenceHandlers::HoverMotion hover_;
  rclcpp_action::Server<Timed>::SharedPtr server_;
  std::shared_ptr<Handle> handle_;
  rclcpp::Service<Timed::Impl::SendGoalService>::SharedPtr modify_;
  rclcpp::Service<Trigger>::SharedPtr pause_,resume_,stop_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<as2_msgs::msg::ControllerInfo>::SharedPtr controller_sub_;
  rclcpp::Subscription<as2_msgs::msg::PlatformInfo>::SharedPtr info_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<as2_msgs::msg::BehaviorStatus>::SharedPtr status_pub_;
};

int main(int argc,char ** argv) {
  rclcpp::init(argc,argv);
  auto options=rclcpp::NodeOptions();
  rclcpp::spin(std::make_shared<ExtendedFollowPath>(options));
  rclcpp::shutdown();
  return 0;
}
