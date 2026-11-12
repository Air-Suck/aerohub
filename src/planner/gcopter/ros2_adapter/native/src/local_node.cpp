// GCOPTER-owned receding-horizon planner. ROG-Map is copied; planning is GCOPTER.
#include <uav_gcopter/core.hpp>
#include <rog_map/rog_map.h>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/string.hpp>
#include <uav_planner_interfaces/msg/timed_path.hpp>
#include <yaml-cpp/yaml.h>
#include <deque>
#include <queue>
#include <set>
#include <filesystem>
#include <ompl/util/RandomNumbers.h>

using Vec = Eigen::Vector3d;
using Boundary = Eigen::Matrix3d;
using Path = uav_planner_interfaces::msg::TimedPath;
using Trigger = std_srvs::srv::Trigger;

class LocalMap : public rog_map::ROGMap {
  const double getSystemWalltimeNow() override {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
  }
public:
  explicit LocalMap(const std::string &file) {
    std::filesystem::create_directories("/tmp/gcopter-local-map");
    cfg_ = rog_map::Config(file);
    if (cfg_.load_pcd_en || cfg_.ros_callback_en || !cfg_.raycasting_en)
      throw std::invalid_argument("local map requires sensor raycasting, no PCD, direct API");
    init();
  }
  bool free(const Vec &p) const {
    // ROG-Map's isKnownFreeInflate means merely nonoccupied when unknown
    // inflation is off. Explicitly query probabilistic known-free as well.
    return isKnownFree(p) && !isOccupiedInflate(p);
  }
};

class LocalPlanner : public rclcpp::Node {
  std::unique_ptr<LocalMap> map_;
  uav_gcopter::Config cfg_;
  std::string frame_, vehicle_, task_;
  Vec target_;
  double lead_, horizon_, range_, goal_tolerance_, interval_, last_attempt_ = -1e9, map_time_ = 0;
  bool requested_ = false, failed_ = false, final_ = false, pending_ = false, enabled_ = false;
  bool future_blocked_ = false;
  uint64_t revision_ = 0;
  Trajectory<5> active_, candidate_, previous_;
  double active_start_ = 0, candidate_start_ = 0, previous_start_ = 0;
  bool candidate_final_ = false;
  std::deque<nav_msgs::msg::Odometry> odoms_;
  std::deque<sensor_msgs::msg::PointCloud2::SharedPtr> scans_;
  rclcpp::Publisher<Path>::SharedPtr plans_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr failure_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr execution_sub_;
  rclcpp::Service<Trigger>::SharedPtr start_, reset_, health_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string reason_ = "waiting_for_lidar";
  double seconds() { return now().seconds(); }
  static Vec position(const nav_msgs::msg::Odometry &s) {
    return Vec(s.pose.pose.position.x, s.pose.pose.position.y, s.pose.pose.position.z);
  }
  bool fresh() {
    const double t = seconds();
    return !odoms_.empty() && t-map_time_ >= 0 && t-map_time_ < 1 &&
      t-rclcpp::Time(odoms_.back().header.stamp).seconds() >= 0 &&
      t-rclcpp::Time(odoms_.back().header.stamp).seconds() < .3;
  }
  void invalidate(const std::string &reason) {
    if(failed_) return;
    std_msgs::msg::String message;message.data=reason;failure_->publish(message);
    plans_->publish(Path{});
    failed_ = true; requested_ = false; reason_ = reason;
    RCLCPP_ERROR(get_logger(), "%s", reason.c_str());
  }
  Boundary at(const Trajectory<5> &curve, double t) const {
    Boundary s;
    t = std::clamp(t, 0.0, curve.getTotalDuration());
    s.col(0) = curve.getPos(t); s.col(1) = curve.getVel(t); s.col(2) = curve.getAcc(t);
    return s;
  }
  double heading(const Trajectory<5> &curve, double t, double fallback) const {
    const double duration=curve.getTotalDuration();
    for(double offset: {0.0,.02,-.02,.05,-.05,.1,-.1,.2,-.2}) {
      const auto velocity=curve.getVel(std::clamp(t+offset,0.0,duration));
      if(velocity.head<2>().norm()>.15) return std::atan2(velocity.y(),velocity.x());
    }
    return fallback;
  }
  bool safe(const Trajectory<5> &curve, double from, double until = INFINITY, bool require_observed = true) {
    if(!curve.getPieceNum() || !std::isfinite(curve.getTotalDuration()) ||
       curve.getTotalDuration()<=0 || curve.getTotalDuration()>120) return false;
    const double end=std::min(until,curve.getTotalDuration());
    for (double t = std::max(0.0, from); t <= end; t += .01) {
      const auto p=curve.getPos(t),v=curve.getVel(t),a=curve.getAcc(t);
      if (!p.allFinite() || !v.allFinite() || !a.allFinite() || v.norm()>cfg_.bounds[0]*1.01) return false;
      const bool valid=require_observed?map_->free(p):(map_->insideLocalMap(p) && !map_->isOccupiedInflate(p));
      if(!valid) {
        RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,"curve check: observed=%d occupied=%d p=[%.2f %.2f %.2f]",
          int(map_->isKnownFree(p)),int(map_->isOccupiedInflate(p)),p.x(),p.y(),p.z());
        return false;
      }
    }
    return end<0 || (require_observed?map_->free(curve.getPos(end)):
      (map_->insideLocalMap(curve.getPos(end)) && !map_->isOccupiedInflate(curve.getPos(end))));
  }
  void consume_scans() {
    while (!scans_.empty() && !odoms_.empty()) {
      const auto msg = scans_.front();
      const double stamp = rclcpp::Time(msg->header.stamp).seconds();
      auto pose = std::min_element(odoms_.begin(), odoms_.end(), [stamp](const auto &a, const auto &b) {
        return std::abs(rclcpp::Time(a.header.stamp).seconds()-stamp) <
               std::abs(rclcpp::Time(b.header.stamp).seconds()-stamp);
      });
      if (seconds()-stamp > 1 || seconds() < stamp) { scans_.pop_front(); continue; }
      if (std::abs(rclcpp::Time(pose->header.stamp).seconds()-stamp) > .05) break;
      scans_.pop_front();
      if (stamp <= map_time_ || msg->header.frame_id != frame_ || msg->is_bigendian) continue;
      try {
        for (const char *name : {"x", "y", "z"}) {
          auto f = std::find_if(msg->fields.begin(), msg->fields.end(), [name](const auto &v) { return v.name == name; });
          if (f == msg->fields.end() || f->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
              f->count != 1 || f->offset+4 > msg->point_step) throw std::runtime_error("invalid XYZ fields");
        }
        if (!msg->point_step || msg->row_step != msg->width*msg->point_step ||
            msg->data.size() != size_t(msg->row_step)*msg->height) throw std::runtime_error("invalid cloud layout");
        rog_map::PointCloud cloud;
        sensor_msgs::PointCloud2ConstIterator<float> x(*msg,"x"), y(*msg,"y"), z(*msg,"z");
        for (; x != x.end(); ++x, ++y, ++z) {
          if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(*z)) continue;
          pcl::PointXYZI p; p.x=*x; p.y=*y; p.z=*z; p.intensity=1; cloud.push_back(p);
        }
        const auto &q = pose->pose.pose.orientation;
        map_->updateMap(cloud, {position(*pose), Eigen::Quaterniond(q.w,q.x,q.y,q.z)});
        map_time_ = stamp;
        if(previous_.getPieceNum() && seconds()<active_start_ &&
           !safe(previous_,seconds()-previous_start_,active_start_-previous_start_,false))
          invalidate("handoff_prefix_blocked_by_lidar");
        // A committed curve already has an observed-free proof at generation.
        // Recheck new obstacles; forgetting a free cell is not a new obstacle.
        if (pending_ && !safe(candidate_,0,INFINITY,false)) invalidate("pending_trajectory_blocked_by_lidar");
        if (active_.getPieceNum()) {
          if(!safe(active_,seconds()-active_start_,seconds()+lead_+.2-active_start_,false))
            invalidate("handoff_prefix_blocked_by_lidar");
          else {
            future_blocked_=!safe(active_,seconds()-active_start_,INFINITY,false);
            if(future_blocked_) last_attempt_=-1e9;
          }
        }
      } catch (const std::exception &e) { reason_ = std::string("cloud_rejected:")+e.what(); }
    }
  }
  // Find a reachable local goal, treating every unobserved cell as blocked.
  // This frontend is independent of SUPER's A*/exploration/backup algorithms.
  bool local_problem(const Boundary &start, Boundary &goal, std::vector<Vec> &route, bool &target_final) {
    const double r = cfg_.resolution;
    const int n = int(std::ceil((horizon_+2)/r))*2, nz = 12;
    Eigen::Vector3i dims(n,n,nz);
    // Center a grid cell on the exact boundary state, rather than nudging the
    // start into an unobserved neighboring cell in the LiDAR blind volume.
    cfg_.low = start.col(0)-Vec(n*r/2,n*r/2,nz*r/2)-Vec::Constant(r/2);
    cfg_.high = cfg_.low + dims.cast<double>()*r;
    const int count = n*n*nz;
    auto id = [=](int x,int y,int z) {return x+n*(y+n*z);};
    auto xyz = [=](int i) { return Eigen::Vector3i(i%n,(i/n)%n,i/(n*n)); };
    auto center = [&](int i) -> Vec {return cfg_.low+(xyz(i).cast<double>()+Vec::Constant(.5))*r;};
    std::vector<uint8_t> free(count), visited(count);
    for (int i=0;i<count;++i) {
      Vec p=center(i);
      free[i] = map_->free(p);
    }
    auto start_id = ((start.col(0)-cfg_.low)/r).array().floor().cast<int>().matrix().eval();
    int root = id(start_id.x(),start_id.y(),start_id.z());
    if (!free[root] || !map_->free(start.col(0))) { reason_="start_not_observed_free"; return false; }
    std::vector<int> parent(count,-1);
    std::queue<int> queue; queue.push(root); visited[root]=1;
    int best=-1; double best_score=(start.col(0)-target_).norm();
    while(!queue.empty()) {
      int i=queue.front(); queue.pop(); auto p=xyz(i); Vec c=center(i);
      const double travel=(c-start.col(0)).norm(), score=(c-target_).norm();
      if(travel <= range_ && score < best_score-1e-6) { best=i; best_score=score; }
      for(int d=0;d<3;++d) for(int s:{-1,1}) {
        auto q=p; q[d]+=s;
        if((q.array()<0).any() || (q.array()>=dims.array()).any()) continue;
        int j=id(q.x(),q.y(),q.z());
        if(!visited[j] && free[j] && (center(j)-start.col(0)).norm() <= range_+r) {
          bool edge_free=true;
          for(double u=.1;u<1;u+=.1) if(!map_->free(c+(center(j)-c)*u)) {edge_free=false;break;}
          if(!edge_free) continue;
          visited[j]=1;parent[j]=i;queue.push(j);
        }
      }
    }
    goal=Boundary::Zero();
    const auto target_cell=((target_-cfg_.low)/r).array().floor().cast<int>().matrix().eval();
    target_final=false;
    if((target_cell.array()>=0).all() && (target_cell.array()<dims.array()).all()) {
      int t=id(target_cell.x(),target_cell.y(),target_cell.z());
      if(visited[t] && map_->free(target_) && (target_-start.col(0)).norm()<=range_) {
        best=t;target_final=true;
      }
    }
    // Navigation goals have spatial tolerance. If the exact target voxel is
    // unobserved (common in a LiDAR blind/FOV boundary), finish at the closest
    // actually observed reachable cell inside the declared tolerance.
    if(best>=0 && best_score<=goal_tolerance_) target_final=true;
    if(best<0 || (!target_final && (center(best)-start.col(0)).norm()<1.0)) {reason_="no_observed_local_progress";return false;}
    goal.col(0)=target_final && map_->free(target_)?target_:center(best);
    for(int i=best;i!=root;i=parent[i]) route.push_back(center(i));
    route.push_back(start.col(0));std::reverse(route.begin(),route.end());route.back()=goal.col(0);
    // Shortcut only through observed free space. This keeps GCOPTER's geometric
    // corridor frontend independent while respecting the sensor-derived route.
    std::vector<Vec> shortened{route.front()};
    size_t begin=0;
    while(begin+1<route.size()) {
      size_t next=begin+1;
      for(size_t j=route.size()-1;j>begin+1;--j) {
        bool clear=true;const Vec delta=route[j]-route[begin];
        for(double t=0;t<=delta.norm();t+=.05)
          if(!map_->free(route[begin]+delta*(t/std::max(.001,delta.norm())))) {clear=false;break;}
        if(clear) {next=j;break;}
      }
      shortened.push_back(route[next]);begin=next;
    }
    route=std::move(shortened);return true;
  }
  uav_gcopter::Result solve(const Boundary &start,const Boundary &goal,const std::vector<Vec> &route) {
    super_utils::vec_E<Vec> occupied;
    map_->boxSearchInflate(cfg_.low,cfg_.high,super_utils::OCCUPIED,occupied);
    std::vector<Vec> surface(occupied.begin(),occupied.end());
    const double margin=std::sqrt(3.0)*map_->getInfResolution()/2;
    uav_gcopter::Result result;
    // Add only unknown cells actually cut by an optimized curve. Materializing
    // the entire unknown volume creates fragmented FIRI seeds with sparse
    // scans; ignoring it permits the optimizer to cut across unknown corners.
    // The complete resulting curve still has to pass the independent check.
    std::set<std::tuple<int,int,int>> added;
    for(int pass=0;pass<4;++pass) {
      result=uav_gcopter::optimizeRoute(cfg_,start,goal,route,surface,margin);
      if(!result || !std::isfinite(result.trajectory.getTotalDuration()) || result.trajectory.getTotalDuration()>120) return result;
      bool unknown=false;
      for(double t=0;t<=result.trajectory.getTotalDuration();t+=.01) {
        Vec p=result.trajectory.getPos(t);
        if(!p.allFinite()) {result.error="trajectory_nonfinite";return result;}
        if(!map_->free(p)) {
          unknown=true;
          auto id=(p/map_->getResolution()).array().floor().cast<int>().matrix().eval();
          auto key=std::make_tuple(id.x(),id.y(),id.z());
          if(added.insert(key).second) surface.push_back((id.cast<double>()+Vec::Constant(.5))*map_->getResolution());
        }
      }
      if(!unknown) return result;
    }
    result.error="trajectory_left_observed_free_space";return result;
  }
  void attempt() {
    const double began=seconds();
    // Wait for actual execution of the last handoff before replacing it again;
    // only initial mode acquisition needs a longer grace period.
    if (pending_ || (active_.getPieceNum() && seconds()<active_start_+(revision_==1?.15:.03))) return;
    if (active_.getPieceNum() && final_ && !future_blocked_) return;
    if (began-last_attempt_<interval_) return;
    last_attempt_=began;
    Boundary start=Boundary::Zero(), goal;
    double handoff=began+lead_;
    if(active_.getPieceNum() && handoff>active_start_+active_.getTotalDuration()-.05) {
      invalidate("local_plan_exhausted:"+reason_);return;
    }
    if(active_.getPieceNum()) start=at(active_,handoff-active_start_);
    else {
      const auto &s=odoms_.back(); const auto &v=s.twist.twist.linear;
      if(Vec(v.x,v.y,v.z).norm()>.15) {reason_="waiting_for_stationary_first_start";return;}
      start.col(0)=position(s);
    }
    std::vector<Vec> route;
    bool target_final=false;
    uav_gcopter::Result result;
    bool feasible=false;
    // A shorter observed-free stopping segment can remain feasible when a
    // distant detour does not. Retry within this same future handoff budget,
    // rather than cancelling after the first long-horizon solve fails.
    for(int retry=0;retry<4 && seconds()<handoff-.2;++retry) {
      route.clear();
      if(!local_problem(start,goal,route,target_final)) break;
      result=solve(start,goal,route);
      if(result && safe(result.trajectory,0)) {feasible=true;break;}
      reason_=result?"trajectory_check_failed":result.error;
      if(range_<=2) break;
      range_=std::max(2.0,range_*.65);
    }
    if(!feasible) {
      if(future_blocked_) invalidate("blocked_route_replan_failed:"+reason_);
      return;
    }
    if(seconds()>handoff-.15) {
      reason_="solve_missed_handoff";if(future_blocked_) invalidate(reason_);return;
    }
    range_=std::min(horizon_,range_+.5);
    Path msg;msg.interface_version=1;msg.task_id=task_;msg.vehicle_id=vehicle_;msg.frame_id=frame_;
    msg.revision=revision_+1;msg.generated_at=now();msg.valid_from=rclcpp::Time(int64_t(handoff*1e9),RCL_ROS_TIME);
    msg.valid_fields=7;msg.max_speed=cfg_.bounds[0];msg.is_final=target_final;
    double duration=result.trajectory.getTotalDuration();
    std::set<double> times{0,duration};
    for(double t=.02;t<duration;t+=.02) times.insert(t);
    double boundary=0;for(const auto &piece:result.trajectory) {boundary+=piece.getDuration();times.insert(boundary);}
    // Face horizontal velocity. At a stationary endpoint retain the closest
    // moving heading; unwrap successive samples so interpolation takes the
    // short angular path. Exact P/V/A handoff makes update yaw continuous too.
    double yaw=heading(result.trajectory,0,get_parameter("yaw").as_double());
    for(double t:times) {
      auto s=at(result.trajectory,t);uav_planner_interfaces::msg::TimedTrajectoryPoint p;
      double next_yaw=heading(result.trajectory,t,yaw);
      yaw+=std::remainder(next_yaw-yaw,2*std::acos(-1.0));
      int64_t ns=std::llround(t*1e9);p.time_from_start.sec=ns/1000000000;p.time_from_start.nanosec=ns%1000000000;
      p.position.x=s(0,0);p.position.y=s(1,0);p.position.z=s(2,0);
      p.velocity.x=s(0,1);p.velocity.y=s(1,1);p.velocity.z=s(2,1);
      p.acceleration.x=s(0,2);p.acceleration.y=s(1,2);p.acceleration.z=s(2,2);p.yaw=yaw;
      if(!msg.points.empty() && p.time_from_start==msg.points.back().time_from_start) continue;
      msg.points.push_back(p);
    }
    candidate_=result.trajectory;candidate_start_=handoff;candidate_final_=target_final;pending_=true;
    plans_->publish(msg);reason_="awaiting_execution_ack";
    RCLCPP_INFO(get_logger(),"revision=%lu solve_s=%.3f local_goal=[%.2f %.2f %.2f] final=%d",
      static_cast<unsigned long>(msg.revision),seconds()-began,goal(0,0),goal(1,0),goal(2,0),int(target_final));
  }
public:
  LocalPlanner():Node("gcopter_local_function") {
    frame_=declare_parameter<std::string>("frame_id","drone0/odom");
    vehicle_=declare_parameter<std::string>("vehicle_id","drone0");
    auto goal=declare_parameter<std::vector<double>>("goal",{0.,20.,2.});
    if(goal.size()!=3) throw std::invalid_argument("goal requires XYZ");
    target_=Eigen::Map<Vec>(goal.data());
    if(!target_.allFinite()) throw std::invalid_argument("goal must be finite");
    lead_=declare_parameter("lead_time",.4);horizon_=declare_parameter("planning_horizon",8.);
    range_=horizon_;
    goal_tolerance_=declare_parameter("goal_tolerance",1.5);
    interval_=declare_parameter("replan_interval",.2);
    declare_parameter("yaw",0.0);
    if(lead_<.2 || horizon_<2 || horizon_>12 || interval_<.1) throw std::invalid_argument("invalid local timing/horizon");
    cfg_.resolution=declare_parameter("resolution",.5);
    cfg_.rrt_timeout=declare_parameter("rrt_timeout",.05);cfg_.bounds[0]=declare_parameter("max_speed",3.);
    cfg_.weight_time=declare_parameter("weight_time",2.);ompl::RNG::setSeed(42);
    if(!std::isfinite(cfg_.resolution) || cfg_.resolution<.2 || cfg_.resolution>1 ||
       !std::isfinite(lead_) || !std::isfinite(horizon_) || !std::isfinite(interval_) ||
       !std::isfinite(goal_tolerance_) || goal_tolerance_<=0 || goal_tolerance_>2 ||
       !std::isfinite(cfg_.bounds[0]) || cfg_.bounds[0]<=0 || cfg_.bounds[0]>4)
      throw std::invalid_argument("invalid local resolution/speed/timing");
    map_=std::make_unique<LocalMap>(declare_parameter<std::string>("map_config",""));
    plans_=create_publisher<Path>("gcopter/plan",1);
    status_=create_publisher<std_msgs::msg::String>("gcopter/local_status",10);
    failure_=create_publisher<std_msgs::msg::String>("gcopter/failure",10);
    odom_sub_=create_subscription<nav_msgs::msg::Odometry>("navigation/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::SharedPtr s) {
      const auto &q=s->pose.pose.orientation;
      if(s->header.frame_id!=frame_ || !position(*s).allFinite() ||
        !std::isfinite(q.w+q.x+q.y+q.z) || std::abs(q.w*q.w+q.x*q.x+q.y*q.y+q.z*q.z-1)>.01) return;
      if(!odoms_.empty() && rclcpp::Time(s->header.stamp)<rclcpp::Time(odoms_.back().header.stamp)) {
        if(requested_) invalidate("state_time_reversed");return;
      }
      odoms_.push_back(*s);while(odoms_.size()>200) odoms_.pop_front();
    });
    cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("navigation/points",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::SharedPtr s) {
      scans_.push_back(s);while(scans_.size()>10) scans_.pop_front();
    });
    execution_sub_=create_subscription<std_msgs::msg::String>("gcopter/state",10,[this](std_msgs::msg::String::SharedPtr msg) {
      try {
        auto s=YAML::Load(msg->data);enabled_=s["enabled"].as<bool>();
        if(!enabled_ && requested_) {requested_=false;reason_="execution_disabled";}
        if(!failed_ && s["error"] && !s["error"].as<std::string>().empty()) {failed_=true;requested_=false;reason_="execution_failed";}
        if(pending_ && s["accepted_revision"] && s["accepted_revision"].as<uint64_t>()==revision_+1 &&
          s["task"].as<std::string>()==task_) {
          previous_=active_;previous_start_=active_start_;
          active_=candidate_;active_start_=candidate_start_;final_=candidate_final_;pending_=false;++revision_;reason_="tracking";
        }
      } catch(const std::exception &) {}
    });
    start_=create_service<Trigger>("gcopter/plan_once",[this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r) {
      r->success=fresh() && enabled_ && !failed_ && !requested_ && !revision_ && !pending_;
      if(r->success) {requested_=true;task_="gcopter-local-"+std::to_string(now().nanoseconds());}
      r->message=r->success?"local_planning_started":(!enabled_?"execution_not_enabled":reason_);
    });
    reset_=create_service<Trigger>("gcopter/reset",[this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r) {
      invalidate("map_reset_requires_restart");r->success=true;r->message="stopped; restart local map before next task";
    });
    health_=create_service<Trigger>("gcopter/health",[this](const Trigger::Request::SharedPtr,Trigger::Response::SharedPtr r) {
      r->success=fresh()&&!failed_;r->message=reason_;
    });
    timer_=create_wall_timer(std::chrono::milliseconds(50),[this] {
      consume_scans();
      if(requested_ && !failed_) {
        if(!fresh()) invalidate("sensor_watchdog");
        else if(pending_ && seconds()>candidate_start_-.05) invalidate("execution_ack_timeout");
        else attempt();
      }
      std_msgs::msg::String m;
      m.data="{\"reason\":\""+reason_+"\",\"revision\":"+std::to_string(revision_)+
        ",\"map_stamp\":"+std::to_string(map_time_)+",\"pending\":"+(pending_?"true":"false")+"}";
      status_->publish(m);
    });
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<LocalPlanner>());rclcpp::shutdown();
}
