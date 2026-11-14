#include "engine.hpp"
#include <rclcpp/rclcpp.hpp>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <uav_follow_reference_acro/msg/control_session.hpp>
#include <uav_follow_reference_acro/msg/control_frame.hpp>
#include <atomic>
#include <fstream>
#include <future>
#include <deque>
#include <mutex>
#include <thread>
using Session=uav_follow_reference_acro::msg::ControlSession;
using Frame=uav_follow_reference_acro::msg::ControlFrame;
class IPCNode: public rclcpp::Node {
 std::unique_ptr<Engine> engine_;
 std::future<std::shared_ptr<Engine>> planning_;
 std::shared_ptr<LoaclAstarClass> spare_grid_;
 uint64_t session_epoch_=0,planning_epoch_=0;
 rclcpp::Time planning_stamp_{0,0,RCL_ROS_TIME};
 std::mutex output_mutex_,input_mutex_;
 rclcpp::CallbackGroup::SharedPtr input_group_;
 uint64_t cloud_generation_=0,used_cloud_generation_=0;
 Frame cached_;
 bool have_cached_=false;
 double cached_wall_=0;
 rclcpp::Time cached_state_stamp_{0,0,RCL_ROS_TIME},cached_cloud_stamp_{0,0,RCL_ROS_TIME};
 std::atomic<bool> stopping_{false};
 std::thread sender_;
 void send_loop(){
   uint64_t sequence=0;
   while(!stopping_ && rclcpp::ok()){
     auto deadline=std::chrono::steady_clock::now()+std::chrono::milliseconds(10);
     {
       std::lock_guard<std::mutex> lock(output_mutex_);
       if(have_cached_){
         Frame out=cached_;out.sequence=++sequence;
         const double age=(now()-rclcpp::Time(out.header.stamp)).seconds();
         if(age<0 || age>.1 || ipc_native::now()-cached_wall_>.1 || !fresh(cached_state_stamp_,.2) || !fresh(cached_cloud_stamp_,.5))out.solver_ok=false;
         output_->publish(out);
       }
     }
     std::this_thread::sleep_until(deadline);
   }
 }

 std::deque<std::pair<rclcpp::Time,pcl::PointCloud<pcl::PointXYZ>::Ptr>> cloud_history_;
 std::vector<Eigen::Vector3d> points_;
 rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ready_;
 Session session_; nav_msgs::msg::Odometry odom_;
 rclcpp::Time cloud_stamp_{0,0,RCL_ROS_TIME},last_solve_{0,0,RCL_ROS_TIME};
 rclcpp::Time last_attempt_stamp_{0,0,RCL_ROS_TIME},session_start_{0,0,RCL_ROS_TIME};
 int nominal_reference_stride_=1;
 bool have_odom_=false,have_cloud_=false,replan_=true;
 bool planar_path_=false;double search_clearance_=0,path_check_clearance_=0;
 uint64_t sequence_=0; double hover_,max_thrust_,last_throttle_,clearance_,reference_speed_,velocity_limit_,reference_speed_ratio_,reference_acceleration_=0.,map_z_min_=0.,map_z_max_=8.;
 std::string planning_frame_,body_frame_,diagnostic_qp_path_;
 bool qp_dumped_=false;
 rclcpp::Subscription<Session>::SharedPtr session_sub_;
 rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
 rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
 rclcpp::Publisher<Frame>::SharedPtr output_;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr health_;
 rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_;
 rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr processed_;
 rclcpp::TimerBase::SharedPtr timer_;
 ipc_native::Parameters parameters_;
 static Eigen::Vector3d vector(const geometry_msgs::msg::Vector3& v){return {v.x,v.y,v.z};}
 void report(const std::string& s){std_msgs::msg::String m;m.data=s;health_->publish(m);}
 bool fresh(const rclcpp::Time& t,double limit){double age=(now()-t).seconds();return t.nanoseconds()>0 && age>=0 && age<=limit;}
 void reset(const Session& s){
   {std::lock_guard<std::mutex> lock(output_mutex_);have_cached_=false;}
   ++session_epoch_;session_=s;session_start_=now();sequence_=0;last_solve_=rclcpp::Time(0,0,RCL_ROS_TIME);last_attempt_stamp_=rclcpp::Time(0,0,RCL_ROS_TIME);replan_=true;
   if(!s.active){engine_.reset();return;}
   auto speed=vector(s.max_speed);
   if(s.target.header.frame_id!=planning_frame_ || s.yaw.mode!=as2_msgs::msg::YawMode::FIXED_YAW || !speed.allFinite() || speed.minCoeff()<=0) {
     session_.active=false;report("unsupported session");return;
   }
   auto p=parameters_;
   const char* axes[]={"x","y","z"};
   for(int i=0;i<3;++i){double bound=std::min(speed[i],velocity_limit_);p.values[std::string("/ipc_node/mpc/v")+axes[i]+"_max"]=bound;p.values[std::string("/ipc_node/mpc/v")+axes[i]+"_min"]=-bound;}
   engine_=std::make_unique<Engine>(p);engine_->record_primary_failure_=!diagnostic_qp_path_.empty();engine_->ref_dis_=1;engine_->path_dis_=std::min(reference_speed_,reference_speed_ratio_*std::min(speed.minCoeff(),velocity_limit_))*engine_->mpc_->MPC_STEP;engine_->ref_dis_=std::max(1,int(std::ceil(engine_->path_dis_/.05-1e-9)));engine_->path_dis_/=engine_->ref_dis_;nominal_reference_stride_=engine_->ref_dis_;engine_->sfc_dis_=clearance_;engine_->obstacle_clearance_=search_clearance_;engine_->path_check_clearance_=path_check_clearance_;engine_->planar_=planar_path_;{std::lock_guard<std::mutex> lock(input_mutex_);engine_->local_pc_=points_;used_cloud_generation_=cloud_generation_;}last_throttle_=hover_;
   report("session_config velocity_limit="+std::to_string(velocity_limit_)+",reference_speed="+std::to_string(reference_speed_)+",path_spacing="+std::to_string(engine_->path_dis_)+",reference_stride="+std::to_string(engine_->ref_dis_)+",planar="+std::to_string(planar_path_));
 }
 void tick(){
   double tick_begin=ipc_native::now(), plan_ms=0;
   nav_msgs::msg::Odometry snapshot_odom;
   rclcpp::Time snapshot_cloud{0,0,RCL_ROS_TIME};
   bool snapshot_have_odom=false,snapshot_have_cloud=false;
   uint64_t generation=0;std::vector<Eigen::Vector3d> new_points;
   {
     std::lock_guard<std::mutex> lock(input_mutex_);
     snapshot_odom=odom_;snapshot_cloud=cloud_stamp_;snapshot_have_odom=have_odom_;snapshot_have_cloud=have_cloud_;generation=cloud_generation_;
     if(engine_ && generation!=used_cloud_generation_)new_points=points_;
   }
   if(engine_ && generation!=used_cloud_generation_){engine_->local_pc_=std::move(new_points);used_cloud_generation_=generation;replan_=true;}

   std_msgs::msg::Bool ready;ready.data=snapshot_have_odom && snapshot_have_cloud && fresh(rclcpp::Time(snapshot_odom.header.stamp),.2) && fresh(snapshot_cloud,.5);ready_->publish(ready);
   if(!session_.active || !engine_){
     RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),1000,"ready=%d odom=%d cloud=%d snapshot_odomage=%f cloud_age=%f",ready.data,snapshot_have_odom,snapshot_have_cloud,(now()-rclcpp::Time(snapshot_odom.header.stamp)).seconds(),(now()-snapshot_cloud).seconds());
     return;
   }
   const auto attempt_stamp=now();
   // ROS timers can dispatch again before the next /clock update. Re-solving
   // with an unchanged time evolves the thrust-model state and gives distinct
   // commands the same identity used by downstream twist/thrust pairing.
   // Reuse the existing cached result with its original age instead.
   if(attempt_stamp<=last_attempt_stamp_)return;
   last_attempt_stamp_=attempt_stamp;
   Frame out;out.header.stamp=attempt_stamp;out.header.frame_id=body_frame_;
   out.session_id=session_.session_id;out.revision=session_.revision;out.sequence=++sequence_;
   out.solver_ok=false;out.last_successful_solve=last_solve_;
   try{
     if(!snapshot_have_odom || !snapshot_have_cloud || !fresh(rclcpp::Time(snapshot_odom.header.stamp),.2) || !fresh(snapshot_cloud,.5))throw std::runtime_error("stale state/cloud");
     auto& p=snapshot_odom.pose.pose.position;auto& q=snapshot_odom.pose.pose.orientation;
     Eigen::Quaterniond rotation(q.w,q.x,q.y,q.z);
     if(!rotation.coeffs().allFinite() || std::abs(rotation.norm()-1)>1e-2)throw std::runtime_error("invalid quaternion");
     engine_->odom_p_={p.x,p.y,p.z};engine_->odom_v_=rotation*vector(snapshot_odom.twist.twist.linear);
     if(!engine_->odom_p_.allFinite() || !engine_->odom_v_.allFinite())throw std::runtime_error("invalid state");
     // Preserve upstream thrust-model acceleration state; no truth acceleration input.
     engine_->odom_a_=rotation*Eigen::Vector3d(0,0,last_throttle_*9.81/hover_)-Eigen::Vector3d(0,0,9.81);
     auto& g=session_.target.point;Eigen::Vector3d goal(g.x,g.y,g.z);
     bool path_changed=false;
     if(planning_.valid() && planning_.wait_for(std::chrono::seconds(0))==std::future_status::ready){
       auto planned=planning_.get();
       if(planned && !planned->follow_path_.empty() && planning_epoch_==session_epoch_ && fresh(planning_stamp_,.5)){
         spare_grid_=engine_->local_astar_;engine_->local_astar_=planned->local_astar_;
         engine_->follow_path_=std::move(planned->follow_path_);engine_->astar_index_=0;path_changed=planned->path_replanned_;
       }else if(planned){spare_grid_=planned->local_astar_;replan_=true;}
     }
     if(replan_ && !planning_.valid()){
       auto snapshot=std::make_shared<Engine>(*engine_);
       if(!spare_grid_){spare_grid_=std::make_shared<LoaclAstarClass>();spare_grid_->InitMap(.1,Eigen::Vector3d(-10,-10,map_z_min_),Eigen::Vector3d(10,10,map_z_max_));}
       snapshot->local_astar_=std::move(spare_grid_);
       planning_epoch_=session_epoch_;planning_stamp_=snapshot_cloud;replan_=false;
       planning_=std::async(std::launch::async,[snapshot,goal]{try{snapshot->plan(goal,true);return snapshot;}catch(const std::exception& e){IPC_LOG("async planning rejected: %s",e.what());snapshot->follow_path_.clear();return snapshot;}});
     }
     if(engine_->follow_path_.empty())throw std::runtime_error("waiting for valid A* path");
     Eigen::Vector3d acceleration,jerk;
     if(reference_acceleration_>0){
       const double progress=std::max(0.,(attempt_stamp-session_start_).seconds())*reference_acceleration_*engine_->mpc_->MPC_STEP/engine_->path_dis_;
       engine_->ref_dis_=std::clamp(int(std::floor(progress)),1,nominal_reference_stride_);
     }
     double begin=ipc_native::now();
     bool solved=engine_->solve(acceleration,jerk);
     if(!diagnostic_qp_path_.empty() && !qp_dumped_ && !engine_->primary_failure_dump_.empty()){std::ofstream stream(diagnostic_qp_path_);if(stream){stream<<engine_->primary_failure_dump_;qp_dumped_=true;engine_->primary_failure_dump_.clear();engine_->record_primary_failure_=false;}}
     if(!solved){
       if(!diagnostic_qp_path_.empty() && !qp_dumped_){std::ofstream stream(diagnostic_qp_path_);if(stream){engine_->mpc_->dumpProblem(stream);qp_dumped_=true;}}
       throw std::runtime_error("MPC solve failed");
     }
     if(acceleration.norm()<1e-6)throw std::runtime_error("degenerate acceleration");
     double yaw=std::atan2(rotation.toRotationMatrix()(1,0),rotation.toRotationMatrix()(0,0));
     Eigen::Vector3d z=acceleration.normalized(),xc(std::cos(yaw),std::sin(yaw),0);
     Eigen::Vector3d y=z.cross(xc);if(y.norm()<1e-6)throw std::runtime_error("degenerate attitude");y.normalize();Eigen::Vector3d x=y.cross(z);
     Eigen::Vector3d hw=(jerk-z.dot(jerk)*z)/acceleration.norm();
     double yaw_error=std::remainder(session_.yaw.angle-yaw,2*M_PI);
     Eigen::Vector3d rates(-hw.dot(y),hw.dot(x),.5*yaw_error*z.z());
     double throttle=acceleration.dot(rotation*Eigen::Vector3d::UnitZ())*hover_/9.81;
     if(!rates.allFinite() || !std::isfinite(throttle) || throttle<.08 || throttle>.9 || rates.cwiseAbs().maxCoeff()>3)throw std::runtime_error("control bounds");
     out.body_rates.x=rates.x();out.body_rates.y=rates.y();out.body_rates.z=rates.z();out.thrust_newtons=throttle*max_thrust_;
     last_throttle_=throttle;
     // Solve age starts before computation; never freshen a reused solution.
     last_solve_=rclcpp::Time(out.header.stamp);out.last_successful_solve=last_solve_;out.solver_ok=true;
     report("solved wall_ms="+std::to_string((ipc_native::now()-begin)*1000)+",iterations="+std::to_string(engine_->mpc_->last_iterations)+",setup_ms="+std::to_string(engine_->mpc_->last_setup_ms)+",optimize_ms="+std::to_string(engine_->mpc_->last_optimize_ms)+",corridor_before="+std::to_string(engine_->mpc_->corridor_rows_before)+",corridor_after="+std::to_string(engine_->mpc_->corridor_rows_after)+",reference_stride="+std::to_string(engine_->ref_dis_)+",conservative_corridor="+std::to_string(engine_->conservative_corridor_used_)+",solve_stamp_ns="+std::to_string(last_solve_.nanoseconds())+",state_stamp_ns="+std::to_string(rclcpp::Time(snapshot_odom.header.stamp).nanoseconds())+",cloud_stamp_ns="+std::to_string(snapshot_cloud.nanoseconds())+",elapsed_to_health_ms="+std::to_string((ipc_native::now()-tick_begin)*1000));
     if(path_changed){nav_msgs::msg::Path path;path.header=out.header;path.header.frame_id=planning_frame_;
     for(const auto& v:engine_->follow_path_){geometry_msgs::msg::PoseStamped point;point.header=path.header;point.pose.position.x=v.x();point.pose.position.y=v.y();point.pose.position.z=v.z();point.pose.orientation.w=1;path.poses.push_back(point);}path_->publish(path);}
   }catch(const std::exception& e){report(e.what());RCLCPP_WARN_THROTTLE(get_logger(),*get_clock(),500,"control rejected: %s",e.what());}
   {std::lock_guard<std::mutex> lock(output_mutex_);cached_=out;cached_wall_=tick_begin;cached_state_stamp_=rclcpp::Time(snapshot_odom.header.stamp);cached_cloud_stamp_=snapshot_cloud;have_cached_=true;}
   RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),500,"tick_ms=%f plan_ms=%f",(ipc_native::now()-tick_begin)*1000,plan_ms);
 }
public:
 ~IPCNode(){stopping_=true;if(sender_.joinable())sender_.join();}
 IPCNode():Node("ipc_native"){
   rcl_interfaces::msg::ParameterDescriptor startup;startup.read_only=true;
   diagnostic_qp_path_=declare_parameter<std::string>("diagnostic_qp_path","",startup);
   hover_=declare_parameter("hover_percentage",.48821,startup);max_thrust_=declare_parameter("max_thrust",15.,startup);
   if(hover_<.1 || hover_>.8 || max_thrust_<=0)throw std::runtime_error("invalid thrust mapping");
   reference_speed_=declare_parameter("reference_speed_mps",.25,startup);
   if(!std::isfinite(reference_speed_) || reference_speed_<=0)throw std::runtime_error("invalid reference speed");
   velocity_limit_=declare_parameter("mpc.velocity_limit",.5,startup);
   reference_speed_ratio_=declare_parameter("reference_speed_ratio",.6,startup);
   reference_acceleration_=declare_parameter("reference_acceleration_mps2",0.,startup);
   if(!std::isfinite(reference_acceleration_) || reference_acceleration_<0)throw std::runtime_error("invalid reference acceleration");
   if(!std::isfinite(velocity_limit_) || velocity_limit_<=0 || !std::isfinite(reference_speed_ratio_) || reference_speed_ratio_<=0 || reference_speed_ratio_>1)throw std::runtime_error("invalid speed envelope");
   clearance_=declare_parameter("obstacle_clearance",.55,startup);
   if(!std::isfinite(clearance_) || clearance_<.35)throw std::runtime_error("invalid clearance");
   planar_path_=declare_parameter("planar_path",false,startup);
   search_clearance_=declare_parameter("astar.search_clearance",clearance_+.2,startup);
   path_check_clearance_=declare_parameter("astar.path_check_clearance",clearance_,startup);
   if(!std::isfinite(search_clearance_) || !std::isfinite(path_check_clearance_) || path_check_clearance_<clearance_ || search_clearance_<=path_check_clearance_)throw std::runtime_error("invalid A* inflation hysteresis");
   planning_frame_=declare_parameter<std::string>("planning_frame","earth",startup);body_frame_=declare_parameter<std::string>("body_frame","drone0/base_link",startup);
   parameters_.values={{"/ipc_node/mpc/horizon",15},{"/ipc_node/mpc/step",.1},{"/ipc_node/mpc/R_p",1000},{"/ipc_node/mpc/R_u",0},{"/ipc_node/mpc/R_u_con",.2},{"/ipc_node/mpc/R_pN",2000},{"/ipc_node/mpc/R_vN",1000},{"/ipc_node/mpc/R_aN",1000}};
   map_z_min_=declare_parameter("map.z_min",0.,startup);map_z_max_=declare_parameter("map.z_max",8.,startup);
   if(!std::isfinite(map_z_min_) || !std::isfinite(map_z_max_) || map_z_max_<=map_z_min_)throw std::runtime_error("invalid map height bounds");
   parameters_.values["/ipc_node/map/z_min"]=map_z_min_;parameters_.values["/ipc_node/map/z_max"]=map_z_max_;
   double acceleration_limit=declare_parameter("mpc.acceleration_limit",1.0,startup);double jerk_limit=declare_parameter("mpc.jerk_limit",5.0,startup);
   if(acceleration_limit<=0 || jerk_limit<=0 || !std::isfinite(acceleration_limit) || !std::isfinite(jerk_limit))throw std::runtime_error("invalid limits");
   for(auto axis:{"x","y","z"})for(auto kind:{"a","u"}) {double limit=std::string(kind)=="a"?acceleration_limit:jerk_limit;parameters_.values[std::string("/ipc_node/mpc/")+kind+axis+"_min"]=-limit;parameters_.values[std::string("/ipc_node/mpc/")+kind+axis+"_max"]=limit;}
   processed_=create_publisher<sensor_msgs::msg::PointCloud2>("ipc/processed_points",rclcpp::SensorDataQoS());
   ready_=create_publisher<std_msgs::msg::Bool>("ipc/ready",1);
   output_=create_publisher<Frame>("external_acro/control",rclcpp::SensorDataQoS());
   health_=create_publisher<std_msgs::msg::String>("ipc/health",10);path_=create_publisher<nav_msgs::msg::Path>("ipc/path",1);
   input_group_=create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
   rclcpp::SubscriptionOptions input_options;input_options.callback_group=input_group_;
   session_sub_=create_subscription<Session>("external_acro/session",rclcpp::QoS(1).reliable().transient_local(),[this](Session::ConstSharedPtr s){if(s->session_id!=session_.session_id || s->revision!=session_.revision || s->active!=session_.active)reset(*s);});
   odom_sub_=create_subscription<nav_msgs::msg::Odometry>("navigation/odom",rclcpp::SensorDataQoS(),[this](nav_msgs::msg::Odometry::ConstSharedPtr m){if(m->header.frame_id==planning_frame_ && m->child_frame_id==body_frame_){std::lock_guard<std::mutex> lock(input_mutex_);odom_=*m;have_odom_=true;}},input_options);
   cloud_sub_=create_subscription<sensor_msgs::msg::PointCloud2>("navigation/points",rclcpp::SensorDataQoS(),[this](sensor_msgs::msg::PointCloud2::ConstSharedPtr m){
     if(m->header.frame_id!=planning_frame_)return;
     try{double begin=ipc_native::now();nav_msgs::msg::Odometry crop_odom;{std::lock_guard<std::mutex> lock(input_mutex_);crop_odom=odom_;}std::vector<Eigen::Vector3d> points; pcl::PointCloud<pcl::PointXYZ>::Ptr raw(new pcl::PointCloud<pcl::PointXYZ>); sensor_msgs::PointCloud2ConstIterator<float>x(*m,"x"),y(*m,"y"),z(*m,"z");
       for(;x!=x.end();++x,++y,++z)if(std::isfinite(*x)&&std::isfinite(*y)&&std::isfinite(*z)&&*z>map_z_min_+.2&&*z<map_z_max_ && std::abs(*x-crop_odom.pose.pose.position.x)<9.5 && std::abs(*y-crop_odom.pose.pose.position.y)<9.5)raw->push_back(pcl::PointXYZ(*x,*y,*z));
       pcl::VoxelGrid<pcl::PointXYZ> filter;filter.setLeafSize(.2,.2,.2);filter.setInputCloud(raw);pcl::PointCloud<pcl::PointXYZ> downsampled;filter.filter(downsampled);
       for(const auto& point:downsampled)points.emplace_back(point.x,point.y,point.z);
       sensor_msgs::msg::PointCloud2 processed;processed.header=m->header;
       sensor_msgs::PointCloud2Modifier modifier(processed);modifier.setPointCloud2FieldsByString(1,"xyz");modifier.resize(points.size());
       sensor_msgs::PointCloud2Iterator<float> px(processed,"x"),py(processed,"y"),pz(processed,"z");
       for(const auto& point:points){*px=point.x();*py=point.y();*pz=point.z();++px;++py;++pz;}processed_->publish(processed);
       // Keep the per-scan diagnostic above separate from the static history map.
       // This also avoids accumulating an already accumulated map during replay.
       rclcpp::Time scan_stamp(m->header.stamp);
       if(!cloud_history_.empty() && scan_stamp<cloud_history_.back().first)cloud_history_.clear();
       cloud_history_.emplace_back(scan_stamp,raw);
       while(cloud_history_.size()>10 || (scan_stamp-cloud_history_.front().first).seconds()>2.)cloud_history_.pop_front();
       pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZ>);
       for(const auto& scan:cloud_history_)for(const auto& point:*scan.second)
         if(std::abs(point.x-crop_odom.pose.pose.position.x)<9.5 && std::abs(point.y-crop_odom.pose.pose.position.y)<9.5)accumulated->push_back(point);
       filter.setInputCloud(accumulated);filter.filter(downsampled);points.clear();
       for(const auto& point:downsampled)points.emplace_back(point.x,point.y,point.z);
       {std::lock_guard<std::mutex> lock(input_mutex_);points_=std::move(points);cloud_stamp_=rclcpp::Time(m->header.stamp);have_cloud_=true;++cloud_generation_;}RCLCPP_INFO_THROTTLE(get_logger(),*get_clock(),2000,"cloud input=%u filtered=%zu preprocess_ms=%f",m->width*m->height,points_.size(),(ipc_native::now()-begin)*1000);
     }catch(const std::exception& e){report(e.what());}
   },input_options);
   timer_=rclcpp::create_timer(this,get_clock(),std::chrono::milliseconds(10),[this]{tick();});
   sender_=std::thread([this]{send_loop();});
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);auto node=std::make_shared<IPCNode>();rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(),2);executor.add_node(node);executor.spin();rclcpp::shutdown();}
