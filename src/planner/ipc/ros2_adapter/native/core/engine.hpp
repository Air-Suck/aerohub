#pragma once
#include "mpc.h"
#include "local_astar.h"

#include <memory>
#include <sstream>
struct Engine {
 std::shared_ptr<MPCPlannerClass> mpc_;
 std::shared_ptr<LoaclAstarClass> local_astar_;
 std::vector<Eigen::Vector3d> local_pc_, follow_path_, mpc_goals_;
 Eigen::Vector3d odom_p_=Eigen::Vector3d::Zero(), odom_v_=Eigen::Vector3d::Zero(), odom_a_=Eigen::Vector3d::Zero();
 bool path_replanned_=false,planar_=false;
 double map_z_min_=0.,map_z_max_=8.;
 int astar_index_=0, ref_dis_=6;
 double path_dis_=0.05, sfc_dis_=0.3, obstacle_clearance_=0.35,path_check_clearance_=0.35;
 explicit Engine(ipc_native::Parameters parameters):mpc_(std::make_shared<MPCPlannerClass>(parameters)),local_astar_(std::make_shared<LoaclAstarClass>()) {
   parameters.param("/ipc_node/map/z_min",map_z_min_,0.);parameters.param("/ipc_node/map/z_max",map_z_max_,8.);
   local_astar_->InitMap(.1, Eigen::Vector3d(-10,-10,map_z_min_),Eigen::Vector3d(10,10,map_z_max_));
 }
 void GenerateAPolytope(Eigen::Vector3d,Eigen::Vector3d,Eigen::Matrix<double,Eigen::Dynamic,4>&,uint8_t);
 void plan(const Eigen::Vector3d& requested_goal,bool retain_valid_path=false) {
   path_replanned_=false;
   Eigen::Vector3d goal=requested_goal;
   Eigen::Vector3d delta=goal-odom_p_;
   double extent=delta.head<2>().cwiseAbs().maxCoeff();
   const bool local_goal=extent>9.;
   if(local_goal) goal=odom_p_+delta*(9./extent);

   if(planar_)goal.z()=requested_goal.z();
   local_astar_->planar_=planar_;
   local_astar_->SetCenter(Eigen::Vector3d(odom_p_.x(),odom_p_.y(),0));
   local_astar_->Reset();
   local_astar_->setObsVector(local_pc_, retain_valid_path ? path_check_clearance_ : obstacle_clearance_);
   if(retain_valid_path && !follow_path_.empty()){
     size_t nearest=0;double distance=1e9;
     for(size_t i=0;i<follow_path_.size();++i){double d=(follow_path_[i]-odom_p_).norm();if(d<distance){nearest=i;distance=d;}}
     std::vector<Eigen::Vector3d> remaining(follow_path_.begin()+nearest,follow_path_.end());
     bool extend=(follow_path_.back()-requested_goal).norm()>.3 && (follow_path_.back()-odom_p_).norm()<2.;
     if(!extend && distance<.5 && local_astar_->CheckPathFree(remaining))return;
   }
   if(retain_valid_path)local_astar_->setObsVector(local_pc_,obstacle_clearance_);
   Eigen::Vector3d search_start=odom_p_;
   if(planar_){
     if(std::abs(odom_p_.z()-requested_goal.z())>.25)throw std::runtime_error("planar route requires established cruise height");
     search_start.z()=requested_goal.z();
     Eigen::Vector3d direction=search_start-odom_p_;double length=direction.squaredNorm();
     if(length>1e-12)for(const auto& obstacle:local_pc_){double t=std::clamp((obstacle-odom_p_).dot(direction)/length,0.,1.);if((obstacle-odom_p_-t*direction).norm()<sfc_dis_)throw std::runtime_error("unsafe cruise-height connector");}
   }
   const Eigen::Vector3d initial_start=search_start;
   if(!local_astar_->CheckStartEnd(search_start)){
     // The upstream cube inflation may occupy a physically clear start cell.
     // Connect to a nearby free cell only if the entire connector has the same
     // Euclidean clearance required by the SFC; never clear obstacle cells.
     double best=1e9;
     for(int x=-6;x<=6;++x)for(int y=-6;y<=6;++y)for(int z=planar_?0:-6;z<=(planar_?0:6);++z){
       Eigen::Vector3d candidate=initial_start+.1*Eigen::Vector3d(x,y,z);
       double length=(candidate-odom_p_).squaredNorm();
       if(length<1e-12 || length>=best || length>.36 || !local_astar_->CheckStartEnd(candidate))continue;
       Eigen::Vector3d direction=candidate-odom_p_;bool clear=true;
       for(const auto& obstacle:local_pc_){
         double t=std::clamp((obstacle-odom_p_).dot(direction)/length,0.,1.);
         if((obstacle-odom_p_-t*direction).norm()<sfc_dis_){clear=false;break;}
       }
       if(clear){search_start=candidate;best=length;}
     }
     if(best==1e9)throw std::runtime_error("start occupied without clearance-valid connector");
   }
   if(local_goal && !local_astar_->CheckStartEnd(goal)){
     for(double reach=8.9;reach>=2.;reach-=.1){
       Eigen::Vector3d candidate=odom_p_+delta*(reach/extent);
       if(planar_)candidate.z()=requested_goal.z();
       if(local_astar_->CheckStartEnd(candidate)){goal=candidate;break;}
     }
   }
   if(!local_astar_->CheckStartEnd(goal))throw std::runtime_error("goal occupied");
   if(!local_astar_->SearchPath(search_start,goal)) throw std::runtime_error("A* failed");
   std::vector<Eigen::Vector3d> path,waypoints;
   local_astar_->GetPath(path); local_astar_->FloydHandle(path,waypoints);
   if(planar_)for(auto& point:waypoints)point.z()=requested_goal.z();
   if(waypoints.empty()) throw std::runtime_error("empty A* path");
   if((search_start-odom_p_).norm()>1e-6)waypoints.insert(waypoints.begin(),odom_p_);
   follow_path_.clear();
   for(size_t i=1;i<waypoints.size();++i) {
     Eigen::Vector3d delta=waypoints[i]-waypoints[i-1];
     int n=std::max(1,int(std::ceil(delta.norm()/path_dis_)));
     for(int j=0;j<n;++j) follow_path_.push_back(waypoints[i-1]+delta*double(j)/n);
   }
   follow_path_.push_back(goal); astar_index_=0;path_replanned_=true;
 }
 bool conservative_corridor_used_=false;
 bool record_primary_failure_=false;
 std::string primary_failure_dump_;
 bool runScheduledMpc(Eigen::Matrix<double,Eigen::Dynamic,4> corridor) {
   conservative_corridor_used_=false;
   if(mpc_->Run())return true;
   if(record_primary_failure_ && primary_failure_dump_.empty()){
     std::ostringstream stream;mpc_->dumpProblem(stream);primary_failure_dump_=stream.str();
   }
   // A path-distance schedule can demand entry to the next corridor before
   // the vehicle can turn/brake. Retry a NEW optimization constrained to the
   // current convex safe region at every prediction step. No old controls,
   // softened obstacle planes, or unconstrained prefix are accepted.
   if(corridor.rows()==0 || !corridor.allFinite() || !mpc_->IsInFSC(odom_p_,corridor))return false;
   mpc_->beginCorridors();
   for(int i=0;i<mpc_->MPC_HORIZON;++i)mpc_->SetFSC(corridor,i);
   conservative_corridor_used_=mpc_->Run();
   return conservative_corridor_used_;
 }
 bool solve(Eigen::Vector3d& acceleration,Eigen::Vector3d& jerk) {
        Eigen::Matrix<double,Eigen::Dynamic,4> initial_corridor;
        mpc_->beginCorridors();
        if (follow_path_.size() > 0) { // follow A* path
            double min_dis = 10000.0;
            for (int i = 0; i < follow_path_.size(); i++) { // find the nearest point
                double dis = (odom_p_ - follow_path_[i]).norm();
                if (dis < min_dis) {
                    min_dis = dis;
                    astar_index_ = i;
                }
            }

            int goal_in_sfc = astar_index_;
            if (follow_path_.size() - astar_index_ <= ref_dis_) { // near the end, find the sfc at current point
                goal_in_sfc = follow_path_.size();
                Eigen::Matrix<double, Eigen::Dynamic, 4> planes;
                GenerateAPolytope(odom_p_, odom_p_, planes, 0);initial_corridor=planes;
                for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                    mpc_->SetFSC(planes, i);
                }
            } else { // find the biggest and the longest sfc
                Eigen::Matrix<double, Eigen::Dynamic, 4> planes, last_planes;
                GenerateAPolytope(follow_path_[astar_index_], follow_path_[astar_index_], planes, 0);initial_corridor=planes; 
                last_planes = planes;
                int init_num = 0;
                if (mpc_->IsInFSC(odom_p_, planes) == false) { // if odom is not in initial sfc
                    init_num = (odom_p_ - follow_path_[astar_index_]).norm() / path_dis_ / ref_dis_;
                    IPC_LOG("\033[35m UAV is out sfc! init num is %d \033[0m", init_num);
                }
                int first_id = astar_index_, mpc_goal_index = 0;
                for (int i = first_id+1; i < follow_path_.size(); i++) { // find last pos in first sfc
                    if (mpc_->IsInFSC(follow_path_[i], planes)) {
                        first_id = i;
                        goal_in_sfc = i;
                    } else {
                        first_id -= 0.2 / path_dis_;
                        if (first_id < astar_index_) first_id = astar_index_;
                        break;
                    }
                }
                mpc_goal_index = (first_id - astar_index_) / ref_dis_ + init_num + 1;
                assert(first_id >= astar_index_);
                for (int i = init_num; i <= mpc_goal_index && i < mpc_->MPC_HORIZON; i++) {
                    mpc_->SetFSC(planes, i);
                }
                int end_index = first_id + (mpc_->MPC_HORIZON-mpc_goal_index) * ref_dis_;
                if (end_index >= follow_path_.size()) end_index = follow_path_.size() - 1;
                for (int i = end_index; i >= first_id && mpc_goal_index < mpc_->MPC_HORIZON - 1; i--) {
                    if (local_astar_->CheckPoint(follow_path_[i]) == false) continue;
                    if (local_astar_->CheckLineObstacleFree(follow_path_[first_id], follow_path_[i]) == false) continue;
                    i = i - 0.2 / path_dis_;
                    if (i <= first_id) break;
                    Eigen::Matrix<double, Eigen::Dynamic, 4> long_planes;
                    GenerateAPolytope(follow_path_[first_id], follow_path_[i], long_planes, 1);
                    if (long_planes.rows() > 0) {
                        for (int j = mpc_goal_index+1; j < mpc_->MPC_HORIZON; j++) {
                            mpc_->SetFSC(long_planes, j);
                        }
                        mpc_goal_index = mpc_->MPC_HORIZON;
                        for (int k = first_id; k < follow_path_.size(); k++) {
                            if (mpc_->IsInFSC(follow_path_[k], long_planes)) goal_in_sfc = k;
                            else break;
                        }
                        // IPC_LOG("\033[43;37m long sfc : %d \033[0m", goal_in_sfc);
                        break;
                    }
                }
                for (int i = mpc_goal_index+1; i < mpc_->MPC_HORIZON; i++) {
                    if (last_planes.rows() > 0) mpc_->SetFSC(last_planes, i);
                }
            }

            // MPC set goal
            mpc_goals_.clear();
            Eigen::Vector3d last_p_ref;
            for (int i = 0; i < mpc_->MPC_HORIZON; i++) {
                int index = astar_index_ + i * ref_dis_;
                if (index >= goal_in_sfc) index = goal_in_sfc;
                if (index >= follow_path_.size()) index = follow_path_.size() - 1;
                Eigen::Vector3d v_r(0, 0, 0);
                if (i == 0) v_r = (follow_path_[index] - odom_p_) / mpc_->MPC_STEP;
                else if (i == mpc_->MPC_HORIZON) v_r.setZero();
                else v_r = (follow_path_[index] - last_p_ref) / mpc_->MPC_STEP;
                last_p_ref = follow_path_[index];
                
                mpc_->SetGoal(follow_path_[index], v_r, Eigen::Vector3d::Zero(), i);
                // std::cout << "mpc goal " << i << ": " << follow_path_[index].transpose() << " v: " << v_r.transpose() << std::endl;
                mpc_goals_.push_back(follow_path_[index]);
            }
        } else { throw std::runtime_error("no path"); }

 mpc_->SetStatus(odom_p_,odom_v_,odom_a_);
 if(!runScheduledMpc(initial_corridor)) return false;
 mpc_->GetOptimCmd(jerk,0);
 Eigen::MatrixXd A,B; mpc_->SystemModel(A,B,mpc_->MPC_STEP);
 Eigen::VectorXd predicted=A*mpc_->X_0_+B*jerk;
 acceleration=predicted.segment<3>(6)+Eigen::Vector3d(0,0,9.81);
 return acceleration.allFinite() && jerk.allFinite();
 }
};
