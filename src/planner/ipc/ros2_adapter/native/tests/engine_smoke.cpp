#include "engine.hpp"
int main(){
 ipc_native::Parameters p;
 p.values={{"/ipc_node/mpc/horizon",15},{"/ipc_node/mpc/step",.1}};
 Engine engine(p);
 engine.odom_p_={0,0,2};engine.odom_v_.setZero();engine.odom_a_.setZero();
 engine.plan(Eigen::Vector3d(.5,0,2));
 Eigen::Vector3d a,j;if(!engine.solve(a,j) || !a.allFinite() || !j.allFinite())return 1;
 for(int i=0;i<20;++i){engine.plan(Eigen::Vector3d(.5,0,2));if(!engine.solve(a,j))return 3;}
 engine.local_pc_.push_back(Eigen::Vector3d(.5,0,2));
 try{engine.plan(Eigen::Vector3d(.5,0,2));return 2;}catch(const std::runtime_error&){}
 engine.local_pc_.clear();engine.local_pc_.push_back(Eigen::Vector3d(0,9,2));
 engine.plan(Eigen::Vector3d(0,20,2));
 if(engine.follow_path_.back().y()>=9)return 4;
 try{engine.plan(Eigen::Vector3d(0,9,2));return 5;}catch(const std::runtime_error&){}
 engine.local_pc_.clear();engine.plan(Eigen::Vector3d(2,0,2));
 auto unchanged=engine.follow_path_;engine.plan(Eigen::Vector3d(2,0,2),true);
 if(engine.path_replanned_ || engine.follow_path_!=unchanged)return 9;
 engine.local_pc_={Eigen::Vector3d(1,0,2)};engine.plan(Eigen::Vector3d(2,0,2),true);
 if(!engine.path_replanned_)return 10;
 engine.obstacle_clearance_=.55;engine.sfc_dis_=.55;
 engine.local_pc_={Eigen::Vector3d(.5,.5,2)};
 engine.plan(Eigen::Vector3d(-1,0,2));
 if((engine.follow_path_.front()-engine.odom_p_).norm()>1e-9)return 6;
 engine.local_pc_={engine.odom_p_};
 try{engine.plan(Eigen::Vector3d(-1,0,2));return 7;}catch(const std::runtime_error&){}
 Eigen::Matrix<double,Eigen::Dynamic,4> invalid(1,4);invalid.setConstant(std::numeric_limits<double>::quiet_NaN());
 try{engine.mpc_->SetFSC(invalid,0);return 8;}catch(const std::runtime_error&){}
 engine.local_pc_={Eigen::Vector3d(1,0,2)};engine.planar_=true;engine.obstacle_clearance_=.75;engine.path_check_clearance_=.55;
 engine.plan(Eigen::Vector3d(2,0,2));double lateral=0;
 for(const auto& point:engine.follow_path_){if(std::abs(point.z()-2)>1e-9)return 11;lateral=std::max(lateral,std::abs(point.y()));}
 if(lateral<.55)return 12;
 engine.plan(Eigen::Vector3d(2,0,2),true);if(engine.path_replanned_)return 13;
 engine.odom_p_.z()=2.02;engine.local_pc_={Eigen::Vector3d(0,9,2)};
 engine.plan(Eigen::Vector3d(0,20,2));
 if(std::abs(engine.follow_path_.back().z()-2)>1e-9 || (engine.follow_path_.front()-engine.odom_p_).norm()>1e-9)return 14;
 try{engine.plan(Eigen::Vector3d(0,20,3));return 15;}catch(const std::runtime_error&){}
 return 0;
}
