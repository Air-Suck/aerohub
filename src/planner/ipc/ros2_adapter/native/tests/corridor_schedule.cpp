#include "engine.hpp"
#include <iostream>
int main(){
 ipc_native::Parameters p;
 p.values={{"/ipc_node/mpc/horizon",15},{"/ipc_node/mpc/step",.1}};
 for(auto axis:{"x","y","z"}) {
   for(auto kind:{"v","a","u"}) {
     double limit=std::string(kind)=="u"?10.:4.;
     p.values[std::string("/ipc_node/mpc/")+kind+axis+"_min"]=-limit;
     p.values[std::string("/ipc_node/mpc/")+kind+axis+"_max"]=limit;
   }
 }
 Engine e(p);auto& m=*e.mpc_;
 Eigen::Matrix<double,Eigen::Dynamic,4> current(6,4);
 current<<1,0,0,-.6,-1,0,0,-2,0,1,0,-2,0,-1,0,-2,0,0,1,-3,0,0,-1,1;
 auto next=current;next(1,3)=.5; // overlap exists at x=.5 ... .6
 auto prepare=[&](double position,double speed){
   e.odom_p_={position,0,2};e.odom_v_={speed,0,0};e.odom_a_.setZero();
   m.SetStatus(e.odom_p_,e.odom_v_,e.odom_a_);m.beginCorridors();
   for(int i=0;i<m.MPC_HORIZON;++i){m.SetGoal({1,0,2},Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),i);auto planes=i<2?current:next;m.SetFSC(planes,i);}
 };
 prepare(0,0);
 if(!e.runScheduledMpc(current)){std::cerr<<"reachable current corridor rejected because next corridor scheduled too early\n";return 1;}
 if(!e.conservative_corridor_used_)return 2;
 Eigen::VectorXd state=m.X_0_;Eigen::MatrixXd A,B;m.SystemModel(A,B,.1);
 for(int i=0;i<m.MPC_HORIZON;++i){Eigen::Vector3d u;m.GetOptimCmd(u,i);state=A*state+B*u;
   for(int r=0;r<current.rows();++r)if(current.row(r).head<3>().dot(state.head<3>())+current(r,3)>2e-3)return 3;
 }
 prepare(-3,0);if(e.runScheduledMpc(current))return 4; // never recover outside the certified corridor
 prepare(.5,4);if(e.runScheduledMpc(current))return 5; // cannot stop before its wall
 return 0;
}
