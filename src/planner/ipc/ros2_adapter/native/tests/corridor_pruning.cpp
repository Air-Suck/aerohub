#include "mpc.h"
int main(){
 ipc_native::Parameters p;MPCPlannerClass m(p);
 m.SetStatus(Eigen::Vector3d(0,0,1),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero());
 for(int i=0;i<m.MPC_HORIZON;++i)m.SetGoal(Eigen::Vector3d(.5,0,1),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),i);
 Eigen::Matrix<double,Eigen::Dynamic,4> plane(1,4);plane<<1,0,0,-100;
 m.SetFSC(plane,m.MPC_HORIZON-1);
 if(!m.Run() || m.corridor_rows_before!=1 || m.corridor_rows_after!=0)return 1;
 plane<<1,0,0,-.0025;m.SetFSC(plane,m.MPC_HORIZON-1);
 if(!m.Run() || m.corridor_rows_after!=1)return 2;
 Eigen::VectorXd x=m.X_0_;Eigen::MatrixXd A,B; m.SystemModel(A,B,m.MPC_STEP);
 for(int i=0;i<m.MPC_HORIZON;++i){Eigen::Vector3d u;m.GetOptimCmd(u,i);x=A*x+B*u;}
 if(x[0]>.0036)return 3;
 return 0;
}
