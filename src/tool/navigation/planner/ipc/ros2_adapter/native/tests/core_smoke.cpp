#include "mpc.h"
#include "polytope/emvp.hpp"
int main() {
 ipc_native::Parameters parameters;
 MPCPlannerClass mpc(parameters);
 mpc.SetStatus(Eigen::Vector3d(0,0,1), Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
 for(int i=0;i<mpc.MPC_HORIZON;++i) mpc.SetGoal(Eigen::Vector3d(0,0,1),Eigen::Vector3d::Zero(),Eigen::Vector3d::Zero(),i);
 if(!mpc.Run()) return 1;
 Eigen::Vector3d u; mpc.GetOptimCmd(u,0);
 if(!u.allFinite() || u.norm()>1e-3) return 2;
 return 0;
}
