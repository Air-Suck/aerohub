#include <optimizer/trajectory_feasibility.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>
void check(bool condition) { static int index=0; ++index; if (!condition) throw std::runtime_error("feasibility gate regression case "+std::to_string(index)); }
int main() {
  using poly_traj::CoefficientMat; using poly_traj::Trajectory;
  const double v=.23, a=.35, tolerance=.05;
  CoefficientMat c=CoefficientMat::Zero();
  check(ego_planner::feasibleTrajectory(Trajectory({1},{c}),v,a,tolerance));
  c(0,4)=2;
  check(!ego_planner::feasibleTrajectory(Trajectory({1},{c}),v,a,tolerance));
  c.setZero(); c(0,3)=1;
  check(!ego_planner::feasibleTrajectory(Trajectory({1},{c}),v,a,tolerance));
  c.setZero(); c(0,0)=std::numeric_limits<double>::quiet_NaN();
  check(!ego_planner::feasibleTrajectory(Trajectory({1},{c}),v,a,tolerance));
  c.setZero();
  check(!ego_planner::feasibleTrajectory(Trajectory({0},{c}),v,a,tolerance));
  check(!ego_planner::feasibleTrajectory(Trajectory({1},{c}),-v,a,tolerance));
  // Endpoints have zero velocity, but the continuous interior peak exceeds v.
  Eigen::Matrix3d head=Eigen::Matrix3d::Zero(), tail=head; tail(0,0)=1;
  poly_traj::MinJerkOpt opt; opt.reset(head,tail,1);
  opt.generate(Eigen::MatrixXd(3,0),Eigen::VectorXd::Constant(1,5));
  check(!ego_planner::feasibleTrajectory(opt.getTraj(),v,a,tolerance));
  opt.generate(Eigen::MatrixXd(3,0),Eigen::VectorXd::Constant(1,10));
  check(ego_planner::feasibleTrajectory(opt.getTraj(),v,a,tolerance));
  // Float32 rounding must not turn an accepted double trajectory infeasible.
  c.setZero(); c(0,4)=v*(1+tolerance)-1e-10;
  check(c(0,4) < v*(1+tolerance));
  check(!ego_planner::feasibleTrajectory(Trajectory({1},{c}),v,a,tolerance));
  std::cout << "PASS: interior extrema, nonfinite/invalid data, hard limits, float32 boundary, feasible recovery\n";
}
