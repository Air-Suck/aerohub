#include <uav_gcopter/core.hpp>
#include <gcopter/geo_utils.hpp>
#include <gcopter/gcopter.hpp>
#include <ompl/util/RandomNumbers.h>
#include <iostream>
#include <limits>
#include <stdexcept>
void check(bool condition, const char *message) {
  if (!condition) throw std::runtime_error(message);
}
int main() {
  ompl::RNG::setSeed(42);
  uav_gcopter::Config c;
  Eigen::Matrix3d start = Eigen::Matrix3d::Zero(), goal = start;
  start.col(0) = Eigen::Vector3d(-3, 0, 2);
  goal.col(0) = Eigen::Vector3d(3, 0, 2);
  Eigen::MatrixX4d box(6, 4);
  box << 1,0,0,-8, -1,0,0,-8, 0,1,0,-4, 0,-1,0,-4, 0,0,1,-4, 0,0,-1,0;
  auto result = uav_gcopter::optimize(c, start, goal, {box});
  check(static_cast<bool>(result), result.error.c_str());
  const auto &t = result.trajectory;
  const double duration = t.getTotalDuration();
  // Independent invocation of the locked upstream entry with the identical corridor.
  gcopter::GCOPTER_PolytopeSFC upstream;
  check(upstream.setup(c.weight_time, start, goal, {box}, INFINITY, c.smoothing,
    c.quadrature, c.bounds, c.weights, c.physics), "upstream setup");
  Trajectory<5> baseline;
  check(std::isfinite(upstream.optimize(baseline, c.tolerance)), "upstream optimize");
  check(std::abs(baseline.getTotalDuration()-duration) < 1e-8, "duration parity");
  check(std::isfinite(duration) && duration > 0, "duration");
  check((t.getPos(0)-start.col(0)).norm() < 1e-5, "start");
  check((t.getPos(duration)-goal.col(0)).norm() < 1e-5, "goal");
  for (double stamp = 0; stamp < duration; stamp += 0.001) {
    check((t.getPos(stamp)-baseline.getPos(stamp)).norm() < 1e-8 &&
          (t.getVel(stamp)-baseline.getVel(stamp)).norm() < 1e-8 &&
          (t.getAcc(stamp)-baseline.getAcc(stamp)).norm() < 1e-8, "upstream PVA parity");
    check(t.getPos(stamp).allFinite() && t.getVel(stamp).allFinite() && t.getAcc(stamp).allFinite(), "finite PVA");
    check(t.getVel(stamp).norm() <= c.bounds[0] * 1.01, "speed bound");
    check((box.leftCols<3>()*t.getPos(stamp)+box.col(3)).maxCoeff() <= 1e-5, "corridor");
  }
  // Local frontend boundary: real moving PVA must survive FIRI+GCOPTER and
  // meet the exact boundary needed by a rolling FollowTimedPath replacement.
  auto moving = start;
  moving(0,1)=.7;moving(0,2)=.1;
  auto local=uav_gcopter::optimizeRoute(c,moving,goal,{moving.col(0),goal.col(0)}, {}, .1);
  check(static_cast<bool>(local),local.error.c_str());
  check((local.trajectory.getPos(0)-moving.col(0)).norm()<1e-5,"local start P");
  check((local.trajectory.getVel(0)-moving.col(1)).norm()<1e-5,"local start V");
  check((local.trajectory.getAcc(0)-moving.col(2)).norm()<1e-5,"local start A");
  check((local.trajectory.getPos(local.trajectory.getTotalDuration())-goal.col(0)).norm()<1e-5,"local end P");
  check(!uav_gcopter::plan(c, start, goal, {start.col(0)}), "occupied endpoint rejected");
  auto invalid = goal;
  invalid(0,0) = std::numeric_limits<double>::quiet_NaN();
  check(!uav_gcopter::plan(c, start, invalid, {}), "NaN rejected");
  invalid = goal; invalid(0,0) = 100;
  check(!uav_gcopter::plan(c, start, invalid, {}), "outside rejected");
  Eigen::MatrixX4d left = box, right = box;
  left(0,3) = 4; right(1,3) = 4;
  gcopter::GCOPTER_PolytopeSFC failed_upstream;
  check(!failed_upstream.setup(c.weight_time, start, goal, {left, right}, INFINITY,
    c.smoothing, c.quadrature, c.bounds, c.weights, c.physics), "upstream disjoint failure");
  auto failed_wrapper = uav_gcopter::optimize(c, start, goal, {left, right});
  check(!failed_wrapper && failed_wrapper.error == "setup_failed", "disjoint failure parity");
  auto full = uav_gcopter::plan(c, start, goal, {Eigen::Vector3d(0, 0, 2)});
  check(static_cast<bool>(full), full.error.c_str());
  std::vector<Eigen::Vector3d> wall;
  c.low = Eigen::Vector3d(-5, -3, 0); c.high = Eigen::Vector3d(5, 3, 5);
  for (double y = -3; y < 3; y += .125)
    for (double z = 0; z < 5; z += .125) wall.emplace_back(0, y, z);
  auto blocked = uav_gcopter::plan(c, start, goal, wall);
  check(!blocked && blocked.error == "no_exact_route", "no solution wall rejected");
  // The same thin wall in a long map must not disappear between OMPL checks.
  c.low.x() = -125; c.high.x() = 125;
  start(0,0) = -100; goal(0,0) = 100;
  auto long_blocked = uav_gcopter::plan(c, start, goal, wall);
  check(!long_blocked && long_blocked.error == "no_exact_route", "long map thin wall rejected");
  std::cout << "core checks passed local_moving_boundary=true duration=" << duration << " pieces=" << t.getPieceNum() << '\n';
}
