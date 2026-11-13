#include <optimizer/poly_traj_utils.hpp>
#include <iomanip>
#include <iostream>

int main() {
  std::cout << std::setprecision(17);
  for (int pieces = 1; pieces <= 4; ++pieces) {
    Eigen::Matrix3d start = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d finish = Eigen::Matrix3d::Zero();
    start.col(0) << -0.5, 1.0, 2.0;
    start.col(1) << 0.1, -0.2, 0.05;
    finish.col(0) << 3.0, -1.5, 2.5;
    Eigen::MatrixXd inner(3, pieces - 1);
    Eigen::VectorXd durations(pieces);
    for (int i = 0; i < pieces; ++i) {
      durations(i) = 1.1 + 0.3 * i;
      if (i < pieces - 1) {
        const double fraction = static_cast<double>(i + 1) / pieces;
        inner.col(i) = (1.0 - fraction) * start.col(0) + fraction * finish.col(0);
        inner(1, i) += 0.2;
      }
    }
    poly_traj::MinJerkOpt optimizer;
    optimizer.reset(start, finish, pieces);
    optimizer.generate(inner, durations);
    const auto trajectory = optimizer.getTraj();
    const double total = durations.sum();
    for (int sample = 0; sample <= 50; ++sample) {
      const double time = total * sample / 50.0;
      const auto position = trajectory.getPos(time);
      const auto velocity = trajectory.getVel(time);
      const auto acceleration = trajectory.getAcc(time);
      std::cout << pieces << ' ' << time;
      for (int axis = 0; axis < 3; ++axis) {
        std::cout << ' ' << position(axis) << ' ' << velocity(axis) << ' ' << acceleration(axis);
      }
      std::cout << '\n';
    }
  }
}
