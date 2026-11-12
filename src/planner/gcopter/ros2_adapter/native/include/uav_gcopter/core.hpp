#pragma once
#include <gcopter/trajectory.hpp>
#include <string>

namespace uav_gcopter {
struct Config {
  Eigen::Vector3d low{-25, -25, 0}, high{25, 25, 5};
  double resolution = 0.25, inflation = 0.5, rrt_timeout = 0.02;
  double weight_time = 20, smoothing = 0.01, tolerance = 1e-5;
  int quadrature = 16;
  Eigen::Matrix<double, 5, 1> bounds{4, 2.1, 1.05, 2, 12};
  Eigen::Matrix<double, 5, 1> weights{1e4, 1e4, 1e4, 1e4, 1e5};
  Eigen::Matrix<double, 6, 1> physics{0.61, 9.8, 0.7, 0.8, 0.01, 1e-4};
};
struct Result {
  Trajectory<5> trajectory;
  std::vector<Eigen::Vector3d> route;
  std::string error;
  explicit operator bool() const { return error.empty() && trajectory.getPieceNum() > 0; }
};
Result optimize(const Config &, const Eigen::Matrix3d &, const Eigen::Matrix3d &,
                const std::vector<Eigen::MatrixX4d> &);
// Caller supplies a sensor-valid route and preinflated obstacle surface, then
// independently validates the optimized curve against its current local map.
Result optimizeRoute(const Config &, const Eigen::Matrix3d &, const Eigen::Matrix3d &,
                     const std::vector<Eigen::Vector3d> &, const std::vector<Eigen::Vector3d> &, double);
Result plan(const Config &, const Eigen::Matrix3d &, const Eigen::Matrix3d &,
            const std::vector<Eigen::Vector3d> &);
}
