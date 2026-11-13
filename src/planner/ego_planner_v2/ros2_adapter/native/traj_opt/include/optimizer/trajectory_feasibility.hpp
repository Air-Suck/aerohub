#pragma once
#include <optimizer/poly_traj_utils.hpp>
#include <cmath>

namespace ego_planner {
// Check the entire polynomial, including the float32 coefficients/durations
// used on the wire. A sampled penalty is not a hard feasibility guarantee.
inline bool feasibleTrajectory(const poly_traj::Trajectory &traj,
                               double max_velocity, double max_acceleration,
                               double tolerance) {
  if (traj.getPieceNum() == 0 || !std::isfinite(max_velocity) ||
      !std::isfinite(max_acceleration) || !std::isfinite(tolerance) ||
      max_velocity <= 0 || max_acceleration <= 0 || tolerance < 0)
    return false;
  std::vector<double> durations;
  std::vector<poly_traj::CoefficientMat> coefficients;
  for (int i = 0; i < traj.getPieceNum(); ++i) {
    const auto &piece = traj.getPiece(i);
    double duration = static_cast<float>(piece.getDuration());
    auto coefficients32 = piece.getCoeffMat().cast<float>().cast<double>().eval();
    if (!std::isfinite(duration) || duration <= 0 || !coefficients32.allFinite())
      return false;
    durations.push_back(duration);
    coefficients.push_back(coefficients32);
  }
  const poly_traj::Trajectory wire(durations, coefficients);
  // The upstream extrema-reporting helper returns zero on constant-rate
  // pieces. Its root-count constraint checker also tests both endpoints.
  for (int i = 0; i < traj.getPieceNum(); ++i) {
    for (const auto *candidate : {&traj, &wire}) {
      const auto &piece = candidate->getPiece(i);
      if (!piece.checkMaxVelRate(max_velocity*(1+tolerance)) ||
          !piece.checkMaxAccRate(max_acceleration*(1+tolerance))) return false;
    }
  }
  return true;
}
}
