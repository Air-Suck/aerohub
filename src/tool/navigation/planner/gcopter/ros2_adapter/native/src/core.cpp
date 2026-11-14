#include <uav_gcopter/core.hpp>
#include <gcopter/geo_utils.hpp>
#include <gcopter/gcopter.hpp>
#include <gcopter/sfc_gen.hpp>
#include <gcopter/voxel_map.hpp>
#include <cmath>

namespace uav_gcopter {
namespace {
bool valid(const Config &c, const Eigen::Matrix3d &s, const Eigen::Matrix3d &g) {
  return s.allFinite() && g.allFinite() && c.low.allFinite() && c.high.allFinite() &&
    (c.high.array() > c.low.array()).all() && std::isfinite(c.resolution) && c.resolution > 0 &&
    std::isfinite(c.inflation) && c.inflation > 0 && c.inflation / c.resolution <= 100 &&
    std::isfinite(c.rrt_timeout) && c.rrt_timeout > 0 && c.rrt_timeout <= 30 &&
    c.bounds.allFinite() && c.weights.allFinite() && c.physics.allFinite() &&
    (c.bounds.array() > 0).all() && (c.weights.array() > 0).all() && c.bounds[4] > c.bounds[3] &&
    c.physics[0] > 0 && c.physics[1] > 0 && c.physics[5] > 0 &&
    (c.physics.segment<3>(2).array() >= 0).all() &&
    std::isfinite(c.weight_time) && c.weight_time > 0 && std::isfinite(c.smoothing) && c.smoothing > 0 &&
    std::isfinite(c.tolerance) && c.tolerance > 0 && c.quadrature > 0 && c.quadrature <= 256;
}
}
Result optimize(const Config &c, const Eigen::Matrix3d &s, const Eigen::Matrix3d &g,
                const std::vector<Eigen::MatrixX4d> &corridor) {
  Result r;
  if (!valid(c, s, g) || corridor.empty()) { r.error = "invalid_input"; return r; }
  for (const auto &poly : corridor) {
    if (poly.rows() < 4 || !poly.allFinite() ||
        (poly.leftCols<3>().rowwise().norm().array() < 1e-12).any()) {
      r.error = "invalid_corridor"; return r;
    }
  }
  gcopter::GCOPTER_PolytopeSFC solver;
  if (!solver.setup(c.weight_time, s, g, corridor, INFINITY, c.smoothing,
                    c.quadrature, c.bounds, c.weights, c.physics)) {
    r.error = "setup_failed"; return r;
  }
  if (!std::isfinite(solver.optimize(r.trajectory, c.tolerance)) || r.trajectory.getPieceNum() == 0) {
    r.trajectory.clear(); r.error = "optimization_failed";
  }
  return r;
}
Result optimizeRoute(const Config &c, const Eigen::Matrix3d &s, const Eigen::Matrix3d &g,
                     const std::vector<Eigen::Vector3d> &route,
                     const std::vector<Eigen::Vector3d> &surface, double margin) {
  Result r;
  if(!valid(c,s,g) || route.size()<2 || !std::isfinite(margin) || margin<0 ||
     (route.front()-s.col(0)).norm()>1e-6 || (route.back()-g.col(0)).norm()>1e-6) {
    r.error="invalid_local_route";return r;
  }
  for(const auto &p:route) if(!p.allFinite()) {r.error="invalid_local_route";return r;}
  for(const auto &p:surface) if(!p.allFinite()) {r.error="invalid_local_surface";return r;}
  std::vector<Eigen::MatrixX4d> corridor;
  sfc_gen::convexCover(route,surface,c.low,c.high,7,3,corridor);
  for(auto &poly:corridor)
    poly.col(3).array()+=margin*poly.leftCols<3>().rowwise().norm().array();
  for(size_t i=0;i<corridor.size();++i) {
    Eigen::Matrix3Xd vertices;
    if(!geo_utils::enumerateVs(corridor[i],vertices) ||
       (i && !geo_utils::overlap(corridor[i-1],corridor[i],1e-6))) {
      r.error="local_corridor_disconnected";return r;
    }
  }
  sfc_gen::shortCut(corridor);
  r=optimize(c,s,g,corridor);r.route=route;return r;
}
Result plan(const Config &c, const Eigen::Matrix3d &s, const Eigen::Matrix3d &g,
            const std::vector<Eigen::Vector3d> &cloud) {
  Result r;
  if (!valid(c, s, g)) { r.error = "invalid_input"; return r; }
  const Eigen::Array3d sizes = ((c.high - c.low) / c.resolution).array().floor();
  if ((sizes < 1).any() || (sizes > 1000000).any() || sizes.prod() > 1e8) {
    r.error = "map_size_limit"; return r;
  }
  voxel_map::VoxelMap map(sizes.matrix().cast<int>(), c.low, c.resolution);
  const auto inside = [&](const Eigen::Vector3d &p) {
    return (p.array() >= c.low.array()).all() && (p.array() < map.getCorner().array()).all();
  };
  for (const auto &p : cloud) {
    if (!p.allFinite()) { r.error = "invalid_cloud"; return r; }
    if (inside(p)) map.setOccupied(p);
  }
  // Search seeds need clearance for the later voxel-volume corridor shrink.
  // Keep the declared collision map unchanged for FIRI and the final check.
  auto search_map = map;
  map.dilate(static_cast<int>(std::ceil(c.inflation / c.resolution)));
  search_map.dilate(static_cast<int>(std::ceil(c.inflation / c.resolution)) + 1);
  if (!inside(s.col(0)) || !inside(g.col(0)) || map.query(Eigen::Vector3d(s.col(0))) || map.query(Eigen::Vector3d(g.col(0)))) {
    r.error = "invalid_endpoint"; return r;
  }
  if ((s.col(0) - g.col(0)).norm() < 1e-6) { r.error = "coincident_endpoints"; return r; }
  if (search_map.query(Eigen::Vector3d(s.col(0))) || search_map.query(Eigen::Vector3d(g.col(0)))) {
    r.error = "insufficient_search_clearance"; return r;
  }
  sfc_gen::planPath(s.col(0), g.col(0), map.getOrigin(), map.getCorner(), &search_map,
                    c.rrt_timeout, r.route);
  if (r.route.size() < 2 || (r.route.back() - g.col(0)).norm() > 1e-6) {
    r.error = "no_exact_route"; return r;
  }
  std::vector<Eigen::Vector3d> surface;
  map.getSurf(surface);
  std::vector<Eigen::MatrixX4d> corridor;
  sfc_gen::convexCover(r.route, surface, map.getOrigin(), map.getCorner(), 7, 3, corridor);
  // FIRI sees voxel centers; reserve a voxel half-diagonal before optimizing.
  // The same inflated occupancy is still checked at 1ms after solving.
  for (auto &poly : corridor)
    poly.col(3).array() += 0.5 * std::sqrt(3.0) * c.resolution *
                          poly.leftCols<3>().rowwise().norm().array();
  // Distinguish geometric failures before the optimizer's generic setup error.
  for (size_t i = 0; i < corridor.size(); ++i) {
    Eigen::Matrix3Xd vertices;
    if (!geo_utils::enumerateVs(corridor[i], vertices)) {
      r.error = "empty_safe_corridor:" + std::to_string(i); return r;
    }
    if (i > 0 && !geo_utils::overlap(corridor[i-1], corridor[i], 1e-6)) {
      r.error = "disconnected_safe_corridor:" + std::to_string(i); return r;
    }
  }
  sfc_gen::shortCut(corridor);
  auto optimized = optimize(c, s, g, corridor);
  optimized.route = std::move(r.route);
  if (optimized) {
    const double duration = optimized.trajectory.getTotalDuration();
    if (!std::isfinite(duration) || duration <= 0 || duration > 600) {
      optimized.trajectory.clear(); optimized.error = "duration_limit"; return optimized;
    }
    for (double t = 0; t <= duration; t = std::min(t + 0.001, duration)) {
      const Eigen::Vector3d p = optimized.trajectory.getPos(t);
      const Eigen::Vector3d v = optimized.trajectory.getVel(t);
      if (!p.allFinite() || !v.allFinite() || !inside(p) || map.query(p) ||
          v.norm() > c.bounds[0] * 1.01) {
        optimized.error = !p.allFinite() || !v.allFinite() ? "trajectory_nonfinite" :
          !inside(p) ? "trajectory_outside" : map.query(p) ? "trajectory_collision" : "trajectory_speed";
        optimized.trajectory.clear(); break;
      }
      if (t == duration) break;
    }
  }
  return optimized;
}
}
