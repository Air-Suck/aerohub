#include "engine.hpp"
#include <iostream>

// A wall spans the entire local X range. At the unchanged target height a
// planar route is impossible; a valid path must cross above/below the wall.
int main() {
  for (bool over : {true, false}) {
    ipc_native::Parameters parameters;
    Engine engine(parameters);
    engine.obstacle_clearance_ = .75;
    engine.path_check_clearance_ = engine.sfc_dis_ = .55;
    engine.odom_p_ = {0, -2, 3};
    const double low = over ? 0. : 2.;
    const double high = over ? 4. : 8.;
    for (int x = -50; x <= 50; ++x)
      for (double z = low; z <= high + 1e-6; z += .2)
        engine.local_pc_.emplace_back(.2*x, 0., z);
    engine.plan({0, 2, 3});
    double extreme = 3;
    for (const auto& point : engine.follow_path_) {
      extreme = over ? std::max(extreme, point.z()) : std::min(extreme, point.z());
      if (!point.allFinite()) return 1;
      for (const auto& obstacle : engine.local_pc_)
        if ((point-obstacle).norm() < engine.sfc_dis_ - 1e-6) return 2;
    }
    if ((engine.follow_path_.front()-engine.odom_p_).norm() > 1e-9 ||
        (engine.follow_path_.back()-Eigen::Vector3d(0, 2, 3)).norm() > 1e-9) return 3;
    if (over ? extreme < 4.7 : extreme > 1.3) return 4;
    std::cout << (over ? "over" : "under") << " wall extreme_z=" << extreme << '\n';
    engine.planar_ = true;
    try {
      engine.plan({0, 2, 3});
      std::cerr << "planar route unexpectedly crossed a full-width wall\n";
      return 5;
    } catch (const std::runtime_error&) {
      // The same fixture must reject planar planning, proving that changing
      // height is necessary rather than incidental in the successful route.
    }
  }
}
