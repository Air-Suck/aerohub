#pragma once

#include <cmath>

namespace fsm {
// Goal completion and a stationary intermediate stop are separate decisions.
inline bool stationaryStopEligible(bool finished, double speed, double acceleration) {
    return finished && std::isfinite(speed) && std::isfinite(acceleration)
           && speed >= 0.0 && speed < 1e-3
           && acceleration >= 0.0 && acceleration < 1e-3;
}
inline bool stopHoldActive(bool planning, double elapsed, double timeout) {
    return planning && std::isfinite(elapsed) && std::isfinite(timeout)
           && elapsed >= 0.0 && timeout > 0.0 && elapsed < timeout;
}
inline bool terminalGoalHoldEligible(bool finished, bool main_trajectory,
                                     double endpoint_error, double speed,
                                     double acceleration) {
    return finished && main_trajectory && std::isfinite(endpoint_error)
           && std::isfinite(speed) && std::isfinite(acceleration)
           && endpoint_error >= 0.0 && endpoint_error < 0.1
           && speed >= 0.0 && speed < 1e-3
           && acceleration >= 0.0 && acceleration < 1e-3;
}
}
