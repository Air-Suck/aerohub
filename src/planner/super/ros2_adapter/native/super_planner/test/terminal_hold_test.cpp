#include "fsm/terminal_hold.hpp"
#include <cassert>
#include <limits>

int main() {
    using fsm::terminalGoalHoldEligible;
    // A valid terminal target remains available while measured tracking error
    // settles; measured vehicle position is deliberately not this predicate.
    assert(terminalGoalHoldEligible(true, true, 1e-6, 1e-7, 1e-7));
    assert(!terminalGoalHoldEligible(false, true, 0, 0, 0));
    assert(!terminalGoalHoldEligible(true, false, 0, 0, 0));
    assert(!terminalGoalHoldEligible(true, true, 2.0, 0, 0));
    assert(!terminalGoalHoldEligible(true, true, 0.1, 0, 0));
    assert(!terminalGoalHoldEligible(true, true, 0, 0.3, 0));
    assert(!terminalGoalHoldEligible(true, true, 0, 0, 0.1));
    assert(fsm::stationaryStopEligible(true, 0, 0));
    assert(!fsm::stationaryStopEligible(false, 0, 0));
    assert(!fsm::stationaryStopEligible(true, 0.3, 0));
    assert(!fsm::stationaryStopEligible(true, 0, 0.1));
    assert(fsm::stopHoldActive(true, 9.99, 10));
    assert(!fsm::stopHoldActive(true, 10, 10));
    assert(!fsm::stopHoldActive(false, 1, 10));
    assert(!fsm::stopHoldActive(true, -1, 10));
    assert(!fsm::stopHoldActive(true, 0, 0));
    const auto nan = std::numeric_limits<double>::quiet_NaN();
    assert(!fsm::stationaryStopEligible(true, nan, 0));
    assert(!fsm::stopHoldActive(true, nan, 10));
    assert(!terminalGoalHoldEligible(true, true, nan, 0, 0));
    assert(!terminalGoalHoldEligible(true, true, 0, nan, 0));
    assert(!terminalGoalHoldEligible(true, true, 0, 0, nan));
}
