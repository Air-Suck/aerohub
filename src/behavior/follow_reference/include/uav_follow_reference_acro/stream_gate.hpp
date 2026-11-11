#pragma once
#include <cmath>
#include <cstdint>
#include <string>
#include <utility>

namespace uav_follow_reference_acro {
// ROS-independent acceptance rules; arrival of invalid/old frames never extends a lease.
class StreamGate {
public:
  void start(std::string session, std::uint64_t revision) {
    session_ = std::move(session); revision_ = revision; sequence_ = 0;
    active_ = true; received_ = false;
  }
  void stop() {active_ = false; received_ = false;}
  bool accept(const std::string & session, std::uint64_t revision, std::uint64_t sequence,
      double stamp, double solve, double now, double timeout, double solve_timeout) {
    if (!active_ || session != session_ || revision != revision_ || sequence == 0 ||
        (received_ && (sequence <= sequence_ || stamp < stamp_)) ||
        !fresh(stamp, now, timeout) || !fresh(solve, now, solve_timeout) || solve > stamp) {
      return false;
    }
    sequence_ = sequence; stamp_ = stamp; solve_ = solve; received_ = true;
    return true;
  }
  bool valid(double now, double timeout, double solve_timeout) const {
    return active_ && received_ && fresh(stamp_, now, timeout) &&
      fresh(solve_, now, solve_timeout);
  }
private:
  static bool fresh(double stamp, double now, double timeout) {
    return std::isfinite(stamp) && std::isfinite(now) && stamp > 0 &&
      now >= stamp && now - stamp <= timeout;
  }
  std::string session_;
  std::uint64_t revision_{0}, sequence_{0};
  double stamp_{0}, solve_{0};
  bool active_{false}, received_{false};
};
}
