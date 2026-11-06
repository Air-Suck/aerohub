// External timed-path execution math. No ROS dependency; coordinates are ENU.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <deque>
#include <stdexcept>
#include <string>
#include <vector>

namespace uav_follow_path_execution {
using Vec = std::array<double, 3>;
struct Point {
  double t{};
  Vec p{}, v{}, a{};
  double yaw{};
};
struct Path {
  std::string task, vehicle, frame;
  uint64_t revision{};
  double generated{}, start{}, max_speed{};
  bool final{};
  uint8_t fields{};
  std::vector<Point> points;
  double end() const {return start + points.back().t;}
};
inline double distance(const Vec & a, const Vec & b) {
  double sum = 0;
  for (size_t i = 0; i < 3; ++i) {sum += (a[i]-b[i])*(a[i]-b[i]);}
  return std::sqrt(sum);
}
inline bool finite(const Vec & v) {
  return std::all_of(v.begin(), v.end(), [](double x) {return std::isfinite(x);});
}
inline void validate(const Path & p, double now) {
  if (p.task.empty() || p.vehicle.empty() || p.frame.empty() || p.revision == 0 ||
    !std::isfinite(now) || !std::isfinite(p.start) || !std::isfinite(p.generated) ||
    p.start <= now || p.generated > now || p.generated > p.start ||
    !std::isfinite(p.max_speed) || p.max_speed <= 0 || p.points.size() < 2 ||
    p.fields > 7 || ((p.fields & 2) && !(p.fields & 1)))
  {throw std::invalid_argument("invalid metadata or late trajectory");}
  double previous = -1;
  for (const auto & point : p.points) {
    if (!std::isfinite(point.t) || point.t < 0 || point.t <= previous ||
      !finite(point.p) || ((p.fields & 1) && !finite(point.v)) ||
      ((p.fields & 2) && !finite(point.a)) ||
      ((p.fields & 4) && !std::isfinite(point.yaw)))
    {throw std::invalid_argument("invalid trajectory point");}
    previous = point.t;
  }
  if (p.points.front().t != 0) {throw std::invalid_argument("first time must be zero");}
  if (!std::isfinite(p.end())) {throw std::invalid_argument("non-finite coverage end");}
}

// Linear position-only; cubic Hermite with velocity; quintic with velocity and
// acceleration. Derivatives come from the SAME curve, not independent lerps.
inline Point sample(const Path & path, double absolute_time) {
  const double t = std::clamp(absolute_time-path.start, 0.0, path.points.back().t);
  auto upper = std::upper_bound(path.points.begin(), path.points.end(), t,
    [](double value, const Point & p) {return value < p.t;});
  if (upper == path.points.end()) {upper = path.points.end()-1;}
  if (upper == path.points.begin()) {++upper;}
  const auto & l = *(upper-1);
  const auto & r = *upper;
  const double h = r.t-l.t;
  const double u = (t-l.t)/h;
  Point out;
  out.t = t;
  for (size_t i = 0; i < 3; ++i) {
    std::array<double, 6> c{l.p[i], r.p[i]-l.p[i], 0, 0, 0, 0};
    if (path.fields & 1) {
      c[1] = h*l.v[i];
      const double d = r.p[i]-l.p[i];
      c[2] = 3*d-h*(2*l.v[i]+r.v[i]);
      c[3] = -2*d+h*(l.v[i]+r.v[i]);
      if (path.fields & 2) {
        c[2] = h*h*l.a[i]/2;
        const double D = d-c[1]-c[2];
        const double V = h*r.v[i]-c[1]-2*c[2];
        const double A = h*h*r.a[i]-2*c[2];
        c[3] = 10*D-4*V+A/2;
        c[4] = -15*D+7*V-A;
        c[5] = 6*D-3*V+A/2;
      }
    }
    for (int k = 5; k >= 0; --k) {out.p[i] = out.p[i]*u+c[k];}
    for (int k = 5; k >= 1; --k) {out.v[i] = out.v[i]*u+k*c[k];}
    for (int k = 5; k >= 2; --k) {out.a[i] = out.a[i]*u+k*(k-1)*c[k];}
    out.v[i] /= h;
    out.a[i] /= h*h;
  }
  out.yaw = l.yaw + u*std::remainder(r.yaw-l.yaw, 2*std::acos(-1.0));
  return out;
}

class Timeline {
public:
  Path active;
  std::deque<Path> pending;
  bool loaded{false};
  void reset() {loaded = false; pending.clear();}
  void start(const Path & p, double now) {
    validate(p, now);
    active = p;
    loaded = true;
    pending.clear();
  }
  void advance(double now) {
    while (!pending.empty() && now >= pending.front().start) {
      active = pending.front();
      pending.pop_front();
    }
  }
  void update(const Path & p, double now, double position_tolerance,
    double velocity_tolerance, double acceleration_tolerance = 0.5,
    double yaw_tolerance = 0.15) {
    validate(p, now);
    advance(now);
    const auto & previous = pending.empty() ? active : pending.back();
    if (!loaded || pending.size() >= 32 || p.task != previous.task || p.vehicle != previous.vehicle ||
      p.frame != previous.frame || p.fields != previous.fields || p.revision <= previous.revision ||
      p.generated < previous.generated || p.start <= previous.start || p.start > previous.end())
    {throw std::invalid_argument("stale, excessive pending, or incompatible update");}
    const auto boundary = sample(previous, p.start);
    if (distance(boundary.p, p.points.front().p) > position_tolerance ||
      ((p.fields & 1) && distance(boundary.v, p.points.front().v) > velocity_tolerance) ||
      ((p.fields & 2) && distance(boundary.a, p.points.front().a) > acceleration_tolerance) ||
      ((p.fields & 4) && std::abs(std::remainder(
        boundary.yaw-p.points.front().yaw, 2*std::acos(-1.0))) > yaw_tolerance))
    {throw std::invalid_argument("discontinuous trajectory replacement: position=" +
      std::to_string(distance(boundary.p, p.points.front().p)) + " velocity=" +
      std::to_string(distance(boundary.v, p.points.front().v)) + " acceleration=" +
      std::to_string(distance(boundary.a, p.points.front().a)));}
    pending.push_back(p);
  }
};
}  // namespace uav_follow_path_execution
