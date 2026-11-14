#pragma once
#include <cstdlib>
#include <limits>
#include <algorithm>
namespace stage3_height {
// These are launch-time settings; changing them requires restarting the run.
// Avoid scanning the process environment for every A* neighbor/map query.
inline double low() {
  static const double value=[] { const char *p=std::getenv("SUPER_CENTER_Z_MIN"); return p ? std::atof(p) : -std::numeric_limits<double>::infinity(); }();
  return value;
}
inline double high() {
  static const double value=[] { const char *p=std::getenv("SUPER_CENTER_Z_MAX"); return p ? std::atof(p) : std::numeric_limits<double>::infinity(); }();
  return value;
}
inline bool allowed(double z) { return z >= low() && z <= high(); }
inline double cell_height(double z, double resolution) {
  if (z < low() && z + resolution*.5 > low()) return std::min(low()+.01,z+resolution*.5-.001);
  if (z > high() && z - resolution*.5 < high()) return std::max(high()-.01,z-resolution*.5+.001);
  return z;
}
template<class V> void clip(V &lo, V &hi) { lo.z()=std::max(lo.z(),low()); hi.z()=std::min(hi.z(),high()); }
}
