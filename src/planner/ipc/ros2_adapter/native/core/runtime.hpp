#pragma once
#include <chrono>
#include <cstdio>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>
#include <algorithm>
#define IPC_LOG(...) do { std::fprintf(stderr, __VA_ARGS__); std::fputc('\n', stderr); } while (false)
namespace ipc_native {
// Monotonic profiling only. Control timestamps belong to the ROS node clock.
inline double now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
struct Parameters {
  std::map<std::string, double> values;
  template<class T> void param(const std::string& key, T& value, T fallback) const {
    auto it = values.find(key); value = it == values.end() ? fallback : static_cast<T>(it->second);
  }
};
}
