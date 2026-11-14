#include <iomanip>
#include <iostream>
#include "uav_follow_path_execution/timeline.hpp"
int main() {
  namespace uf=uav_follow_path_execution;
  uf::Path path;
  path.start=10.; path.fields=7;
  size_t count, queries;
  std::cin >> count >> queries;
  for (size_t i=0;i<count;++i) {
    uf::Point p;
    std::cin >> p.t;
    for (auto& x:p.p) std::cin >> x;
    for (auto& x:p.v) std::cin >> x;
    for (auto& x:p.a) std::cin >> x;
    std::cin >> p.yaw;
    path.points.push_back(p);
  }
  std::cout << std::setprecision(17);
  for (size_t i=0;i<queries;++i) {
    double t; std::cin >> t;
    auto p=uf::sample(path,10.+t);
    for (auto x:p.p) std::cout << x << ' ';
    for (auto x:p.v) std::cout << x << ' ';
    for (auto x:p.a) std::cout << x << ' ';
    std::cout << p.yaw << '\n';
  }
}
