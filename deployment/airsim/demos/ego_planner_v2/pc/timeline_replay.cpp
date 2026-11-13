#include <iomanip>
#include <iostream>
#include "uav_follow_path_execution/timeline.hpp"
int main() {
  namespace uf = uav_follow_path_execution;
  uf::Path path;
  size_t count, queries;
  std::cin >> path.start >> count;
  path.fields = 3;
  for (size_t i=0;i<count;++i) {
    uf::Point p;
    std::cin >> p.t;
    for (auto* v : {&p.p,&p.v,&p.a}) for (double& x:*v) std::cin >> x;
    path.points.push_back(p);
  }
  std::cin >> queries;
  std::cout << std::setprecision(17);
  for(size_t i=0;i<queries;++i) {
    double t; std::cin >> t;
    auto p=uf::sample(path,t);
    for (const auto* v : {&p.p,&p.v,&p.a}) for(double x:*v) std::cout << x << ' ';
    std::cout << '\n';
  }
  return std::cin.fail() ? 1 : 0;
}
