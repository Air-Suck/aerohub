// Check the actual Behavior interpolator against a seventh-order source curve.
#include "uav_follow_path_execution/timeline.hpp"
#include <cassert>
#include <iostream>
int main() {
  namespace uf=uav_follow_path_execution;
  uf::Path path; path.start=0; path.fields=3;
  for (int i=0;i<=50;++i) {
    double t=i*.02;
    path.points.push_back({t,{std::pow(t,7),0,0},{7*std::pow(t,6),0,0},{42*std::pow(t,5),0,0},0});
  }
  double ep=0,ev=0,ea=0;
  for (int i=0;i<=1000;++i) {
    double t=i*.001;auto s=uf::sample(path,t);
    ep=std::max(ep,std::abs(s.p[0]-std::pow(t,7)));
    ev=std::max(ev,std::abs(s.v[0]-7*std::pow(t,6)));
    ea=std::max(ea,std::abs(s.a[0]-42*std::pow(t,5)));
  }
  assert(ep<1e-8 && ev<1e-6 && ea<1e-3);
  std::cout<<"seventh-order vs Behavior quintic max P/V/A error: "<<ep<<" "<<ev<<" "<<ea<<"\n";
}
