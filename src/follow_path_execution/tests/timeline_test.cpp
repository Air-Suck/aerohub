#include "uav_follow_path_execution/timeline.hpp"
#include <cassert>
#include <iostream>
using namespace uav_follow_path_execution;
template<class F> void rejects(F f) {
  bool rejected = false;
  try {f();} catch (const std::invalid_argument &) {rejected = true;}
  assert(rejected);
}
int main() {
  Path p;
  p.task="task"; p.vehicle="drone0"; p.frame="drone0/odom";
  p.revision=1; p.generated=9; p.start=10; p.max_speed=2;
  p.points={{0,{0,0,0},{1,0,0},{0,0,0},3.0},
    {2,{2,0,0},{1,0,0},{0,0,0},-3.0}};
  for (auto fields : {0,1,3,7}) {
    p.fields=fields;
    validate(p,9.5);
    auto s=sample(p,11);
    assert(std::abs(s.p[0]-1)<1e-9);
    assert(std::abs(s.v[0]-1)<1e-9);
    assert(std::abs(s.a[0])<1e-9);
  }
  assert(std::abs(sample(p,11).yaw-std::acos(-1.0))<1e-9);
  rejects([&]{validate(p,10);});
  auto bad=p; bad.points[1].t=0;
  rejects([&]{validate(bad,9.5);});
  bad=p; bad.points[0].p[0]=NAN;
  rejects([&]{validate(bad,9.5);});
  Timeline timeline; timeline.start(p,9.5);
  auto next=p; next.revision=2; next.start=11; next.generated=10;
  next.points[0].p[0]=1; next.points[1].p[0]=3;
  timeline.update(next,10.2,0.01,0.01);
  timeline.advance(10.9); assert(timeline.active.revision==1);
  timeline.advance(11); assert(timeline.active.revision==2);
  rejects([&]{timeline.update(next,10.5,0.01,0.01);});
  bad=next; bad.revision=3; bad.start=12; bad.generated=11;
  bad.points[0].p[0]=100;
  rejects([&]{timeline.update(bad,11.5,0.01,0.01);});
  // More than one future revision can be acknowledged without replacing the
  // currently executing curve early.
  p.fields=0;
  timeline.start(p,9.5);
  next.fields=0;
  timeline.update(next,10.2,0.01,0.01);
  auto third=next; third.revision=3; third.start=11.5; third.generated=10.3;
  third.points[0].p[0]=1.5; third.points[1].p[0]=3.5;
  timeline.update(third,10.4,0.01,0.01);
  timeline.advance(11.2); assert(timeline.active.revision==2);
  timeline.advance(11.5); assert(timeline.active.revision==3);
  // Quintic must preserve position, velocity AND acceleration at both ends.
  p.fields=3; p.points[0].v={0,0,0}; p.points[1].v={0,0,0};
  p.points[0].a={0.3,0,0}; p.points[1].a={-0.4,0,0};
  const auto first=sample(p,10), last=sample(p,12);
  assert(std::abs(first.a[0]-0.3)<1e-9);
  assert(std::abs(last.a[0]+0.4)<1e-9);
  assert(std::abs(last.v[0])<1e-9);
  assert(std::abs(last.p[0]-2)<1e-9);
  std::cout << "timed path validation, interpolation and replacement passed\n";
}
