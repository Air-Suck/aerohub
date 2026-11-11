#include <cassert>
#include "uav_follow_reference_acro/stream_gate.hpp"
int main() {
  using uav_follow_reference_acro::StreamGate;
  StreamGate gate;
  gate.start("one", 1);
  assert(!gate.accept("old", 1, 1, 10, 10, 10, .1, .1));
  assert(gate.accept("one", 1, 1, 10, 10, 10, .1, .1));
  assert(!gate.accept("one", 1, 1, 10.01, 10.01, 10.01, .1, .1));
  assert(!gate.accept("one", 1, 2, 9.99, 9.99, 10.01, .1, .1));
  assert(!gate.accept("one", 1, 2, 10.1, 10.1, 10.01, .1, .1));
  assert(!gate.valid(10.2, .1, .1));
  assert(!gate.valid(9.9, .1, .1));
  gate.start("one", 2);
  assert(!gate.accept("one", 1, 2, 10.2, 10.2, 10.2, .1, .1));
  assert(!gate.accept("one", 2, 1, 10.2, 10, 10.2, .1, .1));
  assert(gate.accept("one", 2, 1, 10.2, 10.2, 10.2, .1, .1));
  gate.stop();
  assert(!gate.valid(10.2, .1, .1));
  assert(!gate.accept("one", 2, 2, 10.21, 10.21, 10.21, .1, .1));
}
