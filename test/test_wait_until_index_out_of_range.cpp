// An index that is not on the path is a programming mistake, like a checkpoint that can never be reached: the wait prints what is wrong,
// once, and returns at once. It does not wait on some other point of the path, and it does not mark the motion interfered, since nothing
// went wrong with the robot.
#include <cmath>
#include <string>

#include "doctest.h"
#include "exit_gate_rig.hpp"
#include "stdout_capture.hpp"

using namespace ez;
using namespace gate;

namespace {

size_t count_of(const std::string& text, const std::string& what) {
  size_t n = 0;
  for (size_t at = text.find(what); at != std::string::npos; at = text.find(what, at + what.size())) n++;
  return n;
}

void start_path(Rig& r) { r.chassis.pid_odom_set({{{0_in, 12_in}, fwd, 100}, {{12_in, 24_in}, fwd, 100}, {{24_in, 24_in}, fwd, 100}}); }

}  // namespace

TEST_CASE("pid_wait_until_index() with an index off the path returns at once, prints once and is not interfered") {
  // Two passes per poll on the heavy robot: the pace at which a healthy run used to be called interfered
  for (int index : {-1, -5, 3, 99}) {
    Rig r(sim::archetype_heavy_slow(), 2);
    start_path(r);
    double elapsed = -1;
    bool ok = false;
    std::string out = test_stub::capture_stdout([&]() { ok = r.wait([&]() { r.chassis.pid_wait_until_index(index); }, 2000, &elapsed); });
    CAPTURE(index);
    REQUIRE(ok);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(elapsed == doctest::Approx(0.0));
    CHECK(count_of(out, "is not within range") == 1);
    // The motion itself carries on and is still waited for the normal way
    std::string out2 = test_stub::capture_stdout([&]() { ok = r.wait([&]() { r.chassis.pid_wait(); }, 3000, &elapsed); });
    REQUIRE(ok);
    CHECK_FALSE(r.chassis.interfered);
  }
}

TEST_CASE("pid_wait_until_index_started() with an index off the path prints once and is not interfered") {
  for (int index : {-1, 3, 99}) {
    Rig r(sim::archetype_heavy_slow(), 2);
    start_path(r);
    bool ok = false;
    std::string out = test_stub::capture_stdout([&]() { ok = r.wait([&]() { r.chassis.pid_wait_until_index_started(index); }, 2000); });
    CAPTURE(index);
    REQUIRE(ok);
    CHECK_FALSE(r.chassis.interfered);
    CHECK(count_of(out, "is not within range") == 1);
  }
}

TEST_CASE("pid_wait_until_index() with the last index on the path still waits for the robot to get there") {
  Rig r(sim::archetype_heavy_slow(), 2);
  start_path(r);
  double elapsed = 0;
  bool ok = r.wait([&]() { r.chassis.pid_wait_until_index(2); }, 3000, &elapsed);
  REQUIRE(ok);
  CHECK(elapsed > 100.0);
  CHECK_FALSE(r.chassis.interfered);
}
