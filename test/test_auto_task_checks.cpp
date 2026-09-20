// The checks ez_auto_task makes about itself, and the marker every EZ-Template task sets when it starts.
//
//  - Every task EZ-Template creates starts by saying the scheduler is running. Until then a guard must not touch
//    priorities (task_get_current() is not the caller before the scheduler starts).
//  - ez_auto_task prints one warning if the PROS system daemon is not below the priority guards raise to.
//  - ez_auto_task counts the passes that start at a different priority than the task was created with. That should
//    never happen, and if it does the guard has leaked a priority.
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "EZ-Template/lock.hpp"
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;
using test_stub::g_sched;

namespace {
namespace fs = std::filesystem;

Drive make_chassis() {
  test_stub::reset_all();
  detail::scheduler_running.store(false);
  detail::daemon_reported.store(false);
  detail::stats.auto_task_priority_starts.store(0);
  detail::stats.auto_task_passes.store(0);
  detail::print_sink = nullptr;
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360, 1.0);
}

// Runs ez_auto_task for `passes` passes. pros::Task never runs its callable in the host build, so the fake
// pros::delay() throws once the countdown reaches zero to unwind the otherwise-infinite loop.
void run_auto_task(Drive& chassis, int passes) {
  test_stub::g_clock.delay_calls_until_stop = passes - 1;
  try {
    DriveTestAccess::ez_auto_task(chassis);
  } catch (test_stub::StopLoop&) {
  }
  test_stub::g_clock.delay_calls_until_stop = -1;
}

std::string printed;
void sink(const char* text) { printed += text; }
void capture_prints() {
  printed.clear();
  detail::print_sink = sink;
}

void drop_priority_between_passes() {
  g_sched.tasks[0].base = 11;
  g_sched.tasks[0].effective = 11;
  test_stub::g_clock.on_delay = nullptr;
}

// `make -C test` runs from the test directory. Fall back to this file's own location for other launchers.
fs::path library_root() {
  for (const fs::path& candidate : {fs::path(".."), fs::path(__FILE__).parent_path() / ".."}) {
    if (fs::exists(candidate / "src" / "EZ-Template") && fs::exists(candidate / "include" / "EZ-Template")) return candidate;
  }
  return {};
}

// The first statement of the function whose definition starts with `signature`, or "" if it is not found.
std::string first_statement(const std::string& source_file, const std::string& signature) {
  std::ifstream in(library_root() / source_file, std::ios::binary);
  std::ostringstream all;
  all << in.rdbuf();
  std::string text = all.str();
  std::size_t at = text.find(signature);
  if (at == std::string::npos) return "";
  std::size_t start = at + signature.size();
  for (;;) {  // skip whitespace and comment lines
    while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start]))) start++;
    if (text.compare(start, 2, "//") != 0) break;
    start = text.find('\n', start);
    if (start == std::string::npos) return "";
  }
  return text.substr(start, text.find(';', start) - start + 1);
}
}  // namespace

TEST_CASE("ez_auto_task tells EZ-Template the scheduler is running") {
  Drive chassis = make_chassis();
  CHECK_FALSE(detail::scheduler_running.load());
  run_auto_task(chassis, 1);
  CHECK(detail::scheduler_running.load());
}

TEST_CASE("every task EZ-Template creates starts by saying the scheduler is running") {
  REQUIRE_MESSAGE(!library_root().empty(), "could not find the library source. Run the tests from the test directory (make -C test).");
  CHECK(first_statement("src/EZ-Template/drive/pid_tasks.cpp", "void Drive::ez_auto_task() {") == "ez::detail::mark_scheduler_running();");
  // sdcard.cpp is not part of the host build, so this one is checked by reading the source.
  CHECK(first_statement("src/EZ-Template/sdcard.cpp", "void limitSwitchTask() {") == "ez::detail::mark_scheduler_running();");
}

TEST_CASE("a daemon below the guard's priority is not reported") {
  Drive chassis = make_chassis();
  capture_prints();
  g_sched.tasks[3] = {14, 14};
  run_auto_task(chassis, 2);
  CHECK(printed.empty());
}

TEST_CASE("a daemon at or above the guard's priority, or missing, is reported once") {
  Drive chassis = make_chassis();
  capture_prints();
  g_sched.tasks[3] = {15, 15};
  run_auto_task(chassis, 3);
  CHECK(printed.find("priority 15") != std::string::npos);
  CHECK(printed.find("not below 15") != std::string::npos);
  detail::report_daemon_priority_once();  // a second chassis asks again
  std::size_t first = printed.find("PROS system daemon");
  CHECK(printed.find("PROS system daemon", first + 1) == std::string::npos);  // once, however many ask

  Drive missing = make_chassis();
  capture_prints();
  g_sched.daemon_present = false;
  run_auto_task(missing, 2);
  CHECK(printed.find("could not find the PROS system daemon") != std::string::npos);

  Drive too_high = make_chassis();
  capture_prints();
  g_sched.tasks[3] = {16, 16};
  run_auto_task(too_high, 1);
  CHECK(printed.find("priority 16") != std::string::npos);
}

TEST_CASE("ez_auto_task counts a pass that starts at a different priority than the task was created with") {
  Drive chassis = make_chassis();
  test_stub::g_clock.on_delay = drop_priority_between_passes;  // something changes the task's priority after its first pass
  run_auto_task(chassis, 3);
  CHECK(detail::stats.auto_task_priority_starts.load() == 2);  // passes 2 and 3 both start at 11, the task was created at 8
}

TEST_CASE("ez_auto_task compares against the priority it started with, not a fixed default") {
  Drive chassis = make_chassis();
  g_sched.tasks[0] = {5, 5};  // this task was created at 5
  run_auto_task(chassis, 3);
  CHECK(detail::stats.auto_task_priority_starts.load() == 0);
}

TEST_CASE("ez_auto_task counts its passes") {
  Drive chassis = make_chassis();
  run_auto_task(chassis, 4);
  CHECK(detail::stats.auto_task_passes.load() == 4);
  CHECK(detail::stats.auto_task_priority_starts.load() == 0);
}
