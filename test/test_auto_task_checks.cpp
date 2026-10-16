// The marker every EZ-Template task sets when it starts, and ez_auto_task's pass counter.
//
//  - Every task EZ-Template creates starts by saying the scheduler is running. Until then a guard must not touch
//    priorities (task_get_current() is not the caller before the scheduler starts).
//  - ez_auto_task counts its passes, a heartbeat for debug displays.
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "EZ-Template/lock.hpp"
#include "doctest.h"
#include "drive_test_access.hpp"

using namespace ez;

namespace {
namespace fs = std::filesystem;

Drive make_chassis() {
  test_stub::reset_all();
  detail::scheduler_running.store(false);
  detail::stats.auto_task_passes.store(0);
  detail::print_sink = nullptr;
  return Drive({1, -2}, {-3, 4}, 5, 3.25, 360);
}

// Runs ez_auto_task for `passes` passes. pros::Task never runs its callable in the host build, so the fake
// pros::delay() throws once the countdown reaches zero to unwind the otherwise-infinite loop.
void run_auto_task(Drive& chassis, int passes) {
  test_stub::g_clock.delay_calls_until_stop = passes - 1;
  try {
    DriveTestAccess::ez_auto_task(chassis);
  } catch (test_stub::StopLoop&) {}
  test_stub::g_clock.delay_calls_until_stop = -1;
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

TEST_CASE("ez_auto_task counts its passes") {
  Drive chassis = make_chassis();
  run_auto_task(chassis, 4);
  CHECK(detail::stats.auto_task_passes.load() == 4);
}
