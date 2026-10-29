// Promise 5: PROS deletes the competition task on every mode change, even while it holds a lock. So every lock is an ez::Lock
// taken with a KillSafeGuard (a PlainGuard only where PROS never deletes the task), and nothing blocks while a guard is held.
// This reads the library source, the way CODING-STYLE.md describes. The scan is lexical: a helper that only runs with the lock
// held is invisible to it unless it is on kLockedHelpers below.
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "../doctest.h"

namespace {

namespace fs = std::filesystem;

// Functions that are only ever called with the chassis lock held, and are not themselves inside a guard's scope.
const std::vector<std::string> kLockedHelpers = {
    "Drive::check_imu_task(", "Drive::raw_pid_odom_ptp_set(", "Drive::smooth_path(", "Drive::inject_points(", "Drive::raw_pid_odom_pp_set(",
};

// Functions where a PlainGuard is allowed, because the task that runs them is never deleted by PROS: ez_auto_task, and
// lines_replay, the display daemon's callback.
const std::vector<std::string> kPlainGuardFunctions = {
    "Drive::ez_auto_task(",
    "lines_replay(",
};

void erase_all(std::string& text, const std::string& what) {
  for (std::size_t at = text.find(what); at != std::string::npos; at = text.find(what, at)) text.erase(at, what.size());
}

// Blanks out comments and the contents of string and character literals, one output line per input line, so
// a doc comment or a printf format that mentions std::lock_guard, or contains a brace, does not confuse the scans.
std::vector<std::string> strip_comments_and_literals(const std::string& text) {
  std::vector<std::string> lines;
  std::istringstream in(text);
  std::string line;
  bool in_block_comment = false;
  while (std::getline(in, line)) {
    std::string kept;
    for (std::size_t i = 0; i < line.size(); i++) {
      if (in_block_comment) {
        if (line.compare(i, 2, "*/") == 0) {
          in_block_comment = false;
          i++;
        }
      } else if (line.compare(i, 2, "/*") == 0) {
        in_block_comment = true;
        i++;
      } else if (line.compare(i, 2, "//") == 0) {
        break;
      } else if (line[i] == '"' || line[i] == '\'') {
        char quote = line[i];
        kept += quote;
        for (i++; i < line.size() && line[i] != quote; i++)
          if (line[i] == '\\') i++;  // skip the escaped character
        kept += quote;
      } else {
        kept += line[i];
      }
    }
    lines.push_back(kept);
  }
  return lines;
}

// Rule 1. Line numbers (1-based) in `text` that use a lock other than ez::Lock / the ez guards.
std::vector<int> bare_lock_lines(const std::string& text) {
  static const std::vector<std::string> forbidden = {
      "std::lock_guard",
      "std::unique_lock",
      "std::scoped_lock",
      "std::mutex",
      "std::recursive_mutex",
      "std::shared_mutex",
      "pros::Mutex",
      "pros::RecursiveMutex",
      ".take(",
      "->take(",
      ".give(",
      "->give(",
      "mutex_take",
      "mutex_give",
  };

  std::vector<int> found;
  std::vector<std::string> lines = strip_comments_and_literals(text);
  for (std::size_t i = 0; i < lines.size(); i++) {
    std::string line = lines[i];
    // The mutex type is allowed as the argument of a Lock or a guard, and nowhere else.
    for (const char* wrapper : {"Lock", "PlainGuard", "KillSafeGuard"}) {
      erase_all(line, std::string(wrapper) + "<pros::RecursiveMutex>");
      erase_all(line, std::string(wrapper) + "<pros::Mutex>");
    }
    for (const std::string& token : forbidden) {
      if (line.find(token) != std::string::npos) {
        found.push_back(static_cast<int>(i) + 1);
        break;
      }
    }
  }
  return found;
}

// Rule 2. Line numbers (1-based) of blocking calls made while a guard is in scope. A guard is in scope from
// its declaration to the end of the block that contains it. A function on kLockedHelpers only ever runs with the
// lock held, so its whole body counts.
std::vector<int> slow_call_lines_under_lock(const std::string& text) {
  static const std::regex guard(R"(\b(?:PlainGuard|KillSafeGuard)<[^>]*>\s+\w+\s*[({])");
  static const std::regex slow(R"(\b(printf|fprintf|puts|delay|task_delay|screen_print)\b|std::cout|std::cerr|pros::screen|pros::lcd::)");

  std::vector<int> found;
  std::vector<std::string> lines = strip_comments_and_literals(text);
  for (std::size_t start = 0; start < lines.size(); start++) {
    bool declares_guard = std::regex_search(lines[start], guard);
    bool defines_helper = false;
    for (const std::string& helper : kLockedHelpers) {
      const std::string& line = lines[start];
      std::size_t brace = line.find_last_not_of(" \t\r");
      if (line.find(helper) != std::string::npos && brace != std::string::npos && line[brace] == '{') defines_helper = true;
    }
    if (!declares_guard && !defines_helper) continue;

    int depth = defines_helper ? -1 : 0;  // the body's opening brace is on this line, so it is not yet inside a block
    for (std::size_t i = start; i < lines.size(); i++) {
      if (i > start && std::regex_search(lines[i], slow)) found.push_back(static_cast<int>(i) + 1);
      for (char c : lines[i]) depth += (c == '{') - (c == '}');
      if (depth < 0) break;  // the block holding the guard has ended
    }
  }
  return found;
}

// Rule 3. Line numbers (1-based) that declare a PlainGuard anywhere except inside a function on kPlainGuardFunctions.
// A PlainGuard does not protect its task from a competition mode change, so it is only for tasks PROS never deletes.
// The function a line is in is the nearest definition above it: a line that starts in column 0 and ends with a brace.
std::vector<int> stray_plain_guard_lines(const std::string& text) {
  static const std::regex plain(R"(\bPlainGuard<[^>]*>\s+\w+\s*[({])");
  std::vector<int> found;
  std::vector<std::string> lines = strip_comments_and_literals(text);
  for (std::size_t i = 0; i < lines.size(); i++) {
    if (!std::regex_search(lines[i], plain)) continue;

    std::string header;
    for (std::size_t j = i + 1; j-- > 0;) {
      const std::string& line = lines[j];
      std::size_t last = line.find_last_not_of(" \t\r");
      if (!line.empty() && !std::isspace(static_cast<unsigned char>(line[0])) && line[0] != '}' && line[0] != '#' && last != std::string::npos &&
          line[last] == '{') {
        header = line;
        break;
      }
    }
    bool allowed = false;
    for (const std::string& function : kPlainGuardFunctions)
      if (header.find(function) != std::string::npos) allowed = true;
    if (!allowed) found.push_back(static_cast<int>(i) + 1);
  }
  return found;
}

std::string read_file(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

// `make -C test/v2` runs from test/v2. Fall back to this file's own location for other launchers.
fs::path library_root() {
  for (const fs::path& candidate : {fs::path("../.."), fs::path(__FILE__).parent_path() / ".." / ".."}) {
    if (fs::exists(candidate / "src" / "EZ-Template") && fs::exists(candidate / "include" / "EZ-Template")) return candidate;
  }
  return {};
}

}  // namespace

// The scan can only pass the library if it can fail on a rule break, so each rule is shown catching one and letting one through.
TEST_CASE("Promise 5: the locking scan catches each kind of break, and ignores comments and strings") {
  CHECK(bare_lock_lines("int a;\nint b;\npros::Mutex m;\nint c;") == std::vector<int>{3});
  CHECK(bare_lock_lines("std::lock_guard<pros::RecursiveMutex> lock(drive_mutex);").size() == 1);
  CHECK(bare_lock_lines("mutex->give();").size() == 1);
  CHECK(bare_lock_lines("ez::KillSafeGuard<pros::RecursiveMutex> lock(drive_mutex);").empty());
  CHECK(bare_lock_lines("/* std::lock_guard */ printf(\"a pros::Mutex in text\");").empty());

  CHECK(slow_call_lines_under_lock("void f() {\n  KillSafeGuard<pros::Mutex> lock(m);\n  if (a) {\n    printf(\"x\");\n  }\n}") == std::vector<int>{4});
  CHECK(slow_call_lines_under_lock("void f() {\n  KillSafeGuard<pros::Mutex> lock(m);\n  pros::delay(10);\n}").size() == 1);
  CHECK(slow_call_lines_under_lock("void Drive::check_imu_task() {\n  printf(\"x\");\n}") == std::vector<int>{2});
  CHECK(slow_call_lines_under_lock("void f() {\n  KillSafeGuard<pros::Mutex> lock(m);\n  drive_mutex.print_after_unlock(\"x\");\n}").empty());
  CHECK(slow_call_lines_under_lock("void f() {\n  {\n    KillSafeGuard<pros::Mutex> lock(m);\n  }\n  printf(\"done\");\n}").empty());

  CHECK(stray_plain_guard_lines("void Drive::odom_x_set(double x) {\n  ez::PlainGuard<pros::RecursiveMutex> lock(drive_mutex);\n}") == std::vector<int>{2});
  CHECK(stray_plain_guard_lines("void Drive::ez_auto_task() {\n  while (true) {\n    ez::PlainGuard<pros::RecursiveMutex> lock(drive_mutex);\n  }\n}").empty());
  CHECK(stray_plain_guard_lines("void Drive::ez_auto_task() {\n}\nvoid other() {\n  PlainGuard<pros::Mutex> guard(m);\n}") == std::vector<int>{4});
}

TEST_CASE("Promise 5: no library source breaks the locking rules") {
  fs::path root = library_root();
  REQUIRE_MESSAGE(!root.empty(), "could not find the library source. Run the tests from test/v2 (make -C test/v2).");

  std::vector<std::string> bare_locks, slow_calls, stray_plain_guards;
  std::size_t files_scanned = 0;
  for (const char* dir : {"src", "include/EZ-Template"}) {
    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(root / dir)) {
      if (!entry.is_regular_file()) continue;
      std::string ext = entry.path().extension().string();
      if (ext != ".cpp" && ext != ".hpp" && ext != ".h" && ext != ".c") continue;
      if (entry.path().filename() == "lock.hpp") continue;  // the one place that wraps the real mutexes

      files_scanned++;
      std::string text = read_file(entry.path());
      std::string where = entry.path().generic_string();
      for (int line : bare_lock_lines(text)) bare_locks.push_back(where + ":" + std::to_string(line));
      for (int line : slow_call_lines_under_lock(text)) slow_calls.push_back(where + ":" + std::to_string(line));

      for (int line : stray_plain_guard_lines(text)) stray_plain_guards.push_back(where + ":" + std::to_string(line));
    }
  }

  CHECK_MESSAGE(files_scanned > 20, "the scan found suspiciously few files, so it is not checking the library");

  std::string bare_message =
      "These lines take a lock directly. Use an ez::Lock with an ez::KillSafeGuard instead, so a task deleted while "
      "holding it cannot leave it held forever (see include/EZ-Template/lock.hpp):";
  for (const std::string& problem : bare_locks) bare_message += "\n  " + problem;
  CHECK_MESSAGE(bare_locks.empty(), bare_message);

  std::string slow_message =
      "These lines do slow work while holding a lock. Use lock.print_after_unlock() for text, and move anything else "
      "that waits outside the guard's scope, so a task cannot be deleted while it is blocked holding the lock "
      "(see include/EZ-Template/lock.hpp):";
  for (const std::string& problem : slow_calls) slow_message += "\n  " + problem;
  CHECK_MESSAGE(slow_calls.empty(), slow_message);

  std::string plain_message =
      "A PlainGuard does not protect its task from a competition mode change. It is only for tasks PROS never deletes "
      "(see kPlainGuardFunctions at the top of this test). Use an ez::KillSafeGuard here:";
  for (const std::string& problem : stray_plain_guards) plain_message += "\n  " + problem;
  CHECK_MESSAGE(stray_plain_guards.empty(), plain_message);
}
