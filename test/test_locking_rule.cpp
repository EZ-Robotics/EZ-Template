// The locking rules (see the top of include/EZ-Template/lock.hpp):
//
//  1. Every lock in the library is an ez::RecoverableMutex taken with an ez::LockGuard. No bare
//     pros::Mutex, pros::RecursiveMutex, std::lock_guard, or take()/give() on a mutex outside lock.hpp.
//  2. Nothing slow happens while a lock is held: no printf, no delay, no screen printing. Text goes through
//     guard.print_after_unlock(), which prints once the lock is free.
//
// Why a test and not just a convention: PROS deletes the running competition task on every mode change
// without releasing the mutexes it held, so a lock that waits forever can leave odometry dead until the
// brain restarts. One forgotten bare lock brings that failure back, and it only shows up rarely on a real
// robot, so this test reads the source and fails the build the moment one is added.
//
// Pinned here:
//  - the scanners flag each kind of violation, and ignore comments, strings, and RecoverableMutex declarations
//  - no library source file breaks either rule (display.cpp is scanned even though the host build skips it)
//
// The scan is lexical. It cannot see a slow call made by a function that a locked function calls, so keep
// slow work out of such helpers too.
#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>
#include <vector>

#include "doctest.h"

namespace {

namespace fs = std::filesystem;

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

// Rule 1. Line numbers (1-based) in `text` that use a lock other than ez::RecoverableMutex / ez::LockGuard.
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
    // The mutex type is allowed as the argument of a RecoverableMutex, and nowhere else.
    erase_all(line, "RecoverableMutex<pros::RecursiveMutex>");
    erase_all(line, "RecoverableMutex<pros::Mutex>");
    // A guard passed to a helper, so the helper can print through it.
    erase_all(line, "LockGuard<pros::RecursiveMutex>");
    erase_all(line, "LockGuard<pros::Mutex>");
    for (const std::string& token : forbidden) {
      if (line.find(token) != std::string::npos) {
        found.push_back(static_cast<int>(i) + 1);
        break;
      }
    }
  }
  return found;
}

// Rule 2. Line numbers (1-based) of slow calls made while an ez::LockGuard is in scope. A guard is in
// scope from its declaration to the end of the block that contains it. A function that takes a guard by
// reference, so it can print through it, only ever runs with the lock held, so its whole body counts.
std::vector<int> slow_call_lines_under_lock(const std::string& text) {
  static const std::regex guard(R"(\bLockGuard\s+\w+\s*[({])");
  static const std::regex guard_parameter(R"(\bLockGuard<[^>]*>&\s*\w+\s*\)\s*\{)");
  static const std::regex slow(R"(\b(printf|fprintf|puts|delay|task_delay|screen_print)\b|std::cout|pros::lcd::)");

  std::vector<int> found;
  std::vector<std::string> lines = strip_comments_and_literals(text);
  for (std::size_t start = 0; start < lines.size(); start++) {
    bool declares_guard = std::regex_search(lines[start], guard);
    bool takes_guard = std::regex_search(lines[start], guard_parameter);
    if (!declares_guard && !takes_guard) continue;

    int depth = takes_guard ? -1 : 0;  // the body's opening brace is on this line, so it is not yet inside a block
    for (std::size_t i = start; i < lines.size(); i++) {
      if (i > start && std::regex_search(lines[i], slow)) found.push_back(static_cast<int>(i) + 1);
      for (char c : lines[i]) depth += (c == '{') - (c == '}');
      if (depth < 0) break;  // the block holding the guard has ended
    }
  }
  return found;
}

std::string read_file(const fs::path& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream out;
  out << in.rdbuf();
  return out.str();
}

// `make -C test` runs from the test directory. Fall back to this file's own location for other launchers.
fs::path library_root() {
  for (const fs::path& candidate : {fs::path(".."), fs::path(__FILE__).parent_path() / ".."}) {
    if (fs::exists(candidate / "src" / "EZ-Template") && fs::exists(candidate / "include" / "EZ-Template")) return candidate;
  }
  return {};
}

}  // namespace

TEST_CASE("rule 1 scanner flags every kind of bare lock") {
  CHECK(bare_lock_lines("std::lock_guard<std::mutex> lock(m);").size() == 1);
  CHECK(bare_lock_lines("std::lock_guard<pros::RecursiveMutex> lock(drive_mutex);").size() == 1);
  CHECK(bare_lock_lines("std::unique_lock<pros::Mutex> lock(m);").size() == 1);
  CHECK(bare_lock_lines("std::scoped_lock lock(a, b);").size() == 1);
  CHECK(bare_lock_lines("pros::Mutex mutex;").size() == 1);
  CHECK(bare_lock_lines("pros::RecursiveMutex drive_mutex;").size() == 1);
  CHECK(bare_lock_lines("line_mutex().take();").size() == 1);
  CHECK(bare_lock_lines("mutex->give();").size() == 1);
  CHECK(bare_lock_lines("mutex_take(handle, 100);").size() == 1);
  CHECK(bare_lock_lines("int a;\nint b;\npros::Mutex m;\nint c;").at(0) == 3);  // reports the right line
}

TEST_CASE("rule 1 scanner accepts the recoverable lock, and ignores comments and strings") {
  CHECK(bare_lock_lines("RecoverableMutex<pros::RecursiveMutex> drive_mutex{\"chassis\"};").empty());
  CHECK(bare_lock_lines("static ez::RecoverableMutex<pros::Mutex> mutex(\"screen line\");").empty());
  CHECK(bare_lock_lines("ez::LockGuard lock(drive_mutex);").empty());
  CHECK(bare_lock_lines("// std::lock_guard is not allowed here").empty());
  CHECK(bare_lock_lines("int a;  // pros::Mutex used to be here").empty());
  CHECK(bare_lock_lines("/* std::lock_guard\n   pros::Mutex */\nint a;").empty());
  CHECK(bare_lock_lines("/**\n * Recursive so a std::lock_guard was fine\n */").empty());
  CHECK(bare_lock_lines("printf(\"a pros::Mutex in text\");").empty());
  CHECK(bare_lock_lines("void check(ez::LockGuard<pros::RecursiveMutex>& lock);").empty());

  // But a bare lock beside a RecoverableMutex on the same line is still caught.
  CHECK(bare_lock_lines("RecoverableMutex<pros::Mutex> a; pros::Mutex b;").size() == 1);
}

TEST_CASE("rule 2 scanner flags slow calls made while a guard is held") {
  CHECK(slow_call_lines_under_lock("void f() {\n  ez::LockGuard lock(m);\n  printf(\"x\");\n}").at(0) == 3);
  CHECK(slow_call_lines_under_lock("void f() {\n  ez::LockGuard lock(m);\n  fprintf(stderr, \"x\");\n}").size() == 1);
  CHECK(slow_call_lines_under_lock("void f() {\n  ez::LockGuard lock(m);\n  pros::delay(10);\n}").size() == 1);
  CHECK(slow_call_lines_under_lock("void f() {\n  ez::LockGuard lock(m);\n  ez::screen_print(\"x\", 1);\n}").size() == 1);
  CHECK(slow_call_lines_under_lock("void f() {\n  ez::LockGuard lock(m);\n  if (a) {\n    printf(\"x\");\n  }\n}").at(0) == 4);     // nested block
  CHECK(slow_call_lines_under_lock("void f() {\n  ez::LockGuard lock(m);\n  a = 1;\n}\nvoid g() {\n  printf(\"x\");\n}").empty());  // next function
}

TEST_CASE("rule 2 scanner treats the whole body of a function that takes a guard as locked") {
  CHECK(slow_call_lines_under_lock("void f(ez::LockGuard<pros::RecursiveMutex>& lock) {\n  printf(\"x\");\n}").at(0) == 2);
  CHECK(slow_call_lines_under_lock("void f(LockGuard<pros::Mutex>& lock) {\n  a = 1;\n  if (a) {\n    pros::delay(5);\n  }\n}").at(0) == 4);
  CHECK(slow_call_lines_under_lock("void f(LockGuard<pros::RecursiveMutex>& lock) {\n  lock.print_after_unlock(\"x\");\n}").empty());
  CHECK(slow_call_lines_under_lock("void f(LockGuard<pros::RecursiveMutex>& lock) {\n  a = 1;\n}\nvoid g() {\n  printf(\"x\");\n}").empty());  // next function
  CHECK(slow_call_lines_under_lock("void f(LockGuard<pros::RecursiveMutex>& lock);\nvoid g() {\n  printf(\"x\");\n}").empty());              // a declaration has no body
}

TEST_CASE("rule 2 scanner accepts print_after_unlock, and slow calls after the guard's block ends") {
  CHECK(slow_call_lines_under_lock("void f() {\n  ez::LockGuard lock(m);\n  lock.print_after_unlock(\"x\");\n}").empty());
  CHECK(slow_call_lines_under_lock("void f() {\n  {\n    ez::LockGuard lock(m);\n    a = 1;\n  }\n  printf(\"done\");\n  pros::delay(10);\n}").empty());
  CHECK(slow_call_lines_under_lock("void f() {\n  printf(\"before\");\n  ez::LockGuard lock(m);\n  a = 1;\n}").empty());
  CHECK(slow_call_lines_under_lock("void f() {\n  ez::LockGuard lock(m);\n  // printf(\"a comment\");\n  a = \"printf(\";\n}").empty());
  CHECK(slow_call_lines_under_lock("void f() {\n  ez::LockGuard lock(m);\n  a = \"{ unbalanced\";\n  b = 2;\n}\nvoid g() { printf(\"x\"); }").empty());  // braces in strings ignored
}

TEST_CASE("no library source breaks the locking rules") {
  fs::path root = library_root();
  REQUIRE_MESSAGE(!root.empty(), "could not find the library source. Run the tests from the test directory (make -C test).");

  std::vector<std::string> bare_locks, slow_calls;
  std::size_t files_scanned = 0;
  for (const char* dir : {"src", "include/EZ-Template"}) {
    for (const fs::directory_entry& entry : fs::recursive_directory_iterator(root / dir)) {
      if (!entry.is_regular_file()) continue;
      std::string ext = entry.path().extension().string();
      if (ext != ".cpp" && ext != ".hpp" && ext != ".h" && ext != ".c") continue;
      if (entry.path().filename() == "lock.hpp") continue;  // the one place that wraps the real mutexes

      files_scanned++;
      std::string text = read_file(entry.path());
      for (int line : bare_lock_lines(text)) bare_locks.push_back(entry.path().generic_string() + ":" + std::to_string(line));
      for (int line : slow_call_lines_under_lock(text)) slow_calls.push_back(entry.path().generic_string() + ":" + std::to_string(line));
    }
  }

  CHECK_MESSAGE(files_scanned > 20, "the scan found suspiciously few files, so it is not checking the library");

  std::string bare_message =
      "These lines take a lock directly. Use an ez::RecoverableMutex with an ez::LockGuard instead, so a task "
      "deleted while holding it cannot leave it held forever (see include/EZ-Template/lock.hpp):";
  for (const std::string& problem : bare_locks) bare_message += "\n  " + problem;
  CHECK_MESSAGE(bare_locks.empty(), bare_message);

  std::string slow_message =
      "These lines do slow work while holding a lock. Use guard.print_after_unlock() for text, and move anything "
      "else that waits outside the guard's scope, so a stuck terminal or a slow call cannot make other tasks "
      "think the lock's owner is gone (see include/EZ-Template/lock.hpp):";
  for (const std::string& problem : slow_calls) slow_message += "\n  " + problem;
  CHECK_MESSAGE(slow_calls.empty(), slow_message);
}
