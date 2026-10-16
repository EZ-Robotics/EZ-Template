// Captures what the code under test writes to stdout, so a test can pin the exact text printed to the terminal.
//
// It redirects the process's stdout file descriptor into a scratch file for the length of the capture. That
// reaches printf and std::cout alike, which matters because the library calls printf directly. The scratch file
// is created in the current directory (`make -C test` runs from test/), because tmpfile() on Windows writes to
// the drive root and is often refused.
#pragma once

#include <cstdio>
#include <string>

#ifdef _WIN32
#include <io.h>
#define EZ_TEST_DUP _dup
#define EZ_TEST_DUP2 _dup2
#define EZ_TEST_FILENO _fileno
#define EZ_TEST_CLOSE _close
#else
#include <unistd.h>
#define EZ_TEST_DUP dup
#define EZ_TEST_DUP2 dup2
#define EZ_TEST_FILENO fileno
#define EZ_TEST_CLOSE close
#endif

namespace test_stub {

// Runs `action` and returns everything it wrote to stdout, with Windows line endings folded to "\n".
template <typename F>
std::string capture_stdout(F&& action) {
  const char* path = "ez_stdout_capture.tmp";
  std::fflush(stdout);
  int saved = EZ_TEST_DUP(EZ_TEST_FILENO(stdout));
  std::FILE* file = std::fopen(path, "w+b");
  EZ_TEST_DUP2(EZ_TEST_FILENO(file), EZ_TEST_FILENO(stdout));

  // Put stdout back even if `action` throws.
  struct Restore {
    int saved;
    ~Restore() {
      std::fflush(stdout);
      EZ_TEST_DUP2(saved, EZ_TEST_FILENO(stdout));
      EZ_TEST_CLOSE(saved);
    }
  };
  {
    Restore restore{saved};
    action();
  }

  std::fflush(file);
  std::rewind(file);
  std::string text;
  char buffer[512];
  for (std::size_t n; (n = std::fread(buffer, 1, sizeof(buffer), file)) > 0;) text.append(buffer, n);
  std::fclose(file);
  std::remove(path);

  std::string folded;
  for (char c : text)
    if (c != '\r') folded += c;
  return folded;
}

}  // namespace test_stub
