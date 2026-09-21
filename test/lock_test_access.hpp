// Lets tests see an ez::Lock's private state. Declared a friend inside Lock, defined only in the tests. Defined
// here, once, so every test file that includes it sees the same definition.
#pragma once

#include "EZ-Template/lock.hpp"

namespace ez {
struct LockTestAccess {
  template <typename M>
  static M& mutex(Lock<M>& lock) {
    return lock.mutex_;
  }
  template <typename M>
  static pros::task_t owner(Lock<M>& lock) {
    return lock.owner_.load();
  }
  template <typename M>
  static const std::atomic<pros::task_t>& owner_field(Lock<M>& lock) {
    return lock.owner_;
  }
  template <typename M>
  static int depth(Lock<M>& lock) {
    return lock.depth_;
  }
};
}  // namespace ez
