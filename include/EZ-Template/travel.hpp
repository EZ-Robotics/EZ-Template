/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace ez {
namespace detail {

/**
 * How far one measured thing (a drive side, the heading, the odom xy pose) has travelled over the last W milliseconds.
 *
 * This is what the library means by "stopped": the PATH LENGTH of the last W ms divided by W is under a speed floor.  Not
 * the speed of one tick (one encoder count in one 10 ms tick is already 2.55 in/s on a 450 rpm 3.25 in drive, so a per
 * tick speed cannot see a floor of 1.5 in/s), and not the start-to-end difference (a robot oscillating fast across its
 * target reads as stopped to that).
 *
 * Every sample goes through a backlash band first.  Position only counts as travel once it leaves a band of +/- `band`
 * (one sensor count) around where it last stopped moving; when it leaves, the band follows it and only the distance past
 * the edge counts.  A sensor flickering one count at rest then adds nothing (without this it would add one count every
 * flicker, which at 10 ms ticks is 2.55 in/s of fake travel), and a real swing larger than one count is still counted in
 * full.
 *
 * Sampled by the auto task once per fresh pass, not by whoever asks.  Two samples with the same timestamp (a task that
 * catches up runs several passes in one tick, all reading the same sensors) are one sample, and a pass counter that has not
 * moved is no sample at all.  Asking about a window the history does not cover, or one whose latest sample is old (a starved
 * or dead task), answers "not stopped": the existing starved-task fallbacks own that case.
 *
 * A fixed ring, no allocation.  CAPACITY samples at the 10 ms pass time is 2.5 s of history, so any window up to 2 s is
 * answered in full; a longer one is answered as the average speed over the stored history, so motion older than the ring
 * is forgotten and such a window can read "stopped" where the full window would not.  Exit windows beyond 2 s are not
 * supported.
 *
 * Not thread safe on its own: the drive samples and asks under its mutex.
 */
class PathTracker {
public:
  static constexpr int CAPACITY = 256;
  // The shortest window anything is asked about: two auto task passes.  A window of 0 or 10 ms would otherwise be a single
  // sample interval, which has no travel in it at all for a flat reading.
  static constexpr int MIN_WINDOW_MS = 20;
  // A latest sample older than this means the task that feeds the tracker is not running (3 passes).
  static constexpr int STALE_MS = 30;

  void band_set(double band) { band_ = band; }

  // Forget the history.  The next sample is the new baseline.
  void reset() {
    has_baseline_ = false;
    count_ = 0;
    head_ = 0;
    wrapped_ = false;
    cum_ = 0.0;
  }

  // Whether anything has been sampled since the last reset
  bool active() const { return has_baseline_; }

  // `pass` is the auto task's pass counter.  A position that is not finite is ignored, like a pass that did not happen.
  void sample(double x, double y, std::uint32_t t_ms, std::uint32_t pass) {
    if (!std::isfinite(x) || !std::isfinite(y)) return;
    if (!has_baseline_) {
      cx_ = x;
      cy_ = y;
      cum_ = 0.0;
      last_pass_ = pass;
      push(t_ms);
      has_baseline_ = true;
      return;
    }
    if (pass == last_pass_) return;  // nothing fresh
    last_pass_ = pass;
    advance(x, y);
    if (t_ms == newest_t()) {
      cum_at(newest_index()) = cum_;  // the same instant again: one sample
      return;
    }
    push(t_ms);
  }

  // The path length over the last window_ms (at least MIN_WINDOW_MS) and the span of time it really covers, which is at least
  // that.  False when it cannot be told: no history, not enough of it to cover the window, or a stale latest sample.
  bool travel_over(int window_ms, std::uint32_t now_ms, double& travel, double& span_ms) const {
    if (!has_baseline_ || count_ < 2) return false;
    if ((std::int32_t)(now_ms - newest_t()) > STALE_MS) return false;
    if (window_ms < MIN_WINDOW_MS) window_ms = MIN_WINDOW_MS;
    std::uint32_t want = newest_t() - (std::uint32_t)window_ms;
    // Newest sample at or before `want`
    int base = -1;
    for (int k = 1; k < count_; k++) {
      int idx = index_from_newest(k);
      if ((std::int32_t)(t_at(idx) - want) <= 0) {
        base = idx;
        break;
      }
    }
    if (base < 0) {
      // Not covered.  If the ring has not lost anything the window is longer than the motion has been running, which says
      // nothing; if it has, use what is left (the average speed over a shorter span, which forgets older motion).
      if (!wrapped_) return false;
      base = index_from_newest(count_ - 1);
    }
    span_ms = (double)(std::int32_t)(newest_t() - t_at(base));
    if (span_ms <= 0.0) return false;
    travel = cum_at_const(newest_index()) - cum_at_const(base);
    return true;
  }

  // Whether the thing was under floor_per_s (units per second) on average over the last window_ms
  bool stopped(int window_ms, double floor_per_s, std::uint32_t now_ms) const {
    double travel = 0.0, span = 0.0;
    if (!travel_over(window_ms, now_ms, travel, span)) return false;
    return travel < floor_per_s * span / 1000.0;
  }

private:
  void advance(double x, double y) {
    double dx = x - cx_, dy = y - cy_;
    double d = std::hypot(dx, dy);
    // The epsilon keeps a reading that is exactly one count away (which float noise can put a hair over) inside the band
    if (d > band_ + 1e-9) {
      double move = d - band_;
      cx_ += dx / d * move;
      cy_ += dy / d * move;
      cum_ += move;
    }
  }

  int newest_index() const { return (head_ + CAPACITY - 1) % CAPACITY; }
  int index_from_newest(int k) const { return (head_ + CAPACITY - 1 - k) % CAPACITY; }
  std::uint32_t newest_t() const { return t_at(newest_index()); }
  std::uint32_t t_at(int idx) const { return t_[idx]; }
  double& cum_at(int idx) { return cum_buf_[idx]; }
  double cum_at_const(int idx) const { return cum_buf_[idx]; }

  void push(std::uint32_t t_ms) {
    t_[head_] = t_ms;
    cum_buf_[head_] = cum_;
    head_ = (head_ + 1) % CAPACITY;
    if (count_ < CAPACITY)
      count_++;
    else
      wrapped_ = true;
  }

  std::array<std::uint32_t, CAPACITY> t_{};
  std::array<double, CAPACITY> cum_buf_{};
  int head_ = 0;
  int count_ = 0;
  bool wrapped_ = false;
  bool has_baseline_ = false;
  double band_ = 0.0;
  double cx_ = 0.0, cy_ = 0.0;
  double cum_ = 0.0;
  std::uint32_t last_pass_ = 0;
};

}  // namespace detail
}  // namespace ez
