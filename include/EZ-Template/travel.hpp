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
 * Sampled by the auto task once per fresh pass, not by whoever asks.  When the task stalls, the movement during the stall arrives in
 * the first sample after it and counts in full against the window asked about, not spread over the stall.  Two samples with the same timestamp (a task that
 * catches up runs several passes in one tick, all reading the same sensors) are one sample, and a pass counter that has not
 * moved is no sample at all.  Asking about a window the history does not cover, or one whose latest sample is old (a starved
 * or dead task), answers "not stopped": the existing starved-task fallbacks own that case.
 *
 * Net displacement is the other question a wait asks of the same history: has the thing gone anywhere over a window, a distance and not a
 * path length (see net_over()).  A robot oscillating in place has a long path and no net displacement, a robot being dragged through its
 * target has both.  It is read off the same band-followed position, so one count of flicker is no displacement.  It compares where the
 * thing sat on average in the first half of the window with where it sat on average in the second, not the first and last sample: a robot
 * that is hunting by one sample (a limit cycle that flips sign on every pass, or every third) is on the same side of its centre at both
 * ends of any window whose sample count is a multiple of the cycle and on the opposite side at both ends of any other, so the end points
 * read the whole amplitude on every pass, for ever, however long the robot sits there.  Averaging over half a window cannot be fooled by
 * a cycle that short: it reads at most the amplitude of the cycle times the cycle's length over the half window.
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
      store_position(newest_index(), count_ >= 2 ? index_from_newest(1) : -1);
      return;
    }
    push(t_ms);
  }

  // The path length over the last window_ms (at least MIN_WINDOW_MS) and the span of time it is measured over, which is the window
  // itself, or less only when the ring has lost the start of it.  When the samples have a gap in them the travel since the last
  // sample before the window is all charged to the window, so a gap can only make the robot read faster, never slower.  False
  // when it cannot be told: no history, not enough of it to cover the window, or a stale latest sample.
  bool travel_over(int window_ms, std::uint32_t now_ms, double& travel, double& span_ms) const {
    int base = 0;
    if (!locate(window_ms, now_ms, false, base, span_ms)) return false;
    travel = cum_at_const(newest_index()) - cum_at_const(base);
    return true;
  }

  // How far the thing went over the last window_ms (at least MIN_WINDOW_MS): the straight line from where it sat on average in the first
  // half of the window to where it sat on average in the second, and the time between those two averages, which is half the window.  A
  // thing moving at a steady speed v reads v times that span, whatever the window.  It is a distance and not a path length, so a thing
  // going back and forth about one place reads little however far it travels.  Unlike travel_over() a window the history does not cover
  // is answered over what is stored, from the oldest sample, whether the ring has wrapped or the motion is younger than the window: a
  // caller that holds a wait until this reads "moving" must never be left with nothing to read, so it can only be told "not enough to
  // say anything" (false) when there is no history at all or the latest sample is stale (a starved or dead task, which the starved-task
  // fallbacks own).  The position is taken to move in a straight line from one sample to the next, so movement during a gap in the samples
  // is spread over the gap.
  bool net_over(int window_ms, std::uint32_t now_ms, double& net, double& span_ms) const {
    int base = 0;
    double window = 0.0;
    if (!locate(window_ms, now_ms, true, base, window)) return false;
    double start_x, start_y, mid_x, mid_y, end_x, end_y;
    integral_at(-window, start_x, start_y);
    integral_at(-window / 2.0, mid_x, mid_y);
    integral_at(0.0, end_x, end_y);
    double half = window / 2.0;
    net = std::hypot(((end_x - mid_x) - (mid_x - start_x)) / half, ((end_y - mid_y) - (mid_y - start_y)) / half);
    span_ms = half;
    return true;
  }

  // Whether the thing was under floor_per_s (units per second) on average over the last window_ms
  bool stopped(int window_ms, double floor_per_s, std::uint32_t now_ms) const {
    double travel = 0.0, span = 0.0;
    if (!travel_over(window_ms, now_ms, travel, span)) return false;
    return travel < floor_per_s * span / 1000.0;
  }

  // Whether the thing went nowhere: its net displacement over the last window_ms (see net_over()) is under floor_per_s (units per second)
  // times the span that is measured over.  A thing that is oscillating about one place has a long path and no net displacement and is in
  // place; one that is being carried through it, or away from it, is not however slowly.  When there is nothing to say (see net_over()) it
  // is in place, so a caller that asks this of a robot it would otherwise let go is never left holding it by a tracker that has no history.
  bool in_place(int window_ms, double floor_per_s, std::uint32_t now_ms) const {
    double net = 0.0, span = 0.0;
    if (!net_over(window_ms, now_ms, net, span)) return true;
    return net < floor_per_s * span / 1000.0;
  }

private:
  // Finds the sample a window reaches back to: the newest one at or before `window_ms` ago, and the span of time from it to the newest.
  // `short_ok`: answer from the oldest sample when the history does not reach that far back (see net_over()).  Otherwise that is
  // only answered when the ring has lost the start of the window, as the average over what is left, which forgets older motion.
  bool locate(int window_ms, std::uint32_t now_ms, bool short_ok, int& base_out, double& span_ms) const {
    if (!has_baseline_ || count_ < 2) return false;
    if ((std::int32_t)(now_ms - newest_t()) > STALE_MS) return false;
    if (window_ms < MIN_WINDOW_MS) window_ms = MIN_WINDOW_MS;
    std::uint32_t want = newest_t() - (std::uint32_t)window_ms;
    int base = -1;
    bool covered = true;
    for (int k = 1; k < count_; k++) {
      int idx = index_from_newest(k);
      if ((std::int32_t)(t_at(idx) - want) <= 0) {
        base = idx;
        break;
      }
    }
    if (base < 0) {
      // Not covered.  If the ring has not lost anything the window is longer than the motion has been running, which says
      // nothing to travel_over(); if it has, use what is left (the average speed over a shorter span, which forgets older motion).
      if (!wrapped_ && !short_ok) return false;
      base = index_from_newest(count_ - 1);
      covered = false;
    }
    span_ms = (double)(std::int32_t)(newest_t() - t_at(base));
    if (span_ms <= 0.0) return false;
    // The sample the window reaches back to can be much older than the window when the task missed a stretch of time.  Whatever
    // the robot did in that stretch shows up in the first sample after it, and it is charged to the window that was asked about:
    // spread over the whole stretch it would be diluted (a 3 in/s shove after a 300 ms stall would read as 1 in/s).
    if (covered && span_ms > window_ms) span_ms = (double)window_ms;
    base_out = base;
    return true;
  }

  // The time integral of the band-followed position from the start of the history to `rel_ms` (zero or less) from the newest sample, with
  // the position moving in a straight line from one sample to the next.  What an average over any stretch of the history is made from:
  // the average over [a, b] is (integral_at(b) - integral_at(a)) / (b - a).  A time before the oldest sample reads as the oldest.
  void integral_at(double rel_ms, double& ix, double& iy) const {
    int newest = newest_index();
    if (rel_ms >= 0.0) {
      ix = ix_buf_[newest];
      iy = iy_buf_[newest];
      return;
    }
    std::uint32_t tn = newest_t();
    for (int k = 1; k < count_; k++) {
      int idx = index_from_newest(k);
      double rel = (double)(std::int32_t)(t_at(idx) - tn);
      if (rel > rel_ms) continue;
      int next = index_from_newest(k - 1);
      double rel_next = (double)(std::int32_t)(t_at(next) - tn);
      double frac = rel_next > rel ? (rel_ms - rel) / (rel_next - rel) : 0.0;
      double x_at = cx_buf_[idx] + (cx_buf_[next] - cx_buf_[idx]) * frac;
      double y_at = cy_buf_[idx] + (cy_buf_[next] - cy_buf_[idx]) * frac;
      ix = ix_buf_[idx] + 0.5 * (cx_buf_[idx] + x_at) * (rel_ms - rel);
      iy = iy_buf_[idx] + 0.5 * (cy_buf_[idx] + y_at) * (rel_ms - rel);
      return;
    }
    int oldest = index_from_newest(count_ - 1);
    ix = ix_buf_[oldest];
    iy = iy_buf_[oldest];
  }

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

  // Records the band-followed position at sample idx (its time already set) and the integral up to it, from the sample before it (-1: none)
  void store_position(int idx, int prev) {
    cx_buf_[idx] = cx_;
    cy_buf_[idx] = cy_;
    if (prev < 0) {
      ix_buf_[idx] = iy_buf_[idx] = 0.0;
      return;
    }
    double dt = (double)(std::int32_t)(t_at(idx) - t_at(prev));
    ix_buf_[idx] = ix_buf_[prev] + 0.5 * (cx_buf_[prev] + cx_) * dt;
    iy_buf_[idx] = iy_buf_[prev] + 0.5 * (cy_buf_[prev] + cy_) * dt;
  }

  void push(std::uint32_t t_ms) {
    t_[head_] = t_ms;
    cum_buf_[head_] = cum_;
    store_position(head_, count_ > 0 ? newest_index() : -1);
    head_ = (head_ + 1) % CAPACITY;
    if (count_ < CAPACITY)
      count_++;
    else
      wrapped_ = true;
  }

  std::array<std::uint32_t, CAPACITY> t_{};
  std::array<double, CAPACITY> cum_buf_{};
  // The band-followed position at each sample, and the time integral of it up to that sample (in units times ms), what net_over() reads
  std::array<double, CAPACITY> cx_buf_{}, cy_buf_{}, ix_buf_{}, iy_buf_{};
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
