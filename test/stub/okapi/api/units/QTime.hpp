// Minimal stand-in for okapi's QTime. See QLength.hpp for why this stores
// its value directly in milliseconds rather than replicating okapi's second-
// based internals.
#pragma once

namespace okapi {

class QTime {
 public:
  constexpr QTime() : value_(0.0) {}
  constexpr explicit QTime(double value) : value_(value) {}

  constexpr double convert(QTime unit) const { return value_ / unit.value_; }
  constexpr QTime operator*(double scalar) const { return QTime(value_ * scalar); }
  constexpr QTime operator+(QTime other) const { return QTime(value_ + other.value_); }
  constexpr QTime operator-(QTime other) const { return QTime(value_ - other.value_); }
  constexpr bool operator==(QTime other) const { return value_ == other.value_; }
  constexpr bool operator<(QTime other) const { return value_ < other.value_; }
  constexpr bool operator>(QTime other) const { return value_ > other.value_; }

 private:
  double value_;
};

constexpr QTime millisecond{1.0};
constexpr QTime second{1000.0};

namespace literals {
constexpr QTime operator""_ms(long double v) { return QTime(static_cast<double>(v)); }
constexpr QTime operator""_ms(unsigned long long v) { return QTime(static_cast<double>(v)); }
}  // namespace literals

}  // namespace okapi
