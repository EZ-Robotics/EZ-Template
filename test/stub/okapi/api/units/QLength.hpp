// Minimal stand-in for okapi's QLength. Real okapi is a full dimensional-
// analysis unit system (RQuantity<...>) with meters as the SI canonical unit;
// EZ-Template only ever constructs a QLength from a literal and immediately
// calls .convert(okapi::inch) on it (or, in tests, the reverse), so this
// stores the value in inches directly rather than replicating okapi's meter-
// based internals -- convert() against any other family of unit was never
// going to be meaningful here anyway, since EZ-Template never asks for one.
#pragma once

namespace okapi {

class QLength {
 public:
  constexpr QLength() : value_(0.0) {}
  constexpr explicit QLength(double value) : value_(value) {}

  constexpr double convert(QLength unit) const { return value_ / unit.value_; }
  constexpr QLength operator*(double scalar) const { return QLength(value_ * scalar); }
  constexpr QLength operator+(QLength other) const { return QLength(value_ + other.value_); }
  constexpr QLength operator-(QLength other) const { return QLength(value_ - other.value_); }
  constexpr bool operator==(QLength other) const { return value_ == other.value_; }
  constexpr bool operator<(QLength other) const { return value_ < other.value_; }
  constexpr bool operator>(QLength other) const { return value_ > other.value_; }

 private:
  double value_;
};

constexpr QLength inch{1.0};
constexpr QLength foot{12.0};

namespace literals {
constexpr QLength operator""_in(long double v) { return QLength(static_cast<double>(v)); }
constexpr QLength operator""_in(unsigned long long v) { return QLength(static_cast<double>(v)); }
}  // namespace literals

}  // namespace okapi
