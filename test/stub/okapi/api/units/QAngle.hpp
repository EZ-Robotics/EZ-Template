// Minimal stand-in for okapi's QAngle. See QLength.hpp for why this stores
// its value directly in degrees rather than replicating okapi's radian-based
// internals.
#pragma once

namespace okapi {

class QAngle {
 public:
  constexpr QAngle() : value_(0.0) {}
  constexpr explicit QAngle(double value) : value_(value) {}

  constexpr double convert(QAngle unit) const { return value_ / unit.value_; }
  constexpr QAngle operator*(double scalar) const { return QAngle(value_ * scalar); }
  constexpr QAngle operator+(QAngle other) const { return QAngle(value_ + other.value_); }
  constexpr QAngle operator-(QAngle other) const { return QAngle(value_ - other.value_); }
  constexpr bool operator==(QAngle other) const { return value_ == other.value_; }
  constexpr bool operator<(QAngle other) const { return value_ < other.value_; }
  constexpr bool operator>(QAngle other) const { return value_ > other.value_; }

 private:
  double value_;
};

constexpr QAngle degree{1.0};
constexpr QAngle radian{57.29577951308232};

namespace literals {
constexpr QAngle operator""_deg(long double v) { return QAngle(static_cast<double>(v)); }
constexpr QAngle operator""_deg(unsigned long long v) { return QAngle(static_cast<double>(v)); }
}  // namespace literals

}  // namespace okapi
