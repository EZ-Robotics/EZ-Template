/*
 * This code is a modified version of Benjamin Jurke's work in 2015. You can read his blog post
 * here:
 * https://benjaminjurke.com/content/articles/2015/compile-time-numerical-unit-dimension-checking/
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 *
 * Derived from OkapiLib's units implementation:
 * https://github.com/purduesigbots/OkapiLib/tree/master/include/okapi/api/units
 */
#pragma once

#include <cmath>
#include <ratio>

// EZ-Units version. EZ-Template static_asserts on EZ_UNITS_VERSION_MAJOR so a
// version mismatch between a standalone EZ-Units install and the copy vendored
// inside EZ-Template becomes a compile error instead of silent breakage.
#define EZ_UNITS_VERSION_MAJOR 1
#define EZ_UNITS_VERSION_MINOR 0

// Optional bridge to OkapiLib. If a project has OkapiLib's units headers on its
// include path (e.g. an EZ-Template project that still depends on OkapiLib
// elsewhere), ez::RQuantity and the three named types gain an implicit
// converting constructor from the matching okapi::RQuantity type. This lets a
// function taking an ez-typed parameter accept an okapi value without change.
// It does NOT extend to arithmetic operators - `ez::QLength + okapi::QLength`
// still fails to compile, see the README for details. This guard means
// EZ-Units compiles cleanly in projects with no okapi headers at all.
#if defined(__has_include)
#if __has_include("okapi/api/units/RQuantity.hpp")
#include "okapi/api/units/RQuantity.hpp"
#define EZ_UNITS_HAS_OKAPI_BRIDGE 1
#endif
#endif

namespace ez {
template <typename MassDim, typename LengthDim, typename TimeDim, typename AngleDim>
class RQuantity {
  public:
  explicit constexpr RQuantity() : value(0.0) {
  }

  explicit constexpr RQuantity(double val) : value(val) {
  }

  explicit constexpr RQuantity(long double val) : value(static_cast<double>(val)) {
  }

#ifdef EZ_UNITS_HAS_OKAPI_BRIDGE
  constexpr RQuantity(const okapi::RQuantity<MassDim, LengthDim, TimeDim, AngleDim> &val)
    : value(val.getValue()) {
  }
#endif

  // The intrinsic operations for a quantity with a unit is addition and subtraction
  constexpr RQuantity const &operator+=(const RQuantity &rhs) {
    value += rhs.value;
    return *this;
  }

  constexpr RQuantity const &operator-=(const RQuantity &rhs) {
    value -= rhs.value;
    return *this;
  }

  constexpr RQuantity operator-() {
    return RQuantity(value * -1);
  }

  constexpr RQuantity const &operator*=(const double rhs) {
    value *= rhs;
    return *this;
  }

  constexpr RQuantity const &operator/=(const double rhs) {
    value /= rhs;
    return *this;
  }

  // Returns the value of the quantity in multiples of the specified unit
  constexpr double convert(const RQuantity &rhs) const {
    return value / rhs.value;
  }

  // returns the raw value of the quantity (should not be used)
  constexpr double getValue() const {
    return value;
  }

  constexpr RQuantity<MassDim, LengthDim, TimeDim, AngleDim> abs() const {
    return RQuantity<MassDim, LengthDim, TimeDim, AngleDim>(std::fabs(value));
  }

  constexpr RQuantity<std::ratio_divide<MassDim, std::ratio<2>>,
                      std::ratio_divide<LengthDim, std::ratio<2>>,
                      std::ratio_divide<TimeDim, std::ratio<2>>,
                      std::ratio_divide<AngleDim, std::ratio<2>>>
  sqrt() const {
    return RQuantity<std::ratio_divide<MassDim, std::ratio<2>>,
                     std::ratio_divide<LengthDim, std::ratio<2>>,
                     std::ratio_divide<TimeDim, std::ratio<2>>,
                     std::ratio_divide<AngleDim, std::ratio<2>>>(std::sqrt(value));
  }

  private:
  double value;
};

// Predefined (physical unit) quantity types:
// ------------------------------------------
#define QUANTITY_TYPE(_Mdim, _Ldim, _Tdim, _Adim, name)                                            \
  typedef RQuantity<std::ratio<_Mdim>, std::ratio<_Ldim>, std::ratio<_Tdim>, std::ratio<_Adim>>    \
    name;

// Unitless
QUANTITY_TYPE(0, 0, 0, 0, Number)
constexpr Number number(1.0);

// Standard arithmetic operators:
// ------------------------------
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator+(const RQuantity<M, L, T, A> &lhs,
                                          const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(lhs.getValue() + rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator-(const RQuantity<M, L, T, A> &lhs,
                                          const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(lhs.getValue() - rhs.getValue());
}
template <typename M1,
          typename L1,
          typename T1,
          typename A1,
          typename M2,
          typename L2,
          typename T2,
          typename A2>
constexpr RQuantity<std::ratio_add<M1, M2>,
                    std::ratio_add<L1, L2>,
                    std::ratio_add<T1, T2>,
                    std::ratio_add<A1, A2>>
operator*(const RQuantity<M1, L1, T1, A1> &lhs, const RQuantity<M2, L2, T2, A2> &rhs) {
  return RQuantity<std::ratio_add<M1, M2>,
                   std::ratio_add<L1, L2>,
                   std::ratio_add<T1, T2>,
                   std::ratio_add<A1, A2>>(lhs.getValue() * rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator*(const double &lhs, const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(lhs * rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator*(const RQuantity<M, L, T, A> &lhs, const double &rhs) {
  return RQuantity<M, L, T, A>(lhs.getValue() * rhs);
}
template <typename M1,
          typename L1,
          typename T1,
          typename A1,
          typename M2,
          typename L2,
          typename T2,
          typename A2>
constexpr RQuantity<std::ratio_subtract<M1, M2>,
                    std::ratio_subtract<L1, L2>,
                    std::ratio_subtract<T1, T2>,
                    std::ratio_subtract<A1, A2>>
operator/(const RQuantity<M1, L1, T1, A1> &lhs, const RQuantity<M2, L2, T2, A2> &rhs) {
  return RQuantity<std::ratio_subtract<M1, M2>,
                   std::ratio_subtract<L1, L2>,
                   std::ratio_subtract<T1, T2>,
                   std::ratio_subtract<A1, A2>>(lhs.getValue() / rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_subtract<std::ratio<0>, M>,
                    std::ratio_subtract<std::ratio<0>, L>,
                    std::ratio_subtract<std::ratio<0>, T>,
                    std::ratio_subtract<std::ratio<0>, A>>
operator/(const double &x, const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<std::ratio_subtract<std::ratio<0>, M>,
                   std::ratio_subtract<std::ratio<0>, L>,
                   std::ratio_subtract<std::ratio<0>, T>,
                   std::ratio_subtract<std::ratio<0>, A>>(x / rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> operator/(const RQuantity<M, L, T, A> &rhs, const double &x) {
  return RQuantity<M, L, T, A>(rhs.getValue() / x);
}

// Comparison operators for quantities:
// ------------------------------------
template <typename M, typename L, typename T, typename A>
constexpr bool operator==(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() == rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator!=(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() != rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator<=(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() <= rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator>=(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() >= rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator<(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() < rhs.getValue());
}
template <typename M, typename L, typename T, typename A>
constexpr bool operator>(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return (lhs.getValue() > rhs.getValue());
}

// Common math functions:
// ------------------------------

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> abs(const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(std::abs(rhs.getValue()));
}

template <typename R, typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_multiply<M, R>,
                    std::ratio_multiply<L, R>,
                    std::ratio_multiply<T, R>,
                    std::ratio_multiply<A, R>>
pow(const RQuantity<M, L, T, A> &lhs) {
  return RQuantity<std::ratio_multiply<M, R>,
                   std::ratio_multiply<L, R>,
                   std::ratio_multiply<T, R>,
                   std::ratio_multiply<A, R>>(std::pow(lhs.getValue(), double(R::num) / R::den));
}

template <int R, typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_multiply<M, std::ratio<R>>,
                    std::ratio_multiply<L, std::ratio<R>>,
                    std::ratio_multiply<T, std::ratio<R>>,
                    std::ratio_multiply<A, std::ratio<R>>>
pow(const RQuantity<M, L, T, A> &lhs) {
  return RQuantity<std::ratio_multiply<M, std::ratio<R>>,
                   std::ratio_multiply<L, std::ratio<R>>,
                   std::ratio_multiply<T, std::ratio<R>>,
                   std::ratio_multiply<A, std::ratio<R>>>(std::pow(lhs.getValue(), R));
}

template <int R, typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_divide<M, std::ratio<R>>,
                    std::ratio_divide<L, std::ratio<R>>,
                    std::ratio_divide<T, std::ratio<R>>,
                    std::ratio_divide<A, std::ratio<R>>>
root(const RQuantity<M, L, T, A> &lhs) {
  return RQuantity<std::ratio_divide<M, std::ratio<R>>,
                   std::ratio_divide<L, std::ratio<R>>,
                   std::ratio_divide<T, std::ratio<R>>,
                   std::ratio_divide<A, std::ratio<R>>>(std::pow(lhs.getValue(), 1.0 / R));
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_divide<M, std::ratio<2>>,
                    std::ratio_divide<L, std::ratio<2>>,
                    std::ratio_divide<T, std::ratio<2>>,
                    std::ratio_divide<A, std::ratio<2>>>
sqrt(const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<std::ratio_divide<M, std::ratio<2>>,
                   std::ratio_divide<L, std::ratio<2>>,
                   std::ratio_divide<T, std::ratio<2>>,
                   std::ratio_divide<A, std::ratio<2>>>(std::sqrt(rhs.getValue()));
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_divide<M, std::ratio<3>>,
                    std::ratio_divide<L, std::ratio<3>>,
                    std::ratio_divide<T, std::ratio<3>>,
                    std::ratio_divide<A, std::ratio<3>>>
cbrt(const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<std::ratio_divide<M, std::ratio<3>>,
                   std::ratio_divide<L, std::ratio<3>>,
                   std::ratio_divide<T, std::ratio<3>>,
                   std::ratio_divide<A, std::ratio<3>>>(std::cbrt(rhs.getValue()));
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_multiply<M, std::ratio<2>>,
                    std::ratio_multiply<L, std::ratio<2>>,
                    std::ratio_multiply<T, std::ratio<2>>,
                    std::ratio_multiply<A, std::ratio<2>>>
square(const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<std::ratio_multiply<M, std::ratio<2>>,
                   std::ratio_multiply<L, std::ratio<2>>,
                   std::ratio_multiply<T, std::ratio<2>>,
                   std::ratio_multiply<A, std::ratio<2>>>(std::pow(rhs.getValue(), 2));
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio_multiply<M, std::ratio<3>>,
                    std::ratio_multiply<L, std::ratio<3>>,
                    std::ratio_multiply<T, std::ratio<3>>,
                    std::ratio_multiply<A, std::ratio<3>>>
cube(const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<std::ratio_multiply<M, std::ratio<3>>,
                   std::ratio_multiply<L, std::ratio<3>>,
                   std::ratio_multiply<T, std::ratio<3>>,
                   std::ratio_multiply<A, std::ratio<3>>>(std::pow(rhs.getValue(), 3));
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> hypot(const RQuantity<M, L, T, A> &lhs,
                                      const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(std::hypot(lhs.getValue(), rhs.getValue()));
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> mod(const RQuantity<M, L, T, A> &lhs,
                                    const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(std::fmod(lhs.getValue(), rhs.getValue()));
}

template <typename M1,
          typename L1,
          typename T1,
          typename A1,
          typename M2,
          typename L2,
          typename T2,
          typename A2>
constexpr RQuantity<M1, L1, T1, A1> copysign(const RQuantity<M1, L1, T1, A1> &lhs,
                                             const RQuantity<M2, L2, T2, A2> &rhs) {
  return RQuantity<M1, L1, T1, A1>(std::copysign(lhs.getValue(), rhs.getValue()));
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> ceil(const RQuantity<M, L, T, A> &lhs,
                                     const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(std::ceil(lhs.getValue() / rhs.getValue()) * rhs.getValue());
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> floor(const RQuantity<M, L, T, A> &lhs,
                                      const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(std::floor(lhs.getValue() / rhs.getValue()) * rhs.getValue());
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> trunc(const RQuantity<M, L, T, A> &lhs,
                                      const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(std::trunc(lhs.getValue() / rhs.getValue()) * rhs.getValue());
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<M, L, T, A> round(const RQuantity<M, L, T, A> &lhs,
                                      const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<M, L, T, A>(std::round(lhs.getValue() / rhs.getValue()) * rhs.getValue());
}

// Common trig functions:
// ------------------------------

constexpr Number
sin(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>> &rhs) {
  return Number(std::sin(rhs.getValue()));
}

constexpr Number
cos(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>> &rhs) {
  return Number(std::cos(rhs.getValue()));
}

constexpr Number
tan(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>> &rhs) {
  return Number(std::tan(rhs.getValue()));
}

constexpr RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>
asin(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>(
    std::asin(rhs.getValue()));
}

constexpr RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>
acos(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>(
    std::acos(rhs.getValue()));
}

constexpr RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>
atan(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>(
    std::atan(rhs.getValue()));
}

constexpr Number
sinh(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>> &rhs) {
  return Number(std::sinh(rhs.getValue()));
}

constexpr Number
cosh(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>> &rhs) {
  return Number(std::cosh(rhs.getValue()));
}

constexpr Number
tanh(const RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>> &rhs) {
  return Number(std::tanh(rhs.getValue()));
}

constexpr RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>
asinh(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>(
    std::asinh(rhs.getValue()));
}

constexpr RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>
acosh(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>(
    std::acosh(rhs.getValue()));
}

constexpr RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>
atanh(const Number &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>(
    std::atanh(rhs.getValue()));
}

template <typename M, typename L, typename T, typename A>
constexpr RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>
atan2(const RQuantity<M, L, T, A> &lhs, const RQuantity<M, L, T, A> &rhs) {
  return RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>>(
    std::atan2(lhs.getValue(), rhs.getValue()));
}

inline namespace literals {
constexpr long double operator"" _pi(long double x) {
  return static_cast<double>(x) * 3.1415926535897932384626433832795;
}
constexpr long double operator"" _pi(unsigned long long int x) {
  return static_cast<double>(x) * 3.1415926535897932384626433832795;
}
} // namespace literals
} // namespace ez

// Conversion macro, which utilizes the string literals
#define ConvertTo(_x, _y) (_x).convert(1.0_##_y)

namespace ez {
// Named quantity classes, deriving from RQuantity instead of a bare typedef, so
// compiler errors read "ez::QLength" instead of the fully expanded RQuantity<...>
// template. Arithmetic results (which are still typed as the base RQuantity, per
// the original operators above) convert back implicitly via the constructor
// below, so `QLength d = meter + meter;` keeps working the same as it always did.
class QLength : public RQuantity<std::ratio<0>, std::ratio<1>, std::ratio<0>, std::ratio<0>> {
  public:
  explicit constexpr QLength() : RQuantity() {
  }
  explicit constexpr QLength(double val) : RQuantity(val) {
  }
  constexpr QLength(const RQuantity &val) : RQuantity(val) {
  }
#ifdef EZ_UNITS_HAS_OKAPI_BRIDGE
  constexpr QLength(const okapi::RQuantity<std::ratio<0>, std::ratio<1>, std::ratio<0>, std::ratio<0>> &val)
    : RQuantity(val) {
  }
#endif
};

constexpr QLength meter(1.0); // SI base unit
constexpr QLength decimeter = meter / 10;
constexpr QLength centimeter = meter / 100;
constexpr QLength millimeter = meter / 1000;
constexpr QLength kilometer = 1000 * meter;
constexpr QLength inch = 2.54 * centimeter;
constexpr QLength foot = 12 * inch;
constexpr QLength yard = 3 * foot;
constexpr QLength mile = 5280 * foot;
constexpr QLength tile = 24 * inch;

inline namespace literals {
constexpr QLength operator"" _mm(long double x) {
  return static_cast<double>(x) * millimeter;
}
constexpr QLength operator"" _cm(long double x) {
  return static_cast<double>(x) * centimeter;
}
constexpr QLength operator"" _m(long double x) {
  return static_cast<double>(x) * meter;
}
constexpr QLength operator"" _km(long double x) {
  return static_cast<double>(x) * kilometer;
}
constexpr QLength operator"" _mi(long double x) {
  return static_cast<double>(x) * mile;
}
constexpr QLength operator"" _yd(long double x) {
  return static_cast<double>(x) * yard;
}
constexpr QLength operator"" _ft(long double x) {
  return static_cast<double>(x) * foot;
}
constexpr QLength operator"" _in(long double x) {
  return static_cast<double>(x) * inch;
}
constexpr QLength operator"" _tile(long double x) {
  return static_cast<double>(x) * tile;
}
constexpr QLength operator"" _mm(unsigned long long int x) {
  return static_cast<double>(x) * millimeter;
}
constexpr QLength operator"" _cm(unsigned long long int x) {
  return static_cast<double>(x) * centimeter;
}
constexpr QLength operator"" _m(unsigned long long int x) {
  return static_cast<double>(x) * meter;
}
constexpr QLength operator"" _km(unsigned long long int x) {
  return static_cast<double>(x) * kilometer;
}
constexpr QLength operator"" _mi(unsigned long long int x) {
  return static_cast<double>(x) * mile;
}
constexpr QLength operator"" _yd(unsigned long long int x) {
  return static_cast<double>(x) * yard;
}
constexpr QLength operator"" _ft(unsigned long long int x) {
  return static_cast<double>(x) * foot;
}
constexpr QLength operator"" _in(unsigned long long int x) {
  return static_cast<double>(x) * inch;
}
constexpr QLength operator"" _tile(unsigned long long int x) {
  return static_cast<double>(x) * tile;
}
} // namespace literals
} // namespace ez

namespace ez {
class QAngle : public RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>> {
  public:
  explicit constexpr QAngle() : RQuantity() {
  }
  explicit constexpr QAngle(double val) : RQuantity(val) {
  }
  constexpr QAngle(const RQuantity &val) : RQuantity(val) {
  }
#ifdef EZ_UNITS_HAS_OKAPI_BRIDGE
  constexpr QAngle(const okapi::RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<0>, std::ratio<1>> &val)
    : RQuantity(val) {
  }
#endif
};

constexpr QAngle radian(1.0);
constexpr QAngle degree = static_cast<double>(2_pi / 360.0) * radian;

inline namespace literals {
constexpr QAngle operator"" _rad(long double x) {
  return QAngle(x);
}
constexpr QAngle operator"" _rad(unsigned long long int x) {
  return QAngle(static_cast<double>(x));
}
constexpr QAngle operator"" _deg(long double x) {
  return static_cast<double>(x) * degree;
}
constexpr QAngle operator"" _deg(unsigned long long int x) {
  return static_cast<double>(x) * degree;
}
} // namespace literals
} // namespace ez

namespace ez {
class QTime : public RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>> {
  public:
  explicit constexpr QTime() : RQuantity() {
  }
  explicit constexpr QTime(double val) : RQuantity(val) {
  }
  constexpr QTime(const RQuantity &val) : RQuantity(val) {
  }
#ifdef EZ_UNITS_HAS_OKAPI_BRIDGE
  constexpr QTime(const okapi::RQuantity<std::ratio<0>, std::ratio<0>, std::ratio<1>, std::ratio<0>> &val)
    : RQuantity(val) {
  }
#endif
};

constexpr QTime second(1.0); // SI base unit
constexpr QTime millisecond = second / 1000;
constexpr QTime minute = 60 * second;
constexpr QTime hour = 60 * minute;
constexpr QTime day = 24 * hour;

inline namespace literals {
constexpr QTime operator"" _s(long double x) {
  return QTime(x);
}
constexpr QTime operator"" _ms(long double x) {
  return static_cast<double>(x) * millisecond;
}
constexpr QTime operator"" _min(long double x) {
  return static_cast<double>(x) * minute;
}
constexpr QTime operator"" _h(long double x) {
  return static_cast<double>(x) * hour;
}
constexpr QTime operator"" _day(long double x) {
  return static_cast<double>(x) * day;
}
constexpr QTime operator"" _s(unsigned long long int x) {
  return QTime(static_cast<double>(x));
}
constexpr QTime operator"" _ms(unsigned long long int x) {
  return static_cast<double>(x) * millisecond;
}
constexpr QTime operator"" _min(unsigned long long int x) {
  return static_cast<double>(x) * minute;
}
constexpr QTime operator"" _h(unsigned long long int x) {
  return static_cast<double>(x) * hour;
}
constexpr QTime operator"" _day(unsigned long long int x) {
  return static_cast<double>(x) * day;
}
} // namespace literals
} // namespace ez
