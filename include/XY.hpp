#pragma once

#include <cmath>
#include <tuple>

struct XY {
  double x;
  double y;

  template <typename T>
  inline operator std::tuple<T &, T &>() {
    return std::tuple<T &, T &>{x, y};
  }
};

inline constexpr bool operator==(const XY &a, const XY &b) noexcept {
  return (a.x == b.x) && (a.y == b.y);
}

inline constexpr bool operator!=(const XY &a, const XY &b) noexcept {
  return !(a == b);
}

inline constexpr XY operator+(const XY &a, const XY &b) {
  return XY{a.x + b.x, a.y + b.y};
}

inline constexpr XY operator-(const XY &a, const XY &b) {
  return XY{a.x - b.x, a.y - b.y};
}

inline constexpr XY operator*(const double &a, const XY &b) {
  return XY{a * b.x, a * b.y};
}

inline double l2norm(const XY &p) { return std::hypot(p.x, p.y); }

inline constexpr double quadrance(const XY &p) { return p.x * p.x + p.y * p.y; }

inline XY normalise(const XY &p) {
  const double norm = l2norm(p);
  return {p.x / norm, p.y / norm};
}
