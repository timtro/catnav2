#pragma once

#include <variant>

/* #include <boost/hana/functional/overload.hpp> */
/* using boost::hana::overload; */

namespace ob {

  struct Null {};

  struct Point {
    /*
     *           1
     *  φ(𝑟) = ------ , 𝑟 = √(𝑥² + 𝑦²)
     *          ρ
     *         𝑟  + ε
     */
    double x;
    double y;
    double pwr;      // pwr ≔ ρ
    double epsilon;  // epsilon ≔ ε
  };

  using Obstacle = std::variant<ob::Null, ob::Point>;

  std::pair<double, double> g_phi(double, double, const ob::Obstacle&);

  constexpr auto g_phi_accuml(double x, double y) {
    return [x, y](std::pair<double, double>& accum,
                  const Obstacle& o) -> std::pair<double, double> {
      std::pair<double, double> xy = g_phi(x, y, o);
      return {accum.first + xy.first, accum.second + xy.second};
    };
  }

}  // namespace ob
