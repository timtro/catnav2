#pragma once

#include <variant>
#include <string>

#include "../include/XY.hpp"

namespace ob {

  struct Null {};

  struct Point {
    /*
     *           1
     *  φ(𝑟) = ────── , 𝑟 = √(𝑥² + 𝑦²)
     *          ρ
     *         𝑟  + ε
     */
    XY position;
    double pwr;      // pwr ≔ ρ
    double epsilon;  // epsilon ≔ ε
  };

  using Obstacle = std::variant<ob::Null, ob::Point>;

  XY g_phi(XY, const ob::Obstacle);

  std::string to_json(const Obstacle);

  constexpr auto g_phi_accuml(XY p) {
    return
        [p](XY accum, const Obstacle o) -> XY { return accum + g_phi(p, o); };
  }

}  // namespace ob
