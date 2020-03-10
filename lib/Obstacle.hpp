#pragma once

#include <variant>

/* #include <boost/hana/functional/overload.hpp> */
/* using boost::hana::overload; */

struct NullObstacle {};

struct PointObstacle {
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

using Obstacle = std::variant<NullObstacle, PointObstacle>;

std::pair<double,double> g_phi(double, double, Obstacle);
