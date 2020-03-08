#pragma once

#include <variant>

#include <boost/hana/functional/overload.hpp>

using boost::hana::overload;

struct NullObstacle {};

struct PointObstacle {
  double x;
  double y;
  double pwr;
  double epsilon;
};

using Obstacle = std::variant<NullObstacle, PointObstacle>;
