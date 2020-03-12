#define CATCH_CONFIG_ENABLE_PAIR_STRINGMAKER

#include <limits>
#include <list>
#include <vector>

#include <boost/hana/functional/curry.hpp>
#include <catch2/catch.hpp>

#include "../include/list-fmap.hpp"
#include "../lib/Obstacle.hpp"

using boost::hana::curry;

using XY = std::pair<double, double>;

const double big = std::numeric_limits<double>::max() / 2;

TEST_CASE("Null obstacles produce no potential", "[NullObstacle]") {
  std::vector<Obstacle> obs{NullObstacle{}, NullObstacle{}};

  REQUIRE(g_phi(0, 0, NullObstacle()) == XY{0, 0});
  REQUIRE(g_phi(100, 100, NullObstacle()) == XY{0, 0});
}

TEST_CASE("Testing point obstacles at origin with hand-computed values",
          "[PointObstacle]") {
  constexpr auto o1 = PointObstacle{0, 0, 2, 1};
  constexpr auto g_o1 = [o1](double x, double y) { return g_phi(x, y, o1); };

  REQUIRE(g_o1(0, 0) == XY{0, 0});
  REQUIRE(g_o1(1, 0) == XY{0.5, 0});
  REQUIRE(g_o1(0, 1) == XY{0, 0.5});
  REQUIRE(g_o1(1, -1) == XY{2. / 9, -2. / 9});
  REQUIRE(g_o1(-10, 100) == XY{-20. / 102030201, 200. / 102030201});
  // Vanish at infinity:
  REQUIRE(g_o1(0, big) == XY{0, 0});
  REQUIRE(g_o1(big, 0) == XY{0, 0});

  constexpr auto o2 = PointObstacle{0, 0, 1, 1};
  constexpr auto g_o2 = [o2](double x, double y) { return g_phi(x, y, o2); };

  // By quirk of the math, with exponent 1, g_phi is undef at the origin.
  REQUIRE((std::isnan(g_o2(0, 0).first) && std::isnan(g_o2(0, 0).second)));

  REQUIRE(g_o2(1, 0) == XY{1. / 4, 0});
  REQUIRE(g_o2(0, 1) == XY{0, 1. / 4});
  REQUIRE(g_o2(1, -1)
          == XY{1 / (M_SQRT2 * (M_SQRT2 + 1) * (M_SQRT2 + 1)),
                -1 / (M_SQRT2 * (M_SQRT2 + 1) * (M_SQRT2 + 1))});
  // Vanish at infinity:
  REQUIRE(g_o2(0, big) == XY{0, 0});
  REQUIRE(g_o2(big, 0) == XY{0, 0});

  constexpr auto o3 = PointObstacle{0, 0, 2, 2};
  constexpr auto g_o3 = [o3](double x, double y) { return g_phi(x, y, o3); };

  REQUIRE(g_o3(0, 0) == XY{0, 0});
  REQUIRE(g_o3(1, 0) == XY{2. / 9, 0});
  REQUIRE(g_o3(0, 1) == XY{0, 2. / 9});
  REQUIRE(g_o3(1, -1) == XY{1. / 8, -1. / 8});
  // Vanish at infinity:
  REQUIRE(g_o3(0, big) == XY{0, 0});
  REQUIRE(g_o3(big, 0) == XY{0, 0});
}

TEST_CASE("Testing point obstacles off-origin", "[PointObstacle]") {
  constexpr auto o1 = PointObstacle{1, 1, 2, 1};
  constexpr auto g_o1 = [o1](double x, double y) { return g_phi(x, y, o1); };

  REQUIRE(g_o1(1, 1) == XY{0, 0});
  REQUIRE(g_o1(2, 1) == XY{0.5, 0});
  REQUIRE(g_o1(1, 2) == XY{0, 0.5});
  REQUIRE(g_o1(2, 0) == XY{2. / 9, -2. / 9});
  REQUIRE(g_o1(-9, 101) == XY{-20. / 102030201, 200. / 102030201});
}

TEST_CASE(
    "Working in aggregates, summing over path with both Null and Point "
    "obstacles together.",
    "[NullObstacle][PointObstacle]") {
  const std::list<Obstacle> obs{PointObstacle{0, 0, 2, 1}, NullObstacle{},
                                PointObstacle{0, 0, 2, 1}, NullObstacle{}};

  constexpr auto fl = [](XY accum, const Obstacle& o) -> XY {
    XY xy = g_phi(1, 0, o);
    return {accum.first + xy.first, accum.second + xy.second};
  };

  const XY result = foldl(fl, XY{0, 0}, obs);

  REQUIRE(result == XY{1, 0});
}
