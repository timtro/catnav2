#define CATCH_CONFIG_ENABLE_PAIR_STRINGMAKER

#include <limits>
#include <vector>

#include <boost/hana/functional/curry.hpp>
#include <catch2/catch.hpp>

#include "../lib/Obstacle.hpp"

using boost::hana::curry;

using XY = std::pair<double, double>;

TEST_CASE("Null obstacles produce no potential", "[NullObstacle]") {
  std::vector<Obstacle> obs{NullObstacle(), NullObstacle()};

  REQUIRE(g_phi(0, 0, NullObstacle()) == XY{0, 0});
  REQUIRE(g_phi(100, 100, NullObstacle()) == XY{0, 0});
}

TEST_CASE("Testing point obstacle at origin with hand-computed values",
          "[PointObstacle]") {
  const auto o1 = PointObstacle{0, 0, 2, 1};
  const double big = std::numeric_limits<double>::max() / 2;

  REQUIRE(g_phi(0, 0, o1) == XY{0, 0});
  REQUIRE(g_phi(1, 0, o1) == XY{0.5, 0});
  REQUIRE(g_phi(0, 1, o1) == XY{0, 0.5});
  REQUIRE(g_phi(1, -1, o1) == XY{2. / 9, -2. / 9});
  REQUIRE(g_phi(-10, 100, o1) == XY{-20. / 102030201, 200. / 102030201});
  // Vanish at infinity:
  REQUIRE(g_phi(0, big, o1) == XY{0, 0});
  REQUIRE(g_phi(big, 0, o1) == XY{0, 0});

  const auto o2 = PointObstacle{0, 0, 1, 1};

  REQUIRE((std::isnan(g_phi(0, 0, o2).first)
           && std::isnan(g_phi(0, 0, o2).second)));
  REQUIRE(g_phi(1, 0, o2) == XY{1. / 4, 0});
  REQUIRE(g_phi(0, 1, o2) == XY{0, 1. / 4});
  REQUIRE(g_phi(1, -1, o2)
          == XY{1 / (M_SQRT2 * (M_SQRT2 + 1) * (M_SQRT2 + 1)),
                -1 / (M_SQRT2 * (M_SQRT2 + 1) * (M_SQRT2 + 1))});
  // Vanish at infinity:
  REQUIRE(g_phi(0, big, o2) == XY{0, 0});
  REQUIRE(g_phi(big, 0, o2) == XY{0, 0});
}
