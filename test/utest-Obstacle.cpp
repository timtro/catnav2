#include <limits>
#include <vector>

#include <catch2/catch.hpp>

#include "../include/list-fmap.hpp"
#include "../lib/Obstacle.hpp"

std::ostream& operator<<(std::ostream& os, XY const& p) {
  os << "( " << p.x << ", " << p.y << " )";
  return os;
}

const double big = std::numeric_limits<double>::max() / 2;

TEST_CASE("Null obstacles produce no potential", "[ob::Null]") {
  REQUIRE(ob::g_phi(XY{0, 0}, ob::Null()) == XY{0, 0});
  REQUIRE(ob::g_phi(XY{100, 100}, ob::Null()) == XY{0, 0});
}

TEST_CASE("Testing point obstacles at origin with hand-computed values",
          "[ob::Point]") {
  constexpr auto o1 = ob::Point{{0, 0}, 2, 1};
  constexpr auto g_o1 = [o1](XY p) { return ob::g_phi(p, o1); };

  REQUIRE(g_o1({0, 0}) == XY{0, 0});
  REQUIRE(g_o1({1, 0}) == XY{-0.5, 0});
  REQUIRE(g_o1({0, 1}) == XY{0, -0.5});
  REQUIRE(g_o1({1, -1}) == XY{-2. / 9, 2. / 9});
  REQUIRE(g_o1({-10, 100}) == XY{20. / 102030201, -200. / 102030201});
  // // Vanish at infinity:
  REQUIRE(g_o1({0, big}) == XY{0, 0});
  REQUIRE(g_o1({big, 0}) == XY{0, 0});

  constexpr auto o2 = ob::Point{{0, 0}, 1, 1};
  constexpr auto g_o2 = [o2](XY p) { return ob::g_phi(p, o2); };

  // By quirk of the math, with exponent 1, ob::g_phi is undef at the origin.
  REQUIRE((std::isnan(g_o2({0, 0}).x) && std::isnan(g_o2({0, 0}).y)));

  REQUIRE(g_o2({1, 0}) == XY{-1. / 4, 0});
  REQUIRE(g_o2({0, 1}) == XY{0, -1. / 4});
  REQUIRE(g_o2({-1, 1})
          == XY{1 / (M_SQRT2 * (M_SQRT2 + 1) * (M_SQRT2 + 1)),
                -1 / (M_SQRT2 * (M_SQRT2 + 1) * (M_SQRT2 + 1))});
  // // Vanish at infinity:
  REQUIRE(g_o2({0, big}) == XY{0, 0});
  REQUIRE(g_o2({big, 0}) == XY{0, 0});

  constexpr auto o3 = ob::Point{{0, 0}, 2, 2};
  constexpr auto g_o3 = [o3](XY p) { return ob::g_phi(p, o3); };

  REQUIRE(g_o3({0, 0}) == XY{0, 0});
  REQUIRE(g_o3({1, 0}) == XY{-2. / 9, 0});
  REQUIRE(g_o3({0, 1}) == XY{0, -2. / 9});
  REQUIRE(g_o3({1, -1}) == XY{-1. / 8, 1. / 8});
  // // Vanish at infinity:
  REQUIRE(g_o3({0, big}) == XY{0, 0});
  REQUIRE(g_o3({big, 0}) == XY{0, 0});
}

TEST_CASE("Testing point obstacles off-origin", "[ob::Point]") {
  constexpr auto o1 = ob::Point{{1, 1}, 2, 1};
  constexpr auto g_o1 = [o1](XY p) { return ob::g_phi(p, o1); };

  REQUIRE(g_o1({1, 1}) == XY{0, 0});
  REQUIRE(g_o1({2, 1}) == XY{-0.5, 0.});
  REQUIRE(g_o1({1, 2}) == XY{0, -0.5});
  REQUIRE(g_o1({2, 0}) == XY{-2. / 9, 2. / 9});
  REQUIRE(g_o1({-9, 101}) == XY{20. / 102030201, -200. / 102030201});
}

TEST_CASE(
    "Working in aggregates, summing over path with both Null and Point "
    "obstacles together.",
    "[ob::Null][ob::Point]") {
  const std::vector<ob::Obstacle> obs{ob::Point{{0, 0}, 2, 1}, ob::Null{},
                                      ob::Point{{0, 0}, 2, 1}, ob::Null{}};

  const XY result = foldl(ob::g_phi_accuml({1, -1}), XY{0, 0}, obs);

  REQUIRE(result == XY{-4. / 9, 4. / 9});
}
