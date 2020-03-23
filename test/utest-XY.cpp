#include <cmath>
#include <iostream>
#include <vector>

#include <catch2/catch.hpp>

#include "../include/XY.hpp"

std::ostream& operator<<(std::ostream& os, XY const& p) {
  os << "( " << p.x << ", " << p.y << " )";
  return os;
}

std::vector<double> as_vector(XY p) { return std::vector{p.x, p.y}; }

TEST_CASE("Test points for non/equality.") {
  XY p1{0, 0}, p2{0, 0};
  REQUIRE(p1 == p2);
  REQUIRE(XY{4, 4} == XY{4, 4});
  REQUIRE(XY{4, 5} != XY{4, 4});
}

TEST_CASE("Test componentwise addition.") {
  XY p1{4., 4.}, p2{1, 1};
  REQUIRE(p1 + p2 == XY{5., 5.});
  REQUIRE(XY{3, -3} + XY{-3, 3} == XY{0, 0});
}

TEST_CASE("Test componentwise subtraction.") {
  REQUIRE(XY{0, 0} - XY{0, 0} == XY{0, 0});
  REQUIRE(XY{0, 0} - XY{-3, 3} == XY{3, -3});
}

TEST_CASE("Test that the l2norm gives expected results.") {
  CHECK(l2norm(XY{0, 0}) == 0);
  CHECK(l2norm(XY{1, 1}) == M_SQRT2);
  CHECK(l2norm(XY{1, 1}) == M_SQRT2);
}

TEST_CASE("Test that quandrance() gives expected results.") {
  CHECK(quadrance(XY{0, 0}) == 0);
  CHECK(quadrance(XY{1, 1}) == 2);
}

TEST_CASE("Should convert to a tuple of references to double for std::tie") {
  double x = 0, y = 0;
  std::tie(x, y) = XY{-5, 10};
  REQUIRE(x == -5);
  REQUIRE(y == 10);
}

TEST_CASE(
    "Function normalise should produce expected unit-sized vectors, except "
    "when it shouldn't.") {

  XY badness = normalise({0, 0});
  REQUIRE((std::isnan(badness.x) && std::isnan(badness.y)));

  REQUIRE_THAT(as_vector(normalise({2, 2})),
               Catch::Approx<double>({M_SQRT1_2, M_SQRT1_2}));
  REQUIRE_THAT(as_vector(normalise({0.1, 0.1})),
               Catch::Approx<double>({M_SQRT1_2, M_SQRT1_2}));

  REQUIRE(normalise({2, 2}) == normalise({0.1, 0.1}));

  REQUIRE_THAT(as_vector(normalise({-5, 0})),
               Catch::Approx<double>({-1, 0}));

  REQUIRE_THAT(as_vector(normalise({0, 7})),
               Catch::Approx<double>({0, 1}));
}
