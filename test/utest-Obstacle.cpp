#include <vector>

#include <catch2/catch.hpp>

#include "../lib/Obstacle.hpp"

template <typename T, typename U>
std::pair<T, U> operator+(const std::pair<T, U>& l, const std::pair<T, U>& r) {
  return {l.first + r.first, l.second + r.second};
}

template <typename T, typename U>
std::pair<T, U> operator+=(const std::pair<T, U>& l, const std::pair<T, U>& r) {
  return {l.first + r.first, l.second + r.second};
}

TEST_CASE("Null obstacles produce no potential") {
  std::vector<Obstacle> obs{NullObstacle(), NullObstacle()};
  std::pair<double, double> total = {0, 0};

  double x = 0;
  double y = 0;

  for (auto& ob : obs) total += g_phi(x, y, ob);

  REQUIRE(total == std::pair<double, double>{0, 0});
}

