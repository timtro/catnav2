#include <catch2/catch.hpp>
#include <cmath>
#include <vector>

#include "../include/nmpc_algebra.hpp"

using namespace std::chrono_literals;

template <typename T, std::size_t N>
constexpr auto as_vector(T (&x)[N]) {
  return std::vector(x, x + N);
}
template <typename T, std::size_t N>
constexpr auto as_vector(std::array<T, N> x) {
  return std::vector(x.data(), x.data() + N);
}

template <typename T, std::size_t N>
constexpr void set_array(T (&x)[N], std::array<T, N>&& list) {
  assert(list.size() == N);
  for (std::size_t i = 0; i < N; ++i) x[i] = list[i];
}

TEST_CASE("Check for zero-initialisation", "[NMPCState]") {
  NMPCState<5> c;
  c.obstacles.push_back(ob::Point{{0, 0}, 2, 1});
  c.obstacles.push_back(ob::Null{});

  REQUIRE(util::to_json(c) == "{\n"
  "\"time\": 0,\n"
  "\"R\": 0.000000,\n"
  "\"Q\": 0.000000,\n"
  "\"Q₀\": 0.000000,\n"
  "\"dt\": 0.333333,\n"
  "\"x\": [0.000000,0.000000,0.000000,0.000000,0.000000],\n"
  "\"y\": [0.000000,0.000000,0.000000,0.000000,0.000000],\n"
  "\"th\": [0.000000,0.000000,0.000000,0.000000,0.000000],\n"
  "\"Dx\": [0.000000,0.000000,0.000000,0.000000,0.000000],\n"
  "\"Dy\": [0.000000,0.000000,0.000000,0.000000,0.000000],\n"
  "\"Dth\": [0.000000,0.000000,0.000000,0.000000],\n"
  "\"v\": [0.000000,0.000000,0.000000,0.000000],\n"
  "\"xref\": [0.000000,0.000000,0.000000,0.000000],\n"
  "\"yref\": [0.000000,0.000000,0.000000,0.000000],\n"
  "\"ex\": [0.000000,0.000000,0.000000,0.000000],\n"
  "\"ey\": [0.000000,0.000000,0.000000,0.000000],\n"
  "\"obstacles\": [{\"type\": \"Point\", \"x\": 0.000000, \"y\": 0.000000, \"pwr\": 2.000000, \"epsilon\": 1.000000}, {\"type\": \"Null\"}]\n}");
}
