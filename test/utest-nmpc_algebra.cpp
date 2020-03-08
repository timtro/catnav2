#include <catch2/catch.hpp>

#include "../include/nmpc_algebra.hpp"

TEST_CASE("Forcasting the pose of a Nav2 robot starting at the origin…") {
  constexpr std::size_t N = 5;

  NMPCState<N> cx;
  for (auto& each : cx.dt) each = 1s;
  cx.x[0] = 0.f;
  cx.y[0] = 0.f;
  cx.th[0] = 0.f;
  cx.Dx[0] = 0.f;
  cx.Dy[0] = 0.f;
  cx.Dth[0] = 0.f;

  REQUIRE(cx.x[0] == 0.f);
  REQUIRE(cx.y[0] == 0.f);
  REQUIRE(cx.v[N - 2] == 0.f);

  SECTION(
      "With no steering and velocity, the robot should remain stationary "
      "throughout the forecast horizon.") {
    for (auto& each : cx.v) each = 0.f;
    for (auto& each : cx.Dth) each = 0.f;

    auto result = dtl::compute_forecast(std::move(cx));

    REQUIRE_THAT(std::vector<double>(result.x, result.x + N),
                 Catch::Equals<double>({0.f, 0.f, 0.f, 0.f, 0.f}));
    REQUIRE_THAT(std::vector<double>(result.y, result.y + N),
                 Catch::Equals<double>({0.f, 0.f, 0.f, 0.f, 0.f}));
    REQUIRE_THAT(std::vector<double>(result.th, result.th + N),
                 Catch::Equals<double>({0.f, 0.f, 0.f, 0.f, 0.f}));
    REQUIRE_THAT(std::vector<double>(result.Dx, result.Dx + N),
                 Catch::Equals<double>({0.f, 0.f, 0.f, 0.f, 0.f}));
    REQUIRE_THAT(std::vector<double>(result.Dy, result.Dy + N),
                 Catch::Equals<double>({0.f, 0.f, 0.f, 0.f, 0.f}));
  }

  SECTION(
      "With no steering and constant speed, pointed in the x-direction, the "
      "robot should predictably follow along the axis.") {
    for (auto& each : cx.v) each = 1.f;
    for (auto& each : cx.Dth) each = 0.f;
    for (auto& each : cx.th) each = 0.f;
    cx.Dx[0] = 1.f;
    cx.Dy[0] = 0.f;

    auto result = dtl::compute_forecast(std::move(cx));

    REQUIRE_THAT(std::vector<double>(result.x, result.x + N),
                 Catch::Equals<double>({0.f, 1.f, 2.f, 3.f, 4.f}));
    REQUIRE_THAT(std::vector<double>(result.y, result.y + N),
                 Catch::Equals<double>({0.f, 0.f, 0.f, 0.f, 0.f}));
    REQUIRE_THAT(std::vector<double>(result.th, result.th + N),
                 Catch::Equals<double>({0.f, 0.f, 0.f, 0.f, 0.f}));
    REQUIRE_THAT(std::vector<double>(result.Dx, result.Dx + N),
                 Catch::Equals<double>({1.f, 1.f, 1.f, 1.f, 1.f}));
    REQUIRE_THAT(std::vector<double>(result.Dy, result.Dy + N),
                 Catch::Equals<double>({0.f, 0.f, 0.f, 0.f, 0.f}));
  }
}
