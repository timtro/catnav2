#include <vector>

#include <catch2/catch.hpp>

#include "../include/nmpc_algebra.hpp"

TEST_CASE("Forcasting the pose of a Nav2 robot starting at the origin…") {
  /* Catch::StringMaker<double>::precision = 20; */

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
  REQUIRE(cx.th[0] == 0.f);
  REQUIRE(cx.Dx[0] == 0.f);
  REQUIRE(cx.Dy[0] == 0.f);
  REQUIRE(cx.Dth[0] == 0.f);

  SECTION(
      "With no steering and velocity, the robot should remain stationary "
      "throughout the forecast horizon.") {
    for (auto& each : cx.v) each = 0.f;
    for (auto& each : cx.Dth) each = 0.f;

    auto result = dtl::compute_forecast(std::move(cx));

    REQUIRE_THAT(std::vector(result.x, result.x + N),
                 Catch::Equals<double>({0, 0, 0, 0, 0}));
    REQUIRE_THAT(std::vector(result.y, result.y + N),
                 Catch::Equals<double>({0, 0, 0, 0, 0}));
    REQUIRE_THAT(std::vector(result.th, result.th + N),
                 Catch::Equals<double>({0, 0, 0, 0, 0}));
    REQUIRE_THAT(std::vector(result.Dx, result.Dx + N),
                 Catch::Equals<double>({0, 0, 0, 0, 0}));
    REQUIRE_THAT(std::vector(result.Dy, result.Dy + N),
                 Catch::Equals<double>({0, 0, 0, 0, 0}));
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

    REQUIRE_THAT(std::vector(result.x, result.x + N),
                 Catch::Equals<double>({0, 1, 2, 3, 4}));
    REQUIRE_THAT(std::vector(result.y, result.y + N),
                 Catch::Equals<double>({0, 0, 0, 0, 0}));
    REQUIRE_THAT(std::vector(result.th, result.th + N),
                 Catch::Equals<double>({0, 0, 0, 0, 0}));
    REQUIRE_THAT(std::vector(result.Dx, result.Dx + N),
                 Catch::Equals<double>({1, 1, 1, 1, 1}));
    REQUIRE_THAT(std::vector(result.Dy, result.Dy + N),
                 Catch::Equals<double>({0, 0, 0, 0, 0}));
  }

  SECTION(
      "Drive in a square by controlling Dth. With v = 1, dt=1 and Dth = π/2, "
      "this should drive the unit-box.") {
    for (auto& each : cx.v) each = 1.f;
    for (auto& each : cx.Dth) each = M_PI / 2;
    for (auto& each : cx.th) each = 0.f;
    cx.Dx[0] = 1.f;
    cx.Dy[0] = 0.f;

    auto result = dtl::compute_forecast(std::move(cx));

    REQUIRE_THAT(std::vector(result.x, result.x + N),
                 Catch::Approx<double>({0, 1, 1, 0, 0}).margin(1e-15));
    REQUIRE_THAT(std::vector(result.y, result.y + N),
                 Catch::Approx<double>({0, 0, 1, 1, 0}).margin(1e-15));
    REQUIRE_THAT(
        std::vector(result.th, result.th + N),
        Catch::Approx<double>({0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI})
            .margin(1e-15));
    REQUIRE_THAT(std::vector(result.Dx, result.Dx + N),
                 Catch::Approx<double>({1, 0, -1, 0, 1}).margin(1e-15));
    REQUIRE_THAT(std::vector(result.Dy, result.Dy + N),
                 Catch::Approx<double>({0, 1, 0, -1, 0}).margin(1e-15));
  }
}
