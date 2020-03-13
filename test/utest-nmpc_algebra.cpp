#include <vector>

#include <catch2/catch.hpp>

#include "../include/nmpc_algebra.hpp"

template <typename T, std::size_t N>
constexpr auto as_vector(T (&x)[N]) {
  return std::vector(x, x + N);
}

template <typename T, std::size_t N>
constexpr void set_array(T (&x)[N], std::array<T, N>&& list) {
  assert(list.size() == N);
  for (std::size_t i = 0; i < N; ++i) x[i] = list[i];
}

TEST_CASE("Use 'iterate_while' to make a toy factorial function.",
          "[dtl][iterate_while]") {
  constexpr auto minus_one_and_times = [](std::pair<int, int> p) {
    return (p.first > 1) ? std::pair{p.first - 1, p.first * p.second}
                         : std::pair{1, p.second};
  };

  constexpr auto first_gt_1 = [](const std::pair<int, int>& p) {
    return p.first > 1;
  };

  constexpr auto factorial = [minus_one_and_times, first_gt_1](int i) {
    return dtl::iterate_while(minus_one_and_times, first_gt_1, std::pair{i, 1})
        .second;
  };

  std::vector<int> ints{0, 1, 2, 3, 4, 5, 6};
  std::vector<int> expected{1, 1, 2, 6, 24, 120, 720};
  std::vector<int> ans;

  for (auto& each : ints) ans.push_back(factorial(each));

  REQUIRE(ans == expected);
}

TEST_CASE(
    "Forcasting the pose and tracking error of a Nav2 robot starting at the "
    "origin.",
    "[dtl][compute_forecast]") {
  constexpr std::size_t N = 5;

  NMPCState<N> c;
  for (auto& each : c.dt) each = 1s;
  c.x[0] = 0;
  c.y[0] = 0;
  c.th[0] = 0;
  c.Dx[0] = 0;
  c.Dy[0] = 0;
  c.Dth[0] = 0;

  SECTION(
      "With no steering and velocity, the robot should remain stationary "
      "throughout the forecast horizon. With tracking reference at the origin, "
      "there should be 0 tracking error.") {
    for (auto& each : c.v) each = 0;
    for (auto& each : c.Dth) each = 0;

    for (auto& each : c.xref) each = 0;
    for (auto& each : c.yref) each = 0;

    auto result = dtl::compute_forecast(std::move(c));

    REQUIRE(as_vector(result.x) == std::vector<double>{0, 0, 0, 0, 0});
    REQUIRE(as_vector(result.y) == std::vector<double>{0, 0, 0, 0, 0});
    REQUIRE(as_vector(result.th) == std::vector<double>{0, 0, 0, 0, 0});
    REQUIRE(as_vector(result.Dx) == std::vector<double>{0, 0, 0, 0, 0});
    REQUIRE(as_vector(result.Dy) == std::vector<double>{0, 0, 0, 0, 0});

    REQUIRE(as_vector(result.ex) == std::vector<double>{0, 0, 0, 0});
    REQUIRE(as_vector(result.ey) == std::vector<double>{0, 0, 0, 0});
  }

  SECTION(
      "With no steering and constant speed, pointed in the 𝑥-direction, the "
      "robot should follow along the 𝑥-axis.") {
    for (auto& each : c.v) each = 1.f;
    for (auto& each : c.Dth) each = 0.f;
    for (auto& each : c.th) each = 0.f;
    c.Dx[0] = 1.f;
    c.Dy[0] = 0.f;

    set_array(c.xref, {1, 2, 3, 4});
    set_array(c.yref, {0, 0, 0, 0});

    auto result = dtl::compute_forecast(std::move(c));

    REQUIRE(as_vector(result.x) == std::vector<double>{0, 1, 2, 3, 4});
    REQUIRE(as_vector(result.y) == std::vector<double>{0, 0, 0, 0, 0});
    REQUIRE(as_vector(result.th) == std::vector<double>{0, 0, 0, 0, 0});
    REQUIRE(as_vector(result.Dx) == std::vector<double>{1, 1, 1, 1, 1});
    REQUIRE(as_vector(result.Dy) == std::vector<double>{0, 0, 0, 0, 0});

    REQUIRE(as_vector(result.ex) == std::vector<double>{0, 0, 0, 0});
    REQUIRE(as_vector(result.ey) == std::vector<double>{0, 0, 0, 0});
  }

  SECTION(
      "Drive in a square by controlling Dth. With v = 1, dt=1 and Dth = π/2, "
      "this should drive the unit-box with no tracking error.") {
    for (auto& each : c.v) each = 1.f;
    for (auto& each : c.Dth) each = M_PI_2l;
    for (auto& each : c.th) each = 0.f;
    c.Dx[0] = 1.f;
    c.Dy[0] = 0.f;

    set_array(c.xref, {1, 1, 0, 0});
    set_array(c.yref, {0, 1, 1, 0});

    NMPCState<N> result = dtl::compute_forecast(std::move(c));

    REQUIRE(as_vector(result.th)
            == std::vector<double>{0, M_PI_2, M_PI, 3 * M_PI_2, 2 * M_PI});
    REQUIRE_THAT(as_vector(result.x),
                 Catch::Approx<double>({0, 1, 1, 0, 0}).margin(1e-15));
    REQUIRE_THAT(as_vector(result.y),
                 Catch::Approx<double>({0, 0, 1, 1, 0}).margin(1e-15));
    REQUIRE_THAT(as_vector(result.Dx),
                 Catch::Approx<double>({1, 0, -1, 0, 1}).margin(1e-15));
    REQUIRE_THAT(as_vector(result.Dy),
                 Catch::Approx<double>({0, 1, 0, -1, 0}).margin(1e-15));

    REQUIRE_THAT(as_vector(result.ex),
                 Catch::Approx<double>({0, 0, 0, 0}).margin(1e-15));
    REQUIRE_THAT(as_vector(result.ey),
                 Catch::Approx<double>({0, 0, 0, 0}).margin(1e-15));
  }

  SECTION(
      "If we drive along 𝑦 = 𝑥 in the direction of increasing 𝑥 and 𝑦, but "
      "tracking reference expects the opposite, we should get tracking "
      "error vectors double of the position vectors") {
    for (auto& each : c.v) each = 1.f;
    for (auto& each : c.Dth) each = 0;
    for (auto& each : c.th) each = M_PI_4;
    c.Dx[0] = M_SQRT1_2;
    c.Dy[0] = M_SQRT1_2;

    set_array(c.xref,
              {-M_SQRT1_2, -2 * M_SQRT1_2, -3 * M_SQRT1_2, -4 * M_SQRT1_2});
    set_array(c.yref,
              {-M_SQRT1_2, -2 * M_SQRT1_2, -3 * M_SQRT1_2, -4 * M_SQRT1_2});

    auto result = dtl::compute_forecast(std::move(c));

    REQUIRE_THAT(as_vector(result.ex),
                 Catch::Approx<double>({2 * M_SQRT1_2, 4 * M_SQRT1_2,
                                        6 * M_SQRT1_2, 8 * M_SQRT1_2})
                     .margin(1e-25));
    REQUIRE_THAT(as_vector(result.ey),
                 Catch::Approx<double>({2 * M_SQRT1_2, 4 * M_SQRT1_2,
                                        6 * M_SQRT1_2, 8 * M_SQRT1_2})
                     .margin(1e-25));
  }
}

TEST_CASE(
    "Reherse some of the hand-calculations in the Object unit tests, this time "
    "checking that we fold properly over a trajectory of 𝑥𝑦-coordinates.",
    "[dtl][compute_forecast][potential_gradient]") {
  NMPCState<5> c;
  c.x[0] = 0;
  c.y[0] = 0;
  c.th[0] = M_PI_4;
  for (auto& each : c.dt) each = 1s;
  set_array(c.v, {M_SQRT2, 1, 1, 1});
  set_array(c.Dth, {0, -3 * M_PI_4, 0, 0});
  //                ↑
  //                We don't compute potential at the fixed “current” position.
  //
  // This combination of v and Dth should walk to corners of a triangle:
  //    (0,0) → (0,0) → (1,1) → (1,0) → (1,-1)
  //
  //            │  • (1,1)
  //            │ ↗↓
  //      (0,0)—•——•—(1,0)
  //            │  ↓
  //            │  • (1,-1)
  //
  // We'll check this later.

  c.obstacles.push_back(ob::Point{0, 0, 2, 1});
  c.obstacles.push_back(ob::Null{});
  c.obstacles.push_back(ob::Point{0, 0, 2, 1});
  c.obstacles.push_back(ob::Null{});

  const NMPCState<5> result = compute_forecast(c);

  CHECK_THAT(as_vector(result.x),
             Catch::Approx<double>({0, 0, 1, 1, 1}).margin(1e-25));
  CHECK_THAT(as_vector(result.y),
             Catch::Approx<double>({0, 0, 1, 0, -1}).margin(1e-25));
  CHECK_THAT(as_vector(result.th),
             Catch::Approx<double>({M_PI_4, M_PI_4, -M_PI_2, -M_PI_2, -M_PI_2})
                 .margin(1e-25));
  REQUIRE_THAT(as_vector(result.DPhiX),
               Catch::Approx<double>({0, 4. / 9, 1, 4. / 9}).margin(1e-25));
  REQUIRE_THAT(as_vector(result.DPhiY),
               Catch::Approx<double>({0, 4. / 9, 0, -4. / 9}).margin(1e-25));
}
