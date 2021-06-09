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

  REQUIRE(c.dt.count() == 1. / 3);
  REQUIRE(as_vector(c.x) == std::vector<double>{0, 0, 0, 0, 0});
  REQUIRE(as_vector(c.y) == std::vector<double>{0, 0, 0, 0, 0});
  REQUIRE(as_vector(c.th) == std::vector<double>{0, 0, 0, 0, 0});
  REQUIRE(as_vector(c.Dx) == std::vector<double>{0, 0, 0, 0, 0});
  REQUIRE(as_vector(c.Dy) == std::vector<double>{0, 0, 0, 0, 0});
  REQUIRE(as_vector(c.Dth) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.v) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.xref) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.yref) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.ex) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.ey) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.DPhiX) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.DPhiY) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.px) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.py) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.pDx) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.pDy) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.pth) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(c.grad) == std::vector<double>{0, 0, 0, 0});
}

TEST_CASE("Use 'iterate_while' to make a toy factorial function.",
          "[dtl][iterate_while]") {
  constexpr auto minus_one_and_times = [](std::pair<int, int> p) noexcept {
    return (p.first > 1) ? std::pair{p.first - 1, p.first * p.second}
                         : std::pair{1, p.second};
  };

  constexpr auto first_gt_1 = [](const std::pair<int, int>& p) noexcept {
    return p.first > 1;
  };

  constexpr auto factorial = [minus_one_and_times, first_gt_1](int i) {
    return dtl::iterate_while(minus_one_and_times, first_gt_1, std::pair{i, 1})
        .second;
  };

  const std::vector<int> ints{0, 1, 2, 3, 4, 5, 6};
  const std::vector<int> expected{1, 1, 2, 6, 24, 120, 720};

  std::vector<int> ans = fmap(factorial, ints);

  REQUIRE(ans == expected);
}

TEST_CASE(
    "Forcasting the pose and tracking error of a Nav2 robot starting at the "
    "origin.",
    "[dtl][forecast]") {
  const NMPCState<5> base = [] {
    NMPCState<5> base;
    base.dt = 1s;
    return base;
  }();

  SECTION(
      "With no steering and velocity, the robot should remain stationary "
      "throughout the forecast horizon. With tracking reference at the origin, "
      "there should be 0 tracking error.") {
    const auto c = [base] {
      auto c = base;
      // These should be zero anyway, but let's emphasize it:
      for (auto& each : c.v) each = 0;
      for (auto& each : c.Dth) each = 0;
      for (auto& each : c.xref) each = 0;
      for (auto& each : c.yref) each = 0;
      return c;
    }();

    auto result = dtl::forecast(c);

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
    const auto c = [base] {
      auto c = base;
      c.Dx[0] = 1;
      c.Dy[0] = 0;
      for (auto& each : c.v) each = 1;
      for (auto& each : c.Dth) each = 0;
      for (auto& each : c.th) each = 0;
      c.xref = {1, 2, 3, 4};
      c.yref = {0, 0, 0, 0};
      return c;
    }();

    const auto result = dtl::forecast(c);

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
    const auto c = [base] {
      auto c = base;
      c.Dx[0] = 1;
      c.Dy[0] = 0;
      for (auto& each : c.v) each = 1;
      for (auto& each : c.Dth) each = M_PI_2;
      for (auto& each : c.th) each = 0;
      c.xref = {1, 1, 0, 0};
      c.yref = {0, 1, 1, 0};
      return c;
    }();

    const auto result = dtl::forecast(c);

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
    const auto c = [base] {
      auto c = base;
      c.Dx[0] = M_SQRT1_2;
      c.Dy[0] = M_SQRT1_2;
      for (auto& each : c.v) each = 1;
      for (auto& each : c.Dth) each = 0;
      for (auto& each : c.th) each = M_PI_4;
      c.xref =
        {-M_SQRT1_2, -2 * M_SQRT1_2, -3 * M_SQRT1_2, -4 * M_SQRT1_2};
      c.yref =
        {-M_SQRT1_2, -2 * M_SQRT1_2, -3 * M_SQRT1_2, -4 * M_SQRT1_2};
      return c;
    }();

    const auto result = dtl::forecast(c);

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
    "[dtl][forecast][potential_gradient]") {
  const auto c = [] {
    NMPCState<5> c;
    c.x[0] = 0;
    c.y[0] = 0;
    c.th[0] = M_PI_4;
    c.Dx[0] = 0;
    c.Dy[0] = 0;
    c.dt = 1s;
    //
    // Following combination of v and Dth should walk to corners of a triangle:
    //    (0,0) → (0,0) → (1,1) → (1,0) → (1,-1)
    //
    //            │  • (1,1)
    //            │ ↗↓
    //      (0,0)—•——•—(1,0)
    //            │  ↓
    //            │  • (1,-1)
    //
    // We'll check this later.
    //
    c.v = {M_SQRT2, 1, 1, 1};
    c.Dth = {0, -3 * M_PI_4, 0, 0};
    //                ↑
    //                dtl::forecast doesn't compute potential at the fixed
    //                “current” position, since it doesn't factor into the
    //                control output sequence.
    //
    c.obstacles.push_back(ob::Point{{0, 0}, 2, 1});
    c.obstacles.push_back(ob::Null{});
    c.obstacles.push_back(ob::Point{{0, 0}, 2, 1});
    c.obstacles.push_back(ob::Null{});
    return c;
  }();

  const auto result = dtl::forecast(c);

  CHECK_THAT(as_vector(result.x),
             Catch::Approx<double>({0, 0, 1, 1, 1}).margin(1e-25));
  CHECK_THAT(as_vector(result.y),
             Catch::Approx<double>({0, 0, 1, 0, -1}).margin(1e-25));
  CHECK_THAT(as_vector(result.th),
             Catch::Approx<double>({M_PI_4, M_PI_4, -M_PI_2, -M_PI_2, -M_PI_2})
                 .margin(1e-25));
  REQUIRE_THAT(as_vector(result.DPhiX),
               Catch::Approx<double>({0, -4. / 9, -1, -4. / 9}).margin(1e-25));
  REQUIRE_THAT(as_vector(result.DPhiY),
               Catch::Approx<double>({0, -4. / 9, 0, 4. / 9}).margin(1e-25));
}

TEST_CASE(
    "When the forecast trajectory follows the reference path, and there are no "
    "obstacles, the Lagrange multipliers and gradient along the trajectory "
    "should vanish. `gradNorms` prv— and cur— should get swapped.",
    "[dtl][lagrange_gradient]") {
  const auto c = [] {
    NMPCState<5> c;
    c.dt = 1s;
    c.x[0] = 0;
    c.y[0] = 0;
    c.th[0] = M_PI_4;
    c.Dx[0] = 1;
    c.Dy[0] = 1;
    c.xref = {1, 2, 3, 4};
    c.yref = {1, 2, 3, 4};
    c.v = {M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
    c.Dth = {0, 0, 0, 0};
    c.Q0 = 1;
    c.Q = 1;
    c.R = 1;
    c.curGradNorm = 5;  // REQUIREd to end up in prvGradNorm.
    return c;
  }();

  const auto result = dtl::lagrange_gradient(dtl::forecast(c));

  CHECK(as_vector(result.ex) == std::vector<double>{0, 0, 0, 0});
  CHECK(as_vector(result.ey) == std::vector<double>{0, 0, 0, 0});

  REQUIRE(as_vector(result.px) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(result.py) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(result.pDx) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(as_vector(result.pDy) == std::vector<double>{0, 0, 0, 0});

  REQUIRE(as_vector(result.grad) == std::vector<double>{0, 0, 0, 0});
  REQUIRE(result.curGradNorm == 0);
  REQUIRE(result.prvGradNorm == 5);
}

TEST_CASE(
    "With the outcome of the forecast engineered to match the reference path "
    "(a straight line along 𝑦 = 𝑥 at constant speed), the steepest descent "
    "should terminate in one step, with Dth unchanged.",
    "[dtl][sd_optimise]") {
  const auto c = [] {
    NMPCState<5> c;
    c.dt = 1s;
    c.x[0] = 0;
    c.y[0] = 0;
    c.th[0] = M_PI_4;
    c.Dx[0] = 1;
    c.Dy[0] = 1;
    c.xref = {1, 2, 3, 4};
    c.yref = {1, 2, 3, 4};
    c.v = {M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
    c.Dth = {0, 0, 0, 0};
    c.Q0 = 1;
    c.Q = 1;
    c.R = 1;
    c.curGradNorm = std::numeric_limits<double>::max();
    return c;
  }();

  auto result = dtl::sd_optimise(c);

  CHECK_THAT(as_vector(result.x), Catch::Equals<double>({0, 1, 2, 3, 4}));
  CHECK_THAT(as_vector(result.y), Catch::Equals<double>({0, 1, 2, 3, 4}));
  REQUIRE_THAT(as_vector(result.Dth), Catch::Equals<double>({0, 0, 0, 0}));
  // This surreptitiously tests that there was only one iteration:
  REQUIRE(result.prvGradNorm == std::numeric_limits<double>::max());
}

TEST_CASE(
    "Starting in the direction to follow a straight line reference at a "
    "constant speed, but with a wonky Dth, after sd_optimisation, we should "
    "still end up near the reference path.",
    "[dtl][sd_optimise]") {
  const auto c = [] {
    NMPCState<5> c;
    c.dt = 1s;
    c.x[0] = 0;
    c.y[0] = 0;
    c.th[0] = M_PI_4;
    c.Dx[0] = 1;
    c.Dy[0] = 1;
    c.xref = {1, 2, 3, 4};
    c.yref = {1, 2, 3, 4};
    c.v = {M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
    c.Dth = {M_PI_4, M_PI_4, M_PI_4, M_PI_4};
    c.Q0 = 0;
    c.Q = 1;
    c.R = 0.5;
    c.curGradNorm = std::numeric_limits<double>::max();
    return c;
  }();

  auto result = dtl::sd_optimise(c);

  CHECK_THAT(as_vector(result.x),
             Catch::Approx<double>({0, 1, 2, 3, 4}).margin(0.1));
  CHECK_THAT(as_vector(result.y),
             Catch::Approx<double>({0, 1, 2, 3, 4}).margin(0.1));
}

TEST_CASE(
    "Starting at the origin with a target along 𝑦 = 𝑥, at a speed of 𝑣 = √2 "
    "and time intervals d𝑡 = 1s, we should plan a reference [(1,1), (2,2), "
    "(3,3), (4,4)].",
    "[dtl][plan_reference]") {
  const auto c = [] {
    NMPCState<5> c;
    c.v = {M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
    c.dt = 1s;
    return c;
  }();
  const auto w = [] {
    WorldState<> w;
    w.target = {{1, 1}, 0.1};
    w.nav2pose = {std::chrono::steady_clock::now(), {0, 0}, M_PI_4};
    return w;
  }();

  const auto result = dtl::plan_reference(c, w);

  REQUIRE_THAT(as_vector(result.xref), Catch::Approx<double>({1, 2, 3, 4}));
  REQUIRE_THAT(as_vector(result.yref), Catch::Approx<double>({1, 2, 3, 4}));
}

TEST_CASE(
    "Starting at (8, 8) with a target along 𝑦 = 𝑥, at a speed of 𝑣 = √2 and "
    "time intervals d𝑡 = 1s, we should plan a reference [(7,7), (6,6), (5,5), "
    "(4,4)].",
    "[dtl][plan_reference]") {
  const auto c = [] {
    NMPCState<5> c;
    c.v = {M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
    c.dt = 1s;
    return c;
  }();
  const auto w = [] {
    WorldState<> w;
    w.target = {{1, 1}, 0.1};
    w.nav2pose = {std::chrono::steady_clock::now(), {8, 8}, -3 * M_PI_4};
    return w;
  }();

  const auto result = dtl::plan_reference(c, w);

  REQUIRE_THAT(as_vector(result.xref), Catch::Approx<double>({7, 6, 5, 4}));
  REQUIRE_THAT(as_vector(result.yref), Catch::Approx<double>({7, 6, 5, 4}));
}

TEST_CASE(
    "Starting at (0, 0) with a target along 𝑦, at a speed of 𝑣 = 1 and time "
    "intervals d𝑡 = 1s, we should plan a reference [(0,1), (0,2), (0,3), "
    "(0,4)].",
    "[dtl][plan_reference]") {
  const auto c = [] {
    NMPCState<5> c;
    c.v = {1, 1, 1, 1};
    c.dt = 1s;
    return c;
  }();
  const auto w = [] {
    WorldState<> w;
    w.target = {{0, 1}, 0.1};
    w.nav2pose = {std::chrono::steady_clock::now(), {0, 0}, M_PI_2};
    return w;
  }();

  const auto result = dtl::plan_reference(c, w);

  REQUIRE_THAT(as_vector(result.xref), Catch::Approx<double>({0, 0, 0, 0}));
  REQUIRE_THAT(as_vector(result.yref), Catch::Approx<double>({1, 2, 3, 4}));
}

TEST_CASE(
    "Starting at (0, 0) with a target along 𝑥, at a speed of 𝑣 = 1 and time "
    "intervals d𝑡 = 1s, we should plan a reference [(1,0), (2,0), (3,0), "
    "(4,0)].",
    "[dtl][plan_reference]") {
  const auto c = [] {
    NMPCState<5> c;
    c.v = {1, 1, 1, 1};
    c.dt = 1s;
    return c;
  }();
  const auto w = [] {
    WorldState<> w;
    w.target = {{1, 0}, 0.1};
    w.nav2pose = {std::chrono::steady_clock::now(), {0, 0}, 0};
    return w;
  }();

  const auto result = dtl::plan_reference(c, w);

  REQUIRE_THAT(as_vector(result.xref), Catch::Approx<double>({1, 2, 3, 4}));
  REQUIRE_THAT(as_vector(result.yref), Catch::Approx<double>({0, 0, 0, 0}));
}

TEST_CASE(
    "Starting at the origin with a target along 𝑦 = 𝑥, at a speed of 𝑣 = √2 "
    "and time intervals d𝑡 = 1s, we should obtain an optimal path of [(1,1), "
    "(2,2), (3,3), (4,4)].",
    "[nmpc_algebra]") {
  auto c = [] {
    NMPCState<5> c;
    c.dt = 1s;
    c.v = {M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
    c.Q0 = 0;
    c.Q = 1;
    c.R = 0.5;
    return c;
  }();
  auto w = [] {
    WorldState<> w;
    w.target = {{1, 1}, 0.1};
    w.nav2pose = {std::chrono::steady_clock::now(), {0, 0}, M_PI_4};
    w.obstacles = std::vector<ob::Obstacle>{ob::Null()};
    return w;
  }();

  auto result = nmpc_algebra(c, w);

  CHECK(result.obstacles.size() == w.obstacles.size());
  CHECK(as_vector(c.x) != as_vector(result.x));
  CHECK(as_vector(c.y) != as_vector(result.y));
  REQUIRE_THAT(as_vector(result.x), Catch::Approx<double>({0, 1, 2, 3, 4}));
  REQUIRE_THAT(as_vector(result.y), Catch::Approx<double>({0, 1, 2, 3, 4}));
}

TEST_CASE(
    "With the previous test as a starting point, adding point-obstacles below "
    "(in terms of 𝑦 coordinates) the reference trajectory should push it "
    "upward (toward higher 𝑦-values).",
    "[nmpc-algebra]") {
  auto c = [] {
    NMPCState<5> c;
    c.dt = 1s;
    c.v = {M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
    c.Dth = {M_PI_4, M_PI_4, M_PI_4, M_PI_4};
    c.Q0 = 1;
    c.Q = 1;
    c.R = 0.5;
    return c;
  }();
  auto w = [] {
    WorldState<> w;
    w.target = {{1, 1}, 0.1};
    w.nav2pose = {std::chrono::steady_clock::now(), {0, 0}, M_PI_4};
    w.obstacles = std::vector<ob::Obstacle>{ob::Point{{2.5, 0}, 1, .01},
                                            ob::Point{{3.5, 1}, 2, 0.01}};
    return w;
  }();

  auto result = nmpc_algebra(c, w);

  CHECK(result.obstacles.size() == w.obstacles.size());

  CHECK_THAT(as_vector(result.grad),
             Catch::Approx<double>({0, 0, 0, 0}).margin(0.01));

  CHECK(as_vector(result.Dth) != std::vector<double>{0, 0, 0, 0});
  CHECK(as_vector(result.DPhiX) != std::vector<double>{0, 0, 0, 0});
  CHECK(as_vector(result.DPhiY) != std::vector<double>{0, 0, 0, 0});
  CHECK(as_vector(result.ex) != std::vector<double>{0, 0, 0, 0});
  CHECK(as_vector(result.ex) != std::vector<double>{0, 0, 0, 0});
}

TEST_CASE(
    "The optimal solution presented by the algebra should not depend on the "
    "initial values of the turn rates. The setup from the previous example is "
    "used as a starting point.",
    "[nmpc-algebra]") {
  auto c1 = [] {
    NMPCState<5> c;
    c.dt = 1s;
    c.v = {M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
    c.Dth = {0.1, 0.1, 0.1, 0.1};
    c.Q0 = 1;
    c.Q = 1;
    c.R = 0.5;
    return c;
  }();
  auto c2 = [c = c1]() mutable {
    c.Dth = {M_PI_4, M_PI_4, M_PI_4, M_PI_4};
    return c;
  }();
  auto w = [] {
    WorldState<> w;
    w.target = {{1, 1}, 0.1};
    w.nav2pose = {std::chrono::steady_clock::now(), {0, 0}, 0};
    w.obstacles = std::vector<ob::Obstacle>{ob::Point{{2.5, 0}, 1, .01},
                                            ob::Point{{3.5, 1}, 2, 0.01}};
    return w;
  }();

  auto result1 = nmpc_algebra(c1, w);
  auto result2 = nmpc_algebra(c1, w);

  CHECK(as_vector(result1.x) == as_vector(result2.x));
  CHECK(as_vector(result1.y) == as_vector(result2.y));
}
