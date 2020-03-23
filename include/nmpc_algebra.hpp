#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <list>
#include <variant>

#include <boost/hana/functional/compose.hpp>
#include <boost/hana/functional/curry.hpp>

#include "../include/list-fmap.hpp"
#include "../lib/Nav2remote.hpp"
#include "../lib/Obstacle.hpp"

template <typename Clock = std::chrono::steady_clock>
struct WorldState {
  XY tgt = {0, 0};
  double tgtTolerance = 0.1;
  nav2::XYState<Clock> robot;
  std::vector<ob::Obstacle> obstacles;
};

template <std::size_t N, typename Clock = std::chrono::steady_clock>
struct NMPCState {
  std::chrono::time_point<Clock> time;
  std::chrono::duration<double> dt[N - 1];
  // State Vector:
  double x[N] = {{0}};
  double y[N] = {{0}};
  double th[N] = {{0}};
  double Dx[N] = {{0}};
  double Dy[N] = {{0}};
  double Dth[N - 1] = {{0}};
  // Nav2 gives (𝑥, 𝑦) and robot orientation θ. For line following,
  // we want (v, θ), and we control ω = D θ. So we need 𝑣:
  double v[N - 1] = {{0}};
  // Tracking reference and resulting error (𝑥,𝑦) - (𝑥_ref, 𝑦_ref):
  double xref[N - 1] = {{0}};
  double yref[N - 1] = {{0}};
  double ex[N - 1] = {{0}};
  double ey[N - 1] = {{0}};
  // Obstacle potential gradient for each point of the trajectory:
  double DPhiX[N - 1] = {{0}};
  double DPhiY[N - 1] = {{0}};
  // Lagrange Multipliers:
  double px[N - 1] = {{0}};
  double py[N - 1] = {{0}};
  double pDx[N - 1] = {{0}};
  double pDy[N - 1] = {{0}};
  double pth[N - 1] = {{0}};
  // Optimisation gradients:
  double grad[N - 1] = {{0}};
  double curGradNorm = 0;
  double prvGradNorm = 0;
  // Coefficients
  double R = 0,  // Control effort penalty
      Q = 0,     // Tracking error penalty
      Q0 = 0;    // Terminal error penalty
  // Collection of obstacles used to compute (DPhiX, DPhiY).
  std::vector<ob::Obstacle> obstacles;
};

namespace dtl {

  // iterate_while : (I → I) × (I → bool) × I → I
  //                └───────┘ └──────────┘
  //                  f : F      p : P
  //
  // Iterates and endofunction, I → I, while a predicate, I → bool, returns
  // true, starting from an initial value of I.
  template <typename F, typename P, typename I>
  constexpr auto iterate_while(F f, P p, I i) noexcept {
    static_assert(std::is_nothrow_invocable_r_v<I, F, I>);
    static_assert(std::is_nothrow_invocable_r_v<bool, P, I>);
    do {
      i = std::invoke(f, i);
    } while (std::invoke(p, i));
    return i;
  }

  // forecast : NMPCState → NMPCState
  //
  // Based on the initial pose, time intervals, speed plan, tracking reference &
  // obstacles:
  //   1. Euler integrate th, x, y, Dx, Dy,
  //   2. compute tracking errors, and
  //   3. compute the gradient of the obstacle potential.
  template <std::size_t N, typename Clock = std::chrono::steady_clock>
  constexpr NMPCState<N, Clock> forecast(NMPCState<N, Clock> c) noexcept {
    for (std::size_t k = 1; k < N; ++k) {
      c.th[k] = fma(c.Dth[k - 1], c.dt[k - 1].count(), c.th[k - 1]);
      c.x[k] = fma(c.Dx[k - 1], c.dt[k - 1].count(), c.x[k - 1]);
      c.Dx[k] = c.v[k - 1] * std::cos(c.th[k]);
      c.y[k] = fma(c.Dy[k - 1], c.dt[k - 1].count(), c.y[k - 1]);
      c.Dy[k] = c.v[k - 1] * std::sin(c.th[k]);

      c.ex[k - 1] = c.x[k] - c.xref[k - 1];
      c.ey[k - 1] = c.y[k] - c.yref[k - 1];

      std::tie(c.DPhiX[k - 1], c.DPhiY[k - 1]) =
          foldl(ob::g_phi_accuml(XY{c.x[k], c.y[k]}), XY{0, 0}, c.obstacles);
    }
    return c;
  }

  // lagrange_gradient : NMPCState → NMPCState
  //
  // Calculate gradient from ∂J = ∑∂ℋ/∂u ∂u. In doing so, the Lagrange
  // multipliers are populated.
  template <std::size_t N, typename Clock = std::chrono::steady_clock>
  constexpr NMPCState<N, Clock> lagrange_gradient(
      NMPCState<N, Clock> c) noexcept {
    c.prvGradNorm = c.curGradNorm;
    c.px[N - 2] = c.Q0 * c.ex[N - 2];
    c.py[N - 2] = c.Q0 * c.ey[N - 2];
    c.pDx[N - 2] = 0;
    c.pDy[N - 2] = 0;
    c.pth[N - 2] = 0;
    c.grad[N - 2] = c.pth[N - 2] * c.dt[N - 2].count() + c.Dth[N - 2] * c.R;
    c.curGradNorm = c.grad[N - 2] * c.grad[N - 2];
    // Get the gradient ∂ℋ/∂uₖ, for each step, k in the horizon, loop
    // through each k in N. This involves computing the obstacle potential
    // and Lagrange multiplierc.
    for (int k = N - 3; k >= 0; --k) {
      c.px[k] = c.Q * c.ex[k] + c.DPhiX[k] + c.px[k + 1];
      c.pDx[k] = c.px[k + 1] * c.dt[k].count();
      c.py[k] = c.Q * c.ey[k] + c.DPhiY[k] + c.py[k + 1];
      c.pDy[k] = c.py[k + 1] * c.dt[k].count();
      c.pth[k] = c.pth[k + 1] + c.pDy[k + 1] * c.v[k] * std::cos(c.th[k])
                 - c.pDx[k + 1] * c.v[k] * std::sin(c.th[k]);
      c.grad[k] = c.R * c.Dth[k] + c.pth[k + 1] * c.dt[k].count();
      c.curGradNorm += c.grad[k] * c.grad[k];
    }
    c.curGradNorm = sqrt(c.curGradNorm);
    return c;
  }

  // descend :: NMPCState → NMPCState
  //
  // Uses the scaling factor to descend the control varible in the direction of
  // the computed gradient.
  template <std::size_t N, typename Clock = std::chrono::steady_clock>
  constexpr NMPCState<N, Clock> descend(NMPCState<N, Clock> c) noexcept {
    constexpr double sdStepFactor = 0.1;
    for (std::size_t i = 0; i < N - 1; ++i)
      c.Dth[i] -= sdStepFactor * c.grad[i];
    return c;
  }

  // sd_optimise : NMPCState → NMPCState
  //
  // A steepest/gradient descent algorithm that iteratively refines the control
  // plan to minimize the cost functional.
  template <std::size_t N, typename Clock = std::chrono::steady_clock>
  constexpr NMPCState<N, Clock> sd_optimise(NMPCState<N, Clock> c) noexcept {
    //
    // step : NMPCState → NMPCState
    //
    constexpr auto step = [](auto c) noexcept {
      return descend(lagrange_gradient(forecast(c)));
    };

    // gradNorm_outside : double → NMPCState → bool
    //
    constexpr auto gradNorm_outside = [](double epsilon) {
      return [epsilon](auto c) noexcept {
        return (c.curGradNorm >= epsilon) ? true : false;
      };
    };

    return iterate_while(step, gradNorm_outside(0.01), c);
  }

  // setup : NMPCState × ( ⋯ ) → NMPCState
  // (x₀, y₀, th₀, Dx₀, Dy₀, Dth₀), default to 0
  //
  template <std::size_t N, typename Clock = std::chrono::steady_clock>
  constexpr NMPCState<N, Clock> with_init_from_world(
      NMPCState<N, Clock> c, const WorldState<Clock> w) noexcept {
    c.time = w.robot.localTimestamp;
    c.x[0] = w.robot.position.x;
    c.y[0] = w.robot.position.y;
    c.Dx[0] = w.robot.velocity.x;
    c.Dy[0] = w.robot.velocity.y;
    c.th[0] = std::atan2(w.robot.velocity.y, w.robot.velocity.x);
    c.v[0] = std::hypot(w.robot.velocity.x, w.robot.velocity.y);
    return c;
  }

  // plan_reference : NMPCState × (double × double) → NMPCState
  //
  // Given a target destination, the tracking reference is set to the sampled
  // straight line from the current position (x₀, y₀) to the target with point
  // separations determined by dtₖ [and 𝑣ₖ, or should I set these as part of the
  // plan?].
  template <std::size_t N, typename Clock = std::chrono::steady_clock>
  constexpr NMPCState<N, Clock> plan_reference(NMPCState<N, Clock> c,
                                               const WorldState<Clock> w) {
    XY unit = normalise(w.tgt - w.robot.position);
    for (auto [k, t] = std::tuple{std::size_t{0}, c.dt[0]}; k < N - 1; ++k) {
      c.xref[k] = w.robot.position.x + c.v[k] * unit.x * t.count();
      c.yref[k] = w.robot.position.y + c.v[k] * unit.y * t.count();
      t += c.dt[k];
    }
    c.obstacles = w.obstacles;
    return c;
  }

}  // namespace dtl

//                                      __         __
//    ___  __ _  ___  ____        ___ _/ /__  ___ / /  _______ _
//   / _ \/  ' \/ _ \/ __/       / _ `/ / _ `/ -_) _ \/ __/ _ `/
//  /_//_/_/_/_/ .__/\__/  ____  \_,_/_/\_, /\__/_.__/_/  \_,_/
//            /_/         /___/        /___/
//
// nmpc_algebra : NMPCState × PlantSignal → NMPCState
//
template <std::size_t N, typename Clock = std::chrono::steady_clock>
constexpr NMPCState<N, Clock> nmpc_algebra(NMPCState<N, Clock> c,
                                           const WorldState<Clock> w) {
  const std::chrono::duration<double> deltaT = w.robot.localTimestamp - c.time;
  if (deltaT <= std::chrono::seconds{0}) return c;

  return dtl::sd_optimise(
      dtl::plan_reference(dtl::with_init_from_world(c, w), w));
}
