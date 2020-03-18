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
  // List of obstacles used to compute (DPhiX, DPhiY).
  std::list<ob::Obstacle> obstacles;
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
  constexpr NMPCState<N, Clock> forecast(NMPCState<N, Clock> s) noexcept {
    for (std::size_t k = 1; k < N; ++k) {
      s.th[k] = fma(s.Dth[k - 1], s.dt[k - 1].count(), s.th[k - 1]);
      s.x[k] = fma(s.Dx[k - 1], s.dt[k - 1].count(), s.x[k - 1]);
      s.Dx[k] = s.v[k - 1] * std::cos(s.th[k]);
      s.y[k] = fma(s.Dy[k - 1], s.dt[k - 1].count(), s.y[k - 1]);
      s.Dy[k] = s.v[k - 1] * std::sin(s.th[k]);

      s.ex[k - 1] = s.x[k] - s.xref[k - 1];
      s.ey[k - 1] = s.y[k] - s.yref[k - 1];

      std::tie(s.DPhiX[k - 1], s.DPhiY[k - 1]) = foldl(
          ob::g_phi_accuml(s.x[k], s.y[k]), std::pair{0., 0.}, s.obstacles);
    }
    return s;
  }

  // lagrange_gradient : NMPCState → NMPCState
  //
  // Calculate gradient from ∂J = ∑∂ℋ/∂u ∂u. In doing so, the Lagrange
  // multipliers are populated.
  template <std::size_t N, typename Clock = std::chrono::steady_clock>
  constexpr NMPCState<N, Clock> lagrange_gradient(
      NMPCState<N, Clock> s) noexcept {
    s.prvGradNorm = s.curGradNorm;
    s.curGradNorm = 0;
    s.px[N - 2] = s.Q0 * s.ex[N - 2];
    s.py[N - 2] = s.Q0 * s.ey[N - 2];
    s.pDx[N - 2] = 0;
    s.pDy[N - 2] = 0;
    s.pth[N - 2] = 0;
    s.grad[N - 2] = 0;
    // Get the gradient ∂ℋ/∂uₖ, for each step, k in the horizon, loop
    // through each k in N. This involves computing the obstacle potential
    // and Lagrange multipliers.
    for (int k = N - 3; k >= 0; --k) {
      s.px[k] = s.Q * s.ex[k] + s.DPhiX[k] + s.px[k + 1];
      s.pDx[k] = s.px[k + 1] * s.dt[k].count();
      s.py[k] = s.Q * s.ey[k] + s.DPhiY[k] + s.py[k + 1];
      s.pDy[k] = s.py[k + 1] * s.dt[k].count();
      s.pth[k] = s.pth[k + 1] + s.pDy[k + 1] * s.v[k] * std::cos(s.th[k])
                 - s.pDx[k + 1] * s.v[k] * std::sin(s.th[k]);
      s.grad[k] = s.R * s.Dth[k] + s.pth[k + 1] * s.dt[k].count();
      s.curGradNorm += s.grad[k] * s.grad[k];
    }
    s.curGradNorm = sqrt(s.curGradNorm);

    return s;
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
  constexpr NMPCState<N, Clock> sd_optimise(NMPCState<N, Clock> s) noexcept {
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

    return iterate_while(step, gradNorm_outside(0.01), s);
  }

  // initialize : NMPCState × ( ⋯ ) → NMPCState
  // (x₀, y₀, th₀, Dx₀, Dy₀, Dth₀), default to 0

  // plan_reference : NMPCState × (double × double) → NMPCState
  //
  // Given a target destination, the tracking reference is set to the sampled
  // straight line from the current position (x₀, y₀) to the target with point
  // separations determined by dtₖ [and 𝑣ₖ, or should I set these as part of the
  // plan?].

}  // namespace dtl

//                                      __         __
//    ___  __ _  ___  ____        ___ _/ /__  ___ / /  _______ _
//   / _ \/  ' \/ _ \/ __/       / _ `/ / _ `/ -_) _ \/ __/ _ `/
//  /_//_/_/_/_/ .__/\__/  ____  \_,_/_/\_, /\__/_.__/_/  \_,_/
//            /_/         /___/        /___/
//
// nmpc_algebra : NMPCState × PlantSignal → NMPCState
//
// template <std::size_t N, typename Clock = std::chrono::steady_clock>
// constexpr auto nmpc_algebra() {
//   return [](NMPCState<N, Clock>&& ctrlState,
//             PlantSignal<Clock> pSigl) -> NMPCState<N, Clock> {
//     const std::chrono::duration<double> deltaT = pSigl.time - ctrlState.time;
//     if (deltaT <= std::chrono::seconds{0}) return ctrlState;

//     return ctrlState;
//   };
// }
