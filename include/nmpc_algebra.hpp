#pragma once

#include <chrono>
#include <cmath>
#include <functional>
#include <type_traits>

#include "../include/list-fmap.hpp"
#include "../include/nav2_Pose.hpp"
#include "../lib/Obstacle.hpp"

enum class InfoFlag {
  OK,
  TargetReached,
  NoTarget,
  MissingWorldData,
  STOP,
  Null
};

struct Target {
  XY position = {0, 0};
  double tolerance = 0.1;
};

template <typename Clock = std::chrono::steady_clock>
struct WorldState {
  std::optional<Target> target;
  std::optional<nav2::Pose<Clock>> nav2pose;
  std::vector<ob::Obstacle> obstacles;
};

template <std::size_t N, typename Clock = std::chrono::steady_clock>
struct NMPCState {
  std::chrono::time_point<Clock> time;
  std::chrono::duration<double> dt = std::chrono::duration<double>(1. / 3);
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
  // Obstacle potential gradient for all but starting point of the trajectory:
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
  InfoFlag infoFlag = InfoFlag::OK;
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
    static_assert(std::is_nothrow_invocable_r_v<bool, P, I&>);
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
      c.th[k] = fma(c.Dth[k - 1], c.dt.count(), c.th[k - 1]);
      c.x[k] = fma(c.Dx[k - 1], c.dt.count(), c.x[k - 1]);
      c.Dx[k] = c.v[k - 1] * std::cos(c.th[k]);
      c.y[k] = fma(c.Dy[k - 1], c.dt.count(), c.y[k - 1]);
      c.Dy[k] = c.v[k - 1] * std::sin(c.th[k]);

      c.ex[k - 1] = c.x[k] - c.xref[k - 1];
      c.ey[k - 1] = c.y[k] - c.yref[k - 1];

      std::tie(c.DPhiX[k - 1], c.DPhiY[k - 1]) =
          foldl(ob::g_phi_accuml({c.x[k], c.y[k]}), XY{0, 0}, c.obstacles);
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
    c.grad[N - 2] = c.pth[N - 2] * c.dt.count() + c.Dth[N - 2] * c.R;
    c.curGradNorm = c.grad[N - 2] * c.grad[N - 2];
    // Get the gradient ∂ℋ/∂uₖ, for each step, k in the horizon, loop
    // through each k in N. This involves computing the obstacle potential
    // and Lagrange multiplierc.
    for (int k = N - 3; k >= 0; --k) {
      c.px[k] = c.Q * c.ex[k] + c.DPhiX[k] + c.px[k + 1];
      c.pDx[k] = c.px[k + 1] * c.dt.count();
      c.py[k] = c.Q * c.ey[k] + c.DPhiY[k] + c.py[k + 1];
      c.pDy[k] = c.py[k + 1] * c.dt.count();
      c.pth[k] = c.pth[k + 1] + c.pDy[k + 1] * c.v[k] * std::cos(c.th[k])
                 - c.pDx[k + 1] * c.v[k] * std::sin(c.th[k]);
      c.grad[k] = c.R * c.Dth[k] + c.pth[k + 1] * c.dt.count();
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
    for (std::size_t i = 0; i < N - 1; ++i) {
      c.Dth[i] -= sdStepFactor * c.grad[i];
    }
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
      return [epsilon](auto& c) noexcept { return c.curGradNorm >= epsilon; };
    };

    const double tol = ((c.R + c.Q) * (N - 1) + c.Q0) / N / c.dt.count() / 100;

    return iterate_while(step, gradNorm_outside(tol), c);
  }

  // setup : NMPCState × WorldState → NMPCState
  // (x₀, y₀, th₀, Dx₀, Dy₀, Dth₀), default to 0
  //
  template <std::size_t N, typename Clock = std::chrono::steady_clock>
  constexpr NMPCState<N, Clock> with_init_from_world(
      NMPCState<N, Clock> c, const WorldState<Clock> w) noexcept {
    c.time = w.nav2pose->time;
    c.x[0] = w.nav2pose->position.x;
    c.y[0] = w.nav2pose->position.y;
    c.th[0] = w.nav2pose->orientation;
    c.Dx[0] = c.v[0] * std::cos(c.th[0]);
    c.Dy[0] = c.v[0] * std::sin(c.th[0]);
    // TODO: This is from vme-nmpc, but has a bug. v is an N-1 size vector,
    // so v[0] doesn't mean the speed at (x[0],y[0]). I need to coordinate
    // with changes to telep-base to get the real Dx and Dy.
    c.obstacles = w.obstacles;
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
    XY unit = normalise(w.target->position - w.nav2pose->position);
    c.xref[0] = w.nav2pose->position.x + c.v[0] * unit.x * c.dt.count();
    c.yref[0] = w.nav2pose->position.y + c.v[0] * unit.y * c.dt.count();
    for (std::size_t k = 1; k < N - 1; ++k) {
      c.xref[k] = c.xref[k - 1] + c.v[k] * unit.x * c.dt.count();
      c.yref[k] = c.yref[k - 1] + c.v[k] * unit.y * c.dt.count();
    }
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
  if (!w.target) {
    c.infoFlag = InfoFlag::NoTarget;
    return c;
  }
  if (!w.nav2pose) {
    c.infoFlag = InfoFlag::MissingWorldData;
    return c;
  }
  if (w.nav2pose->time - c.time <= std::chrono::seconds{0}) {
    c.infoFlag = InfoFlag::STOP;
    return c;
  }
  if (l2norm(w.target->position - w.nav2pose->position) < w.target->tolerance) {
    c.infoFlag = InfoFlag::TargetReached;
    return c;
  }

  c.infoFlag = InfoFlag::OK;
  return dtl::sd_optimise(
      dtl::plan_reference(dtl::with_init_from_world(c, w), w));
}
