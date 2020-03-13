#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <list>
#include <variant>

#include "../include/list-fmap.hpp"
#include "../lib/Nav2remote.hpp"
#include "../lib/Obstacle.hpp"

namespace chrono = std::chrono;
using namespace std::chrono_literals;

template <std::size_t N, typename Clock = chrono::steady_clock>
struct NMPCState {
  chrono::time_point<Clock> time;
  std::array<chrono::duration<float>, N> dt;
  // State Vector:
  double x[N];
  double y[N];
  double th[N];
  double Dx[N];
  double Dy[N];
  double Dth[N - 1];
  // Other:
  double v[N - 1];
  double ex[N - 1];
  double ey[N - 1];
  double DPhiX[N - 1];
  double DPhiY[N - 1];
  // Lagrange Multipliers:
  double px[N - 1];
  double py[N - 1];
  double pDx[N - 1];
  double pDy[N - 1];
  double pth[N - 1];
  // Gradients:
  double grad[N - 1];
  double gradNorm;
  // Tracking reference
  double xref[N - 1];
  double yref[N - 1];
  // Coefficients
  double R,  // Control effort penalty
      Q,     // Tracking error penalty
      Q0;    // Terminal error penalty
  std::list<ob::Obstacle> obstacles;
};

namespace dtl {

  template <typename F, typename P, typename I>
  constexpr auto iterate_while(F f, P p, I i) noexcept {
    auto result = std::move(i);
    do {
      result = std::invoke(f, result);
    } while (std::invoke(p, result));

    return result;
  }

  template <std::size_t N, typename Clock>
  auto gradNorm_within(NMPCState<N, Clock> x, double epsilon) noexcept {
    return (x.gradNorm <= epsilon) ? true : false;
  }

  template <std::size_t N, typename Clock = chrono::steady_clock>
  constexpr NMPCState<N, Clock> compute_forecast(
      NMPCState<N, Clock> s) noexcept {
    for (unsigned k = 1; k < N; ++k) {
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

  /*!
   * Calculate gradient from ∂J = ∑∂H/∂u ∂u. In doing so, the Lagrange
   * multipliers are computed.
   */
  template <std::size_t N, typename Clock = chrono::steady_clock>
  constexpr NMPCState<N, Clock> compute_gradient(
      NMPCState<N, Clock> s) noexcept {
    s.gradNorm = 0.;
    s.px[N - 2] = s.Q0 * s.ex[N - 2];
    s.py[N - 2] = s.Q0 * s.ey[N - 2];
    /*!
     * Get the gradient ∂H/∂u_k, for each step, k in the horizon, loop
     * through each k in N. This involves computing the obstacle potential
     * and Lagrange multipliers. Then, the control plan is updated by
     * stepping against the direction of the gradient.
     */
    for (int k = N - 3; k >= 0; --k) {
      s.px[k] = s.Q * s.ex[k] + s.DPhiX[k] + s.px[k + 1];
      s.pDx[k] = s.px[k + 1] * s.dt[k];
      s.py[k] = s.Q * s.ey[k] + s.DPhiY[k] + s.py[k + 1];
      s.pDy[k] = s.py[k + 1] * s.dt[k];
      s.pth[k] = s.pth[k + 1] + s.pDy[k + 1] * s.v[k] * std::cos(s.th[k])
                 - s.pDx[k + 1] * s.v[k] * std::sin(s.th[k]);
      s.grad[k] = s.R * s.Dth[k] + s.pth[k + 1] * s.dt[k];
      s.gradNorm += s.grad[k] * s.grad[k];
    }

    s.gradNorm = sqrt(s.gradNorm);

    return s;
  }

}  // namespace dtl

using namespace dtl;

template <std::size_t N, typename Clock = chrono::steady_clock>
constexpr auto nmpc_algebra() {
  return [](NMPCState<N, Clock>&& ctrlState,
            VMePose<Clock> errSigl) -> NMPCState<N, Clock> {
    const chrono::duration<double> deltaT = errSigl.time - ctrlState.time;
    if (deltaT <= chrono::seconds{0}) return ctrlState;

    ctrlState.time = errSigl.time;

    return std::move(ctrlState);
  };

  /* template <std::size_t N, typename Clock = chrono::steady_clock> */
  /* auto nmpc_algebra() { */
  /*   return [](NMPCState<horizonSteps, Clock>&& ctrlState, */
  /*             VMePose<Clock> errSigl) -> NMPCState<N, Clock> { */
  /*     const chrono::duration<double> deltaT = errSigl.time - ctrlState.time;
   */
  /*     if (deltaT <= chrono::seconds{0}) return ctrlState; */

  /*     ctrlState.time = errSigl.time; */

  /*     // clang-format off */
  /*     return std::move( */
  /*       iterate_until( */
  /*         compose */
  /*           ( compute_gradient */
  /*           , compute_path_potential_gradient */
  /*           , compute_forecast */
  /*           ), */
  /*         gradNorm_within(0.1))); */
  /*     // clang-format on */
  /*   }; */
}
