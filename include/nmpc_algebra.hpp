#pragma once

#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <list>
#include <variant>

#include <boost/hana/functional/compose.hpp>
/* #include <boost/hana/functional/curry.hpp> */

#include "../include/Obstacle.hpp"
#include "../include/util.hpp"
#include "../lib/Nav2remote.hpp"

namespace chrono = std::chrono;
using namespace std::chrono_literals;

using boost::hana::compose;
/* using boost::hana::curry; */

constexpr std::size_t horizonSteps = 25;

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
  std::list<Obstacle> obstacles;
};

namespace dtl {

  template <typename F, typename P, typename I>
  auto iterate_while(F f, P p, I&& i) {
    auto result = std::move(i);
    do {
      result = std::invoke(f, result);
    } while (std::invoke(p, result));

    return std::move(result);
  }

  template <std::size_t N, typename Clock>
  auto gradNorm_within(NMPCState<N, Clock>& x, double epsilon) {
    return (x.gradNorm <= epsilon) ? true : false;
  }

  template <std::size_t N, typename Clock = chrono::steady_clock>
  NMPCState<N, Clock> compute_forecast(NMPCState<N, Clock>&& s) noexcept {
    for (unsigned k = 1; k < N; ++k) {
      s.th[k] = fma(s.Dth[k - 1], s.dt[k - 1].count(), s.th[k - 1]);
      s.x[k] = fma(s.Dx[k - 1], s.dt[k - 1].count(), s.x[k - 1]);
      s.Dx[k] = s.v[k - 1] * std::cos(s.th[k]);
      s.y[k] = fma(s.Dy[k - 1], s.dt[k - 1].count(), s.y[k - 1]);
      s.Dy[k] = s.v[k - 1] * std::sin(s.th[k]);
    }

    return std::move(s);
  }

  template <std::size_t N, typename Clock = chrono::steady_clock>
  NMPCState<N, Clock>&& compute_tracking_errors(
      NMPCState<N, Clock>&& s) noexcept {
    /* for (unsigned k = 0; k < N - 1; ++k) { */
    /*   s.ex[k] = s.x[k + 1] - s.xref[k]; */
    /*   s.ey[k] = s.y[k + 1] - s.yref[k]; */
    /* } */

    return std::move(s);
  }

  template <std::size_t N, typename Clock = chrono::steady_clock>
  NMPCState<N, Clock>&& compute_path_potential_gradient(
      NMPCState<N, Clock> s) noexcept {
    /* for (unsigned k = 0; k < N - 2; ++k) { */
    /*   fp_point2d gradVec = */
    /*       obstacles.gradient_phi(fp_point2d{x[k + 1], y[k + 1]}); */
    /*   DPhiX[k] = gradVec.x; */
    /*   DPhiY[k] = gradVec.y; */
    /* } */
    return std::move(s);
  }

  /*!
   * Calculate gradient from ∂J = ∑∂H/∂u ∂u. In doing so, the Lagrange
   * multipliers are computed.
   */
  template <std::size_t N, typename Clock = chrono::steady_clock>
  NMPCState<N, Clock>&& compute_gradient(NMPCState<N, Clock>&& s) noexcept {
    /* s.gradNorm = 0.; */
    /* s.px[N - 2] = s.Q0 * s.ex[N - 2]; */
    /* s.py[N - 2] = s.Q0 * s.ey[N - 2]; */
    /*!
     * Get the gradient ∂H/∂u_k, for each step, k in the horizon, loop
     * through each k in N. This involves computing the obstacle potential
     * and Lagrange multipliers. Then, the control plan is updated by
     * stepping against the direction of the gradient.
     */
    /* for (int k = N - 3; k >= 0; --k) { */
    /*   s.px[k] = s.Q * s.ex[k] + s.DPhiX[k] + s.px[k + 1]; */
    /*   s.pDx[k] = s.px[k + 1] * s.dt[k]; */
    /*   s.py[k] = s.Q * s.ey[k] + s.DPhiY[k] + s.py[k + 1]; */
    /*   s.pDy[k] = s.py[k + 1] * s.dt[k]; */
    /*   s.pth[k] = s.pth[k + 1] + s.pDy[k + 1] * s.v[k] * std::cos(s.th[k]) */
    /*              - s.pDx[k + 1] * s.v[k] * std::sin(s.th[k]); */
    /*   s.grad[k] = s.R * s.Dth[k] + s.pth[k + 1] * s.dt[k]; */
    /*   s.gradNorm += s.grad[k] * s.grad[k]; */
    /* } */

    /* s.gradNorm = sqrt(s.gradNorm); */

    return std::move(s);
  }

}  // namespace dtl

using namespace dtl;

template <std::size_t N, typename Clock = chrono::steady_clock>
auto nmpc_algebra() {
  return [](NMPCState<horizonSteps, Clock>&& ctrlState,
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
  /*           , compute_tracking_errors */
  /*           , compute_forecast */
  /*           ), */
  /*         gradNorm_within(0.1))); */
  /*     // clang-format on */
  /*   }; */
}
