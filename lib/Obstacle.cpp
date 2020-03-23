#include <cmath>
#include <utility>
#include <variant>

#include "Obstacle.hpp"

namespace ob {

  auto g_phi(XY, const Null) -> XY { return {0, 0}; }

  auto g_phi(XY p, const Point o) -> XY {
    /*
     * For a point-obstacle, the gradient vector is:
     *
     *                ρ - 1
     *   dφ        ρ 𝑟
     *  ──── = - ───────────── , 𝑟 = √(𝑥² + 𝑦²)
     *   d𝑟          ρ     2
     *             (𝑟  + ε)
     */
    auto r = p - o.coords;
    const double q = quadrance(r);
    // TODO: Maybe missing minus sign in numer:
    const double numer = o.pwr * std::pow(q, o.pwr / 2 - 1);
    const double denom = (std::pow(q, o.pwr / 2) + o.epsilon)
                         * (std::pow(q, o.pwr / 2) + o.epsilon);

    return {r.x * numer / denom, r.y * numer / denom};
  }

  auto g_phi(const XY p, const Obstacle o) -> XY {
    return std::visit([p](const auto& o) { return g_phi(p, o); }, o);
  }

}  // namespace ob
