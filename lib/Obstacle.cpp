#include <cmath>
#include <utility>
#include <variant>

#include "Obstacle.hpp"

using XY = std::pair<double, double>;

namespace ob {

  auto g_phi(double, double, const Null&) -> XY { return {0, 0}; }

  auto g_phi(double x, double y, const Point& p) -> XY {
    /*
     * For a point-obstacle, the gradient vector is:
     *
     *                ρ - 1
     *   dφ        ρ 𝑟
     *  ---- = - ------------- , 𝑟 = √(𝑥² + 𝑦²)
     *   d𝑟          ρ     2
     *             (𝑟  + ε)
     */
    const double rx = x - p.x;
    const double ry = y - p.y;
    // Pnemonic: 𝑞 for quadrance, in the sense described by Canadian
    //   mathematician Norman J. Wildberger.
    const double q = rx * rx + ry * ry;
    // TODO: Maybe missing minus sign in numer:
    const double numer = p.pwr * std::pow(q, p.pwr / 2 - 1);
    const double denom = (std::pow(q, p.pwr / 2) + p.epsilon)
                         * (std::pow(q, p.pwr / 2) + p.epsilon);

    return {rx * numer / denom, ry * numer / denom};
  }

  auto g_phi(const double x, const double y, const Obstacle& o) -> XY {
    return std::visit([x, y](auto& o) { return g_phi(x, y, o); }, o);
  }

}  // namespace ob
