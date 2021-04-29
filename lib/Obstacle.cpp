#include <cmath>
#include <utility>
#include <variant>

#include "Obstacle.hpp"

using namespace std::string_literals;

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
    auto r = o.position - p;
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

  // clang-format off
  inline std::string to_json(Point o) {
    return "{\"type\": \"Point\", "s
      + "\"x\": " + std::to_string(o.position.x) + ", "
      + "\"y\": " + std::to_string(o.position.y) + ", "
      + "\"pwr\": " + std::to_string(o.pwr) + ", "
      + "\"epsilon\": " + std::to_string(o.epsilon) + "}";
  }
  // clang-format on

  std::string to_json(Null) {
    return "{\"type\": \"Null\"}"s;
  }

  std::string to_json(Obstacle ob) {
    return std::visit([](const auto& o) {return to_json(o);} , ob);
  }

}  // namespace ob
