#pragma once

#include <chrono>

#include "XY.hpp"

namespace nav2 {

  template <typename Clock = std::chrono::steady_clock>
  struct Pose {
    std::chrono::time_point<Clock> time;
    XY position;
    double orientation;
  };

}  // namespace nav2
