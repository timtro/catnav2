#pragma once

#include <chrono>
#include <functional>

namespace chrono = std::chrono;
using namespace std::chrono_literals;

template <typename T = double, typename Clock = chrono::steady_clock>
struct SignalPt {
  chrono::time_point<Clock> time;
  T value;
};

namespace util {
  template <typename A, typename F>
  [[nodiscard]] auto fmap(F f, const SignalPt<A>& a) {
    return SignalPt{a.time, std::invoke(f, a.value)};
  }
}  // namespace util
