#include <fstream>
#include <iomanip>

#include "../include/nmpc_algebra.hpp"

struct JsonLogger {
  std::ofstream logFile;

  static constexpr auto separator = ",\n";
  const char* sep = "";

  JsonLogger(const JsonLogger&) = delete;
  JsonLogger(const JsonLogger&&) = delete;
  JsonLogger& operator=(const JsonLogger&) = delete;
  JsonLogger& operator=(const JsonLogger&&) = delete;
  explicit JsonLogger(std::string outputFilePath);
  ~JsonLogger();

  void log(const std::string&);
};

std::string to_string(InfoFlag);
std::string to_string(Target);
std::string to_json(ob::Point o);
std::string to_json(ob::Null);
std::string to_json(ob::Obstacle ob);
std::string ob_vec_to_json_array(const std::vector<ob::Obstacle>&);

template <typename Clock = std::chrono::steady_clock>
static inline std::string to_string(const typename Clock::time_point& tp) {
  return std::to_string(tp.time_since_epoch().count());
}

template <typename T, size_t N>
std::string array_to_json_array(const std::array<T, N> xs) {
  static_assert(N > 2);
  std::string output = "[";
  for (unsigned int i = 0; i <= N - 2; ++i)
    output += std::to_string(xs[i]) + ",";
  output += std::to_string(xs[N - 1]) + "]";
  return output;
}

// clang-format off
template <std::size_t N, typename Clock = std::chrono::steady_clock>
std::string to_json(NMPCState<N, Clock> s) {
  return std::string{"{\n"} + "\"time\": " + to_string(s.time) + ",\n"
         + "\"infoFlag\": \"" + to_string(s.infoFlag) + "\",\n"
         + "\"target\": \"" + to_string(s.target) + "\",\n"
         + "\"R\": " + std::to_string(s.R) + ",\n"
         + "\"Q\": " + std::to_string(s.Q)
         + ",\n" + "\"Q₀\": " + std::to_string(s.Q0) + ",\n"
         + "\"dt\": " + std::to_string(s.dt.count()) + ",\n"
         + "\"x\": " + array_to_json_array(s.x) + ",\n"
         + "\"y\": " + array_to_json_array(s.y) + ",\n"
         + "\"th\": " + array_to_json_array(s.th) + ",\n"
         + "\"Dx\": " + array_to_json_array(s.Dx) + ",\n"
         + "\"Dy\": " + array_to_json_array(s.Dy) + ",\n"
         + "\"Dth\": " + array_to_json_array(s.Dth) + ",\n"
         + "\"v\": " + array_to_json_array(s.v) + ",\n"
         + "\"xref\": " + array_to_json_array(s.xref) + ",\n"
         + "\"yref\": " + array_to_json_array(s.yref) + ",\n"
         + "\"ex\": " + array_to_json_array(s.ex) + ",\n"
         + "\"ey\": " + array_to_json_array(s.ey) + ",\n"
         + "\"obstacles\": " + ob_vec_to_json_array(s.obstacles) + "\n" + "}";
}
// clang-format on
