#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <queue>
#include <rxcpp/rx.hpp>
#include <rxcpp/subjects/rx-subject.hpp>
#include <utility>

#include "../include/nmpc_algebra.hpp"
#include "../lib/Nav2remote.hpp"

using namespace std::string_literals;
using namespace std::chrono_literals;
using std::chrono::steady_clock;
using std::chrono::time_point;

constexpr std::size_t N = 15;

using CState = NMPCState<N, steady_clock>;
using PState = WorldState<steady_clock>;

namespace {

  class WorldInterface {
    const time_point<steady_clock> timeAtStartup = steady_clock::now();
    nav2::Remote remoteNav2;
    std::vector<ob::Obstacle> obstacles;

   public:
    const rxcpp::observable<PState> worldStates =
        rxcpp::observable<>::interval(100ms).map([this](auto) {
          obstacles.clear();
          constexpr double omega = 1E-9;
          obstacles = std::vector<ob::Obstacle>{
              ob::Point{
                  {5
                       +  2 * std::sin(
                           omega
                           * steady_clock::now().time_since_epoch().count()),
                   2 * std::cos(omega
                            * steady_clock::now().time_since_epoch().count())},
                  2,
                  0.3333},
              ob::Point{
                  {5
                       - 2 * std::sin(
                           omega
                           * steady_clock::now().time_since_epoch().count()),
                   -2 * std::cos(omega
                            * steady_clock::now().time_since_epoch().count())},
                  2,
                  0.3333}};
          return PState{std::nullopt, remoteNav2.estimatePosition(), obstacles};
        });

    const rxcpp::observable<std::optional<Target>> targetSetpoint;

    WorldInterface(PState w0, const std::string& addr, int port = 5010)
        : remoteNav2(addr.c_str(), port),
          obstacles(w0.obstacles),
          targetSetpoint(
              rxcpp::observable<>::just<std::optional<Target>>(w0.target)) {
      auto [x0, y0] = w0.nav2pose->position;
      remoteNav2.setPosition(x0, y0, w0.nav2pose->orientation);
    }

    void controlled_step(CState& c) {
      switch (c.infoFlag) {
        case InfoFlag::OK:
          remoteNav2.execute(
              nav2::actions::SetRelativeVelocity{0, c.v[0], c.Dth[0]});
          break;

        case InfoFlag::STOP:
          remoteNav2.execute(nav2::actions::Stop{});
          break;

        case InfoFlag::MissingWorldData:
        case InfoFlag::NoTarget:
        case InfoFlag::TargetReached:
          remoteNav2.execute(nav2::actions::Stop{});
          std::this_thread::sleep_for(1s);
          break;

        case InfoFlag::Null:
          break;

        default:
          remoteNav2.execute(nav2::actions::Stop{});
      }
    };
  };

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

    void log(std::string);
  };

  JsonLogger::JsonLogger(const std::string outputFilePath) {
    logFile.open(outputFilePath);
    logFile << "[\n";
  }

  JsonLogger::~JsonLogger() {
    std::cout << "Closing json file" << std::endl;
    logFile << "\n]\n";
    logFile.close();
  }

  void JsonLogger::log(std::string s) {
    logFile << sep << s;
    sep = separator;
  }

}  // namespace

int main() {
  const CState c0 = []() {
    CState c;
    c.infoFlag = InfoFlag::NoTarget;
    for (auto& each : c.v) each = 1.25;
    c.dt = 1.s / 5;
    for (auto& each : c.Dth) each = 0.5;
    c.Q0 = 1;
    c.Q = 1;
    c.R = 0.15;
    return c;
  }();

  JsonLogger logger("testlog.json");

  const PState w0 = []() {
    PState w;
    w.target = {{10, 0}, 0.5};
    w.obstacles = {ob::Point{{5, 0}, 2, 0.3333}, ob::Point{{3, -1}, 2, 0.3333}};
    w.nav2pose = nav2::Pose<>{steady_clock::now(), {0, 0}, 0};
    return w;
  }();

  WorldInterface worldIface(w0, "localhost");
  auto thr = rxcpp::synchronize_event_loop();

  // A classical confiuration for a feedback controller is illustrated as:
  //                  err   u
  //   setPoint ──➤ ⊕ ──➤ C ──➤ P ──┬──➤ plantState
  //               -↑               │
  //                │               │
  //                ╰───────────────┘
  // If you open the loop and place an interface to the imperative world, ◼,
  // at the endpoints, the configuration becomes:
  //
  //    setPoint   err     u
  //    ◼ ─────> ⊗ ───> C ───> ◼
  //             ^
  //             │ plantState
  //             ◼
  // which is in 1:1 correspondence with the code below.
  //   (NB: generalization of ⊕ to ⊗.
  // clang-format off
  const auto sControls =
                  worldIface                          //        plantState
                    .worldStates                      //         ◼ ─────⮧  𝑒
                    .combine_latest(                  // target  ◼ ───> ⊗ ───> ⋯
                      [](WorldState<> w, std::optional<Target> t) {
                        w.target = t;
                        return w;
                      },
                      worldIface.targetSetpoint)
                    .observe_on(rxcpp::identity_current_thread())
                                                      //    𝑒
                    .scan(c0, nmpc_algebra<N>);       //   ───> C

  sControls.subscribe(
                      [&worldIface, &logger](CState c) {  //      𝑢
                        worldIface.controlled_step(c);    //   C ───> ◼
                        logger.log(util::to_json(c));
                      });
  // clang-format on
  // Think of subscribe as `for_each`, but for an asynchronous stream of
  // values instead of an iterable list.
}
