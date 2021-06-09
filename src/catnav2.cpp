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

   public:
    nav2::Remote plant;
    nav2::Remote remoteObst;
    const rxcpp::observable<PState> worldStates =
        rxcpp::observable<>::interval(100ms).map([this](auto) {
          const auto obstPose = remoteObst.estimatePosition();
          const auto leadAhead = XY{std::cos(obstPose->orientation),
            std::sin(obstPose->orientation)};
          const std::optional<Target> tgtFromObst = {{obstPose->position + leadAhead, 0.1}};
          return PState{tgtFromObst,
                        plant.estimatePosition(),
                        {ob::Point{obstPose->position, 3, 0.1}}};
        });

    const rxcpp::observable<std::optional<Target>> targetSetpoint;

    WorldInterface(PState w0, const std::string& addr,
                   const std::string& addrObst, int port = 5010)
        : plant(addr.c_str(), port),
          remoteObst(addrObst.c_str(), 5010),
          targetSetpoint(
              rxcpp::observable<>::just<std::optional<Target>>(w0.target)) {
      auto [x0, y0] = w0.nav2pose->position;
      plant.setPosition(x0, y0, w0.nav2pose->orientation);
    }

    void controlled_step(CState& c) {
      switch (c.infoFlag) {
        case InfoFlag::OK:
          plant.execute(
              nav2::actions::SetRelativeVelocity{0, c.v[0], c.Dth[0]});
          break;

        case InfoFlag::STOP:
          plant.execute(nav2::actions::Stop{});
          break;

        case InfoFlag::MissingWorldData:
        case InfoFlag::NoTarget:
        case InfoFlag::TargetReached:
          plant.execute(nav2::actions::Stop{});
          break;

        case InfoFlag::Null:
          break;

        default:
          plant.execute(nav2::actions::Stop{});
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

    void log(const std::string&);
  };

  JsonLogger::JsonLogger(const std::string outputFilePath) {
    logFile.open(outputFilePath);
    logFile << "[\n";
  }

  JsonLogger::~JsonLogger() {
    logFile << "\n]\n";
    logFile.close();
  }

  void JsonLogger::log(const std::string& s) {
    logFile << sep << s;
    sep = separator;
  }

}  // namespace

int main() {
  const CState c0 = []() {
    CState c;
    c.infoFlag = InfoFlag::NoTarget;
    for (auto& each : c.v) each = 0.8;
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
    w.nav2pose = nav2::Pose<>{steady_clock::now(), {0, 0}, 0};
    return w;
  }();

  WorldInterface worldIface(w0, "localhost", "positron.local");

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
                    // .combine_latest(                  // target  ◼ ───> ⊗ ───> ⋯
                    //   [](WorldState<> w, std::optional<Target> ) {
                    //     // w.target = t;
                    //     return w;
                    //   },
                    //   worldIface.targetSetpoint)
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
