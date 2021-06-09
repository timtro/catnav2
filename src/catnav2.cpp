#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <queue>
#include <rxcpp/rx.hpp>
#include <rxcpp/subjects/rx-subject.hpp>
#include <utility>

#include "../include/nmpc_algebra.hpp"
#include "../lib/JsonNMPCLogger.hpp"
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
    PState curWorldState;
    nav2::Remote remoteNav2;

   public:
    const rxcpp::subjects::subject<PState> worldStates;
    const rxcpp::subjects::subject<std::optional<Target>> targetSetpoint;

    WorldInterface(const std::string& addr, PState w0)
        : curWorldState{std::move(w0)}, remoteNav2(addr.c_str()) {
      auto [x0, y0] = curWorldState.nav2pose->position;
      remoteNav2.setPosition(x0, y0, curWorldState.nav2pose->orientation);
    }
    WorldInterface(const std::string& addr, int port, PState w0)
        : curWorldState{std::move(w0)}, remoteNav2(addr.c_str(), port) {
      auto [x0, y0] = curWorldState.nav2pose->position;
      remoteNav2.setPosition(x0, y0, curWorldState.nav2pose->orientation);
    }

    void push_next_worldState() const {
      worldStates.get_subscriber().on_next(PState{curWorldState.target,
                                                  remoteNav2.estimatePosition(),
                                                  curWorldState.obstacles});
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
      push_next_worldState();
    };
  };

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

  const PState w0 = []() {
    PState w;
    w.target = std::nullopt;
    w.obstacles = {ob::Point{{5, 0}, 2, 0.3333}};
    w.nav2pose = nav2::Pose<>{steady_clock::now(), {0, 0}, M_PI_2};
    return w;
  }();

  WorldInterface worldIface("localhost", w0);

  JsonLogger logger("testlog.json");

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
  //
  const auto sControls = worldIface
                             .worldStates       //       plantState
                             .get_observable()  //        ◼ ─────⮧
                             .combine_latest(  // target  ◼ ───> ⊗ ─> ⋯
                                 [](WorldState<> w, std::optional<Target> t) {
                                   w.target = t;
                                   return w;
                                 },
                                 worldIface.targetSetpoint.get_observable())
                             .observe_on(rxcpp::identity_current_thread())
                             .scan(c0, nmpc_algebra<N>);  // → C

  // This completes the loop:      u
  sControls.subscribe(  //      C ───> ◼
      [&worldIface, &logger](CState c) {
        worldIface.controlled_step(c);
        logger.log(to_json(c));
      });

  worldIface.targetSetpoint.get_subscriber().on_next(Target{{10, 0}, 0.5});
  worldIface.push_next_worldState();
}
