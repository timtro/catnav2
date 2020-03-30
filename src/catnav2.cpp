#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <rxcpp/rx.hpp>
#include <rxcpp/subjects/rx-subject.hpp>
#include <utility>

#include "../include/nmpc_algebra.hpp"

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
    std::vector<Target> waypoints;
    PState curWorldState;

   public:
    nav2::Remote remoteNav2;
    const rxcpp::subjects::subject<PState> worldStates;

    WorldInterface(const std::string& addr, PState w0)
        : curWorldState{std::move(w0)}, remoteNav2(addr.c_str()) {}
    WorldInterface(const std::string& addr, int port, PState w0)
        : curWorldState{std::move(w0)}, remoteNav2(addr.c_str(), port) {}

    void push_next_worldState() const {
      auto mPose = remoteNav2.estimatePosition();
      if (mPose) {
        worldStates.get_subscriber().on_next(
            PState{curWorldState.target, *mPose, curWorldState.obstacles});
      }
    }

    void controlled_step(CState& c) {
      remoteNav2.execute(
          nav2::actions::SetRelativeVelocity{0, c.v[0], c.Dth[0]});
      push_next_worldState();
    };

    auto get_world_observable() const { return worldStates.get_observable(); }
  };

}  // namespace

int main() {
  const CState c0 = []() {
    CState c;
    for (auto& each : c.v) each = 1.0;
    c.dt = 1.s / 5;
    for (auto& each : c.Dth) each = 0.5;
    c.Q0 = 1;
    c.Q = 1;
    c.R = 0.33;
    return c;
  }();

  const PState w0 = []() {
    PState w;
    w.target = {{20,0}, 0.1};
    w.obstacles = {ob::Point{{10, 0}, 2, 0.15}};
    return w;
  }();

  WorldInterface worldIface("localhost", w0);

  // A classical confiuration for a feedback controller is illustrated as:
  //                  err   u
  //   setPoint ──➤ ⊕ ──➤ C ──➤ P ──┬──➤ plantState
  //               -↑               │
  //                │               │
  //                ╰───────────────┘
  // If you open the loop and place an interface to the imperative world, ◼,
  // at the endpoints, the controller becomes:
  //
  // setPoint  err   u
  //    ◼ ─➤ ⊕ ──➤ C ──➤ ◼
  //        -↑
  //         │ plantState
  //         ◼
  // which is in 1:1 correspondence with the code below (read backward from C)
  //
  const auto sControls = worldIface
                             .get_world_observable()  // worldIface == ◼.
                             .observe_on(rxcpp::identity_current_thread())
                             .scan(c0, nmpc_algebra<N>);  //   This is C

  sControls.subscribe(
      [&worldIface](CState c) { worldIface.controlled_step(c); });

  worldIface.remoteNav2.setPosition(0, 0, 0);
  worldIface.push_next_worldState();
}
