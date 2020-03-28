#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <rxcpp/rx.hpp>
#include <rxcpp/subjects/rx-subject.hpp>

#include "../include/nmpc_algebra.hpp"

using namespace std::string_literals;
using namespace std::chrono_literals;

constexpr std::size_t N = 15;

using CState = NMPCState<N, std::chrono::steady_clock>;
using PState = WorldState<std::chrono::steady_clock>;


struct WorldInterface {
  const std::chrono::time_point<std::chrono::steady_clock> timeAtStartup =
      std::chrono::steady_clock::now();
  nav2::Remote remoteNav2;
  const rxcpp::subjects::subject<PState> worldStates;
  PState curWorld;

  WorldInterface(std::string addr, PState w0)
      : remoteNav2(addr.c_str()), curWorld{w0} {}
  WorldInterface(std::string addr, int port, PState w0)
      : remoteNav2(addr.c_str(), port), curWorld{w0} {}

  void try_push_next_worldState() const {
    auto mPose = remoteNav2.estimatePosition();
    if (mPose) {
      worldStates.get_subscriber().on_next(PState{
          curWorld.tgt, curWorld.tgtTolerance, *mPose, curWorld.obstacles});
    }
  }

  void controlled_step(CState& c) {
    remoteNav2.execute(nav2::actions::SetRelativeVelocity{0, c.v[0], c.Dth[0]});
    try_push_next_worldState();
  };

  auto get_world_observable() const { return worldStates.get_observable(); }
};

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
    w.tgt = {5, 5};
    w.obstacles = {ob::Point{{2.5,2.5}, 2, 0.15}};
    return w;
  }();

  WorldInterface worldIface("localhost", w0);

  // std::vector<PState> plantStateRecord;
  // worldIface.get_world_observable().subscribe(
  //     [&](PState x) { plantStateRecord.push_back(x); });

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
  worldIface.try_push_next_worldState();
}
