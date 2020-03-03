
#include "../lib/Nav2remote.hpp"


using CState = PIDState<>;

struct WorldInterface {
  const rxcpp::subjects::behavior<PState> txSubject;
  const rxcpp::observable<PState> setPoint =
      // Setpoint to x = 1, for step response.
      rxcpp::observable<>::just(PState{now, {1., 0.}});

  WorldInterface(PState x0) : txSubject(x0) {}

  void controlled_step(CState u) {
    auto x = txSubject.get_value();
    sim::PState xAugmented = {x.value[0], x.value[1], u.ctrlVal};

    if ((x.time - now) >= simDuration)
      txSubject.get_subscriber().on_completed();

    // do_step uses the second argument for both input and output.
    _stepper.do_step(_plant, xAugmented, 0, dt);
    x.time += dts;
    x.value = {xAugmented[0], xAugmented[1]};
    txSubject.get_subscriber().on_next(x);
  };

  auto get_state_observable() { return txSubject.get_observable(); }
  auto time_elapsed() { return txSubject.get_value().time - now; }

 private:
  ode::runge_kutta4<sim::PState> _stepper;
  const sim::Plant _plant = sim::Plant(staticForce, damp, spring);
};
