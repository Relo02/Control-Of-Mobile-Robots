#include <boost/numeric/odeint.hpp>

using namespace boost::numeric::odeint;

typedef std::vector<double> state_type;

// Simulation of the unyicycle robot model
class ODESimulator {

private:
    // Simulator and integrator variables
    double t, dt;
    double Ta;
    std::vector<double> u;
    bool modelParams_set;

    state_type state;
    runge_kutta_dopri5 < state_type > stepper; // Dormand-Prince 5th order method of Runge-Kutta type of integration

    // ODE function
    void simulator_ode(const state_type &state, state_type &dstate, double t);

public:
    ODESimulator(double deltaT);
    void setInitialState(std::vector<double> initial_state);
    void setModelParams(double Ta, double dt);
    void integrate();
    void setInputValues(std::vector<double> u);
    void getState(state_type &current_state) { current_state = state;};
    void getTime(double &time) { time = t;};
};