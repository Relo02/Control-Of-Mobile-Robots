#include "odesimulator.h"

ODESimulator::ODESimulator(double deltaT) : t(0.0), dt(deltaT), state(5), modelParams_set(false) {

    // Initialize state vector to zero
    state.assign(5, 0.0);
}

void ODESimulator::setInitialState(std::vector<double> initial_state) {
    // Initial state values
    state = initial_state;
}

void ODESimulator::setModelParams(double Ta, double dt) {
    // Set model parameters
    this->Ta = Ta;
    this->dt = dt;
    modelParams_set = true;
}

void ODESimulator::setInputValues(std::vector<double> u) {
    // Set input values
    this->u = u;
}

void ODESimulator::integrate()
{
    // Check model parameters are set
    if (!modelParams_set) {
        throw std::invalid_argument( "Model parameters not set!" );
    }

    // Integrate for one step ahead
    using namespace std::placeholders;
    stepper.do_step(std::bind(&ODESimulator::simulator_ode, this, _1, _2, _3), state, t, dt);

    // Update time
    t += dt;
}

void ODESimulator::simulator_ode(const state_type &state, state_type &dstate, double t) {
    // Unicycle robot model ODEs
    double v = state[3]; // current linear velocity
    double theta = state[2]; // current orientation
    double V_cmd = this->u[0]; // commanded linear velocity
    double omega_cmd = this->u[1]; // commanded angular velocity

    // State derivatives
    dstate[0] = v * cos(theta); // dx/dt
    dstate[1] = v * sin(theta); // dy/dt
    dstate[2] = state[4];   // dv/dt
    dstate[3] = - 1 / this->Ta * v + 1 / this->Ta * V_cmd; // dtheta/dt
    dstate[4] = - 1 / this->Ta * state[4] + 1 / this->Ta * omega_cmd; // domega/dt
}

