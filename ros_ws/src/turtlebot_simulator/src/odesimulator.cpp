#include "odesimulator.h"
#include <cmath>
#include <stdexcept>
#include <functional>

ODESimulator::ODESimulator(double deltaT)
    : t(0.0), dt(deltaT), state(5, 0.0), u(2, 0.0), modelParams_set(false)
{
}

void ODESimulator::setInitialState(const state_type &initial_state)
{
    if (initial_state.size() != 5)
        throw std::invalid_argument("Initial state must be size 5");
    state = initial_state;
}

void ODESimulator::setModelParams(double Ta, double dt)
{
    this->Ta = Ta;
    this->dt = dt;
    modelParams_set = true;
}

void ODESimulator::setInputValues(const state_type &u)
{
    if (u.size() != 2)
        throw std::invalid_argument("Input vector must be size 2");
    this->u = u;
}

void ODESimulator::integrate()
{
    if (!modelParams_set)
        throw std::runtime_error("Model parameters not set!");

    using namespace std::placeholders;
    stepper.do_step(
        std::bind(&ODESimulator::simulator_ode, this, _1, _2, _3),
        state, t, dt);

    t += dt;
}

void ODESimulator::simulator_ode(const state_type &s,
                                 state_type &ds,
                                 double)
{
    double x     = s[0];
    double y     = s[1];
    double theta = s[2];
    double v     = s[3];
    double omega = s[4];

    double v_cmd     = u[0];
    double omega_cmd = u[1];

    ds[0] = v * std::cos(theta);
    ds[1] = v * std::sin(theta);
    ds[2] = omega;
    ds[3] = (-v     + v_cmd)     / Ta;
    ds[4] = (-omega + omega_cmd) / Ta;
}