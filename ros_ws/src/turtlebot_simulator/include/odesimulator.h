#pragma once

#include <vector>
#include <boost/numeric/odeint.hpp>

using state_type = std::vector<double>;

class ODESimulator
{
private:
    double t, dt, Ta;
    bool modelParams_set;
    state_type state;
    state_type u;

    boost::numeric::odeint::runge_kutta_dopri5<state_type> stepper;

    void simulator_ode(const state_type &state,
                       state_type &dstate,
                       double t);

public:
    explicit ODESimulator(double deltaT);
    void setInitialState(const state_type &initial_state);
    void setModelParams(double Ta, double dt);
    void setInputValues(const state_type &u);
    void integrate();

    void getState(state_type &current_state) const { current_state = state; }
    void getTime(double &time) const { time = t; }
};