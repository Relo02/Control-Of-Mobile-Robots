#include "robot_simulation.h"


void RobotSimulation::Prepare(void) {

    std::vector<double> initial_state(5, 0.0);

    // Get parameters from the parameter server
    this->get_parameter("dt", dt_);
    this->get_parameter("Ta", Ta_);
    this->get_parameter("initial_state", initial_state);

    // Set simulation parameters
    simulator_->setInitialState(initial_state);
    simulator_->setModelParams(Ta_);
    simulator_->setTimeStep(dt_);

    simulator_ = new ODESimulator(dt_);
    simulator_->setInitialState(initial_state);
    simulator_->setModelParams(Ta_, dt_);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_),
        std::bind(&RobotSimulation::PeriodicTask, this));

    RCLCPP_INFO("Node %s prepared.", this->get_name());
}

void RobotSimulation::Shutdown(void) {
    RCLCPP_INFO("Node %s shutting down.", this->get_name());
}

void RobotSimulation::PeriodicTask(void) {

    // Integrate the simulator one step ahead
    simulator_->integrate();

    // Get current state
    std::vector<double> current_state;
    simulator_->getState(current_state);

    // Log current state
    RCLCPP_INFO(this->get_logger(), "Time: %.3f, State: [x: %.3f, y: %.3f, theta: %.3f, v: %.3f, omega: %.3f]",
                current_state[0], current_state[1], current_state[2], current_state[3], current_state[4]);

    double current_time;
    simulator_->getTime(current_time);

    // Print simulation every 5 seconds
    if (static_cast<int>(current_time) % 5 == 0) {
        RCLCPP_INFO(this->get_logger(), "Simulation Time: %.3f seconds", current_time);
    }

    // Publish current state
    auto state_msg = geometry_msgs::msg::Twist();
    state_msg.linear.x = current_state[0];  // x position
    state_msg.linear.y = current_state[1];  // y position
    state_msg.angular.z = current_state[2]; // orientation (theta)
    state_msg.twist.linear.x = current_state[3];  // linear longitudinal velocity (v)
    state_msg.twist.angular.z = current_state[4]; // angular velocity (omega)

    state_publisher_->publish(state_msg);
}
