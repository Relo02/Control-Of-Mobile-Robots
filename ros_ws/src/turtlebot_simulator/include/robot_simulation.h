#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/Twist.hpp"
#include "odesimulator.h"

class RobotSimulation : public rclcpp::Node {

private:

    ODESimulator simulator_;

    // ------------- Subscribers -------------
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_cmd_subscription_;

    // ------------- Publishers -------------
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr state_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Extract parameters
    double dt_;
    double Ta_;

    /* Node periodic task */
    void PeriodicTask(void);
    
    /* Node state variables */
    ODESimulator* simulator_;

public:
    RobotSimulation() : Node("robot_simulation"), simulator_(0.1) {
        // Initialize subscriber
        velocity_cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "velocity_cmd", 10,
            std::bind(&RobotSimulation::velocity_command, this, std::placeholders::_1)
        );

        // Initialize publisher
        state_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("robot_state", 10);
    }

    void velocity_command(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // initializing input vector
        // where u[0] is the linear velocity on the x longitudinal axis of the local frame of the robot
        // and u[1] is the angular velocity around the z vertical axis
        std::vector<double> u = {msg->twist.linear.x, msg->twist.angular.z};
        simulator_.setInputValues(u);
    }

    void Prepare(void);
    void RunPeriodically(void);
    void Shutdown(void);
};