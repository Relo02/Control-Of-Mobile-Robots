#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/Twist.hpp"
#include <vector>
#include <chrono>

/*
    Node to publish cmd_vel messages for turtlebot simulator
*/


class CmdVelPub : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    std::vector<double> vel_cmd_;
public:

    CmdVelPub() : Node("cmd_vel_publisher"), vel_cmd_(6, 0.0) {
        // Initialize publisher
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("velocity_cmd", 10);

        // Set default velocities
        vel_cmd_[0] = 0.5; // linear x longitudinal command velocity
        vel_cmd_[1] = 0.0; // angular z rotational command velocity
    }

    // publish once linear and angular velocities commands
    void publish_cmd_vel();
};