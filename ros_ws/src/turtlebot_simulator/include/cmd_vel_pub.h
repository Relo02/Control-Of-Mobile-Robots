#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <chrono>

/*
    Node to publish cmd_vel messages for turtlebot simulator
*/

using namespace std::chrono_literals;

class CmdVelPub : public rclcpp::Node {
private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    std::vector<double> vel_cmd_;
    rclcpp::TimerBase::SharedPtr timer_;

public:

    CmdVelPub() : Node("cmd_vel_publisher"), vel_cmd_(6, 0.0) {
        // Initialize publisher
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("velocity_cmd", 10);

        timer_ = this->create_wall_timer(
                    200ms,
                    std::bind(&CmdVelPub::publish_cmd_vel, this)
);

        // Set default velocities
        vel_cmd_[0] = 0.5; // linear x longitudinal command velocity (m/s)
        vel_cmd_[1] = 0.5; // angular z rotational command velocity (rad/s)
    }

    // publish once linear and angular velocities commands
    void publish_cmd_vel();
};