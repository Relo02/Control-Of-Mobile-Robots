#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "odesimulator.h"

class RobotSimulation : public rclcpp::Node {

private:
    // ------------- Subscribers -------------
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_cmd_subscription_;

    // ------------- Publishers -------------
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr state_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Extract parameters
    double dt_;
    double Ta_;
    
    /* Node state variables */
    ODESimulator *simulator_;

public:

    RobotSimulation();
    ~RobotSimulation() {  // destructor
        if (simulator_) {
            delete simulator_;
        }
    }

    void velocity_command(const geometry_msgs::msg::Twist::SharedPtr msg);
    void Prepare(void);
    // void RunPeriodically(void);
    void Shutdown(void);
    /* Node periodic task */
    void PeriodicTask(void);
};