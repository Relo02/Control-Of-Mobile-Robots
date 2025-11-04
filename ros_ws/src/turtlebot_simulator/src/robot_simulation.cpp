#include "robot_simulation.h"

RobotSimulation::RobotSimulation() : Node("robot_simulation") {

    // Initialize subscriber
    velocity_cmd_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "velocity_cmd", 10,
        std::bind(&RobotSimulation::velocity_command, this, std::placeholders::_1)
    );

    this->declare_parameter("dt", 0.001);
    this->declare_parameter("Ta", 0.070);
    this->declare_parameter("initial_state", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0});   

    // Initialize publisher
    state_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("robot_odometry", 10);
}

void RobotSimulation::Prepare(void) {
    std::vector<double> initial_state(5, 0.0);

    // Get parameters from the parameter server
    this->get_parameter("dt", dt_);
    this->get_parameter("Ta", Ta_);
    this->get_parameter("initial_state", initial_state);

    // Validate parameters
    if (!std::isfinite(dt_) || dt_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid dt parameter; using default 0.01");
        dt_ = 0.01;
    } else {
        RCLCPP_WARN(this->get_logger(), "Using dt = %.4f", dt_);
    }
    if (!std::isfinite(Ta_) || Ta_ <= 0.0) {
        RCLCPP_WARN(this->get_logger(), "Invalid Ta parameter; using default 0.5");
        Ta_ = 0.5;
    } else {
        RCLCPP_WARN(this->get_logger(), "Using Ta = %.4f", Ta_);
    }
    if (initial_state.size() != 5) {
        RCLCPP_WARN(this->get_logger(), "initial_state must have 5 elements; using zeros");
        initial_state = std::vector<double>(5, 0.0);
    }

    // Create simulator instance
    simulator_ = new ODESimulator(dt_);
    simulator_->setModelParams(Ta_, dt_);
    simulator_->setInitialState(initial_state);

    // Create timer with validated dt
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(dt_),
        std::bind(&RobotSimulation::PeriodicTask, this));

    RCLCPP_INFO(this->get_logger(), "Node %s prepared.", this->get_name());
}

void RobotSimulation::Shutdown(void) {
    RCLCPP_INFO(this->get_logger(), "Node %s shutting down.", this->get_name());
}

void RobotSimulation::PeriodicTask(void) {

    // Integrate the simulator one step ahead
    simulator_->integrate();

    // Get current state
    std::vector<double> current_state;
    simulator_->getState(current_state);

    double current_time;
    simulator_->getTime(current_time);

    // Print simulation every 5 seconds
    if (static_cast<int>(current_time) % 5 == 0) {
        RCLCPP_INFO(this->get_logger(), "Simulation Time: %.3f seconds", current_time);
    }

    // Publish current state through odometry type message
    auto odom_msg = nav_msgs::msg::Odometry();
    odom_msg.header.stamp = this->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Fill in the pose information
    odom_msg.pose.pose.position.x = current_state[0];
    odom_msg.pose.pose.position.y = current_state[1];
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, current_state[2]);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    // Fill in the twist information
    odom_msg.twist.twist.linear.x = current_state[3]; // longitudinal velocity
    odom_msg.twist.twist.angular.z = current_state[4];
    state_publisher_->publish(odom_msg);

    // log odometry publishing
    RCLCPP_INFO(this->get_logger(), "Published odometry: position(%.3f, %.3f, %.3f), orientation(%.3f, %.3f, %.3f, %.3f), linear_velocity(%.3f), angular_velocity(%.3f)",
                odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z,
                odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w,
                odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z);
}

void RobotSimulation::velocity_command(const geometry_msgs::msg::Twist::SharedPtr msg) {
    // initializing input vector
    // where u[0] is the linear velocity on the x longitudinal axis of the local frame of the robot
    // and u[1] is the angular velocity around the z vertical axis
    std::vector<double> u = {msg->linear.x, msg->angular.z};
    simulator_->setInputValues(u);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotSimulation>();
  node->Prepare();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
