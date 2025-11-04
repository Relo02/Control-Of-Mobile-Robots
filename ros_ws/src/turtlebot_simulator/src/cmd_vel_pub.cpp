#include "cmd_vel_pub.h"

// publish once linear and angular velocities commands
void CmdVelPub::publish_cmd_vel() {
    auto msg = geometry_msgs::msg::Twist();
    msg.twist.linear.x = vel_cmd_[0];
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    msg.twist.angular.x = 0.0;
    msg.twist.angular.y = 0.0;
    msg.twist.angular.z = vel_cmd_[1];
    cmd_vel_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published cmd_vel: linear(%.2f, %.2f, %.2f), angular(%.2f, %.2f, %.2f)",
                msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
                msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z);
}