#include "cmd_vel_pub.h"

// publish once linear and angular velocities commands
void CmdVelPub::publish_cmd_vel() {
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = vel_cmd_[0];
    msg.linear.y = 0.0;
    msg.linear.z = 0.0;
    msg.angular.x = 0.0;
    msg.angular.y = 0.0;
    msg.angular.z = vel_cmd_[1];
    cmd_vel_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "Published cmd_vel: linear(%.2f, %.2f, %.2f), angular(%.2f, %.2f, %.2f)",
                msg.linear.x, msg.linear.y, msg.linear.z,
                msg.angular.x, msg.angular.y, msg.angular.z);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CmdVelPub>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
