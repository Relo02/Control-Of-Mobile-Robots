#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class CounterNode : public rclcpp::Node
{
public:
    CounterNode();
private:
    void timer_callback();
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
