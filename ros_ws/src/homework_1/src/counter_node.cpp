// Class that create ros2 node counter

#include "counter_node.h"


CounterNode::CounterNode() : Node("counter_node"), count_(0), published_(false)
{
    // Declare and get parameters
    this->declare_parameter<int>("Ci", 0);
    this->declare_parameter<int>("Cf", 10);
    this->declare_parameter<double>("run_period", 1.0);
    int Ci = this->get_parameter("Ci").as_int();
    int Cf = this->get_parameter("Cf").as_int();
    double run_period = this->get_parameter("run_period").as_double();
    this->Ci_ = Ci;
    this->Cf_ = Cf;
    this->count_ = Ci;
    this->published_ = false;

    publisher_ = this->create_publisher<std_msgs::msg::Int32>("counter", 10);

    // Subscription to /set_counter
    subscription_ = this->create_subscription<std_msgs::msg::Int32>(
        "set_counter", 10,
        [this](const std_msgs::msg::Int32::SharedPtr msg) {
            int value = msg->data;
            RCLCPP_INFO(this->get_logger(), "Received: %d", value);
            if (value < this->Cf_) {
                this->count_ = value;
                this->published_ = false;
                // Restart timer if stopped
                if (!this->timer_->is_activated()) {
                    this->timer_->reset();
                }
            } else {
                RCLCPP_WARN(this->get_logger(), "Value %d >= Cf (%d). Ignored.", value, this->Cf_);
            }
        }
    );

    // Timer for incrementing counter
    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(run_period),
        std::bind(&CounterNode::timer_callback, this)
    );
}

void CounterNode::timer_callback()
{
    if (!published_) {
        if (count_ < Cf_) {
            count_++;
        }
        if (count_ >= Cf_ && !published_) {
            auto message = std_msgs::msg::Int32();
            message.data = count_;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Published: %d", count_);
            published_ = true;
            timer_->cancel(); // Stop timer after publishing
        }
    }
}