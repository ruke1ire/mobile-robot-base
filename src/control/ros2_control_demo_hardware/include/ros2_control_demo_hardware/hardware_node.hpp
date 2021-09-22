#ifndef HARDWARE_INTERFACE
#define HARDWARE_INTERFACE

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

using std::placeholders::_1;

class HardwareNode : public rclcpp::Node
{
  public:
    HardwareNode()
    : Node("hardware_node")
    {

      RCLCPP_INFO(rclcpp::get_logger("HardwareNode"), "Starting hardware_node");

      left_wheel_publisher_ = this->create_publisher<std_msgs::msg::Float32>("left_wheel_vel", 10);
      right_wheel_publisher_ = this->create_publisher<std_msgs::msg::Float32>("right_wheel_vel", 10);

	  left_wheel_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float32>("left_wheel_velocity", 10, std::bind(&HardwareNode::left_wheel_velocity_callback, this, _1));
	  right_wheel_velocity_subscriber_ = this->create_subscription<std_msgs::msg::Float32>("right_wheel_velocity", 10, std::bind(&HardwareNode::right_wheel_velocity_callback, this, _1));
    }

    float left_vel = 0.0;
    float right_vel = 0.0;

    void publish_vel();

  private:
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_wheel_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_wheel_publisher_;

	void left_wheel_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);
	void right_wheel_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg);

	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_wheel_velocity_subscriber_;
	rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_wheel_velocity_subscriber_;

};

#endif  
