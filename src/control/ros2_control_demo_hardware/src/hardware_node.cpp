#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include "ros2_control_demo_hardware/hardware_node.hpp"

HardwareNode::HardwareNode() : Node("hardware_node")
{
	left_wheel_publisher_ = this->create_publisher<std_msgs::msg::Float32>("left_wheel_vel", 10);
	right_wheel_publisher_ = this->create_publisher<std_msgs::msg::Float32>("right_wheel_vel", 10);

	left_wheel_velocity_subscriber = this->create_subscription<std_msgs::msg::Float32>("left_wheel_velocity", 10, std::bind(&HardwareNode::left_wheel_velocity_callback, this, _1));
	right_wheel_velocity_subscriber = this->create_subscription<std_msgs::msg::Float32>("right_wheel_velocity", 10, std::bind(&HardwareNode::right_wheel_velocity_callback, this, _1));
}

void HardwareNode::left_wheel_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
	left_vel = msg->data.float_value();
	RCLCPP_INFO(this->get_logger(), "Left wheel velocity '%f'", msg->data.float_value());
}

void HardwareNode::right_wheel_velocity_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
	right_vel = msg->data.float_value();
	RCLCPP_INFO(this->get_logger(), "Right wheel velocity '%f'", msg->data.float_value());
}