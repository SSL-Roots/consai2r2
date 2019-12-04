#ifndef JOYSTICK_COMPONENT_HPP_
#define JOYSTICK_COMPONENT_HPP_

#include "visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <consai2r2_msgs/msg/robot_commands.hpp>

namespace joystick
{

class JoystickComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit JoystickComponent(const rclcpp::NodeOptions & options);

private:
  rclcpp::Publisher<consai2r2_msgs::msg::RobotCommands>::SharedPtr pub_commands_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

  void publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg);
};

} // namespace joystick

#endif
