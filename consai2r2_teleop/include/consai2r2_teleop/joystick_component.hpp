/*
 * Copyright (c) 2019 SSL-Roots
 */
#ifndef CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_
#define CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <consai2r2_msgs/msg/robot_commands.hpp>

#include "visibility_control.hpp"

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

}  // namespace joystick

#endif  // CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_
