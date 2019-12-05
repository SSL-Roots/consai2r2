/*
 * Copyright (c) 2019 SSL-Roots
 */


#include "consai2r2_teleop/joystick_component.hpp"

#include <rclcpp_components/register_node_macro.hpp>

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <cmath>


using namespace std::chrono_literals;


namespace joystick
{
JoystickComponent::JoystickComponent(const rclcpp::NodeOptions & options)
  : Node("consai2r2_teleop", options)
{
  RCLCPP_INFO(this->get_logger(), "hello world");
  auto callback =
    [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void
    {
        publish_robot_commands(msg);
    };

  pub_commands_ = create_publisher<consai2r2_msgs::msg::RobotCommands>("robot_commands", 10);
  sub_joy_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);
}

void JoystickComponent::publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  // TODO(SSL-Roots): WE HAVE TO USE ROS_PARAM
  const int BUTTON_SHUTDOWN_1 = 8;
  const int BUTTON_SHUTDOWN_2 = 8;
  const int BUTTON_MOVE_ENABLE = 4;
  const int AXIS_VEL_SURGE = 1;
  const int AXIS_VEL_SWAY = 0;
  const int AXIS_VEL_ANGULAR = 2;

  const double MAX_VEL_SURGE = 1.0;
  const double MAX_VEL_SWAY = 1.0;
  const double MAX_VEL_ANGULAR = M_PI;

  consai2r2_msgs::msg::RobotCommand command;

  if(msg->buttons[BUTTON_MOVE_ENABLE])
  {
    command.vel_surge = msg->axes[AXIS_VEL_SURGE] * MAX_VEL_SURGE;
    command.vel_sway = msg->axes[AXIS_VEL_SWAY] * MAX_VEL_SWAY;
    command.vel_angular = msg->axes[AXIS_VEL_ANGULAR] * MAX_VEL_ANGULAR;
  }

  command.robot_id = 0;

  consai2r2_msgs::msg::RobotCommands robot_commands;
  robot_commands.header.stamp = this->now();
  robot_commands.is_yellow = false;
  robot_commands.commands.push_back(command);

  pub_commands_->publish(robot_commands);
}



}  // namespace joystick

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
