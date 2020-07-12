// Copyright (c) 2019 SSL-Roots
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.


#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

#include "consai2r2_teleop/joystick_component.hpp"

using namespace std::chrono_literals;


namespace joystick
{
JoystickComponent::JoystickComponent(const rclcpp::NodeOptions & options)
: Node("consai2r2_teleop", options)
{
  this->declare_parameter("button_shutdown_1", 8);
  this->declare_parameter("button_shutdown_2", 9);
  this->declare_parameter("button_move_enable", 4);
  this->declare_parameter("axis_vel_surge", 1);
  this->declare_parameter("axis_vel_sway", 0);
  this->declare_parameter("axis_vel_angular", 2);

  button_shutdown_1_ = this->get_parameter("button_shutdown_1").get_value<int>();
  button_shutdown_2_ = this->get_parameter("button_shutdown_2").get_value<int>();
  button_move_enable_ = this->get_parameter("button_move_enable").get_value<int>();
  axis_vel_surge_ = this->get_parameter("axis_vel_surge").get_value<int>();
  axis_vel_sway_ = this->get_parameter("axis_vel_sway").get_value<int>();
  axis_vel_angular_ = this->get_parameter("axis_vel_angular").get_value<int>();

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
  // FIXME: WE HAVE TO USE ROS_PARAM
  const double MAX_VEL_SURGE = 1.0;
  const double MAX_VEL_SWAY = 1.0;
  const double MAX_VEL_ANGULAR = M_PI;

  consai2r2_msgs::msg::RobotCommand command;

  if (msg->buttons[button_move_enable_]) {
    command.vel_surge = msg->axes[axis_vel_surge_] * MAX_VEL_SURGE;
    command.vel_sway = msg->axes[axis_vel_sway_] * MAX_VEL_SWAY;
    command.vel_angular = msg->axes[axis_vel_angular_] * MAX_VEL_ANGULAR;
  }

  command.robot_id = 0;

  consai2r2_msgs::msg::RobotCommands robot_commands;
  robot_commands.header.stamp = this->now();
  robot_commands.is_yellow = false;
  robot_commands.commands.push_back(command);

  pub_commands_->publish(robot_commands);
}


}  // namespace joystick

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
