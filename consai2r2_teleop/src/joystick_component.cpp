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
  this->declare_parameter("has_analog_d_pad", true);
  this->declare_parameter("d_pad_up", 5);
  this->declare_parameter("d_pad_down", 5);
  this->declare_parameter("d_pad_left", 4);
  this->declare_parameter("d_pad_right", 4);
  this->declare_parameter("d_pad_up_is_positive", true);
  this->declare_parameter("d_pad_right_is_positive", false);

  this->declare_parameter("button_shutdown_1", 8);
  this->declare_parameter("button_shutdown_2", 9);
  this->declare_parameter("button_move_enable", 4);
  this->declare_parameter("button_color_enable", 5);
  this->declare_parameter("axis_vel_surge", 1);
  this->declare_parameter("axis_vel_sway", 0);
  this->declare_parameter("axis_vel_angular", 2);

  this->declare_parameter("d_pad_change_color", "right");

  has_analog_d_pad_ = this->get_parameter("has_analog_d_pad").get_value<bool>();
  d_pad_up_ = this->get_parameter("d_pad_up").get_value<int>();
  d_pad_down_ = this->get_parameter("d_pad_down").get_value<int>();
  d_pad_left_ = this->get_parameter("d_pad_left").get_value<int>();
  d_pad_right_ = this->get_parameter("d_pad_right").get_value<int>();
  d_pad_up_is_positive_ = this->get_parameter("d_pad_up_is_positive").get_value<bool>();
  d_pad_right_is_positive_ = this->get_parameter("d_pad_right_is_positive").get_value<bool>();

  button_shutdown_1_ = this->get_parameter("button_shutdown_1").get_value<int>();
  button_shutdown_2_ = this->get_parameter("button_shutdown_2").get_value<int>();
  button_move_enable_ = this->get_parameter("button_move_enable").get_value<int>();
  button_color_enable_ = this->get_parameter("button_color_enable").get_value<int>();
  axis_vel_surge_ = this->get_parameter("axis_vel_surge").get_value<int>();
  axis_vel_sway_ = this->get_parameter("axis_vel_sway").get_value<int>();
  axis_vel_angular_ = this->get_parameter("axis_vel_angular").get_value<int>();

  d_pad_change_color_ = this->get_parameter("d_pad_change_color").get_value<std::string>();

  is_yellow_ = false;
  has_changed_team_color_ = false;

  auto callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
      shutdown_via_joy(msg);
      change_team_color_via_joy(msg);
      publish_robot_commands(msg);
    };

  pub_commands_ = create_publisher<consai2r2_msgs::msg::RobotCommands>("robot_commands", 10);
  sub_joy_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);
}

bool JoystickComponent::d_pad_pressed(
  const sensor_msgs::msg::Joy::SharedPtr msg, const int target_pad, const bool positive_on)
{
  const double THRESHOLD = 0.5;
  if (has_analog_d_pad_) {
    if (positive_on) {
      return msg->axes[target_pad] > THRESHOLD;
    } else {
      return msg->axes[target_pad] < -THRESHOLD;
    }
  } else {
    return msg->buttons[target_pad];
  }
}

bool JoystickComponent::d_pad_up(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  return d_pad_pressed(msg, d_pad_up_, d_pad_up_is_positive_);
}

bool JoystickComponent::d_pad_down(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  return d_pad_pressed(msg, d_pad_down_, !d_pad_up_is_positive_);
}

bool JoystickComponent::d_pad_left(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  return d_pad_pressed(msg, d_pad_left_, !d_pad_right_is_positive_);
}

bool JoystickComponent::d_pad_right(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  return d_pad_pressed(msg, d_pad_right_, d_pad_right_is_positive_);
}

bool JoystickComponent::d_pad(const sensor_msgs::msg::Joy::SharedPtr msg, const std::string target)
{
  if (target == "up") {
    return d_pad_up(msg);
  } else if (target == "down") {
    return d_pad_down(msg);
  } else if (target == "right") {
    return d_pad_right(msg);
  } else if (target == "left") {
    return d_pad_left(msg);
  } else {
    return false;
  }
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
  robot_commands.is_yellow = is_yellow_;
  robot_commands.commands.push_back(command);

  pub_commands_->publish(robot_commands);
}

void JoystickComponent::shutdown_via_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons[button_shutdown_1_] && msg->buttons[button_shutdown_2_]) {
    RCLCPP_INFO(this->get_logger(), "Shutdown.");
    rclcpp::shutdown();
  }
}

void JoystickComponent::change_team_color_via_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons[button_color_enable_] && d_pad(msg, d_pad_change_color_)) {
    if (has_changed_team_color_ == false) {
      is_yellow_ = !is_yellow_;
      has_changed_team_color_ = true;

      if (is_yellow_) {
        RCLCPP_INFO(this->get_logger(), "Target team color is yellow.");
      } else {
        RCLCPP_INFO(this->get_logger(), "Target team color is blue.");
      }
    }
  } else {
    has_changed_team_color_ = false;
  }
}

}  // namespace joystick

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
