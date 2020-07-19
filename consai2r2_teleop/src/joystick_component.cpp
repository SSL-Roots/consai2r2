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
  this->declare_parameter("button_color_id_enable", 6);
  this->declare_parameter("axis_vel_surge", 1);
  this->declare_parameter("axis_vel_sway", 0);
  this->declare_parameter("axis_vel_angular", 2);

  this->declare_parameter("d_pad_change_color", "right");
  this->declare_parameter("d_pad_increment", "up");
  this->declare_parameter("d_pad_decrement", "down");
  this->declare_parameter("d_pad_reset", "right");

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
  button_color_id_enable_ = this->get_parameter("button_color_id_enable").get_value<int>();
  axis_vel_surge_ = this->get_parameter("axis_vel_surge").get_value<int>();
  axis_vel_sway_ = this->get_parameter("axis_vel_sway").get_value<int>();
  axis_vel_angular_ = this->get_parameter("axis_vel_angular").get_value<int>();

  d_pad_change_color_ = this->get_parameter("d_pad_change_color").get_value<std::string>();
  d_pad_increment_ = this->get_parameter("d_pad_increment").get_value<std::string>();
  d_pad_decrement_ = this->get_parameter("d_pad_decrement").get_value<std::string>();
  d_pad_reset_ = this->get_parameter("d_pad_reset").get_value<std::string>();

  auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this,
      "consai2r2_description");
  while (!parameters_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for theservice. Exiting.");
      rclcpp::shutdown();
    }
    RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
  }
  max_id_ = parameters_client->get_parameter("max_id", 15);
  max_vel_surge_ = parameters_client->get_parameter("max_velocity_surge", 3.0);
  max_vel_sway_ = parameters_client->get_parameter("max_velocity_sway", 3.0);
  max_vel_angular_ = parameters_client->get_parameter("max_velocity_angular", 3.14);

  is_yellow_ = false;
  has_changed_team_color_ = false;
  target_id_ = 0;
  has_changed_target_id_ = false;

  auto callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
      shutdown_via_joy(msg);
      change_color_id_via_joy(msg);
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
  consai2r2_msgs::msg::RobotCommand command;

  if (msg->buttons[button_move_enable_]) {
    command.vel_surge = msg->axes[axis_vel_surge_] * max_vel_surge_;
    command.vel_sway = msg->axes[axis_vel_sway_] * max_vel_sway_;
    command.vel_angular = msg->axes[axis_vel_angular_] * max_vel_angular_;
  }

  command.robot_id = target_id_;

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

void JoystickComponent::change_color_id_via_joy(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (msg->buttons[button_color_id_enable_]) {
    if (d_pad(msg, d_pad_change_color_)) {
      if (!has_changed_team_color_) {
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

    if (d_pad(msg, d_pad_increment_) || d_pad(msg, d_pad_decrement_)) {
      if (!has_changed_target_id_) {
        if (d_pad(msg, d_pad_increment_) && target_id_ < max_id_) {
          target_id_++;
        } else if (d_pad(msg, d_pad_decrement_) && target_id_ > 0) {
          target_id_--;
        }
        has_changed_target_id_ = true;
        RCLCPP_INFO(this->get_logger(), "Target robot ID is %d.", target_id_);
      }
    } else {
      has_changed_target_id_ = false;
    }
  }  // enable button pressed
}

}  // namespace joystick

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
