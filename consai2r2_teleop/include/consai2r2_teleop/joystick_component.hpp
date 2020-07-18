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

#ifndef CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_
#define CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_

#include "consai2r2_msgs/msg/robot_commands.hpp"
#include "consai2r2_teleop/visibility_control.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

namespace joystick
{

class JoystickComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit JoystickComponent(const rclcpp::NodeOptions & options);

private:
  int button_shutdown_1_;
  int button_shutdown_2_;
  int button_move_enable_;
  int button_color_enable_;
  int axis_vel_sway_;
  int axis_vel_surge_;
  int axis_vel_angular_;
  int axis_color_change_;

  bool is_yellow_;
  bool has_changed_team_color_;

  rclcpp::Publisher<consai2r2_msgs::msg::RobotCommands>::SharedPtr pub_commands_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

  void publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg);
  void shutdown_via_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void change_team_color_via_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
};

}  // namespace joystick

#endif  // CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_
