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

#include <string>

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
  bool has_analog_d_pad_;
  int d_pad_up_;
  int d_pad_down_;
  int d_pad_left_;
  int d_pad_right_;
  bool d_pad_up_is_positive_;
  bool d_pad_right_is_positive_;

  int button_shutdown_1_;
  int button_shutdown_2_;
  int button_move_enable_;
  int button_color_id_enable_;
  int axis_vel_sway_;
  int axis_vel_surge_;
  int axis_vel_angular_;

  std::string d_pad_change_color_;
  std::string d_pad_increment_;
  std::string d_pad_decrement_;
  std::string d_pad_reset_;

  int max_id_;
  double max_vel_surge_;
  double max_vel_sway_;
  double max_vel_angular_;

  bool is_yellow_;
  bool has_changed_team_color_;
  int target_id_;
  bool has_changed_target_id_;
  double velocity_gain_;
  bool has_changed_velocity_gain_;

  rclcpp::Publisher<consai2r2_msgs::msg::RobotCommands>::SharedPtr pub_commands_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

  bool d_pad_pressed(
    const sensor_msgs::msg::Joy::SharedPtr msg, const int target_pad, const bool positive_on);
  bool d_pad_up(const sensor_msgs::msg::Joy::SharedPtr msg);
  bool d_pad_down(const sensor_msgs::msg::Joy::SharedPtr msg);
  bool d_pad_left(const sensor_msgs::msg::Joy::SharedPtr msg);
  bool d_pad_right(const sensor_msgs::msg::Joy::SharedPtr msg);
  bool d_pad(const sensor_msgs::msg::Joy::SharedPtr msg, const std::string target);
  void shutdown_via_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void change_color_id_via_joy(const sensor_msgs::msg::Joy::SharedPtr msg);
  void set_move_velocity_via_joy(const sensor_msgs::msg::Joy::SharedPtr msg,
    consai2r2_msgs::msg::RobotCommand & command);
};

}  // namespace joystick

#endif  // CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_
