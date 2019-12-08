
#include "consai2r2_teleop/joystick_component.hpp"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>

using namespace std::chrono_literals;

namespace joystick {
JoystickComponent::JoystickComponent(const rclcpp::NodeOptions &options)
    : Node("consai2r2_teleop", options) {
  RCLCPP_INFO(this->get_logger(), "hello world");
  button_shutdown_1_ = this->declare_parameter("button_shutdown_1", 8);
  button_shutdown_2_ = this->declare_parameter("button_shutdown_2", 8);
  button_move_enable_ = this->declare_parameter("button_move_enable", 4);
  axis_vel_surge_ = this->declare_parameter("axis_vel_surge", 1);
  axis_vel_sway_ = this->declare_parameter("axis_vel_sway", 0);
  axis_vel_angular_ = this->declare_parameter("axis_vel_angular", 2);

  button_kick_enable_ = this->declare_parameter("button_kick_enable");
  button_kick_straight_ = this->declare_parameter("button_kick_straight");
  button_kick_chip_ = this->declare_parameter("button_kick_chip");
  axis_kick_power_ = this->declare_parameter("axis_kick_power");

  button_dribble_enable_ = this->declare_parameter("button_dribble_enable");
  axis_dribble_power_ = this->declare_parameter("axis_dribble_power");

  button_id_enable_ = this->declare_parameter("button_id_enable");
  axis_id_change_ = this->declare_parameter("axis_id_change");

  button_color_enable_ = this->declare_parameter("button_color_enable");
  axis_color_change_ = this->declare_parameter("axis_color_change");

  button_all_id_1_ = this->declare_parameter("button_all_id_1");
  button_all_id_2_ = this->declare_parameter("button_all_id_2");
  button_all_id_3_ = this->declare_parameter("button_all_id_3");
  button_all_id_4_ = this->declare_parameter("button_all_id_4");

  button_path_enable_ = this->declare_parameter("button_path_enable");
  button_add_pose_ = this->declare_parameter("button_add_pose");
  button_delete_path_ = this->declare_parameter("button_delete_path");
  button_send_target_ = this->declare_parameter("button_send_target");

  auto callback = [this](const sensor_msgs::msg::Joy::SharedPtr msg) -> void {
    publish_robot_commands(msg);
  };

  pub_commands_ = create_publisher<consai2r2_msgs::msg::RobotCommands>(
      "robot_commands", 10);
  sub_joy_ = create_subscription<sensor_msgs::msg::Joy>("joy", 10, callback);
}

void JoystickComponent::publish_robot_commands(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  // TODO: WE HAVE TO USE ROS_PARAM
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

  //シャットダウン
  if (msg->buttons[button_shutdown_1_] && msg->buttons[button_shutdown_2_]) {
    rclcpp::shutdown();
    return;
  }

  if (msg->buttons[BUTTON_MOVE_ENABLE]) {
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

} // namespace joystick

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(joystick::JoystickComponent)
