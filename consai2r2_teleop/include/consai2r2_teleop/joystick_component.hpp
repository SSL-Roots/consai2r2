#ifndef CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_
#define CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_

#include <cmath>
#include <consai2r2_msgs/msg/robot_commands.hpp>
#include <consai2r2_teleop/visibility_control.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace joystick {

struct DirectControlParams {
  float kick_power_ = 0.5f;
  float dribble_power = 0.5f;
  int robot_id_ = 0;
  bool is_yellow_ = false;
  bool all_member_control_enable = false;
};

struct ButtonConfig {
  int shutdown_1;
  int shutdown_2;
  int move_enable;
  int kick_enable;
  int kick_straight;
  int kick_chip;
  int dribble_enable;
  int id_enable;
  int color_enable;
  int all_id_1;
  int all_id_2;
  int all_id_3;
  int all_id_4;
  int path_enable;
  int add_pose;
  int delete_path;
  int send_target;
};

struct AxisConfig {
  int surge_vel;
  int sway_vel;
  int vel_angular;
  int kick_power;
  int dribble_power;
  int id_change;
  int color_change;
};
class JoystickComponent : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit JoystickComponent(const rclcpp::NodeOptions &options);

private:
  rclcpp::Publisher<consai2r2_msgs::msg::RobotCommands>::SharedPtr
      pub_commands_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

  ButtonConfig button_conf_;
  AxisConfig axis_conf_;
  static const float MAX_VEL_SURGE_ = 1.f;
  static const float MAX_VEL_SWAY_ = 1.f;
  static const float MAX_VEL_ANGULAR_ = M_PI;

  static const float MAX_KICK_POWER_ = 1.f;
  static const float MAX_POWER_CONTROL_ = 0.1f;

  static const float MAX_DRIBBLE_POWER_ = 1.f;
  static const float DRIBBLE_POWER_CONTROL_ = 0.1f;

  bool indirect_control_enable_ = true;

  DirectControlParams direct_control_;

  void publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg);
};

} // namespace joystick

#endif // CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_
