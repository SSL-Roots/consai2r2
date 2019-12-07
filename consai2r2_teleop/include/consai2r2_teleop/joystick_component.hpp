#ifndef CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_
#define CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_

#include <consai2r2_msgs/msg/robot_commands.hpp>
#include <consai2r2_teleop/visibility_control.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

namespace joystick {

class JoystickComponent : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit JoystickComponent(const rclcpp::NodeOptions &options);

private:
  rclcpp::Publisher<consai2r2_msgs::msg::RobotCommands>::SharedPtr
      pub_commands_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_joy_;

  int button_shutdown_1_;
  int button_shutdown_2_;
  int button_move_enable_;

  int axis_vel_surge_;
  int axis_vel_sway_;
  int axis_vel_angular_;

  int button_kick_enable_;
  int button_kick_straight_;
  int button_kick_chip_;
  int axis_kick_power_;

  int button_dribble_enable_;
  int axis_dribble_power_;

  int button_id_enable_;
  int axis_id_change_;

  int button_color_enable_;
  int axis_color_change_;

  int button_all_id_1_;
  int button_all_id_2_;
  int button_all_id_3_;
  int button_all_id_4_;

  int button_path_enable_;
  int button_add_pose_;
  int button_delete_path_;
  int button_send_target_;

  void publish_robot_commands(const sensor_msgs::msg::Joy::SharedPtr msg);
};

} // namespace joystick

#endif // CONSAI2R2_TELEOP__JOYSTICK_COMPONENT_HPP_
