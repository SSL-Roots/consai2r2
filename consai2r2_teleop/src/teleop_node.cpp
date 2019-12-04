
#include <rclcpp/rclcpp.hpp>
#include <memory>

#include "consai2r2_teleop/joystick_component.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;
  rclcpp::NodeOptions options;
  std::shared_ptr<joystick::JoystickComponent> joystick_node =
    std::make_shared<joystick::JoystickComponent>(options);
  exe.add_node(joystick_node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
}
