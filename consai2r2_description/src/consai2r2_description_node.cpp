#include "rclcpp/rclcpp.hpp"

class DescriptionNode : public rclcpp::Node
{
public:
  DescriptionNode()
  : Node("consai2r2_description")
  {
    this->declare_parameter("game.max_id");
    this->declare_parameter("game.player_num");
    this->declare_parameter("game.our_side");
    this->declare_parameter("game.our_color");
    this->declare_parameter("game.goalie_id");
    this->declare_parameter("game.vision_addr");
    this->declare_parameter("game.vision_port");
    this->declare_parameter("game.referee_addr");
    this->declare_parameter("game.referee_port");
    this->declare_parameter("game.grsim_addr");
    this->declare_parameter("game.grsim_port");
    this->declare_parameter("game.sender_device");
    this->declare_parameter("game.sender_baudrate");

    this->declare_parameter("geometry.robot_radius");
    this->declare_parameter("geometry.ball_radius");
  }
private:
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DescriptionNode>());
  rclcpp::shutdown();
  return 0;
}
