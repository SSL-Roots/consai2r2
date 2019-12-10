#include "rclcpp/rclcpp.hpp"

class DescriptionNode : public rclcpp::Node
{
public:
  DescriptionNode()
  : Node("consai2r2_description")
  {
    this->declare_parameter("max_id");
    this->declare_parameter("our_side");
    this->declare_parameter("our_color");
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
