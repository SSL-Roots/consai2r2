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
#include <exception>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "consai2r2_protobuf/messages_robocup_ssl_robot_status.pb.h"
#include "consai2r2_msgs/msg/robot_status.hpp"
#include "rclcpp/rclcpp.hpp"

#include "consai2r2_receiver/multicast.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RobotStatusReceiver : public rclcpp::Node
{
public:
  RobotStatusReceiver()
  : Node("consai2r2_sim_status_receiver")
  {
    this->declare_parameter("vision_addr", "224.5.23.2");
    this->declare_parameter("status_port_blue", 30011);
    this->declare_parameter("status_port_yellow", 30012);

    this->get_parameter("vision_addr", this->vision_addr);

    auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(this,
        "consai2r2_description");
    if (!parameters_client->wait_for_service(5s)) {
      throw std::runtime_error("Wait for service timed out");
    }

    auto parameters = parameters_client->get_parameters({"max_id", "our_color"});

    this->max_id = parameters[0].as_int();
    this->our_color = parameters[1].as_string();

    for (auto robot_id = 0; robot_id <= max_id; robot_id++) {
      std::ostringstream topic_name;
      topic_name << "~/robot_status_";
      topic_name << std::hex << robot_id;
      this->pub_status.push_back(
        this->create_publisher<consai2r2_msgs::msg::RobotStatus>(
          topic_name.str(), 1));
    }

    if (this->our_color == std::string("blue")) {
      this->get_parameter("status_port_blue", this->status_port);
    } else {
      this->get_parameter("status_port_yellow", this->status_port);
    }

    receiver = std::unique_ptr<MulticastReceiver>(
      new MulticastReceiver(this->vision_addr, this->status_port));

    timer_ = create_wall_timer(16ms, std::bind(&RobotStatusReceiver::timer_callback, this));
  }

private:
  void timer_callback()
  {
    if (receiver->available()) {
      std::vector<char> buf(2048);
      const size_t size = receiver->receive(buf);

      if (size > 0) {
        Robots_Status packet;
        packet.ParseFromString(std::string(buf.begin(), buf.end()));
        this->publish(packet);
      }
    }
  }

  void convert_status_info(
    const Robot_Status & proto_status, consai2r2_msgs::msg::RobotStatus & status)
  {
    status.robot_id = proto_status.robot_id();
    status.infrared = proto_status.infrared();
    status.flat_kick = proto_status.flat_kick();
    status.chip_kick = proto_status.chip_kick();
  }

  void publish(const Robots_Status & packet)
  {
    auto robots_status = packet.robots_status();
    // Convert Protobuf msg to ROS2 msg
    for (
      auto robot_status = robots_status.begin();
      robot_status != robots_status.end();
      robot_status++)
    {
      auto msg = consai2r2_msgs::msg::RobotStatus();
      this->convert_status_info(*robot_status, msg);

      this->pub_status[msg.robot_id]->publish(msg);
    }
  }

  std::vector<rclcpp::Publisher<consai2r2_msgs::msg::RobotStatus>::SharedPtr> pub_status;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<MulticastReceiver> receiver;
  std::string vision_addr;
  int status_port;
  int max_id;
  std::string our_color;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RobotStatusReceiver>());
  rclcpp::shutdown();
  return 0;
}
