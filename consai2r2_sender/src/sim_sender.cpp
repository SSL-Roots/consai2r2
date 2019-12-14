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

// for UDP communication
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <iostream>
#include <memory>
#include <string>

#include "consai2r2_msgs/msg/robot_commands.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "consai2r2_protobuf/grSim_Commands.pb.h"
#include "consai2r2_protobuf/grSim_Packet.pb.h"
#include "consai2r2_protobuf/grSim_Replacement.pb.h"

using std::placeholders::_1;

class simple_udp
{
  int sock;
  struct sockaddr_in addr;

public:
  simple_udp(std::string address, int port)
  {
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = inet_addr(address.c_str());
    addr.sin_port = htons(port);
  }
  void udp_send(std::string word)
  {
    sendto(sock, word.c_str(), word.length(), 0, (struct sockaddr *)&addr, sizeof(addr));
  }

  ~simple_udp() {close(sock);}
};

class SimSender : public rclcpp::Node
{
public:
  SimSender()
  : Node("consai2r2_sim_sender")
  {
    std::string host;
    int port;

    this->declare_parameter("grsim_addr", "127.0.0.1");
    this->declare_parameter("grsim_port", 20011);

    this->get_parameter("grsim_addr", host);
    this->get_parameter("grsim_port", port);

    sub_commands_ = this->create_subscription<consai2r2_msgs::msg::RobotCommands>(
      "robot_commands", 10, std::bind(&SimSender::send_commands, this, std::placeholders::_1));

    udp_ = std::make_shared<simple_udp>(host, port);
  }

private:
  void send_commands(const consai2r2_msgs::msg::RobotCommands::SharedPtr msg) const
  {
    const double MAX_KICK_SPEED = 8.0;  // m/s
    grSim_Commands * packet_commands = new grSim_Commands();

    packet_commands->set_timestamp(msg->header.stamp.sec);
    packet_commands->set_isteamyellow(msg->is_yellow);

    for (auto command : msg->commands) {
      grSim_Robot_Command * robot_command = packet_commands->add_robot_commands();
      robot_command->set_id(command.robot_id);

      // 走行速度
      robot_command->set_veltangent(command.vel_surge);
      robot_command->set_velnormal(command.vel_sway);
      robot_command->set_velangular(command.vel_angular);

      // キック速度
      double kick_speed = command.kick_power * MAX_KICK_SPEED;
      robot_command->set_kickspeedx(kick_speed);

      // チップキック
      if (command.chip_enable) {
        robot_command->set_kickspeedz(kick_speed);
      } else {
        robot_command->set_kickspeedz(0);
      }

      // ドリブル
      robot_command->set_spinner(command.dribble_power > 0);

      // タイヤ個別に速度設定しない
      robot_command->set_wheelsspeed(false);
    }

    grSim_Packet packet;
    packet.set_allocated_commands(packet_commands);

    std::string output;
    packet.SerializeToString(&output);
    udp_->udp_send(output);
  }

  rclcpp::Subscription<consai2r2_msgs::msg::RobotCommands>::SharedPtr sub_commands_;
  std::shared_ptr<simple_udp> udp_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimSender>());
  rclcpp::shutdown();
  return 0;
}
