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

#include "consai2r2_protobuf/referee.pb.h"
#include "consai2r2_msgs/msg/referee.hpp"
#include "rclcpp/rclcpp.hpp"

#include "consai2r2_receiver/multicast.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class RefereeReceiver : public rclcpp::Node
{
public:
  RefereeReceiver()
  : Node("consai2r2_referee_receiver")
  {
    pub = this->create_publisher<consai2r2_msgs::msg::Referee>("~/raw_referee", 1);

    this->declare_parameter("referee_addr", "224.5.23.1");
    this->declare_parameter("referee_port", 10003);

    this->get_parameter("referee_addr", this->referee_addr);
    this->get_parameter("referee_port", this->referee_port);

    timer_ = create_wall_timer(16ms, std::bind(&RefereeReceiver::timer_callback, this));

    receiver = std::unique_ptr<MulticastReceiver>(
      new MulticastReceiver(this->referee_addr, this->referee_port));
  }

private:
  void timer_callback()
  {
    if (receiver->available()) {
      std::vector<char> buf(2048);
      const size_t size = receiver->receive(buf);

      if (size > 0) {
        SSL_Referee packet;
        packet.ParseFromString(std::string(buf.begin(), buf.end()));
        this->publish(packet);
      }
    }
  }

  void convert_team_info(
    const SSL_Referee_TeamInfo & proto_team, consai2r2_msgs::msg::RefereeTeamInfo & team_info)
  {
    team_info.name = proto_team.name();

    team_info.score = proto_team.score();
    team_info.red_cards = proto_team.red_cards();

    auto data = proto_team.yellow_card_times().data();
    std::vector<unsigned int> v(data, data + proto_team.yellow_card_times().size());

    team_info.yellow_card_times = v;
    team_info.yellow_cards = proto_team.yellow_cards();
    team_info.timeouts = proto_team.timeouts();
    team_info.timeout_time = proto_team.timeout_time();
    team_info.goalie = proto_team.goalie();
  }

  void publish(const SSL_Referee & packet)
  {
    auto msg = consai2r2_msgs::msg::Referee();

    // Convert Protobuf msg to ROS2 msg
    msg.stage = packet.stage();
    msg.command = packet.command();
    msg.command_counter = packet.command_counter();

    this->convert_team_info(packet.blue(), msg.blue);
    this->convert_team_info(packet.yellow(), msg.yellow);

    if (packet.has_designated_position()) {
      auto position = packet.designated_position();
      msg.designated_position.x = position.x() * 0.001;  // millimeter to meter
      msg.designated_position.y = position.y() * 0.001;  // millimeter to meter
      msg.designated_position.z = 0.0;
    }

    if (packet.has_gameevent()) {
      auto gameEvent = packet.gameevent();
      msg.game_event.game_event_type = gameEvent.gameeventtype();
      msg.game_event.originator_team = gameEvent.originator().team();
      msg.game_event.originator_bot_id = gameEvent.originator().botid();
      msg.game_event.message = gameEvent.message();
    }

    this->pub->publish(msg);
  }

  rclcpp::Publisher<consai2r2_msgs::msg::Referee>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<MulticastReceiver> receiver;
  std::string referee_addr;
  int referee_port;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RefereeReceiver>());
  rclcpp::shutdown();
  return 0;
}
