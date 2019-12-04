
#include <iostream>
#include <memory>

// for UDP communication
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "consai2_msgs/msg/robot_commands.hpp"

#include "grSim_Packet.pb.h"
#include "grSim_Commands.pb.h"
#include "grSim_Replacement.pb.h"


using std::placeholders::_1;

class simple_udp{
    int sock;
    struct sockaddr_in addr;
public:
    simple_udp(std::string address, int port){
        sock = socket(AF_INET, SOCK_DGRAM, 0);
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = inet_addr(address.c_str());
        addr.sin_port = htons(port);        
    }
    void udp_send(std::string word){
        sendto(sock, word.c_str(), word.length(), 0, (struct sockaddr *)&addr, sizeof(addr));
    }

    ~simple_udp(){
        close(sock);
    }
};

class SimSender : public rclcpp::Node
{
public:
  SimSender()
  : Node("consai2_sim_sender")
  {
    sub_commands_ = this->create_subscription<consai2_msgs::msg::RobotCommands>(
      "robot_commands",
      10,
      std::bind(&SimSender::send_commands, this, std::placeholders::_1)
      );

    // TODO: USE ROS_PARAM FOR ADDR AND PORT
    udp_ = std::make_shared<simple_udp>("127.0.0.1", 20011);
  }

private:
  void send_commands(const consai2_msgs::msg::RobotCommands::SharedPtr msg) const 
  {
    const double MAX_KICK_SPEED = 8.0; // m/s
    grSim_Commands *packet_commands = new grSim_Commands();

    packet_commands->set_timestamp(msg->header.stamp.sec);
    packet_commands->set_isteamyellow(msg->is_yellow);

    for(auto command : msg->commands)
    {
      grSim_Robot_Command *robot_command = packet_commands->add_robot_commands();
      robot_command->set_id(command.robot_id);

      // 走行速度
      robot_command->set_veltangent(command.vel_surge);
      robot_command->set_velnormal(command.vel_sway);
      robot_command->set_velangular(command.vel_angular);

      // キック速度
      double kick_speed = command.kick_power * MAX_KICK_SPEED;
      robot_command->set_kickspeedx(kick_speed);

      // チップキック
      if(command.chip_enable) {
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

  rclcpp::Subscription<consai2_msgs::msg::RobotCommands>::SharedPtr sub_commands_;
  std::shared_ptr<simple_udp> udp_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimSender>());
  rclcpp::shutdown();
  return 0;
}