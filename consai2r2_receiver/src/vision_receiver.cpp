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

#include "consai2r2_protobuf/messages_robocup_ssl_wrapper.pb.h"
#include "consai2r2_msgs/msg/vision_detections.hpp"
#include "consai2r2_msgs/msg/vision_geometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "consai2r2_receiver/multicast.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

static const double TO_METER = 0.001;

class FormatConverter
{
public:
  FormatConverter() {vision_detections = consai2r2_msgs::msg::VisionDetections();}

  void append_frame(const SSL_DetectionFrame & packet_detection)
  {
    auto detection_frame = consai2r2_msgs::msg::DetectionFrame();

    detection_frame.t_capture = packet_detection.t_capture();
    detection_frame.t_sent = packet_detection.t_sent();
    detection_frame.camera_id = packet_detection.camera_id();

    // ball
    auto balls = packet_detection.balls();
    for (auto ball = balls.begin(); ball != balls.end(); ball++) {
      detection_frame.balls.push_back(this->convert_to_ball_topic(*ball));
    }

    // blue robot
    auto robots_blue = packet_detection.robots_blue();
    for (auto robot = robots_blue.begin(); robot != robots_blue.end(); robot++) {
      detection_frame.robots_blue.push_back(this->convert_to_robot_topic(*robot));
    }

    // yellow robot
    auto robots_yellow = packet_detection.robots_yellow();
    for (auto robot = robots_yellow.begin(); robot != robots_yellow.end(); robot++) {
      detection_frame.robots_yellow.push_back(this->convert_to_robot_topic(*robot));
    }

    this->vision_detections.frames.push_back(detection_frame);
  }

  void frame_clear() {this->vision_detections.frames.clear();}

  consai2r2_msgs::msg::VisionDetections get_frame() {return this->vision_detections;}

private:
  consai2r2_msgs::msg::DetectionBall convert_to_ball_topic(const SSL_DetectionBall & raw_ball)
  {
    auto detection_ball = consai2r2_msgs::msg::DetectionBall();

    detection_ball.pose.x = raw_ball.x() * TO_METER;
    detection_ball.pose.y = raw_ball.y() * TO_METER;

    return detection_ball;
  }

  consai2r2_msgs::msg::DetectionRobot convert_to_robot_topic(const SSL_DetectionRobot & raw_robot)
  {
    auto detection_robot = consai2r2_msgs::msg::DetectionRobot();

    detection_robot.robot_id = raw_robot.robot_id();
    detection_robot.pose.x = raw_robot.x() * TO_METER;
    detection_robot.pose.y = raw_robot.y() * TO_METER;
    detection_robot.pose.theta = raw_robot.orientation();

    return detection_robot;
  }

  consai2r2_msgs::msg::VisionDetections vision_detections;
};

class VisionReceiver : public rclcpp::Node
{
public:
  VisionReceiver()
  : Node("consai2r2_vision_receiver"), converter()
  {
    std::string host;
    int port;

    pub_geometry =
      this->create_publisher<consai2r2_msgs::msg::VisionGeometry>("~/raw_vision_geometry", 1);
    pub_detection =
      this->create_publisher<consai2r2_msgs::msg::VisionDetections>("~/raw_vision_detections", 1);

    this->declare_parameter("vision_addr", "224.5.23.2");
    this->declare_parameter("vision_port", 10006);

    this->get_parameter("vision_addr", host);
    this->get_parameter("vision_port", port);

    timer_ = create_wall_timer(16ms, std::bind(&VisionReceiver::timer_callback, this));

    receiver = std::unique_ptr<MulticastReceiver>(new MulticastReceiver(host, port));
  }

private:
  void timer_callback()
  {
    this->converter.frame_clear();

    while (receiver->available()) {
      std::vector<char> buf(2048);
      const size_t size = receiver->receive(buf);

      if (size > 0) {
        SSL_WrapperPacket packet;
        packet.ParseFromString(std::string(buf.begin(), buf.end()));

        if (packet.has_detection()) {
          this->converter.append_frame(packet.detection());
        }
        if (packet.has_geometry()) {
          this->publish_geometry(packet.geometry().field());
        }
      }
    }
    this->publish_detection();
  }

  void publish_detection()
  {
    auto vision_detection = this->converter.get_frame();

    if (vision_detection.frames.size() > 0) {
      rclcpp::Clock ros_clock(RCL_ROS_TIME);
      vision_detection.header.stamp = ros_clock.now();
      this->pub_detection->publish(vision_detection);
    }
  }

  void publish_geometry(const SSL_GeometryFieldSize & packet_field)
  {
    auto geometry = consai2r2_msgs::msg::VisionGeometry();

    geometry.field_length = packet_field.field_length() * TO_METER;
    geometry.field_width = packet_field.field_width() * TO_METER;
    geometry.goal_width = packet_field.goal_width() * TO_METER;
    geometry.goal_depth = packet_field.goal_depth() * TO_METER;
    geometry.boundary_width = static_cast<int>(packet_field.boundary_width() * TO_METER);

    auto lines = packet_field.field_lines();
    for (auto line = lines.begin(); line != lines.end(); line++) {
      auto line_segment = consai2r2_msgs::msg::FieldLineSegment();

      line_segment.name = line->name();
      line_segment.p1_x = line->p1().x() * TO_METER;
      line_segment.p1_y = line->p1().y() * TO_METER;
      line_segment.p2_x = line->p2().x() * TO_METER;
      line_segment.p2_y = line->p2().y() * TO_METER;
      line_segment.thickness = line->thickness() * TO_METER;
      geometry.field_lines.push_back(line_segment);
    }

    auto arcs = packet_field.field_arcs();
    for (auto arc = arcs.begin(); arc != arcs.end(); arc++) {
      auto circular_arc = consai2r2_msgs::msg::FieldCircularArc();

      circular_arc.name = arc->name();
      circular_arc.center_x = arc->center().x() * TO_METER;
      circular_arc.center_y = arc->center().y() * TO_METER;
      circular_arc.radius = arc->radius() * TO_METER;
      circular_arc.a1 = arc->a1();
      circular_arc.a2 = arc->a2();
      circular_arc.thickness = arc->thickness() * TO_METER;
      geometry.field_arcs.push_back(circular_arc);
    }

    this->pub_geometry->publish(geometry);
  }

  rclcpp::Publisher<consai2r2_msgs::msg::VisionGeometry>::SharedPtr pub_geometry;
  rclcpp::Publisher<consai2r2_msgs::msg::VisionDetections>::SharedPtr pub_detection;
  rclcpp::TimerBase::SharedPtr timer_;

  std::unique_ptr<MulticastReceiver> receiver;
  FormatConverter converter;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VisionReceiver>());
  rclcpp::shutdown();
  return 0;
}
