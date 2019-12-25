# Copyright (c) 2019 SSL-Roots
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

import math

from consai2r2_description import consai2r2_parameters

from consai2r2_msgs.msg import BallInfo, RobotInfo, VisionDetections
from geometry_msgs.msg import Pose2D

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.time import Time


class VisionWrapper(Node):

    def __init__(self):
        super().__init__('consai2r2_vision_wrapper')

        QUEUE_SIZE = 10
        self._DISAPPERED_TIME_THRESH = 3.0
        self._PUBLISH_ROBOT = {'blue': False, 'yellow': False}

        self.declare_parameter('publish_ball', True)
        self.declare_parameter('publish_blue', True)
        self.declare_parameter('publish_yellow', True)

        common_params = consai2r2_parameters(self)
        self._OUR_COLOR = common_params.our_color
        self._MAX_ID = common_params.max_id

        self._PUBLISH_BALL = self.get_parameter('publish_ball').value
        self._PUBLISH_ROBOT['blue'] = self.get_parameter('publish_blue').value
        self._PUBLISH_ROBOT['yellow'] = self.get_parameter(
            'publish_yellow').value

        self._pub_ball_info = None
        self._ball_info = BallInfo()

        if self._PUBLISH_BALL:
            self._pub_ball_info = self.create_publisher(
                BallInfo, '~/ball_info', QUEUE_SIZE)

        self._robot_info = {'blue': [], 'yellow': []}
        self._pubs_robot_info = {'blue': [], 'yellow': []}

        for robot_id in range(self._MAX_ID + 1):
            # 末尾に16進数の文字列をつける
            topic_id = hex(robot_id)[2:]

            self._robot_info['blue'].append(RobotInfo())
            self._robot_info['yellow'].append(RobotInfo())

            if self._PUBLISH_ROBOT['blue']:
                topic_name = '~/robot_info_blue_' + topic_id
                pub_robot_info = self.create_publisher(
                    RobotInfo, topic_name, QUEUE_SIZE)
                self._pubs_robot_info['blue'].append(pub_robot_info)

            if self._PUBLISH_ROBOT['yellow']:
                topic_name = '~/robot_info_yellow_' + topic_id
                pub_robot_info = self.create_publisher(
                    RobotInfo, topic_name, QUEUE_SIZE)
                self._pubs_robot_info['yellow'].append(pub_robot_info)

        self.create_subscription(
            VisionDetections, 'consai2r2_vision_receiver/raw_vision_detections',
            self._callback_detections, 1)

    def _callback_detections(self, msg):
        # Visionのデータからロボットとボールの位置を抽出する

        time_stamp = msg.header.stamp

        detection_balls = []
        detection_blues = []
        detection_yellows = []
        for frame in msg.frames:
            for ball in frame.balls:
                detection_balls.append(ball)

            for blue in frame.robots_blue:
                detection_blues.append(blue)

            for yellow in frame.robots_yellow:
                detection_yellows.append(yellow)

        self._extract_ball_pose(detection_balls, time_stamp)
        self._extract_robot_pose('blue', detection_blues, time_stamp)
        self._extract_robot_pose('yellow', detection_yellows, time_stamp)

    def _extract_ball_pose(self, detection_balls, time_stamp):
        # ボール座標を抽出する
        # ボール座標が複数ある場合は、平均値を座標とする
        if detection_balls:
            average_pose = self._average_pose(detection_balls)
            velocity = self._velocity(
                self._ball_info.pose, average_pose, self._ball_info.detection_stamp, time_stamp)

            self._ball_info.pose = average_pose
            self._ball_info.last_detection_pose = average_pose
            self._ball_info.velocity = velocity
            self._ball_info.detected = True
            self._ball_info.detection_stamp = time_stamp
            self._ball_info.disappeared = False
        else:
            self._ball_info.detected = False

            if self._ball_info.disappeared is False:
                # 座標を受け取らなかった場合は、速度を用いて線形予測する

                diff_time_stamp = Clock(clock_type=ClockType.ROS_TIME).now(
                ) - Time.from_msg(self._ball_info.detection_stamp)
                diff_time_secs = diff_time_stamp.nanoseconds / 1000000000

                self._ball_info.pose = self._estimate(
                    self._ball_info.last_detection_pose,
                    self._ball_info.velocity, diff_time_secs)

                # 一定時間、座標を受け取らなかったら消滅判定にする
                if diff_time_secs > self._DISAPPERED_TIME_THRESH:
                    self._ball_info.disappeared = True

        if self._PUBLISH_BALL:
            self._pub_ball_info.publish(self._ball_info)

    def _extract_robot_pose(self, color, detection_robots, time_stamp):
        # ロボット姿勢を抽出する
        # ロボット姿勢が複数ある場合は、平均値を姿勢とする
        detections = [[] for i in range(len(self._robot_info[color]))]

        # ID毎のリストに分ける
        for robot in detection_robots:
            robot_id = robot.robot_id
            detections[robot_id].append(robot)

        for robot_id, robots in enumerate(detections):
            if robots:
                average_pose = self._average_pose(robots)
                velocity = self._velocity(
                    self._robot_info[color][robot_id].pose, average_pose,
                    self._robot_info[color][robot_id].detection_stamp, time_stamp)

                self._robot_info[color][robot_id].robot_id = robot_id
                self._robot_info[color][robot_id].pose = average_pose
                self._robot_info[color][robot_id].last_detection_pose = average_pose
                self._robot_info[color][robot_id].velocity = velocity
                self._robot_info[color][robot_id].detected = True
                self._robot_info[color][robot_id].detection_stamp = time_stamp
                self._robot_info[color][robot_id].disappeared = False
            else:
                self._robot_info[color][robot_id].detected = False
                self._robot_info[color][robot_id].robot_id = robot_id

                # 座標を受け取らなかった場合は、速度を用いて線形予測する
                if self._robot_info[color][robot_id].disappeared is False:
                    duration = Clock(clock_type=ClockType.ROS_TIME).now() - \
                        Time.from_msg(self._robot_info[color][robot_id].detection_stamp)
                    diff_time_secs = duration.nanoseconds / 1000000000

                    self._robot_info[color][robot_id].pose = self._estimate(
                        self._robot_info[color][robot_id].last_detection_pose,
                        self._robot_info[color][robot_id].velocity, diff_time_secs)

                    # 一定時間、座標を受け取らなかったら消滅判定にする
                    if diff_time_secs > self._DISAPPERED_TIME_THRESH:
                        self._robot_info[color][robot_id].disappeared = True

        if self._PUBLISH_ROBOT[color]:
            for robot_id in range(len(self._pubs_robot_info[color])):
                self._pubs_robot_info[color][robot_id].publish(
                    self._robot_info[color][robot_id])

    def _average_pose(self, detections):
        # 姿勢の平均値を求める
        # 角度をpi ~ -piに収めるため、極座標系に角度を変換している

        sum_pose = Pose2D()
        detection_num = float(len(detections))

        # 角度計算用
        sum_x = 0.0
        sum_y = 0.0
        for detection in detections:
            sum_pose.x += detection.pose.x
            sum_pose.y += detection.pose.y
            sum_x += math.cos(detection.pose.theta)
            sum_y += math.sin(detection.pose.theta)
        sum_pose.x = sum_pose.x / detection_num
        sum_pose.y = sum_pose.y / detection_num
        sum_pose.theta = math.fmod(math.atan2(sum_y, sum_x), math.pi)

        return sum_pose

    def _velocity(self, prev_pose, current_pose, prev_stamp, current_stamp):
        # 姿勢の差分から速度を求める

        velocity = Pose2D()
        diff_time_stamp = Time.from_msg(
            current_stamp) - Time.from_msg(prev_stamp)
        diff_time_secs = diff_time_stamp.nanoseconds / 1000000000

        if diff_time_secs > 0:
            diff_pose = Pose2D()
            diff_pose.x = current_pose.x - prev_pose.x
            diff_pose.y = current_pose.y - prev_pose.y
            diff_pose.theta = self._angle_normalize(
                current_pose.theta - prev_pose.theta)

            velocity.x = diff_pose.x / diff_time_secs
            velocity.y = diff_pose.y / diff_time_secs
            velocity.theta = diff_pose.theta / diff_time_secs

        return velocity

    def _estimate(self, pose, velocity, diff_time_secs):
        # 速度と時間をもとに、姿勢を線形予測する

        estimated_pose = Pose2D()

        estimated_pose.x = pose.x + velocity.x * diff_time_secs
        estimated_pose.y = pose.y + velocity.y * diff_time_secs
        estimated_pose.theta = pose.theta + velocity.theta * diff_time_secs

        return estimated_pose

    def _angle_normalize(self, angle):
        # 角度をpi  ~ -piの範囲に変換する
        while angle > math.pi:
            angle -= 2*math.pi

        while angle < -math.pi:
            angle += 2*math.pi

        return angle


def main(args=None):
    rclpy.init(args=args)
    wrapper = VisionWrapper()

    rclpy.spin(wrapper)

    wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
