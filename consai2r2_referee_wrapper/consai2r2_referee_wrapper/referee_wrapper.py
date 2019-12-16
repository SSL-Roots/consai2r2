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
from consai2r2_msgs.msg import BallInfo, DecodedReferee, Referee
from geometry_msgs.msg import Pose2D
import rclpy
from rclpy.node import Node
from referee_pb2 import SSL_Referee

REFEREE_TEXT = {
    0: 'HALT', 1: 'STOP', 3: 'FORCE_START',
    11: 'OUR_KICKOFF_PREPARATION', 12: 'OUR_KICKOFF_START',
    13: 'OUR_PENALTY_PREPARATION', 14: 'OUR_PENALTY_START',
    15: 'OUR_DIRECT_FREE', 16: 'OUR_INDIRECT_FREE',
    17: 'OUR_TIMEOUT', 18: 'OUR_GOAL', 19: 'OUR_BALL_PLACEMENT',
    21: 'THEIR_KICKOFF_PREPARATION', 22: 'THEIR_KICKOFF_START',
    23: 'THEIR_PENALTY_PREPARATION', 24: 'THEIR_PENALTY_START',
    25: 'THEIR_DIRECT_FREE', 26: 'THEIR_INDIRECT_FREE',
    27: 'THEIR_TIMEOUT', 28: 'THEIR_GOAL', 29: 'THEIR_BALL_PLACEMENT',
}
REFEREE_ID = {v: k for k, v in REFEREE_TEXT.items()}


class RefereeWrapper(Node):

    def __init__(self):
        super().__init__('consai2r2_referee_wrapper')

        common_params = consai2r2_parameters(self)
        self._OUR_COLOR = common_params.our_color
        self._OUR_SIDE = common_params.our_side

        self._sub_ball_info = self.create_subscription(
            BallInfo, 'consai2r2_vision_wrapper/ball_info', self._callback_ball_info, 1)
        self._sub_referee = self.create_subscription(
            Referee, 'consai2r2_referee_receiver/raw_referee', self._callback_referee, 1)

        self._pub_decoded_referee = self.create_publisher(DecodedReferee, '~/decoded_referee', 1)

        self._DECODE_ID = {
            'OUR': 10, 'THEIR': 20,
            'HALT': 0, 'STOP': 1, 'FORCE_START': 3,  # 定数の代わりに定義
            'KICKOFF_PREPARATION': 1, 'KICKOFF_START': 2,
            'PENALTY_PREPARATION': 3, 'PENALTY_START': 4,
            'DIRECT_FREE': 5, 'INDIRECT_FREE': 6,
            'TIMEOUT': 7, 'GOAL': 8, 'BALL_PLACEMENT': 9,
        }

        self._ID_BLUE = self._DECODE_ID['OUR']
        self._ID_YELLOW = self._DECODE_ID['THEIR']

        if self._OUR_COLOR != 'blue':
            self._ID_BLUE = self._DECODE_ID['THEIR']
            self._ID_YELLOW = self._DECODE_ID['OUR']

        self._INPLAY_DISTANCE = 0.05  # meter

        self._SPEED_LIMIT_OF_ROBOT = 1.5  # meters / sec
        self._SPEED_LIMIT_OF_BALL = 6.5  # meters / sec
        self._KEEP_OUT_RADIUS_FROM_BALL = 0.5  # meters
        self._KEEP_OUT_DISTANCE_FROM_THEIR_DEFENSE_AREA = 0.2  # meters
        self._NO_LIMIT = -1.0

        self._prev_referee = Referee()
        self._prev_decoded_msg = DecodedReferee()

        self._ball_pose = Pose2D()
        self._stationary_ball_pose = Pose2D()
        self._game_is_inplay = False

    def _callback_ball_info(self, msg):
        self._ball_pose = msg.pose

    def _callback_referee(self, msg):
        # Refereeのデータをチームカラーに合わせて解釈する

        decoded_msg = self._decode_referee(msg)

        self._pub_decoded_referee.publish(decoded_msg)

        # NORMAL_STARTの解釈に前回のコマンドが必要
        self._prev_referee = msg
        self._prev_decoded_msg = decoded_msg

    def _decode_referee_id(self, referee_command):
        decoded_id = 0

        # HALT, STOP, FORCE_STARTはチームカラーに依存しない
        if referee_command == SSL_Referee.HALT:
            decoded_id = SSL_Referee.HALT
        elif referee_command == SSL_Referee.STOP:
            decoded_id = SSL_Referee.STOP
        elif referee_command == SSL_Referee.FORCE_START:
            decoded_id = SSL_Referee.FORCE_START
        elif referee_command == SSL_Referee.NORMAL_START:
            # 複数回IDに加算するのを防ぐため、前回のコマンドと比較する
            if self._prev_referee.command != SSL_Referee.NORMAL_START:
                # PREPARATIONのIDに1を加えたものがSTARTのIDになる
                decoded_id = self._prev_decoded_msg.referee_id + 1
            else:
                decoded_id = self._prev_decoded_msg.referee_id
        else:
            # チームカラーに合わせてコマンドの解釈を変える
            if referee_command == SSL_Referee.PREPARE_KICKOFF_YELLOW:
                decoded_id = self._ID_YELLOW + self._DECODE_ID['KICKOFF_PREPARATION']
            if referee_command == SSL_Referee.PREPARE_KICKOFF_BLUE:
                decoded_id = self._ID_BLUE + self._DECODE_ID['KICKOFF_PREPARATION']
            if referee_command == SSL_Referee.PREPARE_PENALTY_YELLOW:
                decoded_id = self._ID_YELLOW + self._DECODE_ID['PENALTY_PREPARATION']
            if referee_command == SSL_Referee.PREPARE_PENALTY_BLUE:
                decoded_id = self._ID_BLUE + self._DECODE_ID['PENALTY_PREPARATION']
            if referee_command == SSL_Referee.DIRECT_FREE_YELLOW:
                decoded_id = self._ID_YELLOW + self._DECODE_ID['DIRECT_FREE']
            if referee_command == SSL_Referee.DIRECT_FREE_BLUE:
                decoded_id = self._ID_BLUE + self._DECODE_ID['DIRECT_FREE']
            if referee_command == SSL_Referee.INDIRECT_FREE_YELLOW:
                decoded_id = self._ID_YELLOW + self._DECODE_ID['INDIRECT_FREE']
            if referee_command == SSL_Referee.INDIRECT_FREE_BLUE:
                decoded_id = self._ID_BLUE + self._DECODE_ID['INDIRECT_FREE']
            if referee_command == SSL_Referee.TIMEOUT_YELLOW:
                decoded_id = self._ID_YELLOW + self._DECODE_ID['TIMEOUT']
            if referee_command == SSL_Referee.TIMEOUT_BLUE:
                decoded_id = self._ID_BLUE + self._DECODE_ID['TIMEOUT']
            if referee_command == SSL_Referee.GOAL_YELLOW:
                decoded_id = self._ID_YELLOW + self._DECODE_ID['GOAL']
            if referee_command == SSL_Referee.GOAL_BLUE:
                decoded_id = self._ID_BLUE + self._DECODE_ID['GOAL']
            if referee_command == SSL_Referee.BALL_PLACEMENT_YELLOW:
                decoded_id = self._ID_YELLOW + self._DECODE_ID['BALL_PLACEMENT']
            if referee_command == SSL_Referee.BALL_PLACEMENT_BLUE:
                decoded_id = self._ID_BLUE + self._DECODE_ID['BALL_PLACEMENT']

        return decoded_id

    def _decode_referee(self, msg):
        decoded_msg = DecodedReferee()

        decoded_msg.referee_id = self._decode_referee_id(msg.command)
        decoded_msg.referee_text = REFEREE_TEXT.get(decoded_msg.referee_id, 'INVALID_COMMAND')

        # サイドチェンジに対応する
        decoded_msg.placement_position = msg.designated_position
        if self._OUR_SIDE != 'left':
            decoded_msg.placement_position.x = -decoded_msg.placement_position.x
            decoded_msg.placement_position.y = -decoded_msg.placement_position.y

        # Default Settings
        decoded_msg.can_move_robot = False
        decoded_msg.speed_limit_of_robot = self._NO_LIMIT
        decoded_msg.can_kick_ball = False
        decoded_msg.can_enter_their_side = False
        decoded_msg.can_enter_center_circle = False
        decoded_msg.is_inplay = False
        decoded_msg.keep_out_radius_from_ball = self._NO_LIMIT
        decoded_msg.keep_out_distance_from_their_defense_area = self._NO_LIMIT

        # Decode restrictions
        if decoded_msg.referee_id == self._DECODE_ID['HALT'] \
                or decoded_msg.referee_id == self._DECODE_ID['OUR'] + self._DECODE_ID['GOAL'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] + self._DECODE_ID['GOAL']:
            # Reference : Rule 2019, 5.1.2 Halt
            pass  # Keep default settings

        elif decoded_msg.referee_id == self._DECODE_ID['STOP']:
            # Reference : Rule 2019, 5.1.1 Stop
            decoded_msg.can_move_robot = True
            decoded_msg.speed_limit_of_robot = self._SPEED_LIMIT_OF_ROBOT
            decoded_msg.can_enter_their_side = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.keep_out_radius_from_ball = self._KEEP_OUT_RADIUS_FROM_BALL
            decoded_msg.keep_out_distance_from_their_defense_area = \
                self._KEEP_OUT_DISTANCE_FROM_THEIR_DEFENSE_AREA

        elif decoded_msg.referee_id == self._DECODE_ID['FORCE_START']:
            # Reference : Rule 2019, 5.3.5 Force Start
            # Reference : Rule 2019, 8.1.6 Ball Speed
            decoded_msg.can_move_robot = True
            decoded_msg.can_kick_ball = True
            decoded_msg.can_enter_their_side = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.is_inplay = True

        elif decoded_msg.referee_id == self._DECODE_ID['OUR'] \
                + self._DECODE_ID['KICKOFF_PREPARATION']:
            # Reference : Rule 2019, 5.3.2 Kick-Off
            decoded_msg.can_move_robot = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.keep_out_radius_from_ball = 0.0  # No limit but robot do not touch the ball
            decoded_msg.keep_out_distance_from_their_defense_area = \
                self._KEEP_OUT_DISTANCE_FROM_THEIR_DEFENSE_AREA

        elif decoded_msg.referee_id == self._DECODE_ID['OUR'] + self._DECODE_ID['KICKOFF_START']:
            # Reference : Rule 2019, 5.3.1 Normal Start
            # Reference : Rule 2019, 5.3.2 Kick-Off
            decoded_msg.can_move_robot = True
            decoded_msg.can_kick_ball = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.keep_out_distance_from_their_defense_area = \
                self._KEEP_OUT_DISTANCE_FROM_THEIR_DEFENSE_AREA

        elif decoded_msg.referee_id == self._DECODE_ID['OUR'] \
                + self._DECODE_ID['PENALTY_PREPARATION']:
            # Reference : Rule 2019, 5.3.6 Penalty Kick
            decoded_msg.can_move_robot = True
            decoded_msg.can_kick_ball = False
            decoded_msg.can_enter_their_side = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.keep_out_radius_from_ball = 0.0  # No limit but robot do not touch the ball

        elif decoded_msg.referee_id == self._DECODE_ID['OUR'] + self._DECODE_ID['PENALTY_START']:
            # Reference : Rule 2019, 5.3.1 Normal Start
            # Reference : Rule 2019, 5.3.6 Penalty Kick
            decoded_msg.can_move_robot = True
            decoded_msg.can_kick_ball = True
            decoded_msg.can_enter_their_side = True
            decoded_msg.can_enter_center_circle = True

        elif decoded_msg.referee_id == self._DECODE_ID['OUR'] + self._DECODE_ID['DIRECT_FREE'] \
                or decoded_msg.referee_id == self._DECODE_ID['OUR'] \
                + self._DECODE_ID['INDIRECT_FREE']:
            # Reference : Rule 2019, 5.3.3 Direct Free Kick
            # Reference : Rule 2019, 5.3.6 Indirect Free Kick
            decoded_msg.can_move_robot = True
            decoded_msg.can_kick_ball = True
            decoded_msg.can_enter_their_side = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.keep_out_distance_from_their_defense_area = \
                self._KEEP_OUT_DISTANCE_FROM_THEIR_DEFENSE_AREA

        elif decoded_msg.referee_id == self._DECODE_ID['OUR'] + self._DECODE_ID['BALL_PLACEMENT']:
            # Reference : Rule 2019, 5.2 Ball Placement
            # Reference : Rule 2019, 8.2.8 Robot Stop Speed
            decoded_msg.can_move_robot = True
            decoded_msg.can_kick_ball = True
            decoded_msg.can_enter_their_side = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.keep_out_distance_from_their_defense_area = \
                self._KEEP_OUT_DISTANCE_FROM_THEIR_DEFENSE_AREA

        elif decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['KICKOFF_PREPARATION'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['KICKOFF_START']:
            # Reference : Rule 2019, 5.3.2 Kick-Off
            decoded_msg.can_move_robot = True
            decoded_msg.keep_out_radius_from_ball = self._KEEP_OUT_RADIUS_FROM_BALL
            decoded_msg.keep_out_distance_from_their_defense_area = \
                self._KEEP_OUT_DISTANCE_FROM_THEIR_DEFENSE_AREA

        elif decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
            + self._DECODE_ID['PENALTY_PREPARATION'] \
            or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['PENALTY_START']:
            # Reference : Rule 2019, 5.3.6 Penalty Kick
            decoded_msg.can_move_robot = True
            decoded_msg.can_enter_their_side = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.keep_out_radius_from_ball = self._KEEP_OUT_RADIUS_FROM_BALL
            decoded_msg.keep_out_distance_from_their_defense_area = \
                self._KEEP_OUT_DISTANCE_FROM_THEIR_DEFENSE_AREA

        elif decoded_msg.referee_id == self._DECODE_ID['THEIR'] + self._DECODE_ID['DIRECT_FREE'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['INDIRECT_FREE'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['BALL_PLACEMENT']:
            # Reference : Rule 2019, 5.3.3 Direct Free Kick
            # Reference : Rule 2019, 5.3.6 Indirect Free Kick
            # Reference : Rule 2019, 8.2.3 Ball Placement Interference
            decoded_msg.can_move_robot = True
            decoded_msg.can_enter_their_side = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.keep_out_radius_from_ball = self._KEEP_OUT_RADIUS_FROM_BALL
            decoded_msg.keep_out_distance_from_their_defense_area = \
                self._KEEP_OUT_DISTANCE_FROM_THEIR_DEFENSE_AREA

        elif decoded_msg.referee_id == self._DECODE_ID['OUR'] + self._DECODE_ID['TIMEOUT'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] + self._DECODE_ID['TIMEOUT']:
            # Reference : Rule 2019, 4.4.2 Timeouts
            # No limitations
            decoded_msg.can_move_robot = True
            decoded_msg.speed_limit_of_robot = self._NO_LIMIT
            decoded_msg.can_kick_ball = True
            decoded_msg.can_enter_their_side = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.keep_out_radius_from_ball = self._NO_LIMIT
            decoded_msg.keep_out_distance_from_their_defense_area = self._NO_LIMIT

        else:
            decoded_msg.can_move_robot = False
            decoded_msg.speed_limit_of_robot = self._NO_LIMIT
            decoded_msg.can_kick_ball = False
            decoded_msg.can_enter_their_side = False
            decoded_msg.can_enter_center_circle = False
            decoded_msg.keep_out_radius_from_ball = self._NO_LIMIT
            decoded_msg.keep_out_distance_from_their_defense_area = self._NO_LIMIT

        # Consider inplay
        # Reference : Rule 2019, 8.1.3 Double Touch
        # Reference : Rule 2019, A.1 Ball In And Out Of Play
        if decoded_msg.referee_id == self._DECODE_ID['HALT'] \
                or decoded_msg.referee_id == self._DECODE_ID['STOP'] \
                or decoded_msg.referee_id == self._DECODE_ID['OUR'] \
                + self._DECODE_ID['KICKOFF_PREPARATION'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['KICKOFF_PREPARATION'] \
                or decoded_msg.referee_id == self._DECODE_ID['OUR'] \
                + self._DECODE_ID['PENALTY_PREPARATION'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['PENALTY_PREPARATION'] \
                or decoded_msg.referee_id == self._DECODE_ID['OUR'] \
                + self._DECODE_ID['BALL_PLACEMENT'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['BALL_PLACEMENT']:
            self._stationary_ball_pose = self._ball_pose
            self._game_is_inplay = False

        elif decoded_msg.referee_id == self._DECODE_ID['OUR'] \
            + self._DECODE_ID['KICKOFF_START'] \
                or decoded_msg.referee_id == self._DECODE_ID['OUR'] \
                + self._DECODE_ID['PENALTY_START'] \
                or decoded_msg.referee_id == self._DECODE_ID['OUR'] \
                + self._DECODE_ID['DIRECT_FREE'] \
                or decoded_msg.referee_id == self._DECODE_ID['OUR'] \
                + self._DECODE_ID['INDIRECT_FREE'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['KICKOFF_START'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['PENALTY_START'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['DIRECT_FREE'] \
                or decoded_msg.referee_id == self._DECODE_ID['THEIR'] \
                + self._DECODE_ID['INDIRECT_FREE']:

            # ボールが静止位置から動いたかを判断する
            if self._game_is_inplay is False:
                diff_pose = Pose2D()
                diff_pose.x = self._ball_pose.x - self._stationary_ball_pose.x
                diff_pose.y = self._ball_pose.y - self._stationary_ball_pose.y
                move_distance = math.hypot(diff_pose.x, diff_pose.y)

                if move_distance > self._INPLAY_DISTANCE:
                    self._game_is_inplay = True

        # インプレイのときは行動制限を解除する
        if self._game_is_inplay is True:
            decoded_msg.can_move_robot = True
            decoded_msg.speed_limit_of_robot = self._NO_LIMIT
            decoded_msg.can_kick_ball = True
            decoded_msg.can_enter_their_side = True
            decoded_msg.can_enter_center_circle = True
            decoded_msg.keep_out_radius_from_ball = self._NO_LIMIT
            decoded_msg.keep_out_distance_from_their_defense_area = self._NO_LIMIT
            decoded_msg.is_inplay = True

            decoded_msg.referee_text += '(INPLAY)'

        return decoded_msg


def main(args=None):
    rclpy.init(args=args)
    wrapper = RefereeWrapper()

    rclpy.spin(wrapper)

    wrapper.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
