# game status for consai2_game

# Declare of the constants of referee_id
uint8 ID_HALT = 0
uint8 ID_STOP = 1
uint8 ID_FORCE_START = 3
uint8 ID_OUR_KICKOFF_PREPARATION = 11
uint8 ID_OUR_KICKOFF_START = 12
uint8 ID_OUR_PENALTY_PREPARATION = 13
uint8 ID_OUR_PENALTY_START = 14
uint8 ID_OUR_DIRECT_FREE = 15
uint8 ID_OUR_INDIRECT_FREE = 16
uint8 ID_OUR_TIMEOUT = 17
uint8 ID_OUR_GOAL = 18
uint8 ID_OUR_BALL_PLACEMENT = 19
uint8 ID_THEIR_KICKOFF_PREPARATION = 21
uint8 ID_THEIR_KICKOFF_START = 22
uint8 ID_THEIR_PENALTY_PREPARATION = 23
uint8 ID_THEIR_PENALTY_START = 24
uint8 ID_THEIR_DIRECT_FREE = 25
uint8 ID_THEIR_INDIRECT_FREE = 26
uint8 ID_THEIR_TIMEOUT = 27
uint8 ID_THEIR_GOAL = 28
uint8 ID_THEIR_BALL_PLACEMENT = 29


uint8   referee_id
string  referee_text

bool    can_move_robot
float64 speed_limit_of_robot # meter/sec
bool    can_kick_ball
bool    can_enter_their_side
bool    can_enter_center_circle
bool    is_inplay
float64 keep_out_radius_from_ball # meter
float64 keep_out_distance_from_their_defense_area # meter

geometry_msgs/Point placement_position
