cmake_minimum_required(VERSION 3.5)
project(consai2r2_msgs)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(msg_files
  "msg/RefereeGameEvent.msg"
  "msg/RefereeTeamInfo.msg"
  "msg/Referee.msg"
  "msg/FieldCircularArc.msg"
  "msg/FieldLineSegment.msg"
  "msg/VisionGeometry.msg"
  "msg/DetectionBall.msg"
  "msg/DetectionRobot.msg"
  "msg/DetectionFrame.msg"
  "msg/VisionDetections.msg"
  "msg/RobotInfo.msg"
  "msg/BallInfo.msg"
  "msg/DecodedReferee.msg"
  "msg/ReplaceBall.msg"
  "msg/RobotCommand.msg"
  "msg/RobotCommands.msg"
  "msg/RobotStatus.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
