cmake_minimum_required(VERSION 3.5)
project(consai2r2_sender)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(consai2r2_msgs REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

# Protobuf
include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

protobuf_generate_cpp(PROTO_CPP PROTO_H
  include/${PROJECT_NAME}/proto/grSim_Replacement.proto
  include/${PROJECT_NAME}/proto/grSim_Commands.proto
  include/${PROJECT_NAME}/proto/grSim_Packet.proto
)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})
add_executable(sim_sender
  src/sim_sender.cpp
  ${PROTO_CPP}
  ${PROTO_H}
)
ament_target_dependencies(
  sim_sender
  "rclcpp"
  "std_msgs"
  "consai2r2_msgs"
)
target_link_libraries(sim_sender ${PROTOBUF_LIBRARIES} ${Boost_LIBRARIES})


install(TARGETS sim_sender
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
