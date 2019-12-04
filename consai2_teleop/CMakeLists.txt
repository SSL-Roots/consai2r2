cmake_minimum_required(VERSION 3.5)
project(consai2_teleop)

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
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(consai2_msgs REQUIRED)

include_directories(include)

add_library(joystick_component SHARED src/joystick_component.cpp)
target_compile_definitions(joystick_component PRIVATE "RASPIMOUSE_EXAMPLES_BUILDING_DLL")
ament_target_dependencies(joystick_component
  rclcpp
  rclcpp_components
  std_msgs
  sensor_msgs
  consai2_msgs
  )
rclcpp_components_register_nodes(joystick_component "joystick::JoystickComponent")

add_executable(teleop_node src/teleop_node.cpp)
target_link_libraries(teleop_node joystick_component)
ament_target_dependencies(
  teleop_node
  "rclcpp"
)


install(TARGETS
  joystick_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS
  teleop_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
