cmake_minimum_required(VERSION 3.5)
project(consai2r2_receiver)

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
find_package(consai2r2_protobuf REQUIRED)
find_package(Protobuf REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(
  ${PROTOBUF_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${consai2r2_protobuf_INCLUDE_DIRS}
)

include_directories(include ${CMAKE_CURRENT_BINARY_DIR})

add_executable(referee_receiver
  src/referee_receiver.cpp
)
add_executable(vision_receiver
  src/vision_receiver.cpp
)
add_executable(sim_status_receiver
  src/sim_status_receiver.cpp
)
ament_target_dependencies(
  referee_receiver
  "rclcpp"
  "std_msgs"
  "consai2r2_msgs"
  "consai2r2_protobuf"
)
ament_target_dependencies(
  vision_receiver
  "rclcpp"
  "std_msgs"
  "consai2r2_msgs"
  "consai2r2_protobuf"
)
ament_target_dependencies(
  sim_status_receiver
  "rclcpp"
  "std_msgs"
  "consai2r2_msgs"
  "consai2r2_protobuf"
)
target_link_libraries(referee_receiver ${PROTOBUF_LIBRARIES} ${consai2r2_protobuf_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(vision_receiver ${PROTOBUF_LIBRARIES} ${consai2r2_protobuf_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries(sim_status_receiver ${PROTOBUF_LIBRARIES} ${consai2r2_protobuf_LIBRARIES} ${Boost_LIBRARIES})

install(
  DIRECTORY include/
  DESTINATION include)

install(TARGETS referee_receiver
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS vision_receiver
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})
install(TARGETS sim_status_receiver
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

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
