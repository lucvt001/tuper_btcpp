cmake_minimum_required(VERSION 3.8)
project(tuper_btcpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(tuper_interfaces REQUIRED)

include_directories(
  include
)

set(ros_dependencies
  rclcpp
  std_msgs
  behaviortree_ros2
  btcpp_ros2_interfaces
  tuper_interfaces
)

add_executable(generate_tree_node_model src/generate_tree_node_model.cpp)
ament_target_dependencies(generate_tree_node_model ${ros_dependencies})

add_executable(bt_planner src/bt_planner.cpp)
ament_target_dependencies(bt_planner ${ros_dependencies})

install(TARGETS
  generate_tree_node_model
  bt_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
  include/
  DESTINATION include/
  FILES_MATCHING 
    PATTERN "*.h"
    PATTERN "*.hpp"
)

install(DIRECTORY 
  behavior_trees
  btlogs
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
