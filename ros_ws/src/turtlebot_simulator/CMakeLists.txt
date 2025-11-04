cmake_minimum_required(VERSION 3.8)
project(turtlebot_simulator)

# Default C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

# ---- Robot Simulation Node ----
add_executable(robot_sim_node
  src/robot_simulation.cpp
  src/odesimulator.cpp
)
ament_target_dependencies(robot_sim_node
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# ---- Command Velocity Publisher Node ----
add_executable(cmd_vel_pub_node
  src/cmd_vel_pub.cpp
)
ament_target_dependencies(cmd_vel_pub_node
  rclcpp
  geometry_msgs
)

# ---- Install targets ----
install(TARGETS
  robot_sim_node
  cmd_vel_pub_node
  DESTINATION lib/${PROJECT_NAME}
)

# include launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()