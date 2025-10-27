cmake_minimum_required(VERSION 3.5)
project(homework_1)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(counter_node src/counter_node.cpp)
ament_target_dependencies(counter_node rclcpp std_msgs)

install(TARGETS
  counter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()