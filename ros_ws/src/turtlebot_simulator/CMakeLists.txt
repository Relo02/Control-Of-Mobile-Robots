cmake_minimum_required(VERSION 3.8)
project(turtlebot_simulator)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED) # Crucial for rclcpp's dependencies

# Find Boost
find_package(Boost REQUIRED COMPONENTS filesystem system)

# We combine all source files into a single executable based on the provided main()
add_executable(main
  src/main.cpp
  src/cmd_vel_pub.cpp
  src/robot_simulation.cpp
  src/odesimulator.cpp
)

# Define include directories for the compiler
target_include_directories(main PUBLIC
  # Local headers
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  # Crucial ROS 2 includes
  ${rclcpp_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${rcl_interfaces_INCLUDE_DIRS}
)

# ADDED: This handles ROS 2 linking and dependency setup correctly
ament_target_dependencies(main rclcpp std_msgs geometry_msgs rcl_interfaces)

# Promote Boost includes to PUBLIC and link
target_include_directories(main PUBLIC ${Boost_INCLUDE_DIRS})
target_link_libraries(main ${Boost_LIBRARIES})


# Install the single combined executable
install(TARGETS main
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (if any)
install(DIRECTORY include/ DESTINATION include)

# Install package manifest
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()