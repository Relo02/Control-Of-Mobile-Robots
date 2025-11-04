#include "rclcpp/rclcpp.hpp"
#include "cmd_vel_pub.h"
#include "robot_simulation.h"
#include "odesimulator.h"
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Create an Executor (MultiThreaded recommended for concurrent tasks)
  rclcpp::executors::MultiThreadedExecutor executor;

  // 1. Create and Add cmd_vel_pub Node
  auto cmd_vel_node = std::make_shared<CmdVelPub>("cmd_vel_pub_node");
  // The Prepare() method should be called on the node instance
  //cmd_vel_node->Prepare(); 
  executor.add_node(cmd_vel_node);

  // 2. Create and Add robot_simulation Node
  auto robot_sim_node = std::make_shared<RobotSimulation>("robot_simulation_node");
  robot_sim_node->Prepare();
  executor.add_node(robot_sim_node);

  RCLCPP_INFO(rclcpp::get_logger("main"), "Starting Multi-Threaded Executor...");
  
  // Start the execution loop, which drives all timers/callbacks
  executor.spin();

  // The Shutdown() calls are often handled automatically upon destruction or exit.
  cmd_vel_node->Shutdown();
  robot_sim_node->Shutdown();
    
  rclcpp::shutdown();
  return 0;
}

