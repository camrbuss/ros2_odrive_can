#include <cstdio>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ros2_odrive_can/can_service.hpp"


int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;

  auto node1 = std::make_shared<CanService>();
  exec.add_node(node1);

  exec.spin();
  rclcpp::shutdown();
  return 0;
}
