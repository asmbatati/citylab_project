#include "robot_patrol/direction_service_class.h"

int main(int argc, char *argv[]) {
  // Initialize ROS
  rclcpp::init(argc, argv);

  // Spin node
  rclcpp::spin(std::make_shared<DirectionService>());

  // Shutdown
  rclcpp::shutdown();
  return 0;
}