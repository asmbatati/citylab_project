#ifndef DIRECTION_SERVICE_CLASS_H
#define DIRECTION_SERVICE_CLASS_H

#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <vector>
#include <numeric>
#include <algorithm>
#include <limits>
#include <memory>

class DirectionService : public rclcpp::Node {
public:
  // Constructor
  DirectionService();

private:
  // ROS objects
  rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_server;

  // Member method
  void service_callback(
      const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
      const std::shared_ptr<robot_patrol::srv::GetDirection::Response> response);

};

#endif // DIRECTION_SERVICE_CLASS_H