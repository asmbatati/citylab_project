#include "robot_patrol/direction_service_class.h"

DirectionService::DirectionService() : Node("direction_service_node") {
  // ROS objects
  this->service_server =
      create_service<robot_patrol::srv::GetDirection>(
          "/direction_service",
          std::bind(&DirectionService::service_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

  // Node acknowledgement
  RCLCPP_INFO(this->get_logger(),
              "The service /direction_service is available for request.");
}

void DirectionService::service_callback(
    const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
    const std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {

  // Retrieve laser scan data
  const auto& ranges = request->laser_data.ranges;
  const float angle_min = request->laser_data.angle_min;
  const float angle_increment = request->laser_data.angle_increment;

  // Calculate the indices for the three sections (using size_t for consistency)
  size_t right_start_index = static_cast<size_t>((-M_PI / 2 - angle_min) / angle_increment); // -90°
  size_t right_end_index = static_cast<size_t>((-M_PI / 6 - angle_min) / angle_increment);   // -30°

  size_t front_start_index = static_cast<size_t>((-M_PI / 6 - angle_min) / angle_increment); // -30°
  size_t front_end_index = static_cast<size_t>((M_PI / 6 - angle_min) / angle_increment);    // +30°

  size_t left_start_index = static_cast<size_t>((M_PI / 6 - angle_min) / angle_increment);   // +30°
  size_t left_end_index = static_cast<size_t>((M_PI / 2 - angle_min) / angle_increment);     // +90°

  // Validate indices
  if (right_start_index >= ranges.size() || left_end_index >= ranges.size()) {
    RCLCPP_WARN(this->get_logger(), "Invalid laser scan data. Indices out of range.");
    response->direction = "unknown";
    return;
  }

  // Compute total distances for each section
  auto total_dist_sec_right = std::accumulate(ranges.begin() + right_start_index,
                                              ranges.begin() + right_end_index + 1, 0.0);
  auto total_dist_sec_front = std::accumulate(ranges.begin() + front_start_index,
                                              ranges.begin() + front_end_index + 1, 0.0);
  auto total_dist_sec_left = std::accumulate(ranges.begin() + left_start_index,
                                             ranges.begin() + left_end_index + 1, 0.0);

  // Determine the safest direction
  if (total_dist_sec_right > total_dist_sec_front && total_dist_sec_right > total_dist_sec_left) {
    response->direction = "right";
  } else if (total_dist_sec_front > total_dist_sec_left) {
    response->direction = "forward";
  } else {
    response->direction = "left";
  }

  // Log the results
  RCLCPP_INFO(this->get_logger(),
              "Distances (Right: %f, Front: %f, Left: %f) -> Direction: %s",
              total_dist_sec_right, total_dist_sec_front, total_dist_sec_left,
              response->direction.c_str());
}
