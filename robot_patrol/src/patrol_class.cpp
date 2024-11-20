#include "robot_patrol/patrol_class.h"

Patrol::Patrol() : Node("robot_patrol_node") {

    // Subscription to LaserScan topic
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laserCallback, this, std::placeholders::_1));

    // Publisher for velocity commands
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

}

void Patrol::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    float max_distance = 0.0;        // Maximum distance in front FOV
    int max_index = -1;              // Index of the ray with the maximum distance
    float min_distance_in_FOV = 1.0;
    const float FOV_ANGLE = M_PI / 4; // 45° FOV in radians
    auto velocity_msg = geometry_msgs::msg::Twist();

    // Calculate the indices for the front semi-circle (-90° to +90°)
    int front_start_index = static_cast<int>((-M_PI / 2 - msg->angle_min) / msg->angle_increment); // -90°
    int front_end_index = static_cast<int>((M_PI / 2 - msg->angle_min) / msg->angle_increment);   // +90°

    // Calculate the indices for the 45° FOV (-15° to +15°)
    int FOV_start_index = static_cast<int>((-FOV_ANGLE / 2 - msg->angle_min) / msg->angle_increment); // -22.5°
    int FOV_end_index = static_cast<int>((FOV_ANGLE / 2 - msg->angle_min) / msg->angle_increment);   // +22.5°

    // Find the maximum distance within the 45° FOV
    for (int i = FOV_start_index; i <= FOV_end_index; ++i) {
        float distance = msg->ranges[i];
        if (distance < min_distance_in_FOV) {
            min_distance_in_FOV = distance;
        }
    }

    // Check if the min distance in FOV is greater than the safe threshold
    if (min_distance_in_FOV > SAFE_DISTANCE) {
        // No obstacle within the threshold, keep moving forward
        direction_ = 0.0; // Straight ahead
    }

    else{
        // Obstacle detected, analyze the front semi-circle (-90° to +90°)
        for (int i = front_start_index; i <= front_end_index; ++i) {
            float distance = msg->ranges[i];

            // Skip invalid rays (infinity)
            if (!std::isfinite(distance)) continue;

            // Update maximum distance and its index
            if (distance > max_distance) {
                max_distance = distance;
                max_index = i;
                }
            }
        }

    // Calculate the corresponding angle for the largest ray
    if (max_index != -1) {
        direction_ = msg->angle_min + max_index * msg->angle_increment;
    } else {
        // Default to straight ahead if no valid ray is found
        direction_ = 0.0;
    }

    // Send velocity commands
    velocity_msg.linear.x = LINEAR_SPEED;
    velocity_msg.angular.z = direction_;
    cmd_publisher_->publish(velocity_msg);
}

void Patrol::stop() {
    auto velocity_msg = geometry_msgs::msg::Twist();
    velocity_msg.linear.x = 0.0;
    velocity_msg.angular.z = 0.0;
    cmd_publisher_->publish(velocity_msg);

    RCLCPP_INFO(this->get_logger(), "Stop command sent.");
}