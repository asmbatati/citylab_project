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

    // Calculate the indices for the front FOV (-90° to +90°)
    int front_start_index = static_cast<int>((-M_PI / 2 - msg->angle_min) / msg->angle_increment); // -90°
    int front_end_index = static_cast<int>((M_PI / 2 - msg->angle_min) / msg->angle_increment);   // +90°

    // Slice the required vector from LaserScan ranges
    std::vector<float> lidar_scan_of_the_front_only(
        msg->ranges.begin() + front_start_index,
        msg->ranges.begin() + front_end_index + 1
    );

    // Replace infinity values with zero
    std::replace(lidar_scan_of_the_front_only.begin(), lidar_scan_of_the_front_only.end(),
                 std::numeric_limits<float>::infinity(), 0.0f);

    // Extract distances for left, front, and right (adjusting indices based on vector size)
    int left_index = static_cast<int>(lidar_scan_of_the_front_only.size() * 3 / 4);   // Left
    int front_index = static_cast<int>(lidar_scan_of_the_front_only.size() / 2);      // Front
    int right_index = static_cast<int>(lidar_scan_of_the_front_only.size() / 4);      // Right

    left_distance_ = lidar_scan_of_the_front_only[left_index];
    front_distance_ = lidar_scan_of_the_front_only[front_index];
    right_distance_ = lidar_scan_of_the_front_only[right_index];

    // Narrow FOV (-15° to +15° relative to front)
    int FOV_start_index = static_cast<int>((-M_PI / 9 - (-M_PI / 2)) / msg->angle_increment); // -20°
    int FOV_end_index = static_cast<int>((M_PI / 9 - (-M_PI / 2)) / msg->angle_increment);   // +20°

    auto FOV_first_i = lidar_scan_of_the_front_only.begin() + FOV_start_index;
    auto FOV_last_i = lidar_scan_of_the_front_only.begin() + FOV_end_index + 1;
    auto FOV_obstacle = std::min_element(FOV_first_i, FOV_last_i);
    min_distance_ = *FOV_obstacle;

    // Find maximum value and its position for steering
    auto max_element_iter = std::max_element(lidar_scan_of_the_front_only.begin(), lidar_scan_of_the_front_only.end());
    auto max_index = std::distance(lidar_scan_of_the_front_only.begin(), max_element_iter);

    // Calculate steering angle relative to front
    direction_ = msg->angle_min + front_start_index * msg->angle_increment + max_index * msg->angle_increment;

    // Debug information
    RCLCPP_INFO(this->get_logger(), "Left: %f, Front: %f, Right: %f, Min: %f, Direction: %f", left_distance_, front_distance_, right_distance_, min_distance_, direction_);

    // Send velocity commands
    auto velocity_msg = geometry_msgs::msg::Twist();
    velocity_msg.linear.x = LINEAR_SPEED;
    if (min_distance_ < SAFE_DISTANCE) {
        velocity_msg.angular.z = direction_ / 2;
    } else {
        velocity_msg.angular.z = 0.0;
    }
    cmd_publisher_->publish(velocity_msg);
}

void Patrol::stop() {
    auto velocity_msg = geometry_msgs::msg::Twist();
    velocity_msg.linear.x = 0.0;
    velocity_msg.angular.z = 0.0;
    cmd_publisher_->publish(velocity_msg);

    RCLCPP_INFO(this->get_logger(), "Stop command sent.");
}