#include "robot_patrol/patrol_class.h"

Patrol::Patrol() : Node("robot_patrol_node") {

    // Subscription to LaserScan topic
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laserCallback, this, std::placeholders::_1));

    // Publisher for velocity commands
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer for robot control
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Patrol::robotControl, this));

}

void Patrol::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Slice required vector from LaserScan ranges
    auto front_start = msg->ranges.begin() + 180;
    auto front_end = msg->ranges.begin() + 540 + 1;

    std::vector<float> lidar_scan_of_the_front_only(front_end - front_start + 1);
    std::copy(front_start, front_end, lidar_scan_of_the_front_only.begin());

    // Extract distances
    left_distance_ = lidar_scan_of_the_front_only[360];   // Left side
    front_distance_ = lidar_scan_of_the_front_only[180]; // Infront
    right_distance_ = lidar_scan_of_the_front_only[0];    // Right side

    // Narrow FOV
    auto FOV_first_i = lidar_scan_of_the_front_only.begin() + 156; // Start index
    auto FOV_last_i = lidar_scan_of_the_front_only.begin() + 204;   // End index
    auto FOV_obstacle = std::min_element(FOV_first_i, FOV_last_i);
    min_distance_ = *FOV_obstacle;

    // Replace infinity values with zero
    std::replace(lidar_scan_of_the_front_only.begin(), lidar_scan_of_the_front_only.end(),
                 std::numeric_limits<double>::infinity(), 0.0);

    // Find maximum value and its position
    auto max_element_iter = std::max_element(lidar_scan_of_the_front_only.begin(), lidar_scan_of_the_front_only.end());
    auto max_index = std::distance(lidar_scan_of_the_front_only.begin(), max_element_iter);

    // Calculate steering angle
    direction_ = (M_PI / 180) * ((max_index - 180) / 2);

    // Log feedback
    RCLCPP_INFO(this->get_logger(), "LaserScan (Left: %f, Center: %f, Right: %f)", left_distance_, front_distance_, right_distance_);
    RCLCPP_INFO(this->get_logger(), "MaxElement (Index: %ld, Value: %f), Steering Angle: %f", max_index, *max_element_iter, direction_);
}

void Patrol::robotControl() {
    auto velocity_msg = geometry_msgs::msg::Twist();

    // Robot control logic
    if (min_distance_ < SAFE_DISTANCE) {
        velocity_msg.linear.x = 0.0; // Stop
        velocity_msg.angular.z = direction_ / 2; // Rotate
    } else {
        velocity_msg.linear.x = LINEAR_SPEED; // Move forward
        velocity_msg.angular.z = 0.0;         // No rotation
    }

    // Log feedback
    RCLCPP_INFO(this->get_logger(), "Velocity (Linear: %f, Angular: %f)", velocity_msg.linear.x, velocity_msg.angular.z);

    // Publish velocity
    cmd_publisher_->publish(velocity_msg);
}
