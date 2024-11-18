#include "robot_patrol/patrol_class.h"

Patrol::Patrol() : Node("robot_patrol_node") {
    // Initialize callback groups
    scan_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    control_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // Subscription to LaserScan topic
    rclcpp::SubscriptionOptions scan_options;
    scan_options.callback_group = scan_callback_group_;
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laserCallback, this, std::placeholders::_1),
        scan_options);

    // Publisher for velocity commands
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer for robot control
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Patrol::robotControl, this),
        control_callback_group_);

    // Node acknowledgment
    RCLCPP_INFO(this->get_logger(), "The robot_patrol_node started successfully");
}

void Patrol::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Slice required vector from LaserScan ranges
    auto front_start = msg->ranges.begin() + 180;
    auto front_end = msg->ranges.begin() + 540 + 1;

    std::vector<float> front_ranges(front_end - front_start + 1);
    std::copy(front_start, front_end, front_ranges.begin());

    // Extract distances
    left_distance_ = front_ranges[360];   // Left side
    center_distance_ = front_ranges[180]; // Center
    right_distance_ = front_ranges[0];    // Right side

    // Find minimum distance in a specific range (78 to 102 indices)
    auto min_start = front_ranges.begin() + 156; // Start index
    auto min_end = front_ranges.begin() + 204;   // End index
    auto min_element_iter = std::min_element(min_start, min_end);
    min_distance_ = *min_element_iter;

    // Replace infinity values with zero
    std::replace(front_ranges.begin(), front_ranges.end(),
                 std::numeric_limits<double>::infinity(), 0.0);

    // Find maximum value and its position
    auto max_element_iter = std::max_element(front_ranges.begin(), front_ranges.end());
    auto max_index = std::distance(front_ranges.begin(), max_element_iter);

    // Calculate steering angle
    steering_angle_ = (max_index - 180) / 2;

    // Log feedback
    // RCLCPP_INFO(this->get_logger(), "LaserScan (Left: %f, Center: %f, Right: %f)", left_distance_, center_distance_, right_distance_);
    // RCLCPP_INFO(this->get_logger(), "MaxElement (Index: %ld, Value: %f), Steering Angle: %d", max_index, *max_element_iter, steering_angle_);
}

void Patrol::robotControl() {
    auto velocity_msg = geometry_msgs::msg::Twist();

    // Robot control logic
    if (min_distance_ < SAFE_DISTANCE) {
        velocity_msg.linear.x = 0.0; // Stop
        velocity_msg.angular.z = ((M_PI / 180) * steering_angle_) / 2; // Rotate
    } else {
        velocity_msg.linear.x = LINEAR_SPEED; // Move forward
        velocity_msg.angular.z = 0.0;         // No rotation
    }

    // Log feedback
    // RCLCPP_INFO(this->get_logger(), "Velocity (Linear: %f, Angular: %f)", velocity_msg.linear.x, velocity_msg.angular.z);

    // Publish velocity
    cmd_publisher_->publish(velocity_msg);
}
