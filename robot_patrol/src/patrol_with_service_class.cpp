#include "robot_patrol/patrol_with_service_class.h"

PatrolWithService::PatrolWithService() 
    : Node("patrol_with_service_node"), service_in_progress_(false), analyze_scan_(false) {

    // Subscription to LaserScan topic
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        kLidarTopicName, 10,
        std::bind(&PatrolWithService::laserCallback, this, std::placeholders::_1));

    // Publisher for velocity commands
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(kVelTopicName, 10);

    // Service client for direction service
    direction_client_ = this->create_client<robot_patrol::srv::GetDirection>(kServiceName);

    // Timer for periodic checks
    this->create_wall_timer(
        500ms, std::bind(&PatrolWithService::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "PatrolWithService node initialized.");
}

void PatrolWithService::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Store the latest scan for the timer to use
    latest_scan_ = *msg;

    float min_distance_in_FOV = std::numeric_limits<float>::max();
    const float FOV_ANGLE = M_PI / 6; // 30° FOV
    int start_index = static_cast<int>((-FOV_ANGLE / 2 - msg->angle_min) / msg->angle_increment);
    int end_index = static_cast<int>((FOV_ANGLE / 2 - msg->angle_min) / msg->angle_increment);

    // Ensure indices are within bounds
    start_index = std::max(0, start_index);
    end_index = std::min(static_cast<int>(msg->ranges.size()) - 1, end_index);

    for (int i = start_index; i <= end_index; ++i) {
        if (std::isfinite(msg->ranges[i])) {
            min_distance_in_FOV = std::min(min_distance_in_FOV, msg->ranges[i]);
        }
    }

    // If the front is clear, move forward
    if (min_distance_in_FOV > SAFE_DISTANCE) {
        auto velocity_msg = geometry_msgs::msg::Twist();
        velocity_msg.linear.x = LINEAR_SPEED;
        velocity_msg.angular.z = 0.0;
        cmd_publisher_->publish(velocity_msg);

        analyze_scan_ = false; // Reset analysis flag
    } else {
        // Threshold is broken
        RCLCPP_INFO(this->get_logger(), "Now the minimum threshold is broken.");
        analyze_scan_ = true; // Trigger service call
    }

    if (analyze_scan_) {
        auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *msg; // Pass the entire LaserScan data

        auto future = direction_client_->async_send_request(
            request, std::bind(&PatrolWithService::serviceCallback, this, std::placeholders::_1));    
    }

}

void PatrolWithService::timerCallback() {
    // Avoid overlapping service calls
    if (service_in_progress_ || !analyze_scan_) {
        return;
    }

    // Ensure the service is available
    if (!direction_client_->wait_for_service(kWaitTime)) {
        RCLCPP_WARN(this->get_logger(), "Direction service unavailable.");
        return;
    }

    // Debug: Client is ready
    RCLCPP_INFO(this->get_logger(), "Client is ready.");

    // Create and send the service request
    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = latest_scan_;

    auto future = direction_client_->async_send_request(
        request,
        std::bind(&PatrolWithService::serviceCallback, this, std::placeholders::_1));

    service_in_progress_ = true;
}

void PatrolWithService::serviceCallback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
    try {
        auto response = future.get();

        // Debug: Log the service response
        RCLCPP_INFO(this->get_logger(), "Client response: %s", response->direction.c_str());

        // Handle the response
        handleServiceResponse(response->direction);
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());

        // Stop the robot if the service call fails
        auto velocity_msg = geometry_msgs::msg::Twist();
        velocity_msg.linear.x = 0.0;
        velocity_msg.angular.z = 0.0;
        cmd_publisher_->publish(velocity_msg);
    }

    service_in_progress_ = false; // Reset the flag
    analyze_scan_ = false;        // Reset analysis flag
}

void PatrolWithService::handleServiceResponse(const std::string& direction) {
    auto velocity_msg = geometry_msgs::msg::Twist();

    if (direction == "forward") {
        velocity_msg.linear.x = LINEAR_SPEED;
        velocity_msg.angular.z = 0.0;
    } else if (direction == "left") {
        velocity_msg.linear.x = LINEAR_SPEED;
        velocity_msg.angular.z = 0.5;
    } else if (direction == "right") {
        velocity_msg.linear.x = LINEAR_SPEED;
        velocity_msg.angular.z = -0.5;
    } else {
        velocity_msg.linear.x = 0.0;
        velocity_msg.angular.z = 0.0;
    }

    cmd_publisher_->publish(velocity_msg);

    // Debug: Analyzing the scan and moving based on the response
    RCLCPP_INFO(this->get_logger(), "Analyzing the scan: Moving %s.", direction.c_str());
}