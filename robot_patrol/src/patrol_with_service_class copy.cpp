#include "robot_patrol/patrol_with_service_class.h"

PatrolWithService::PatrolWithService() 
    : Node("patrol_with_service_node"), service_called_(false), service_done_(false) {

    // Subscription to LaserScan topic
    lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10, std::bind(&PatrolWithService::laserCallback, this, std::placeholders::_1));

    // Publisher for velocity commands
    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Service client for direction service
    direction_client_ = this->create_client<robot_patrol::srv::GetDirection>("/direction_service");

    // Timer for periodic service calls
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&PatrolWithService::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "PatrolWithService node initialized.");
}

void PatrolWithService::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto velocity_msg = geometry_msgs::msg::Twist();
    float min_distance_in_FOV = std::numeric_limits<float>::max();
    const float FOV_ANGLE = M_PI / 6; // 30° FOV
    int start_index = static_cast<int>((-FOV_ANGLE / 2 - msg->angle_min) / msg->angle_increment);
    int end_index = static_cast<int>((FOV_ANGLE / 2 - msg->angle_min) / msg->angle_increment);

    // Ensure indices are within bounds
    start_index = std::max(0, start_index);
    end_index = std::min(static_cast<int>(msg->ranges.size()) - 1, end_index);

    for (int i = start_index; i <= end_index; ++i) {
        float distance = msg->ranges[i];
        if (distance < min_distance_in_FOV) {
            min_distance_in_FOV = distance;
        }
    }

    // If the front is clear, move forward
    if (min_distance_in_FOV > SAFE_DISTANCE) {
        velocity_msg.linear.x = LINEAR_SPEED;
        velocity_msg.angular.z = 0.0;
        cmd_publisher_->publish(velocity_msg);
    } else {
        // Call the service when the front is blocked
        sendAsyncRequest(msg);
    }
}

void PatrolWithService::sendAsyncRequest(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!direction_client_->wait_for_service(std::chrono::seconds(1))) {
        RCLCPP_WARN(this->get_logger(), "Direction service unavailable.");
        return;
    }

    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = *msg; // Pass the entire LaserScan data

    auto future = direction_client_->async_send_request(
        request, std::bind(&PatrolWithService::responseCallback, this, std::placeholders::_1));
    service_called_ = true;
}

void PatrolWithService::timerCallback() {
    if (service_done_) {
        RCLCPP_INFO(this->get_logger(), "Timer Callback: Service completed.");
    } else if (service_called_) {
        RCLCPP_INFO(this->get_logger(), "Timer Callback: Awaiting service response...");
    } else {
        RCLCPP_INFO(this->get_logger(), "Timer Callback: No service request made.");
    }
}



void PatrolWithService::responseCallback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
    try {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Received direction: %s", response->direction.c_str());
        handleServiceResponse(response->direction);
        service_done_ = true;
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
    }
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
}

void PatrolWithService::stop() {
    auto velocity_msg = geometry_msgs::msg::Twist();
    velocity_msg.linear.x = 0.0;
    velocity_msg.angular.z = 0.0;
    cmd_publisher_->publish(velocity_msg);

    RCLCPP_INFO(this->get_logger(), "Stop command sent.");
}