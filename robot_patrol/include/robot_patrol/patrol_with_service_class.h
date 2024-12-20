#ifndef PATROL_WITH_SERVICE_HPP
#define PATROL_WITH_SERVICE_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "robot_patrol/srv/get_direction.hpp"

#define SAFE_DISTANCE 0.35  // Safety threshold for minimum distance
#define LINEAR_SPEED 0.1    // Linear velocity of the robot

using namespace std::literals::chrono_literals; // Enables chrono literals like 1s

constexpr char kServiceName[]{"direction_service"};
constexpr char kLidarTopicName[]{"/scan"};
constexpr char kVelTopicName[]{"/cmd_vel"};
constexpr static std::chrono::nanoseconds kWaitTime{1s};

class PatrolWithService : public rclcpp::Node {
public:
    // Constructor
    PatrolWithService();

private:
    // ROS objects
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr direction_client_;
    
    // Member variables - Data storage
    sensor_msgs::msg::LaserScan latest_scan_;
    bool service_in_progress_;
    bool analyze_scan_{false};

    // Member methods
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void timerCallback();
    void serviceCallback(rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future);
    void handleServiceResponse(const std::string& direction);
};

#endif // PATROL_WITH_SERVICE_HPP