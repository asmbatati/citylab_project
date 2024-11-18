#ifndef PATROL_HPP
#define PATROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#define SAFE_DISTANCE 0.5  // Safety threshold for minimum distance
#define LINEAR_SPEED 0.2   // Linear velocity of the robot

class Patrol : public rclcpp::Node {
private:
    // Member variables
    int steering_angle_;
    float left_distance_;
    float center_distance_;
    float right_distance_;
    float min_distance_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
    rclcpp::CallbackGroup::SharedPtr control_callback_group_;

    // ROS objects
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Member methods
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void robotControl();

public:
    // Constructor
    Patrol();
};

#endif // PATROL_HPP
