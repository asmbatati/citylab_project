#ifndef PATROL_HPP
#define PATROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#define SAFE_DISTANCE 0.35  // Safety threshold for minimum distance
#define LINEAR_SPEED 0.1   // Linear velocity of the robot

class Patrol : public rclcpp::Node {
public:
    // Constructor
    Patrol();
private:
    // Member variables
    float direction_;
    float left_distance_;
    float front_distance_;
    float right_distance_;
    float min_distance_;

    // ROS objects
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Member methods
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void robotControl();
};

#endif
