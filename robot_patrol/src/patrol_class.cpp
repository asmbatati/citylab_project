#include "robot_patrol/patrol_class.h"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <cmath>

Patrol::Patrol() : Node("patrol"), direction_(0.0)
{
    // Subscription to the laser topic
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&Patrol::laser_callback, this, std::placeholders::_1));

    // Publisher for velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    // Timer for control loop at 10 Hz
    control_loop_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&Patrol::control_loop, this));
}

void Patrol::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    const auto &ranges = msg->ranges;
    compute_safe_direction(ranges, msg->angle_min, msg->angle_increment);
}

void Patrol::compute_safe_direction(const std::vector<float> &ranges, float angle_min, float angle_increment)
{
    float max_distance = 0.0;
    int best_index = -1;

    int start_index = std::ceil((M_PI / 2.0 - angle_min) / angle_increment);
    int end_index = std::floor((M_PI / 2.0 + angle_min) / angle_increment);

    for (int i = start_index; i <= end_index; ++i)
    {
        if (std::isfinite(ranges[i]) && ranges[i] > max_distance)
        {
            max_distance = ranges[i];
            best_index = i;
        }
    }

    if (best_index != -1)
    {
        direction_ = angle_min + best_index * angle_increment;
    }
    else
    {
        direction_ = 0.0; // Default to forward if no valid ranges
    }
}

void Patrol::control_loop()
{
    auto twist_msg = geometry_msgs::msg::Twist();

    if (direction_ != 0.0)
    {
        twist_msg.angular.z = direction_ / 2.0;
    }
    twist_msg.linear.x = 0.1;

    cmd_vel_pub_->publish(twist_msg);
}
