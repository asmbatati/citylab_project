#ifndef PATROL_CLASS_H
#define PATROL_CLASS_H

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Patrol : public rclcpp::Node
{
public:
    Patrol();

private:
    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    void compute_safe_direction(const std::vector<float> &ranges, float angle_min, float angle_increment);

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    void control_loop();

    float direction_; // Direction to the safest area
};

#endif // PATROL_CLASS_H
