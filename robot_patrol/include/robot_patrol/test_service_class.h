#ifndef TEST_SERVICE_CLASS_H
#define TEST_SERVICE_CLASS_H

#include "rclcpp/rclcpp.hpp"
#include "robot_patrol/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <chrono>
#include <memory>

using namespace std::literals::chrono_literals; // Enables chrono literals like 1s

constexpr char kNodeName[]{"test_service"};
constexpr char kServiceName[]{"direction_service"};
constexpr char kTopicName[]{"/scan"};
constexpr static std::chrono::nanoseconds kWaitTime{1s};

class TestService : public rclcpp::Node {
public:
  TestService();

  // Check if the service call is done
  bool is_service_done() const;

private:
  // Timer callback
  void timerCallback();

  // Subscription callback
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);

  // Response callback
  void serviceCallback(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future);

  // ROS entities
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Data storage
  sensor_msgs::msg::LaserScan msg_;
  bool sensor_data_available_{false};
  bool service_done_{false};
};

#endif // TEST_SERVICE_CLASS_H
