#include "robot_patrol/test_service_class.h"

TestService::TestService()
    : Node{kNodeName},
      subscription_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          kTopicName, 10,
          std::bind(&TestService::laserCallback, this,
                    std::placeholders::_1))},
      client_{
          this->create_client<robot_patrol::srv::GetDirection>(kServiceName)},
      timer_{this->create_wall_timer(
          kWaitTime, std::bind(&TestService::timerCallback, this))} {RCLCPP_INFO(this->get_logger(),"Service Client Ready.");}

bool TestService::is_service_done() const { return this->service_done_; }

void TestService::timerCallback() {
  if (!sensor_data_available_) {
    RCLCPP_INFO(this->get_logger(), "Sensor data not available yet.");
    return;
  }

  if (!client_->wait_for_service(kWaitTime)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for service. Terminating...");
      return;
    }
    RCLCPP_WARN(this->get_logger(), "Service unavailable.");
    return;
  }

  auto request{std::make_shared<robot_patrol::srv::GetDirection::Request>()};
  request->laser_data = msg_;

  client_->async_send_request(
      request,
      std::bind(&TestService::serviceCallback, this, std::placeholders::_1));
}

void TestService::serviceCallback(
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
  RCLCPP_INFO(this->get_logger(),"Service Request.");
  auto status{future.wait_for(kWaitTime)};
  if (status == std::future_status::ready) {
    RCLCPP_INFO(this->get_logger(), "Service Response: %s",
                future.get()->direction.c_str());
    service_done_ = true;
  } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
  }
}

void TestService::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  msg_ = *msg;
  sensor_data_available_ = true;
}
