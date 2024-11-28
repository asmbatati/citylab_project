#include "robot_patrol/test_service_class.h"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node{std::make_shared<TestService>()};
  while (!node->is_service_done()) {
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();
  return 0;
}
