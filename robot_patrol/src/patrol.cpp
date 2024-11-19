#include "robot_patrol/patrol_class.h"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create and spin the Patrol node
    auto node = std::make_shared<Patrol>();
    RCLCPP_INFO(node->get_logger(), "Node started. Spinning...");
    rclcpp::spin(node);

    // Explicitly trigger the stop method
    RCLCPP_INFO(node->get_logger(), "Stopping the node and cleaning up...");
    node->stop();

    // Shutdown ROS 2
    rclcpp::shutdown();
    return 0;
}
