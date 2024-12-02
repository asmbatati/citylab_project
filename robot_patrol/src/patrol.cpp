#include "robot_patrol/patrol_class.h"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the Patrol node
    auto node = std::make_shared<Patrol>();
    RCLCPP_INFO(node->get_logger(), "Node started. Spinning...");

    try {
        // Spin the node and handle callbacks
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        // Handle any runtime exceptions
        RCLCPP_ERROR(node->get_logger(), "Exception caught: %s", e.what());
        RCLCPP_INFO(node->get_logger(), "Stopping the robot due to exception...");
        node->stop();
    } catch (...) {
        RCLCPP_ERROR(node->get_logger(), "Unknown exception caught!");
        RCLCPP_INFO(node->get_logger(), "Stopping the robot due to unknown exception...");
        node->stop();
    }

    rclcpp::shutdown();
    return 0;
}
