#include "robot_patrol/patrol_with_service_class.h"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Create the PatrolWithService node
    auto node = std::make_shared<PatrolWithService>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}