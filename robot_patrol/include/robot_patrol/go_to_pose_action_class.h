#ifndef GO_TO_POSE_HPP
#define GO_TO_POSE_HPP

#include <cmath>
#include <functional>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <robot_patrol/action/go_to_pose.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

constexpr char kCmdVelTopicName[]{"/cmd_vel"};
constexpr char kOdomTopicName[]{"/odom"};
constexpr char kActionServerName[]{"go_to_pose"};
constexpr double kTwoPi = 2.0 * M_PI;

using namespace std::placeholders;

class GoToPose : public rclcpp::Node {
public:
    using GoToPoseAction = robot_patrol::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPoseAction>;

    explicit GoToPose(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
    // ROS entities
    rclcpp_action::Server<GoToPoseAction>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

    // Action handle
    rclcpp_action::GoalResponse HandleGoal(const rclcpp_action::GoalUUID &uuid,
                                           std::shared_ptr<const GoToPoseAction::Goal> goal);
    rclcpp_action::CancelResponse HandleCancel(const std::shared_ptr<GoalHandleGoToPose> goalHandle);
    void HandleAccepted(const std::shared_ptr<GoalHandleGoToPose> goalHandle);

    // Position tracking
    geometry_msgs::msg::Pose2D currentPos_;
    geometry_msgs::msg::Pose2D destinationPos_;
    float currentYawNormalized_ = 0;
    float destinationYawNormalized_ = 0;
    float rotation_ = 0;

    // Callbacks
    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Execution
    void Execute(const std::shared_ptr<GoalHandleGoToPose> goalHandle);

    // Calculations
    void CalculateRemainingAngleToTarget();
    double CalculateDestinationDirectionAngle();
    float NormalizeYawRad(float yaw);
    bool AreAlmostEqual(float a, float b, float margin);
    double DistanceBetweenPoses();
};

#endif // GO_TO_POSE_HPP
