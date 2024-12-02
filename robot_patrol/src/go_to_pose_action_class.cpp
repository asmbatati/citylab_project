#include "robot_patrol/go_to_pose_action_class.h"

GoToPose::GoToPose(const rclcpp::NodeOptions &options)
    : Node("go_to_pose_server_node", options) {

    action_server_ = rclcpp_action::create_server<GoToPoseAction>(
        this, kActionServerName,
        std::bind(&GoToPose::HandleGoal, this, _1, _2),
        std::bind(&GoToPose::HandleCancel, this, _1),
        std::bind(&GoToPose::HandleAccepted, this, _1));

    cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopicName, 10);
    odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
        kOdomTopicName, rclcpp::SensorDataQoS(), std::bind(&GoToPose::OdomCallback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Action Server Ready");
}

void GoToPose::OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    currentPos_.set__x(msg->pose.pose.position.x);
    currentPos_.set__y(msg->pose.pose.position.y);
    currentPos_.set__theta(yaw);
}

rclcpp_action::GoalResponse GoToPose::HandleGoal(const rclcpp_action::GoalUUID &uuid,
                                                 std::shared_ptr<const GoToPoseAction::Goal> goal) {
    
    RCLCPP_INFO(this->get_logger(), "Action Called");
    RCLCPP_INFO(this->get_logger(), "Received goal request: x = %f, y = %f, theta = %f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    (void)uuid;
    destinationPos_ = goal->goal_pos;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;

}

rclcpp_action::CancelResponse GoToPose::HandleCancel(const std::shared_ptr<GoalHandleGoToPose> goalHandle) {
    
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goalHandle;
    return rclcpp_action::CancelResponse::ACCEPT;

}

void GoToPose::HandleAccepted(const std::shared_ptr<GoalHandleGoToPose> goalHandle) {

    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&GoToPose::Execute, this, _1), goalHandle}.detach();
}

void GoToPose::Execute(const std::shared_ptr<GoalHandleGoToPose> goalHandle) {
    auto feedback = std::make_shared<GoToPoseAction::Feedback>();
    auto result = std::make_shared<GoToPoseAction::Result>();
    auto twist = geometry_msgs::msg::Twist();

    destinationYawNormalized_ = NormalizeYawRad(CalculateDestinationDirectionAngle());

    rclcpp::Rate loop_rate(5);
    bool goalReached = false;

    // Rotate to face target
    while (rclcpp::ok() && !goalReached) {
        currentYawNormalized_ = NormalizeYawRad(currentPos_.theta);
        CalculateRemainingAngleToTarget();
        goalReached = AreAlmostEqual(rotation_, 0, 2);
        twist.angular.z = goalReached ? 0 : rotation_;
        cmd_publisher_->publish(twist);
        feedback->current_pos = currentPos_;
        goalHandle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    // Move to target
    goalReached = false;
    while (rclcpp::ok() && !goalReached) {
        currentYawNormalized_ = NormalizeYawRad(currentPos_.theta);
        goalReached = AreAlmostEqual(DistanceBetweenPoses(), 0, 5);
        twist.linear.x = goalReached ? 0 : 0.1;
        cmd_publisher_->publish(twist);
        feedback->current_pos = currentPos_;
        goalHandle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    // Rotate to align orientation
    goalReached = false;
    float destinationRadAngle = destinationPos_.theta * (M_PI / 180.0);
    destinationYawNormalized_ = NormalizeYawRad(destinationRadAngle);
    while (rclcpp::ok() && !goalReached) {
        currentYawNormalized_ = NormalizeYawRad(currentPos_.theta);
        CalculateRemainingAngleToTarget();
        goalReached = AreAlmostEqual(rotation_, 0, 1);
        twist.angular.z = goalReached ? 0 : rotation_;
        cmd_publisher_->publish(twist);
        feedback->current_pos = currentPos_;
        goalHandle->publish_feedback(feedback);
        loop_rate.sleep();
    }

    result->status = true;
    goalHandle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Action Completed");
}

void GoToPose::CalculateRemainingAngleToTarget() {
    double clockwiseDiff = fmod((destinationYawNormalized_ - currentYawNormalized_ + kTwoPi), kTwoPi);
    double counterClockwiseDiff = fmod((currentYawNormalized_ - destinationYawNormalized_ + kTwoPi), kTwoPi);
    rotation_ = (clockwiseDiff < counterClockwiseDiff) ? clockwiseDiff : -counterClockwiseDiff;
}

double GoToPose::CalculateDestinationDirectionAngle() {
    return atan2(destinationPos_.y - currentPos_.y, destinationPos_.x - currentPos_.x);
}

float GoToPose::NormalizeYawRad(float yaw) {
    yaw = fmod(yaw, kTwoPi);
    return (yaw < 0) ? yaw + kTwoPi : yaw;
}

bool GoToPose::AreAlmostEqual(float a, float b, float margin) {
    return std::abs(a - b) <= margin;
}

double GoToPose::DistanceBetweenPoses() {
    double dx = currentPos_.x - destinationPos_.x;
    double dy = currentPos_.y - destinationPos_.y;
    return std::sqrt(dx * dx + dy * dy);
}
