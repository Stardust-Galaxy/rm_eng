#include <tf2/LinearMath/Matrix3x3.h>
#include "goal_state_publisher/goal_state_publisher.hpp"
#include "tf2/LinearMath/Quaternion.h"
GoalStatePublisher::GoalStatePublisher(const rclcpp::NodeOptions& options) : Node("goal_joint_state_publisher", options) {
    rclcpp::QoS qos(10);
    goalJointStatePublisher = this->create_publisher<PoseStamped>("goal_state",qos);
    suckerGoalSubscriber = this->create_subscription<msg_interfaces::msg::SlotState>("sucker_goal", qos, [this] (msg_interfaces::msg::SlotState::SharedPtr msg) {
        if(msg->slot_stabled) {
            msg->pose.header.frame_id = "base_link";
            goalJointStatePublisher->publish(msg->pose);
        }
    });
    //wait 5 seconds before publishing the next message using ros2 timer
    //std::this_thread::sleep_for(std::chrono::seconds(5));
    //msg.pose.position.x = 0.4- (0.31 + 0.044) * cos(yaw);
    //msg.pose.position.y = 0.4 - (0.31 + 0.044) * sin(yaw);
    //msg.pose.position.z = 0.59;
    //msg.header.stamp = this->now();
    //goalJointStatePublisher->publish(msg);

    //visionTargetSubscriber = this->create_subscription<PoseStamped>("vision_target",qos,std::bind(&GoalStatePublisher::visionTargetMsgCallBack,this,std::placeholders::_1));
}

void GoalStatePublisher::visionTargetMsgCallBack(const PoseStamped::SharedPtr msg) {
    goalJointStatePublisher->publish(*msg);
}
RCLCPP_COMPONENTS_REGISTER_NODE(GoalStatePublisher)