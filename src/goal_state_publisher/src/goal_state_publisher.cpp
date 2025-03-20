#include <tf2/LinearMath/Matrix3x3.h>
#include "goal_state_publisher/goal_state_publisher.hpp"
#include "tf2/LinearMath/Quaternion.h"
GoalStatePublisher::GoalStatePublisher(const rclcpp::NodeOptions& options) : Node("goal_joint_state_publisher", options) {
    rclcpp::QoS qos(10);
    last_msg.header.stamp = this->now();
    goalJointStatePublisher = this->create_publisher<PoseStamped>("goal_state",qos);
    suckerGoalSubscriber = this->create_subscription<msg_interfaces::msg::SlotState>("sucker_goal", qos, [this] (msg_interfaces::msg::SlotState::SharedPtr msg) {
        if(msg->slot_stabled) {
            msg->pose.header.frame_id = "base_link";
            double duration = msg->pose.header.stamp.sec - last_msg.header.stamp.sec;
            if(duration > 10) {
                RCLCPP_INFO(this->get_logger(), "Goal state: x: %f, y: %f, z: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
                // euler angles
                double roll, pitch, yaw;
                tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
                tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
                RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180);
                goalJointStatePublisher->publish(msg->pose);
                last_msg = msg->pose;
            }
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