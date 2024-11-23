#include "goal_joint_state_publisher/goal_joint_state_publisher.hpp"
#include "tf2/LinearMath/Quaternion.h"
GoalJointStatePublisher::GoalJointStatePublisher(const rclcpp::NodeOptions& options) : Node("goal_joint_state_publisher",options) {
    rclcpp::QoS qos(30);
    goalJointStatePublisher = this->create_publisher<PoseStamped>("goal_joint_state",qos);
    double pitch = 0;
    double yaw = 0;
    double roll = 0;
    msg.pose.position.x = 0.24136;
    msg.pose.position.y = -0.39124;
    msg.pose.position.z = 0.55279;
    tf2::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    msg.pose.orientation.x = 0.85865;
    msg.pose.orientation.y = -0.13781;
    msg.pose.orientation.z = -0.43786;
    msg.pose.orientation.w = -0.22807;
    msg.header.stamp = this->now();
    goalJointStatePublisher->publish(msg);
    //visionTargetSubscriber = this->create_subscription<PoseStamped>("vision_target",qos,std::bind(&GoalJointStatePublisher::visionTargetMsgCallBack,this,std::placeholders::_1));
}

void GoalJointStatePublisher::visionTargetMsgCallBack(const PoseStamped::SharedPtr msg) {
    goalJointStatePublisher->publish(*msg);
}
RCLCPP_COMPONENTS_REGISTER_NODE(GoalJointStatePublisher)