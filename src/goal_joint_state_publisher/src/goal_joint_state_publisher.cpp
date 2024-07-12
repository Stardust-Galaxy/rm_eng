#include "goal_joint_state_publisher/goal_joint_state_publisher.hpp"

GoalJointStatePublisher::GoalJointStatePublisher(const rclcpp::NodeOptions& options) : Node("goal_joint_state_publisher",options) {
    rclcpp::QoS qos(30);
    goalJointStatePublisher = this->create_publisher<PoseStamped>("goal_joint_state",qos);
    t265PoseSubscriber = this->create_subscription<PoseStamped>("T265_pose",qos,std::bind(&GoalJointStatePublisher::t265PoseMsgCallBack,this,std::placeholders::_1));
}

GoalJointStatePublisher::~GoalJointStatePublisher() {}

void GoalJointStatePublisher::t265PoseMsgCallBack(const PoseStamped::SharedPtr msg) {
    PoseStamped goalMsg;
    goalMsg.header.stamp = this->now();
    goalMsg.pose.orientation = msg->pose.orientation;
    goalMsg.pose.position.x =  - msg->pose.position.x;
    goalMsg.pose.position.y =   msg->pose.position.z;
    goalMsg.pose.position.z = msg->pose.position.y;
    goalJointStatePublisher->publish(goalMsg);
}

RCLCPP_COMPONENTS_REGISTER_NODE(GoalJointStatePublisher)