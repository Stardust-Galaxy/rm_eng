#include "goal_joint_state_publisher/goal_joint_state_publisher.hpp"

GoalJointStatePublisher::GoalJointStatePublisher(const rclcpp::NodeOptions& options) : Node("goal_joint_state_publisher",options) {
    rclcpp::QoS qos(30);
    goalJointStatePublisher = this->create_publisher<PoseStamped>("goal_joint_state",qos);
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.3;
    //每隔5s发布一次目标位置每次偏移一点点，模拟视觉目标
    timer = this->create_wall_timer(std::chrono::seconds(5),[this](){
        msg.header.stamp = this->get_clock()->now();
        msg.pose.position.x += 0.05;
        msg.pose.position.y -= 0.05;
        msg.pose.position.z += 0.05;
        goalJointStatePublisher->publish(msg);
        RCLCPP_INFO(this->get_logger(),"Publishing vision target");
    });
    //visionTargetSubscriber = this->create_subscription<PoseStamped>("vision_target",qos,std::bind(&GoalJointStatePublisher::visionTargetMsgCallBack,this,std::placeholders::_1));
}

void GoalJointStatePublisher::visionTargetMsgCallBack(const PoseStamped::SharedPtr msg) {
    goalJointStatePublisher->publish(*msg);
}
RCLCPP_COMPONENTS_REGISTER_NODE(GoalJointStatePublisher)