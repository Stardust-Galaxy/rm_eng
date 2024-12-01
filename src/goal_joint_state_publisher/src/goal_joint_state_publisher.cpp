#include <tf2/LinearMath/Matrix3x3.h>
#include "goal_joint_state_publisher/goal_joint_state_publisher.hpp"
#include "tf2/LinearMath/Quaternion.h"
GoalJointStatePublisher::GoalJointStatePublisher(const rclcpp::NodeOptions& options) : Node("goal_joint_state_publisher",options) {
    rclcpp::QoS qos(30);
    goalJointStatePublisher = this->create_publisher<PoseStamped>("goal_joint_state",qos);
    msg.header.frame_id = "base_link";
    double pitch = 0 * M_PI / 180;
    double yaw = 45 * M_PI / 180;
    double roll = 0 * M_PI / 180;
    msg.pose.position.x = 0.4- (0.32 + 0.044) * cos(yaw);
    msg.pose.position.y = 0.4 - (0.32 + 0.044) * sin(yaw);
    msg.pose.position.z = 0.6;
    tf2::Quaternion q;
    q.setRPY(roll,pitch,yaw);
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();
    msg.header.stamp = this->now();
    tf2::Quaternion q2(-0.017662, -0.15748, 0.45738, 0.87504);
    tf2::Matrix3x3 m(q2);
    double roll2, pitch2, yaw2;
    m.getRPY(roll2,pitch2,yaw2);
    RCLCPP_INFO(this->get_logger(),"roll: %f, pitch: %f, yaw: %f",roll2 / M_PI * 180, pitch2 / M_PI * 180, yaw2 / M_PI * 180);
    goalJointStatePublisher->publish(msg);
    //visionTargetSubscriber = this->create_subscription<PoseStamped>("vision_target",qos,std::bind(&GoalJointStatePublisher::visionTargetMsgCallBack,this,std::placeholders::_1));
}

void GoalJointStatePublisher::visionTargetMsgCallBack(const PoseStamped::SharedPtr msg) {
    goalJointStatePublisher->publish(*msg);
}
RCLCPP_COMPONENTS_REGISTER_NODE(GoalJointStatePublisher)