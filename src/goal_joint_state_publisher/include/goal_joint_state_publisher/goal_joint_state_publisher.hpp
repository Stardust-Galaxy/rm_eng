#ifndef GOAL_JOINT_STATE_PUBLISHER_HPP_
#define GOAL_JOINT_STATE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>

class GoalJointStatePublisher : public rclcpp::Node {
public:
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    GoalJointStatePublisher(const rclcpp::NodeOptions& options);
    ~GoalJointStatePublisher();
private:
    void t265PoseMsgCallBack(const PoseStamped::SharedPtr msg);
    rclcpp::Subscription<PoseStamped>::SharedPtr t265PoseSubscriber;
    rclcpp::Publisher<PoseStamped>::SharedPtr goalJointStatePublisher;
};

#endif