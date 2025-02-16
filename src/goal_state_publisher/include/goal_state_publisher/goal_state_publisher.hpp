#ifndef GOAL_STATE_PUBLISHER_HPP_
#define GOAL_STATE_PUBLISHER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <msg_interfaces/msg/slot_state.hpp>

class GoalStatePublisher : public rclcpp::Node {
public:
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    GoalStatePublisher(const rclcpp::NodeOptions& options);
    ~GoalStatePublisher() override = default;
private:
    rclcpp::Publisher<PoseStamped>::SharedPtr goalJointStatePublisher;
    rclcpp::Subscription<msg_interfaces::msg::SlotState>::SharedPtr suckerGoalSubscriber;
    void visionTargetMsgCallBack(const PoseStamped::SharedPtr msg);
    PoseStamped msg;
    PoseStamped last_msg;
    rclcpp::TimerBase::SharedPtr timer;
};

#endif