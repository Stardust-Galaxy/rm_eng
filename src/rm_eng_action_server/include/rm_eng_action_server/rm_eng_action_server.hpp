#ifndef RM_ENG_ACTION_SERVER_HPP
#define RM_ENG_ACTION_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "../../serial_port/include/serial_port/RMEngJointState.hpp"
class rm_eng_action_server : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
    using JointStateMsg = sensor_msgs::msg::JointState;

    rm_eng_action_server(const rclcpp::NodeOptions& options);

private:
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void execute(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void publish_joint_states();
    std::vector<int16_t> parseJointStates(joint_states_for_send goal_joint_states);
    // 假装真正的关节状态
    std::vector<double> mJointStates;
    rclcpp::Subscription<JointStateMsg>::SharedPtr joint_states_subscriber;
    rclcpp::Publisher<JointStateMsg>::SharedPtr joint_states_publisher;
    rclcpp::Publisher<JointStateMsg>::SharedPtr goal_joint_states_publisher;
    joint_states_for_send goalJointStates;
    uint8_t goalJointStateHeader = 0xFF;
    uint8_t goalJointStateTail = 0xFE;
};

#endif 

