#ifndef RM_ENG_ACTION_SERVER_HPP
#define RM_ENG_ACTION_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "sensor_msgs/msg/joint_state.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "../../serial_port/include/serial_port/RMEngJointState.hpp"
#include "serial_port/SerialPort.hpp"
class rm_eng_action_server : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;


    rm_eng_action_server(const rclcpp::NodeOptions& options);

private:
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void execute(const std::shared_ptr<GoalHandleFJT> goal_handle);
    void publish_joint_states();
    std::vector<uint8_t> parseJointStates(joint_states goal_joint_states);
    std::shared_ptr<SerialPort> serial_port;
    // 假装真正的关节状态
    std::vector<double> mJointStates;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscriber;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_publisher;
    joint_states goalJointStates;
    uint8_t goalJointStateHeader = 0xA8;
};

#endif 

