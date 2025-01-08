#ifndef ARM_CONTROL_TEST_HPP_
#define ARM_CONTROL_TEST_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <tf2/LinearMath/Quaternion.h>

using PoseStamped = geometry_msgs::msg::PoseStamped;
class ArmControlTest : public rclcpp::Node
{
public:
    ArmControlTest(const rclcpp::NodeOptions &options);
    ~ArmControlTest();
private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    rclcpp::Subscription<PoseStamped>::SharedPtr goal_joint_state_subscriber;
    void goal_joint_state_callback(const PoseStamped::SharedPtr msg);
    rclcpp::Rate loop_rate = rclcpp::Rate(10);
};

#endif