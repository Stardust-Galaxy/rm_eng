#ifndef RM_ARM_AUTO_CONTROL_HPP_
#define RM_ARM_AUTO_CONTROL_HPP_
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "tf2/LinearMath/Quaternion.h"
class RMEngAutoControl : public rclcpp::Node
{
public:
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    RMEngAutoControl(const rclcpp::NodeOptions &options);
    ~RMEngAutoControl();
    void goal_joint_state_callback(const PoseStamped::SharedPtr msg);
private:
    void initialize();
    rclcpp::Subscription<PoseStamped>::SharedPtr goal_joint_state_subscriber;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
};

#endif