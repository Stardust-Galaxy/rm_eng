#ifndef RM_ARM_AUTO_CONTROL_HPP_
#define RM_ARM_AUTO_CONTROL_HPP_
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <rm_eng_auto_control/slot.hpp>
#include <msg_interfaces/msg/slot_state.hpp>
class RMEngAutoControl : public rclcpp::Node
{
public:
    using PoseStamped = geometry_msgs::msg::PoseStamped;
    RMEngAutoControl(const rclcpp::NodeOptions &options);
    ~RMEngAutoControl();
    void goalStateCallback(const PoseStamped::SharedPtr msg);
private:
    void publish_main_resource_island();
    void publish_slot(const msg_interfaces::msg::SlotState& slot_state);
    void publish_mine();
    void main_resource_island_action();
    geometry_msgs::msg::Pose last_target_pose;
    bool is_first_goal = true;
    rclcpp::Subscription<msg_interfaces::msg::SlotState>::SharedPtr slot_pose_subscriber;
    rclcpp::Subscription<PoseStamped>::SharedPtr goal_joint_state_subscriber;
    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
    rclcpp::Rate loop_rate = rclcpp::Rate(10);
    shape_msgs::msg::Mesh assimpToShapeMsgMesh(const aiMesh *mesh);
    shape_msgs::msg::Mesh loadSTLAsShapeMsgMesh(const std::string &file_path);
    PoseStamped last_pose;

    void publish_slot();
};

#endif