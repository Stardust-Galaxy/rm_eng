//
// Created by stardust on 2024/12/1.
//

#ifndef BUILD_SLOT_HPP
#define BUILD_SLOT_HPP
#include <moveit_msgs/msg/collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

class SlotObstacle
{
public:
    SlotObstacle(const std::string& frame_id);
    std::vector<moveit_msgs::msg::CollisionObject> generateSlotCollisionObjects(const geometry_msgs::msg::Pose& center_pose);
    std::vector<moveit_msgs::msg::CollisionObject> generateMainResourceIslandCollisionObjects(const geometry_msgs::msg::Pose& center_pose);
private:
    std::string frame_id_;

    //
    moveit_msgs::msg::CollisionObject bottom_, ceiling_, left_wall_, right_wall_, back_wall_;
    moveit_msgs::msg::CollisionObject main_resource_island_top, main_resource_island_left, main_resource_island_right, main_resource_island_bottom;
    //initialize the components of the slot
    void initializeComponents();
    // initialize the component with the given id and dimensions
    void initializeComponent(moveit_msgs::msg::CollisionObject& component, const std::string& id, const std::vector<double>& dimensions);

    moveit_msgs::msg::CollisionObject createComponent(const moveit_msgs::msg::CollisionObject& template_object,
                                                      const geometry_msgs::msg::Pose& center_pose,
                                                      double offset_x, double offset_y, double offset_z);

    geometry_msgs::msg::Pose calculateTransformedPose(const geometry_msgs::msg::Pose& center_pose,
                                                      double offset_x, double offset_y, double offset_z);
};

#endif //BUILD_SLOT_HPP
