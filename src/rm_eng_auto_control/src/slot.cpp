//
// Created by stardust on 2024/12/1.
//
#include <rm_eng_auto_control/slot.hpp>

SlotObstacle::SlotObstacle(const std::string &frame_id) : frame_id_(frame_id) {
    initializeComponents();

}

std::vector<moveit_msgs::msg::CollisionObject>
SlotObstacle::generateCollisionObjects(const geometry_msgs::msg::Pose &center_pose) {
    std::vector<moveit_msgs::msg::CollisionObject> objects;
    // 更新每个组件的位姿并添加到结果中
    objects.push_back(createComponent(bottom_, center_pose, -0.0, 0.0, -0.132));
    objects.push_back(createComponent(left_wall_, center_pose, 0.0, -0.132, 0.0));
    objects.push_back(createComponent(right_wall_, center_pose, 0.0, 0.132, 0.0));
    objects.push_back(createComponent(ceiling_, center_pose, 0.0, 0.0, 0.132));
    objects.push_back(createComponent(back_wall_, center_pose, -0.106, 0.0, 0.0));
    return objects;
}

geometry_msgs::msg::Pose
SlotObstacle::calculateTransformedPose(const geometry_msgs::msg::Pose &center_pose, double offset_x, double offset_y,
                                       double offset_z) {
    // 提取中心位姿的旋转信息
    tf2::Quaternion center_orientation;
    tf2::fromMsg(center_pose.orientation, center_orientation);
    tf2::Matrix3x3 rotation_matrix(center_orientation);

    // 偏移量在槽坐标系下的表示
    tf2::Vector3 local_offset(offset_x, offset_y, offset_z);

    // 计算全局偏移量
    tf2::Vector3 global_offset = rotation_matrix * local_offset;

    // 计算全局位置
    geometry_msgs::msg::Pose transformed_pose;
    transformed_pose.position.x = center_pose.position.x + global_offset.x();
    transformed_pose.position.y = center_pose.position.y + global_offset.y();
    transformed_pose.position.z = center_pose.position.z + global_offset.z();

    // 保持旋转不变
    transformed_pose.orientation = center_pose.orientation;

    return transformed_pose;
}

moveit_msgs::msg::CollisionObject
SlotObstacle::createComponent(const moveit_msgs::msg::CollisionObject &template_object,
                              const geometry_msgs::msg::Pose &center_pose, double offset_x, double offset_y,
                              double offset_z) {
    moveit_msgs::msg::CollisionObject component = template_object;

    // 计算相对位姿
    geometry_msgs::msg::Pose relative_pose = calculateTransformedPose(center_pose, offset_x, offset_y, offset_z);
    component.primitive_poses.clear();
    component.primitive_poses.push_back(relative_pose);
    component.operation = moveit_msgs::msg::CollisionObject::ADD;

    return component;
}

void SlotObstacle::initializeComponent(moveit_msgs::msg::CollisionObject &component, const std::string &id,
                                       const std::vector<double> &dimensions) {
    std::cout << "Initializing components" << std::endl;
    component.id = id;
    component.header.frame_id = frame_id_;
    component.primitives.resize(1);
    component.primitives[0].type = shape_msgs::msg::SolidPrimitive::BOX;
    component.primitives[0].dimensions.resize(3);
    component.primitives[0].dimensions[0] = dimensions[0];
    component.primitives[0].dimensions[1] = dimensions[1];
    component.primitives[0].dimensions[2] = dimensions[2];
}

void SlotObstacle::initializeComponents() {
    // bottom
    initializeComponent(bottom_, "slot_bottom", {0.288, 0.288, 0.024});

    // ceiling
    initializeComponent(ceiling_, "slot_ceiling", {0.288, 0.288, 0.024});

    // left wall
    initializeComponent(left_wall_, "slot_left_wall", {0.288, 0.024, 0.240});

    // right wall
    initializeComponent(right_wall_, "slot_right_wall", {0.288, 0.024, 0.240});

    // back wall
    initializeComponent(back_wall_, "slot_back_wall", {0.076, 0.288, 0.240});
}
