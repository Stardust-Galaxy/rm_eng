#include "rm_eng_auto_control/rm_eng_auto_control.hpp"

RMEngAutoControl::RMEngAutoControl(const rclcpp::NodeOptions& options) : Node("rm_arm_auto_control",options)  {
    rclcpp::QoS qos(30);
    rclcpp::sleep_for(std::chrono::seconds(1));
    auto const node = std::make_shared<rclcpp::Node>(
        "rm_arm_auto_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,"robotic_arm");
//    goal_joint_state_subscriber = this->create_subscription<PoseStamped>("goal_state", qos, std::bind(
//            &RMEngAutoControl::goalStateCallback, this, std::placeholders::_1));
    slot_pose_subscriber = this->create_subscription<msg_interfaces::msg::SlotState>("slot_state", qos, [this](const msg_interfaces::msg::SlotState::SharedPtr msg) {
        if(msg->slot_stabled) {
            last_pose = msg->pose;
            publish_slot(*msg);
            publish_mine();
        }
    });
    planning_scene_diff_publisher =
        this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher->get_subscription_count() < 1) {
        RCLCPP_INFO(this->get_logger(), "Waiting for subscriber");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }


    RCLCPP_INFO(this->get_logger(),"Start to subscribe");

    publish_slot();

    //publish_mine();

    //publish_main_resource_island();
    //main_resource_island_action();
}

RMEngAutoControl::~RMEngAutoControl() = default;

void RMEngAutoControl::publish_main_resource_island() {
    moveit_msgs::msg::PlanningScene planning_scene;
    SlotObstacle slot("base_link");
    geometry_msgs::msg::Pose pose;
    pose.position.x = 1.0;
    pose.position.y = -0.035;
    pose.position.z = 0.2845;
    pose.orientation.w = 1.0;
    moveit_msgs::msg::CollisionObject main_resource_island;
    std::vector<moveit_msgs::msg::CollisionObject> main_resource_island_objects = slot.generateMainResourceIslandCollisionObjects(pose);
    planning_scene.world.collision_objects.insert(planning_scene.world.collision_objects.end(), main_resource_island_objects.begin(), main_resource_island_objects.end());
    planning_scene.is_diff = true;
    planning_scene_diff_publisher->publish(planning_scene);
}

void RMEngAutoControl::publish_slot(const msg_interfaces::msg::SlotState& slot_state) {
    moveit_msgs::msg::PlanningScene planning_scene;
    SlotObstacle slot("base_link");
    geometry_msgs::msg::Pose slot_pose;
    slot_pose.position = slot_state.pose.pose.position;

    slot_pose.orientation = slot_state.pose.pose.orientation;
    //RCLCPP_INFO(this->get_logger(), "Generating Slot Collision Objects");
    std::vector<moveit_msgs::msg::CollisionObject> slot_objects = slot.generateSlotCollisionObjects(slot_pose);
    //RCLCPP_INFO(this->get_logger(), "Publishing Slot into the world");
    planning_scene.world.collision_objects.insert(planning_scene.world.collision_objects.end(), slot_objects.begin(), slot_objects.end());
    planning_scene.is_diff = true;
    planning_scene_diff_publisher->publish(planning_scene);
}

void RMEngAutoControl::publish_slot() {
    moveit_msgs::msg::PlanningScene planning_scene;
    SlotObstacle slot("base_link");
    geometry_msgs::msg::Pose slot_pose;
    slot_pose.position.x = 0.53;  // Fixed position
    slot_pose.position.y = -0.25;  // Fixed position
    slot_pose.position.z = 0.6;  // Fixed position
    //left orientation 45 degrees
    tf2::Quaternion q;
    q.setRPY(0, 0, -M_PI / 4);
    slot_pose.orientation.x = q.x();
    slot_pose.orientation.y = q.y();
    slot_pose.orientation.z = q.z();
    slot_pose.orientation.w = q.w();


    std::vector<moveit_msgs::msg::CollisionObject> slot_objects = slot.generateSlotCollisionObjects(slot_pose);
    planning_scene.world.collision_objects.insert(planning_scene.world.collision_objects.end(), slot_objects.begin(), slot_objects.end());
    planning_scene.is_diff = true;
    planning_scene_diff_publisher->publish(planning_scene);
}

void RMEngAutoControl::publish_mine() {
    /*
     * Gold Mine Definition
     */
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = "roll_link_2";
    attached_object.object.header.frame_id = "roll_link_2";
    attached_object.object.id = "box";
    // define the pose of the Gold Mine
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = 0.0;
    pose.position.y = -0.16;
    pose.position.z = 0.0;
    // define the dimensions of the Gold Mine
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.18;
    primitive.dimensions[1] = 0.18;
    primitive.dimensions[2] = 0.18;
    // give the attributes to the Gold Mine
    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);
    attached_object.object.operation = attached_object.object.ADD;
    attached_object.touch_links = std::vector<std::string>{"roll_link_2"};
    // publishing Gold Mine to the world reference frame
    //RCLCPP_INFO(this->get_logger(), "Publishing attached object into the world at the location of the sucker");
    moveit_msgs::msg::PlanningScene planning_scene;
//    planning_scene.world.collision_objects.push_back(attached_object.object);
//    planning_scene.is_diff = true;
//    planning_scene_diff_publisher->publish(planning_scene);
//    // removing the Gold Mine from the world reference frame
//    moveit_msgs::msg::CollisionObject remove_object;
//    remove_object.id = "box";
//    remove_object.header.frame_id = "roll_link_2";
//    remove_object.operation = remove_object.REMOVE;
//    RCLCPP_INFO(this->get_logger(), "Removing attached object from the world");
    // Attach the Gold Mine to the sucker
    planning_scene.world.collision_objects.clear();
//    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
    planning_scene.is_diff = true;
    planning_scene.robot_state.is_diff = true;
    planning_scene_diff_publisher->publish(planning_scene);
}

void RMEngAutoControl::goalStateCallback(const PoseStamped::SharedPtr msg) {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = msg->pose.position;
    target_pose.orientation = msg->pose.orientation;
    last_target_pose = target_pose;

    move_group->setPoseTarget(target_pose, "roll_link_2");
    auto const [success,plan] = [this] {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto ok = static_cast<bool>(move_group->plan(plan));
        return std::make_pair(ok, plan);
    }();
    if(success) {
        move_group->asyncExecute(plan);
        RCLCPP_INFO(this->get_logger(), "Plan Success!Executing...");
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Plan Failed!");
    }
    loop_rate.sleep();
}



shape_msgs::msg::Mesh RMEngAutoControl::assimpToShapeMsgMesh(const aiMesh *mesh) {
    shape_msgs::msg::Mesh shape_mesh;

    // 处理顶点
    for (unsigned int i = 0; i < mesh->mNumVertices; ++i) {
        geometry_msgs::msg::Point p;
        p.x = mesh->mVertices[i].x;
        p.y = mesh->mVertices[i].y;
        p.z = mesh->mVertices[i].z;
        shape_mesh.vertices.push_back(p);
    }

    // 处理面
    for (unsigned int i = 0; i < mesh->mNumFaces; ++i) {
        const aiFace& face = mesh->mFaces[i];
        if (face.mNumIndices != 3) {
            // 只支持三角面
            continue;
        }
        shape_msgs::msg::MeshTriangle triangle;
        triangle.vertex_indices[0] = face.mIndices[0];
        triangle.vertex_indices[1] = face.mIndices[1];
        triangle.vertex_indices[2] = face.mIndices[2];
        shape_mesh.triangles.push_back(triangle);
    }

    return shape_mesh;
}

shape_msgs::msg::Mesh RMEngAutoControl::loadSTLAsShapeMsgMesh(const std::string &file_path) {
    Assimp::Importer importer;
    const aiScene* scene = importer.ReadFile(file_path, aiProcess_Triangulate);
    if (!scene || !scene->HasMeshes()) {
        throw std::runtime_error("Failed to load STL file: " + file_path + ", error: " + importer.GetErrorString());
    }

    return assimpToShapeMsgMesh(scene->mMeshes[0]);
}

void RMEngAutoControl::main_resource_island_action() {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    publish_main_resource_island();
    geometry_msgs::msg::Pose phase_1_pose;
    moveit_msgs::msg::Constraints constraints;

//    tf2::Quaternion q_;
//    q_.setX(0.706);
//    q_.setY(-0.706);
//    q_.setZ(0.02);
//    q_.setW(-0.02);
//    //print the euler angles
//    double roll, pitch, yaw;
//    tf2::Matrix3x3(q_).getRPY(roll, pitch, yaw);
//    RCLCPP_INFO(this->get_logger(), "Roll: %f, Pitch: %f, Yaw: %f", roll / M_PI * 180, pitch / M_PI * 180, yaw / M_PI * 180);
    phase_1_pose.position.x = 0.65;
    //keep y the same
    phase_1_pose.position.y = -0.035;
    phase_1_pose.position.z = 0.21;
    tf2::Quaternion q;
    q.setRPY(-M_PI, 0, -M_PI / 2);
    phase_1_pose.orientation.x = q.x();
    phase_1_pose.orientation.y = q.y();
    phase_1_pose.orientation.z = q.z();
    phase_1_pose.orientation.w = q.w();
    move_group->setGoalPositionTolerance(0.01);
    move_group->setGoalOrientationTolerance(0.002);
    move_group->setPoseTarget(phase_1_pose, "roll_link_2");
    auto const [success,plan] = [this] {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto ok = static_cast<bool>(move_group->plan(plan));
        return std::make_pair(ok, plan);
    }();
    if(success) {
        move_group->asyncExecute(plan);
        RCLCPP_INFO(this->get_logger(), "Plan_1 Success!Executing...");
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Plan_1 Failed!");
    }
    std::this_thread::sleep_for(std::chrono::seconds(5));
    publish_mine();
//    geometry_msgs::msg::Pose phase_2_pose;
//    phase_2_pose.position.x = 0.65;
//    phase_2_pose.position.y = -0.035;
//    phase_2_pose.position.z = 0.24;
//    q.setRPY(-M_PI, 0, -M_PI / 2);
//    phase_1_pose.orientation.x = q.x();
//    phase_1_pose.orientation.y = q.y();
//    phase_1_pose.orientation.z = q.z();
//    phase_1_pose.orientation.w = q.w();
//    move_group->setGoalPositionTolerance(0.01);
//    move_group->setGoalOrientationTolerance(0.005);
////    move_group->setJointValueTarget("roll_joint_2", 0.0);
////    move_group->setJointValueTarget("roll_joint_1",0.0);
//    move_group->setPoseTarget(phase_2_pose, "roll_link_2");
//    auto const[success_2, plan_2] = [this] {
//        moveit::planning_interface::MoveGroupInterface::Plan plan;
//        auto ok = static_cast<bool>(move_group->plan(plan));
//        return std::make_pair(ok, plan);
//    }();
//    if(success_2) {
//        move_group->asyncExecute(plan_2);
//        RCLCPP_INFO(this->get_logger(), "Plan_2 Success!Executing...");
//    } else {
//        RCLCPP_ERROR(this->get_logger(), "Plan_2 Failed!");
//    }
//
//    moveit_msgs::msg::Constraints constraint;
//    moveit_msgs::msg::PositionConstraint position_constraint;
//    position_constraint.header.frame_id = "base_link";
//    position_constraint.link_name = "roll_link_2";
//
//    shape_msgs::msg::SolidPrimitive tolerance_volume;
//    tolerance_volume.type = shape_msgs::msg::SolidPrimitive::BOX;
//    tolerance_volume.dimensions = {0.5, 0.5, 0.01};
//    position_constraint.constraint_region.primitives.push_back(tolerance_volume);

}


RCLCPP_COMPONENTS_REGISTER_NODE(
    RMEngAutoControl
)