#include "rm_eng_auto_control/rm_eng_auto_control.hpp"

RMEngAutoControl::RMEngAutoControl(const rclcpp::NodeOptions& options) : Node("rm_arm_auto_control",options)  {
    rclcpp::QoS qos(30);
    rclcpp::sleep_for(std::chrono::seconds(1));
    auto const node = std::make_shared<rclcpp::Node>(
        "rm_arm_auto_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,"robotic_arm");
    goal_joint_state_subscriber = this->create_subscription<PoseStamped>("goal_state", qos, std::bind(
            &RMEngAutoControl::goalStateCallback, this, std::placeholders::_1));
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
}

RMEngAutoControl::~RMEngAutoControl() = default;

void RMEngAutoControl::publish_slot(const msg_interfaces::msg::SlotState& slot_state) {
    moveit_msgs::msg::PlanningScene planning_scene;
    SlotObstacle slot("base_link");
    geometry_msgs::msg::Pose slot_pose;
    slot_pose.position = slot_state.pose.pose.position;

    slot_pose.orientation = slot_state.pose.pose.orientation;
    RCLCPP_INFO(this->get_logger(), "Generating Slot Collision Objects");
    std::vector<moveit_msgs::msg::CollisionObject> slot_objects = slot.generateCollisionObjects(slot_pose);
    RCLCPP_INFO(this->get_logger(), "Publishing Slot into the world");
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
    pose.position.y = -0.144;
    pose.position.z = 0.0;
    // define the dimensions of the Gold Mine
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 0.2;
    primitive.dimensions[1] = 0.2;
    primitive.dimensions[2] = 0.2;
    // give the attributes to the Gold Mine
    attached_object.object.primitives.push_back(primitive);
    attached_object.object.primitive_poses.push_back(pose);
    attached_object.object.operation = attached_object.object.ADD;
    attached_object.touch_links = std::vector<std::string>{"roll_link_2"};
    // publishing Gold Mine to the world reference frame
    RCLCPP_INFO(this->get_logger(), "Publishing attached object into the world at the location of the sucker");
    moveit_msgs::msg::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(attached_object.object);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher->publish(planning_scene);
    // removing the Gold Mine from the world reference frame
    moveit_msgs::msg::CollisionObject remove_object;
    remove_object.id = "box";
    remove_object.header.frame_id = "roll_link_2";
    remove_object.operation = remove_object.REMOVE;
    RCLCPP_INFO(this->get_logger(), "Removing attached object from the world");
    // Attach the Gold Mine to the sucker
    planning_scene.world.collision_objects.clear();
    planning_scene.world.collision_objects.push_back(remove_object);
    planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
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




RCLCPP_COMPONENTS_REGISTER_NODE(
    RMEngAutoControl
)