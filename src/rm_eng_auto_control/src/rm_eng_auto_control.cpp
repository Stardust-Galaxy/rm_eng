#include "rm_eng_auto_control/rm_eng_auto_control.hpp"

RMEngAutoControl::RMEngAutoControl(const rclcpp::NodeOptions& options) : Node("rm_arm_auto_control",options)  {
    rclcpp::QoS qos(30);
    rclcpp::sleep_for(std::chrono::seconds(1));
    auto const node = std::make_shared<rclcpp::Node>(
        "rm_arm_auto_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,"robotic_arm");
    goal_joint_state_subscriber = this->create_subscription<PoseStamped>("goal_joint_state", qos, std::bind(&RMEngAutoControl::goal_joint_state_callback, this, std::placeholders::_1));

    rclcpp::Publisher<moveit_msgs::msg::PlanningScene>::SharedPtr planning_scene_diff_publisher =
        this->create_publisher<moveit_msgs::msg::PlanningScene>("planning_scene", 1);
    while(planning_scene_diff_publisher->get_subscription_count() < 1) {
        RCLCPP_INFO(this->get_logger(), "Waiting for subscriber");
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }
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
    pose.position.y = 0.144;
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
    /*
     * Exchange Station Definition
     */
    moveit_msgs::msg::CollisionObject exchange_station;
    exchange_station.id = "exchange_station";
    exchange_station.header.frame_id = "base_link";
    exchange_station.operation = exchange_station.ADD;
    // define the pose of the Exchange Station
    geometry_msgs::msg::Pose exchange_station_pose;
    exchange_station_pose.orientation.w = 1.0;
    exchange_station_pose.position.x = 0.25;
    exchange_station_pose.position.y = -0.6;
    exchange_station_pose.position.z = 0.45;
    // define the dimensions of the Exchange Station
    shape_msgs::msg::SolidPrimitive exchange_station_primitive;
    exchange_station_primitive.type = exchange_station_primitive.BOX;
    exchange_station_primitive.dimensions.resize(3);
    exchange_station_primitive.dimensions[0] = 0.288;
    exchange_station_primitive.dimensions[1] = 0.288;
    exchange_station_primitive.dimensions[2] = 0.288;
    // give the attributes to the Exchange Station
    exchange_station.primitives.push_back(exchange_station_primitive);
    exchange_station.primitive_poses.push_back(exchange_station_pose);
    /*
     * Small box Definition
     */
    moveit_msgs::msg::CollisionObject small_box;
    small_box.id = "small_box";
    small_box.header.frame_id = "base_link";
    small_box.operation = small_box.REMOVE;
    // define the pose of the Small Box
    geometry_msgs::msg::Pose small_box_pose;
    small_box_pose.orientation.w = 1.0;
    small_box_pose.position.x = 0.25 - 0.038;
    small_box_pose.position.y = -0.6;
    small_box_pose.position.z = 0.45;
    // define the dimensions of the Small Box
    shape_msgs::msg::SolidPrimitive small_box_primitive;
    small_box_primitive.type = small_box_primitive.BOX;
    small_box_primitive.dimensions.resize(3);
    small_box_primitive.dimensions[0] = 0.212;
    small_box_primitive.dimensions[1] = 0.240;
    small_box_primitive.dimensions[2] = 0.240;
    // give the attributes to the Small Box
    small_box.primitives.push_back(small_box_primitive);
    small_box.primitive_poses.push_back(small_box_pose);
    // publishing the Exchange Station and the Small Box to the world reference frame
    RCLCPP_INFO(this->get_logger(), "Publishing Exchange Station and Small Box into the world");
    planning_scene.world.collision_objects.push_back(exchange_station);
    planning_scene.world.collision_objects.push_back(small_box);
    planning_scene.is_diff = true;
    planning_scene_diff_publisher->publish(planning_scene);
    RCLCPP_INFO(this->get_logger(),"Start to subscribe");
}

RMEngAutoControl::~RMEngAutoControl() {}

void RMEngAutoControl::initialize() {
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "robotic_arm");
}

void RMEngAutoControl::goal_joint_state_callback(const PoseStamped::SharedPtr msg) {
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

RCLCPP_COMPONENTS_REGISTER_NODE(
    RMEngAutoControl
)