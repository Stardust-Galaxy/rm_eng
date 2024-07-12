#include "rm_eng_auto_control/rm_eng_auto_control.hpp"

RMEngAutoControl::RMEngAutoControl(const rclcpp::NodeOptions& options) : Node("rm_arm_auto_control",options)  {
    rclcpp::QoS qos(30);
    rclcpp::sleep_for(std::chrono::seconds(1));
    auto const node = std::make_shared<rclcpp::Node>(
        "rm_arm_auto_control",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,"rm_eng");
    goal_joint_state_subscriber = this->create_subscription<PoseStamped>("goal_joint_state", qos, std::bind(&RMEngAutoControl::goal_joint_state_callback, this, std::placeholders::_1));
    
    RCLCPP_INFO(this->get_logger(),"Start to subscribe");
}

RMEngAutoControl::~RMEngAutoControl() {}

void RMEngAutoControl::initialize() {
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(shared_from_this(), "rm_eng");
}

void RMEngAutoControl::goal_joint_state_callback(const PoseStamped::SharedPtr msg) {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = msg->pose.position;
    target_pose.orientation = msg->pose.orientation;
    move_group->setPoseTarget(target_pose);
    auto const [success,plan] = [this] {
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto ok = static_cast<bool>(move_group->plan(plan));
        return std::make_pair(ok, plan);
    }();
    if(success) {
        move_group->execute(plan);
        RCLCPP_INFO(this->get_logger(), "Plan Success!Executing...");
    }
    else {
        RCLCPP_ERROR(this->get_logger(), "Plan Failed!");
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(
    RMEngAutoControl
)