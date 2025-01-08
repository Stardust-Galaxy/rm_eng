#include "arm_control_test/arm_control_test.hpp"

ArmControlTest::ArmControlTest(const rclcpp::NodeOptions& options) : Node("arm_control_test", options){
    rclcpp::QoS qos(30);
    rclcpp::sleep_for(std::chrono::seconds(1));
    auto const node = std::make_shared<rclcpp::Node>(
            "arm_control_test",
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node,"robotic_arm");
    goal_joint_state_subscriber = this->create_subscription<PoseStamped>("goal_state", qos, std::bind(&ArmControlTest::goal_joint_state_callback, this, std::placeholders::_1));

}

ArmControlTest::~ArmControlTest() = default;

void ArmControlTest::goal_joint_state_callback(const PoseStamped::SharedPtr msg) {
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = msg->pose.position;
    target_pose.orientation = msg->pose.orientation;
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

RCLCPP_COMPONENTS_REGISTER_NODE(ArmControlTest)