#include "rm_eng_action_server/rm_eng_action_server.hpp"
#include "rclcpp_components/register_node_macro.hpp"
rm_eng_action_server::rm_eng_action_server(const rclcpp::NodeOptions& options) : Node("rm_eng_action_server",options)
{
    //    control_msgs::FollowJointTrajectoryAction
    //serial_port->init();
    this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
                this, "/robotic_arm_controller/follow_joint_trajectory",
                std::bind(&rm_eng_action_server::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
                std::bind(&rm_eng_action_server::handle_cancel, this, std::placeholders::_1),
                std::bind(&rm_eng_action_server::handle_accepted, this, std::placeholders::_1));

    rclcpp::QoS qos(10);
    mJointStates = {-0.872, 2.355, -1.57, 0.0, 0.0, 0.0, 0.0};
    //joint_states_subscriber = this->create_subscription<sensor_msgs::msg::JointState>("joint_states_for_send", qos, [this](const sensor_msgs::msg::JointState::SharedPtr msg){
    //    mJointStates = msg->position;
    //});
    joint_states_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",qos);
    goal_joint_states_publisher = this->create_publisher<sensor_msgs::msg::JointState>("goal_joint_states",qos);
    std::thread{std::bind(&rm_eng_action_server::publish_joint_states, this)}
    .detach();
}

rclcpp_action::GoalResponse rm_eng_action_server::handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const FollowJointTrajectory::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    RCLCPP_INFO(this->get_logger(),goal->trajectory.header.frame_id.c_str());

    int pointSize = goal->trajectory.points.size();
    if(pointSize > 0)
    {
        for(int i = 0; i < pointSize; i++)
        {
            auto point = goal->trajectory.points.at(i);
            RCLCPP_INFO(this->get_logger(), "point[%d]: joint[1]:%f,joint[2]:%f,joint[3]:%f,joint[4]:%f,joint[5]:%f,joint[6]:%f, joint[7]:%f", i, point.positions[0],point.positions[1],point.positions[2],point.positions[3],point.positions[4],point.positions[5]);
        }
    }

   (void)uuid;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse rm_eng_action_server::handle_cancel(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void rm_eng_action_server::execute_move(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    const auto goal = goal_handle->get_goal();

    RCLCPP_INFO(this->get_logger(), "Start to move!");

    auto result = std::make_shared<FollowJointTrajectory::Result>();

    auto trjPointsSize = goal->trajectory.points.size();
    int trjIdx = 0;

    while (rclcpp::ok() && trjIdx < trjPointsSize) {
        auto point =  goal->trajectory.points.at(trjIdx);
        // 按照轨迹指定的时间，进行等待
        if(trjIdx > 0)
        {
            auto last_time = goal->trajectory.points.at(trjIdx - 1).time_from_start;
            auto current_time = goal->trajectory.points.at(trjIdx).time_from_start;

            rclcpp::Time time1(last_time.sec, last_time.nanosec);
            rclcpp::Time time2(current_time.sec, current_time.nanosec);
            auto time_to_sleep = time2 - time1;

            int64_t duration = ((int64_t)time_to_sleep.seconds()) * 1e9 + time_to_sleep.nanoseconds();
            std::chrono::nanoseconds ns(duration / 10);


            rclcpp::sleep_for(ns);
        }

        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->header.frame_id = goal->trajectory.header.frame_id;
        feedback->header.stamp = goal->trajectory.header.stamp;
        feedback->joint_names = goal->trajectory.joint_names;
        //use mJointStates to simulate the actual joint states(needs conversion)
        feedback->actual.positions = mJointStates;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for(int i = 0; i < static_cast<int>(point.positions.size()); i++)
            {
                mJointStates[i] = point.positions.at(i);
            }
        }
        JointStateMsg msg;
        msg.header.frame_id = "base_link";
        msg.header.stamp = this->now();
        msg.name = {"pitch_joint_1", "pitch_joint_2", "pitch_joint_3", "roll_joint_1", "roll_joint_2", "shift_joint", "yaw_joint_1"};
        msg.position = mJointStates;
        goal_joint_states_publisher->publish(msg);

        goal_handle->publish_feedback(feedback);
        /*检测任务是否被取消*/
        if (goal_handle->is_canceling()) {
            result->error_code = -1;
            result->error_string = "has cancel";
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal Canceled");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Publish Feedback"); /*Publish feedback*/

        trjIdx++;

        //        rate.sleep();
    }

//    result->pose = robot.get_current_pose();
    result->error_code = 0;
    result->error_string = "";

    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
}

//void rm_eng_action_server::execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
//{
//    RCLCPP_INFO(this->get_logger(), "Start to move!");
//    auto result = std::make_shared<FollowJointTrajectory::Result>();
//    auto goal = goal_handle->get_goal();
//    auto trjPointsSize = goal->trajectory.points.size();
//    auto finalPoint = goal->trajectory.points.at(trjPointsSize - 1);
//    goalJointStates.header = goalJointStateHeader;
//
//    goalJointStates.yaw_joint_1 = static_cast<int16_t>(finalPoint.positions[0] / M_PI / 2 * 65536);
//    goalJointStates.pitch_joint_1 = static_cast<int16_t>(finalPoint.positions[1] / M_PI / 2 * 65536);
//    goalJointStates.pitch_joint_2 = static_cast<int16_t>(finalPoint.positions[2] / M_PI / 2 * 65536);
//    goalJointStates.roll_joint_1 = static_cast<int16_t>(finalPoint.positions[3] / M_PI / 2 * 65536);
//    goalJointStates.pitch_joint_3 = static_cast<int16_t>(finalPoint.positions[4] / M_PI / 2 * 8192);
//    goalJointStates.roll_joint_2 = static_cast<int16_t>(finalPoint.positions[5] / M_PI / 2 * 8192);
//    std::vector<int16_t> readyToSendJS = parseJointStates(goalJointStates);
//    for(auto& point : goal->trajectory.joint_names)
//    {
//        RCLCPP_INFO(this->get_logger(), "joint name: %s", point.c_str());
//    }
//    RCLCPP_INFO(this->get_logger(), "Send joint states to serial port");
//    result->error_code = 0;
//    result->error_string = "";
//    goal_handle->succeed(result);
//    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
//}

void rm_eng_action_server::handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle) {
    using std::placeholders::_1;
    std::thread{std::bind(&rm_eng_action_server::execute_move, this, _1), goal_handle}.detach();
}

std::vector<int16_t> rm_eng_action_server::parseJointStates(joint_states_for_send goal_joint_states) {
    std::vector<int16_t> jointStates(sizeof(joint_states_for_send) / sizeof(int16_t));
    memcpy(&jointStates[0], &goal_joint_states, sizeof(joint_states_for_send));
    return jointStates;
}

void rm_eng_action_server::publish_joint_states()
{
    rclcpp::Rate rate = rclcpp::Rate(20); // 定时进行操作
    while (rclcpp::ok()) {
        sensor_msgs::msg::JointState jointStates;
        jointStates.header.frame_id = "";
        jointStates.header.stamp = this->now();

        
        jointStates.name = {"pitch_joint_1", "pitch_joint_2", "pitch_joint_3", "roll_joint_1", "roll_joint_2", "shift_joint", "yaw_joint_1"};
        {
            std::lock_guard<std::mutex> lock(mutex_);
            jointStates.position = mJointStates;
        }

        joint_states_publisher->publish(jointStates);
        rate.sleep();
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(
    rm_eng_action_server
)