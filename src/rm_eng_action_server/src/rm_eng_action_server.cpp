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
    mJointStates = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    joint_states_publisher = this->create_publisher<sensor_msgs::msg::JointState>("joint_states",qos);
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
            RCLCPP_INFO(this->get_logger(), "point[%d]: joint[1]:%f,joint[2]:%f,joint[3]:%f,joint[4]:%f,joint[5]:%f,joint[6]:%f", i, point.positions[0],point.positions[1],point.positions[2],point.positions[3],point.positions[4],point.positions[5]);
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
            std::chrono::nanoseconds ns(duration);


            rclcpp::sleep_for(ns);
        }

        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        feedback->header.frame_id = goal->trajectory.header.frame_id;
        feedback->header.stamp = goal->trajectory.header.stamp;
        feedback->joint_names = goal->trajectory.joint_names;
//        feedback->actual = goal->trajectory.joint_names;
//        feedback->desired = ;
//        feedback->error = "";

        for(int i = 0; i < static_cast<int>(point.positions.size()); i++)
        {
            mJointStates[i] = point.positions.at(i);
        }

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

void rm_eng_action_server::execute(const std::shared_ptr<GoalHandleFJT> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Start to move!");
    auto result = std::make_shared<FollowJointTrajectory::Result>();
    auto goal = goal_handle->get_goal();
    auto trjPointsSize = goal->trajectory.points.size();
    auto finalPoint = goal->trajectory.points.at(trjPointsSize - 1);
    goalJointStates.header = goalJointStateHeader;
    for(int i = 0; i < static_cast<int>(finalPoint.positions.size()); i++)
    {
        goalJointStates.positions.at(i) = finalPoint.positions.at(i);
    }
    std::vector<uint8_t> readyToSendJS = parseJointStates(goalJointStates);
    serial_port->write(readyToSendJS);
    result->error_code = 0;
    result->error_string = "";
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal Succeeded");
}

void rm_eng_action_server::handle_accepted(const std::shared_ptr<GoalHandleFJT> goal_handle) {
    using std::placeholders::_1;
    std::thread{std::bind(&rm_eng_action_server::execute_move, this, _1), goal_handle}.detach();
}

std::vector<uint8_t> parseJointStates(joint_states goal_joint_states)
{
    std::vector<uint8_t> jointStates;
    jointStates.push_back(goal_joint_states.header);
    for(int i = 0; i < 6; i++)
    {
        jointStates.push_back(goal_joint_states.positions.at(i));
    }
    return jointStates;
}

void rm_eng_action_server::publish_joint_states()
{
    rclcpp::Rate rate = rclcpp::Rate(15); // 定时进行操作
    while (rclcpp::ok()) {
        sensor_msgs::msg::JointState jointStates;
        jointStates.header.frame_id = "";
// 这样子不对
//        jointStates.header.stamp.sec = this->now().seconds();
//        jointStates.header.stamp.nanosec = this->now().nanoseconds();

        double timeSec = this->now().seconds();
        int32_t sec = timeSec;
        jointStates.header.stamp.sec = sec;
        jointStates.header.stamp.nanosec = (timeSec - sec) * 1e9;
        
        jointStates.name = {"pitch_joint_1","pitch_joint_2", "pitch_joint_3", "roll_joint_1", "roll_joint_2", "yaw_joint_1"};
        jointStates.position = mJointStates;

        joint_states_publisher->publish(jointStates);

//        RCLCPP_INFO(this->get_logger(), "Publish joint states"); /*Publish feedback*/
//        qDebug() << "joint states:" << mJointStates;
        rate.sleep();
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(
    rm_eng_action_server
)