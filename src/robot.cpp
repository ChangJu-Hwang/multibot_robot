#include "multibot_robot/robot.hpp"

#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace Instance;
using namespace Robot;

void MultibotRobot::saveRobotInfo(const std::shared_ptr<RobotInfo::Request> _request,
                                  std::shared_ptr<RobotInfo::Response> _response)
{
    robot_.name_ = _request->config.name;
    robot_.wheel_radius_ = _request->config.wheel_radius;
    robot_.wheel_seperation_ = _request->config.wheel_seperation;

    robot_.max_linVel_ = _request->config.max_linvel;
    robot_.max_linAcc_ = _request->config.max_linacc;
    robot_.max_angVel_ = _request->config.max_angvel;
    robot_.max_angAcc_ = _request->config.max_angacc;

    _response->registration_status = true;
}

// rclcpp_action::GoalResponse MultibotRobot::handle_goal(
//     const rclcpp_action::GoalUUID &_uuid,
//     std::shared_ptr<const Path::Goal> _goal_handle)
// {
//     RCLCPP_INFO(this->get_logger(), "Received request to control %s.", robot_.name_);
//     (void)_uuid;

//     path_segments_.clear();
//     path_segments_ = _goal_handle->path_segments;

//     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
// }

// rclcpp_action::CancelResponse MultibotRobot::handle_cancle(const std::shared_ptr<GoalHandlePath> _goal_handle)
// {
//     RCLCPP_INFO(this->get_logger(), "Received request to cancel %s controller.", robot_.name_);
//     (void)_goal_handle;

//     return rclcpp_action::CancelResponse::ACCEPT;
// }

// void MultibotRobot::execute_controller(const std::shared_ptr<GoalHandlePath> _goal_handle)
// {
//     RCLCPP_INFO(this->get_logger(), "Execute %s controller.", robot_.name_);
//     rclcpp::Rate loop_rate(timeStep_);

//     auto feedback_msg = std::make_shared<Path::Feedback>();

//     time_ = 0;
//     for (const auto path_segment : path_segments_)
//     {
//         // Motion::MotionGenerator motionGenerator;
//         // while(time_ + 1e-8 < path_segment.departure_time.sec and
//         //       rclcpp::ok())
//         while (time_ + 1e-8 < 4.0 and
//                rclcpp::ok())
//         {
//             feedback_msg->odom = odom_;

//             time_ = time_ + timeStep_;

//             _goal_handle->publish_feedback(feedback_msg);
//             loop_rate.sleep();
//         }

//         // Position::Pose goal(path_segment.goal.x, path_segment.goal.y, path_segment.goal.theta);
//         // while(Position::getAngleDiff(robot_.pose_, goal) > 1e-8 and
//         //       rclcpp::ok())
//         // {
//         //     feedback_msg->odom = odom_;

//         //     _goal_handle->publish_feedback(feedback_msg);
//         //     loop_rate.sleep();
//         // }

//         // while(Position::getDistance(robot_.pose_, goal) > 1e-8 and
//         //       rclcpp::ok())
//         // {
//         //     feedback_msg->odom = odom_;

//         //     _goal_handle->publish_feedback(feedback_msg);
//         //     loop_rate.sleep();
//         // }
//     }

//     if (rclcpp::ok())
//     {
//         auto result = std::make_shared<Path::Result>();
//         result->pose = robot_.pose_.component_;

//         _goal_handle->succeed(result);
//     }
// }

void MultibotRobot::odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg)
{
    // odom_ = *_odom_msg.get();

    robot_.pose_.component_.x = _odom_msg->pose.pose.position.x;
    robot_.pose_.component_.y = _odom_msg->pose.pose.position.y;

    tf2::Quaternion q(
        _odom_msg->pose.pose.orientation.x,
        _odom_msg->pose.pose.orientation.y,
        _odom_msg->pose.pose.orientation.z,
        _odom_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    robot_.pose_.component_.theta = yaw;
}

MultibotRobot::MultibotRobot()
    : Node("robot")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    timeStep_ = 0.1;

    this->declare_parameter("namespace");
    std::string robotNamespace = this->get_parameter("namespace").as_string();

    registration_ = this->create_service<RobotInfo>("/" + robotNamespace + "/robotInfo",
                                                    std::bind(&MultibotRobot::saveRobotInfo, this, std::placeholders::_1, std::placeholders::_2));

    // controller_server_ = rclcpp_action::create_server<Path>(
    //     this->get_node_base_interface(),
    //     this->get_node_clock_interface(),
    //     this->get_node_logging_interface(),
    //     this->get_node_waitables_interface(),
    //     "/" +robotNamespace + "/controller",
    //     std::bind(&MultibotRobot::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    //     std::bind(&MultibotRobot::handle_cancle, this, std::placeholders::_1),
    //     std::bind(&MultibotRobot::execute_controller, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + robotNamespace + "/odom", qos,
        std::bind(&MultibotRobot::odom_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "MultibotRobot has been initialized");
}

MultibotRobot::~MultibotRobot()
{
    RCLCPP_INFO(this->get_logger(), "MultibotRobot has been terminated");
}