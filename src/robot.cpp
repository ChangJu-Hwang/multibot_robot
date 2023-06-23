#include "multibot_robot/robot.hpp"

#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

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

void MultibotRobot::odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg)
{
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

void MultibotRobot::report_state()
{
    auto robotStateMsg = RobotState();

    robotStateMsg.name  = robot_.name_;
    robotStateMsg.pose  = robot_.pose_.component_;

    robotState_pub_->publish(robotStateMsg);
}

MultibotRobot::MultibotRobot()
    : Node("robot")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    timeStep_ = 0.1;

    this->declare_parameter("namespace");
    std::string robotNamespace = this->get_parameter("namespace").as_string();

    registration_ = this->create_service<RobotInfo>("/" + robotNamespace + "/info",
                                                    std::bind(&MultibotRobot::saveRobotInfo, this, std::placeholders::_1, std::placeholders::_2));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + robotNamespace + "/odom", qos,
        std::bind(&MultibotRobot::odom_callback, this, std::placeholders::_1));

    robotState_pub_ = this->create_publisher<RobotState>("/" + robotNamespace + "/state", qos);

    update_timer_ = this->create_wall_timer(
        10ms, std::bind(&MultibotRobot::report_state, this));

    RCLCPP_INFO(this->get_logger(), "MultibotRobot has been initialized");
}

MultibotRobot::~MultibotRobot()
{
    RCLCPP_INFO(this->get_logger(), "MultibotRobot has been terminated");
}