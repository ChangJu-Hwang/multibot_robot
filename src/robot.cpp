#include "multibot_robot/robot.hpp"

#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std::chrono_literals;

using namespace Instance;
using namespace Robot;

using LocalPath = multibot_ros2_interface::msg::LocalPath;


MultibotRobot::PathSegment::PathSegment(const LocalPath &_localPath)
{
    start_  = Position::Pose(_localPath.start);
    goal_   = Position::Pose(_localPath.goal);

    departure_time_ = _localPath.departure_time;
    arrival_time_   = _localPath.arrival_time;

    Position::Coordinates start_unit_vector;
        start_unit_vector.x_ = cos(start_.component_.theta);
        start_unit_vector.y_ = sin(start_.component_.theta);
    Position::Coordinates goal_unit_vector;
        goal_unit_vector.x_  = cos(goal_.component_.theta);
        goal_unit_vector.y_  = sin(goal_.component_.theta);

    int angleSign   = Position::crossProduct(start_unit_vector, goal_unit_vector) > 0 ? 1 : -1;

    distance_       = Position::getDistance(start_, goal_);
    angle_          = angleSign * Position::getAngleDiff(start_, goal_);
}

void MultibotRobot::saveRobotInfo(
    const std::shared_ptr<RobotInfo::Request> _request,
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

void MultibotRobot::receivePath(
    const std::shared_ptr<Path::Request>    _request,
    std::shared_ptr<Path::Response>         _response)
{
    is_activated_ = true;
    localPathIdx_ = 0;

    time_ = -1 * _request->start_time;
    path_.clear();
    for (const auto &localPath : _request->path)
    {
        PathSegment pathSegment(localPath);
        pathSegment.rotational_duration_    = Motion::TotalMoveTimeComputer(
            pathSegment.angle_, robot_.max_angVel_, robot_.max_angAcc_);
        pathSegment.translational_duration_ = Motion::TotalMoveTimeComputer(
            pathSegment.distance_, robot_.max_linVel_, robot_.max_linAcc_);
        
        // Todo: Delete(Just for test)
        pathSegment.arrival_time_ = pathSegment.departure_time_ + pathSegment.rotational_duration_ + pathSegment.translational_duration_;

        path_.push_back(pathSegment);
    }

    _response->receive_status = true;
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

void MultibotRobot::publish_topics()
{
    report_state();
    control();
}

void MultibotRobot::report_state()
{
    auto robotStateMsg = RobotState();

    robotStateMsg.name  = robot_.name_;
    robotStateMsg.pose  = robot_.pose_.component_;

    robotState_pub_->publish(robotStateMsg);
}

void MultibotRobot::control()
{
    geometry_msgs::msg::Twist cmd_vel;

    if (not(is_activated_))
    {
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }

    if (time_ > path_[localPathIdx_].arrival_time_+ 1e-8 and 
        Position::getDistance(robot_.pose_, path_[localPathIdx_].goal_) < linear_tolerance_)
        localPathIdx_++;
    
    if (localPathIdx_ >= static_cast<int>(path_.size()))
    {
        is_activated_ = false;
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }
    
    // Move
    if(time_ > path_[localPathIdx_].arrival_time_ - path_[localPathIdx_].translational_duration_ + 1e-8)
    {
        cmd_vel.linear.x = Motion::VelocityComputer(
            path_[localPathIdx_].distance_, robot_.max_linVel_, robot_.max_linAcc_,
            path_[localPathIdx_].translational_duration_ - (path_[localPathIdx_].arrival_time_ - time_));
    }
    // Rotate
    else if(time_ > path_[localPathIdx_].arrival_time_ - path_[localPathIdx_].translational_duration_ - path_[localPathIdx_].rotational_duration_ + 1e-8)
    {
        double w = Motion::VelocityComputer(
            path_[localPathIdx_].angle_, robot_.max_angVel_, robot_.max_angAcc_,
            path_[localPathIdx_].rotational_duration_ - (path_[localPathIdx_].arrival_time_ - time_ - path_[localPathIdx_].translational_duration_));
        
        if (path_[localPathIdx_].angle_ < 0)
            w = -1 * w;
        
        cmd_vel.angular.z = w;
    }

    time_ = time_ + timeStep_;
    cmd_vel_pub_->publish(cmd_vel);
}

MultibotRobot::MultibotRobot()
    : Node("robot")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    auto timeStep = 10ms;
    timeStep_ = timeStep.count() * 1e-3;

    is_activated_ = false;

    this->declare_parameter("namespace");
    this->declare_parameter("linear_tolerance");
    this->declare_parameter("angular_tolerance");

    std::string robotNamespace = this->get_parameter("namespace").as_string();
    this->get_parameter_or("linear_tolerance", linear_tolerance_, 0.10);
    this->get_parameter_or("angular_tolerance", angular_tolerance_, 0.018);

    registration_ = this->create_service<RobotInfo>(
        "/" + robotNamespace + "/info",
        std::bind(&MultibotRobot::saveRobotInfo, this, std::placeholders::_1, std::placeholders::_2));

    control_command_ = this->create_service<Path>(
        "/" + robotNamespace + "/path",
        std::bind(&MultibotRobot::receivePath, this, std::placeholders::_1, std::placeholders::_2));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + robotNamespace + "/odom", qos,
        std::bind(&MultibotRobot::odom_callback, this, std::placeholders::_1));

    robotState_pub_ = this->create_publisher<RobotState>("/" + robotNamespace + "/state", qos);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + robotNamespace + "/cmd_vel", qos);

    update_timer_ = this->create_wall_timer(
        timeStep, std::bind(&MultibotRobot::publish_topics, this));

    RCLCPP_INFO(this->get_logger(), "MultibotRobot has been initialized");
}

MultibotRobot::~MultibotRobot()
{
    RCLCPP_INFO(this->get_logger(), "MultibotRobot has been terminated");
}