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
    start_ = Position::Pose(_localPath.start);
    goal_ = Position::Pose(_localPath.goal);

    departure_time_ = _localPath.departure_time;
    arrival_time_ = _localPath.arrival_time;

    Position::Coordinates start_unit_vector;
    start_unit_vector.x_ = cos(start_.component_.theta);
    start_unit_vector.y_ = sin(start_.component_.theta);
    Position::Coordinates goal_unit_vector;
    goal_unit_vector.x_ = cos(goal_.component_.theta);
    goal_unit_vector.y_ = sin(goal_.component_.theta);

    int angleSign = Position::crossProduct(start_unit_vector, goal_unit_vector) > 0 ? 1 : -1;

    distance_ = Position::getDistance(start_, goal_);
    angle_ = angleSign * Position::getAngleDiff(start_, goal_);
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
    const std::shared_ptr<Path::Request> _request,
    std::shared_ptr<Path::Response> _response)
{
    is_activated_ = true;
    localPathIdx_ = 0;

    time_ = -1 * _request->start_time;
    path_.clear();
    for (const auto &localPath : _request->path)
    {
        PathSegment pathSegment(localPath);
        pathSegment.rotational_duration_ = Motion::TotalMoveTimeComputer(
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

    robotStateMsg.name = robot_.name_;
    robotStateMsg.pose = robot_.pose_.component_;

    robotState_pub_->publish(robotStateMsg);
}

// Kanayama Contoller
// See also: https://edisciplinas.usp.br/pluginfile.php/2333514/mod_resource/content/0/Artigo_Kanayama.pdf
void MultibotRobot::control()
{
    geometry_msgs::msg::Twist cmd_vel;

    if (not(is_activated_))
    {
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }

    if (path_.front().departure_time_ > time_)
    {
        time_ = time_ + timeStep_;
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }

    if (time_ > path_[localPathIdx_].arrival_time_+ 1e-8 )
        localPathIdx_++;

    if (localPathIdx_ >= static_cast<int>(path_.size()))
    {
        is_activated_ = false;
        cmd_vel_pub_->publish(cmd_vel);
        std::cout << "Current Pose: " << robot_.pose_ << std::endl;
        return;
    }

    Position::Pose reference_pose = PoseComputer(path_[localPathIdx_], time_);

    Position::Coordinates cur_unit_vec = Position::Coordinates(
        cos(robot_.pose_.component_.theta), sin(robot_.pose_.component_.theta));

    Position::Coordinates cur_to_ref_vec = Position::Coordinates(
        reference_pose.component_.x - robot_.pose_.component_.x,
        reference_pose.component_.y - robot_.pose_.component_.y);

    double xError = cur_unit_vec * cur_to_ref_vec;
    double yError = Position::crossProduct(cur_unit_vec, cur_to_ref_vec);
    double thetaError = (yError > 0 ? 1 : -1) * Position::getAngleDiff(robot_.pose_, reference_pose);

    double vRef = 0, wRef = 0;
    // Rotate
    if (path_[localPathIdx_].rotational_duration_ + 1e-8 > time_ - path_[localPathIdx_].departure_time_)
    {
        wRef = (path_[localPathIdx_].angle_ > 0 ? 1 : -1) *
               Motion::VelocityComputer(
                   path_[localPathIdx_].angle_, robot_.max_angVel_, robot_.max_angAcc_,
                   time_ - path_[localPathIdx_].departure_time_);
    }
    // Move
    else
    {
        vRef = Motion::VelocityComputer(
            path_[localPathIdx_].distance_, robot_.max_linVel_, robot_.max_linAcc_,
            time_ - path_[localPathIdx_].departure_time_ - path_[localPathIdx_].rotational_duration_);
    }

    cmd_vel.linear.x = vRef * cos(thetaError) + Kx_ * xError;
    cmd_vel.angular.z = wRef + vRef * (Ky_ * yError + Ktheta_ * sin(thetaError));

    time_ = time_ + timeStep_;
    cmd_vel_pub_->publish(cmd_vel);
}

Position::Pose MultibotRobot::PoseComputer(
    const PathSegment _pathSegment,
    const double _time)
{
    try
    {
        if (_pathSegment.departure_time_ > _time + 1e-8)
            throw _time;
    }
    catch (double _wrong_time)
    {
        std::cerr << "[Error] MultibotRobot::PoseComputer(): "
                  << "Out of range(Time): " << _wrong_time << " / "
                  << "Valid range: " << _pathSegment.departure_time_
                  << " ~ " << _pathSegment.arrival_time_ << std::endl;
        std::abort();
    }

    if (_time > _pathSegment.arrival_time_ + 1e-8)
        return _pathSegment.goal_;

    Position::Pose pose = _pathSegment.start_;
    // Rotate
    if (_pathSegment.rotational_duration_ + 1e-8 > _time - _pathSegment.departure_time_)
    {
        pose.component_.theta = pose.component_.theta +
                                (_pathSegment.angle_ > 0 ? 1 : -1) *
                                    Motion::DisplacementComputer(
                                        _pathSegment.angle_, robot_.max_angVel_, robot_.max_angAcc_,
                                        _time - _pathSegment.departure_time_);

        while (std::fabs(pose.component_.theta) - M_PI > 1e-8 or
               pose.component_.theta + 1e-8 > M_PI)
        {
            int sign = pose.component_.theta > 0 ? 1 : -1;
            pose.component_.theta = pose.component_.theta - sign * 2 * M_PI;
        }
    }
    // Move
    else
    {
        double displacement = Motion::DisplacementComputer(
            _pathSegment.distance_, robot_.max_linVel_, robot_.max_linAcc_,
            _time - _pathSegment.departure_time_ - _pathSegment.rotational_duration_);

        double theta = _pathSegment.goal_.component_.theta;

        pose.component_.x = pose.component_.x + displacement * cos(theta);
        pose.component_.y = pose.component_.y + displacement * sin(theta);
        pose.component_.theta = theta;
    }

    return pose;
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

    this->get_parameter_or("Kx", Kx_, 0.25);
    this->get_parameter_or("Ky", Ky_, 1.00);
    this->get_parameter_or("Ktheta", Ktheta_, 1.27);

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