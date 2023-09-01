#include "multibot_robot/robot.hpp"

#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <QApplication>

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

void MultibotRobot::execRobotPanel(int argc, char *argv[])
{
    QApplication app(argc, argv);

    robotPanel_ = std::make_shared<Panel>();
    robotPanel_->setRobotName(robot_.name_);
    robotPanel_->attach(*this);
    robotPanel_->show();

    is_pannel_running_ = true;

    app.exec();
}

void MultibotRobot::init(std::chrono::milliseconds _timeStep)
{
    timeStep_ = _timeStep.count() * 1e-3;

    is_activated_ = false;
    is_pannel_running_ = false;

    this->declare_parameter("name");
    this->declare_parameter("linear_tolerance");
    this->declare_parameter("angular_tolerance");

    robotNamespace_ = this->get_parameter("name").as_string();
    this->get_parameter_or("linear_tolerance", linear_tolerance_, 0.10);
    this->get_parameter_or("angular_tolerance", angular_tolerance_, 0.018);

    this->get_parameter_or("Kx", Kx_, 0.25);
    this->get_parameter_or("Ky", Ky_, 1.00);
    this->get_parameter_or("Ktheta", Ktheta_, 1.27);

    loadRobotInfo();
}

void MultibotRobot::loadRobotInfo()
{
    this->declare_parameter("type");

    robot_.name_ = this->get_parameter("name").as_string();
    robot_.type_ = this->get_parameter("type").as_string();

    if (is_pannel_running_)
        robotPanel_->setRobotName(robot_.name_);

    this->declare_parameter(robot_.type_ + ".size");
    this->declare_parameter(robot_.type_ + ".wheels.separation");
    this->declare_parameter(robot_.type_ + ".wheels.radius");

    this->declare_parameter(robot_.type_ + ".linear.velocity");
    this->declare_parameter(robot_.type_ + ".linear.acceleration");
    this->declare_parameter(robot_.type_ + ".angular.velocity");
    this->declare_parameter(robot_.type_ + ".angular.acceleration");

    this->declare_parameter("goal.x");
    this->declare_parameter("goal.y");
    this->declare_parameter("goal.theta");

    this->get_parameter_or(robot_.type_ + ".size", robot_.size_, 0.0);
    this->get_parameter_or(robot_.type_ + ".wheels.separation", robot_.wheel_seperation_, 0.0);
    this->get_parameter_or(robot_.type_ + ".wheels.radius", robot_.wheel_radius_, 0.0);

    this->get_parameter_or(robot_.type_ + ".linear.velocity", robot_.max_linVel_, 0.0);
    this->get_parameter_or(robot_.type_ + ".linear.acceleration", robot_.max_linAcc_, 0.0);
    this->get_parameter_or(robot_.type_ + ".angular.velocity", robot_.max_angVel_, 0.0);
    this->get_parameter_or(robot_.type_ + ".angular.acceleration", robot_.max_angAcc_, 0.0);

    geometry_msgs::msg::Pose2D goalPose;
    this->get_parameter_or("goal.x", goalPose.x, 0.0);
    this->get_parameter_or("goal.y", goalPose.y, 0.0);
    this->get_parameter_or("goal.theta", goalPose.theta, 0.0);
    robot_.goal_ = goalPose;
}

bool MultibotRobot::request_connection()
{
    while (!connection_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return false;
        }
        RCLCPP_ERROR(this->get_logger(), "Connection not available, waiting again...");
    }

    auto request = std::make_shared<Connection::Request>();

    request->config.name = robot_.name_;
    request->config.type = robot_.type_;
    request->config.size = robot_.size_;
    request->config.wheel_radius = robot_.wheel_radius_;
    request->config.wheel_seperation = robot_.wheel_seperation_;

    request->config.max_linvel = robot_.max_linVel_;
    request->config.max_linacc = robot_.max_linAcc_;
    request->config.max_angvel = robot_.max_angVel_;
    request->config.max_angacc = robot_.max_angAcc_;

    request->goal = robot_.goal_.component_;

    auto response_received_callback = [this](rclcpp::Client<Connection>::SharedFuture _future)
    {
        auto response = _future.get();
        return;
    };

    auto future_result =
        connection_->async_send_request(request, response_received_callback);

    return future_result.get()->is_connected;
}

bool MultibotRobot::request_disconnection()
{
    while (!disconnection_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return false;
        }
        RCLCPP_ERROR(this->get_logger(), "Disconnection not available, waiting again...");
    }

    auto request = std::make_shared<Disconnection::Request>();

    request->name = robot_.name_;

    auto response_received_callback = [this](rclcpp::Client<Disconnection>::SharedFuture _future)
    {
        auto response = _future.get();
        return;
    };

    auto future_result =
        disconnection_->async_send_request(request, response_received_callback);

    return future_result.get()->is_disconnected;
}

bool MultibotRobot::request_modeChange(bool _is_remote)
{
    while (!modeFromRobot_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
            return false;
        }
        RCLCPP_ERROR(this->get_logger(), "ModeChange not available, waiting again...");
    }

    auto request = std::make_shared<ModeSelection::Request>();

    request->name = robot_.name_;
    request->is_remote = _is_remote;

    auto response_received_callback = [this](rclcpp::Client<ModeSelection>::SharedFuture _future)
    {
        auto response = _future.get();
        return;
    };

    auto future_result =
        modeFromRobot_->async_send_request(request, response_received_callback);

    return future_result.get()->is_complete;
}

void MultibotRobot::change_robot_mode(
    const std::shared_ptr<ModeSelection::Request> _request,
    std::shared_ptr<ModeSelection::Response> _response)
{
    if (_request->name != robot_.name_)
        abort();

    if (_request->is_remote == true)
        robotPanel_->setModeState(PanelUtil::Mode::REMOTE);
    else
    {
        robotPanel_->setModeState(PanelUtil::Mode::MANUAL);
        robotPanel_->setVelocity(0.0, 0.0);
    }

    _response->is_complete = true;
}

void MultibotRobot::respond_to_serverScan(const std_msgs::msg::Bool::SharedPtr _msg)
{
    if (_msg->data == false)
        return;

    robotPanel_->set_pushButton_Connect_clicked();
}

void MultibotRobot::respond_to_emergencyStop(const std_msgs::msg::Bool::SharedPtr _msg)
{
    if (_msg->data == false)
        return;

    robotPanel_->set_pushButton_Manual_clicked();
}

void MultibotRobot::respond_to_kill(const std_msgs::msg::Bool::SharedPtr _msg)
{
    if (_msg->data == false)
        return;

    robotPanel_->setConnectionState(false);
    robotPanel_->setModeState(PanelUtil::MANUAL);
    robotPanel_->setVelocity(0.0, 0.0);
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
    robotPanel_->setModeState(PanelUtil::Mode::AUTO);
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

    robot_.linVel_ = _odom_msg->twist.twist.linear.x;
    robot_.angVel_ = _odom_msg->twist.twist.angular.z;

    if (not(robotPanel_->getModeState() == PanelUtil::Mode::MANUAL))
    {
        robotPanel_->setVelocity(
            robot_.linVel_, robot_.angVel_);
    }
}

void MultibotRobot::tf_broadcast()
{
    geometry_msgs::msg::TransformStamped t;

    t.header.frame_id = "map";
    t.child_frame_id = robot_.name_ + "/odom";

    t.transform.translation.x = robot_.pose_.component_.x;
    t.transform.translation.y = robot_.pose_.component_.y;
    t.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, robot_.pose_.component_.theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);
}

void MultibotRobot::publish_topics()
{
    report_state();
    control();
    tf_broadcast();
}

void MultibotRobot::report_state()
{
    auto robotStateMsg = RobotState();

    robotStateMsg.name = robot_.name_;
    robotStateMsg.pose = robot_.pose_.component_;
    robotStateMsg.lin_vel = robot_.linVel_;
    robotStateMsg.ang_vel = robot_.angVel_;

    robotState_pub_->publish(robotStateMsg);
}

// Kanayama Contoller
// See also: https://edisciplinas.usp.br/pluginfile.php/2333514/mod_resource/content/0/Artigo_Kanayama.pdf
void MultibotRobot::control()
{
    geometry_msgs::msg::Twist cmd_vel;

    if (is_pannel_running_ == true and
        robotPanel_->getModeState() == PanelUtil::MANUAL)
    {
        cmd_vel_pub_->publish(robotPanel_->get_cmd_vel());
        return;
    }

    if (is_pannel_running_ == true and
        robotPanel_->getModeState() == PanelUtil::REMOTE)
    {
        return;
    }

    if (not(is_activated_))
    {
        return;
    }

    if (path_.front().departure_time_ > time_)
    {
        time_ = time_ + timeStep_;
        cmd_vel_pub_->publish(cmd_vel);
        return;
    }

    if (time_ > path_[localPathIdx_].arrival_time_ + 1e-8)
        localPathIdx_++;

    if (localPathIdx_ >= static_cast<int>(path_.size()))
    {
        is_activated_ = false;
        cmd_vel_pub_->publish(cmd_vel);
        std::cout << "Current Pose: " << robot_.pose_ << std::endl;

        robotPanel_->set_pushButton_Manual_clicked();

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

void MultibotRobot::update(const PanelUtil::Msg &_msg)
{
    if (not(is_pannel_running_))
        return;

    switch (_msg.first)
    {
    case PanelUtil::Request::CONNECTION_REQUEST:
    {
        bool is_connected = request_connection();
        robotPanel_->setConnectionState(is_connected);
        break;
    }

    case PanelUtil::Request::DISCONNECTION_REQUEST:
    {
        bool is_disconnected = request_disconnection();
        robotPanel_->setConnectionState(not(is_disconnected));
        if (is_disconnected)
        {
            robotPanel_->setModeState(PanelUtil::Mode::MANUAL);
            robotPanel_->setVelocity(0.0, 0.0);
        }

        break;
    }

    case PanelUtil::Request::MANUAL_REQUEST:
    {
        if (robotPanel_->getConnectionState() == false)
            break;

        bool is_complete = request_modeChange(false);

        if (is_complete)
        {
            robotPanel_->setModeState(PanelUtil::Mode::MANUAL);
            robotPanel_->setVelocity(0.0, 0.0);
        }

        break;
    }

    case PanelUtil::Request::REMOTE_REQUEST:
    {
        if (robotPanel_->getConnectionState() == false)
            break;

        bool is_complete = request_modeChange(true);

        if (is_complete)
            robotPanel_->setModeState(PanelUtil::Mode::REMOTE);

        break;
    }

    default:
        break;
    }
}

MultibotRobot::MultibotRobot()
    : Node("robot")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    auto timeStep = 10ms;

    init(timeStep);

    connection_ = this->create_client<Connection>("/connection");
    disconnection_ = this->create_client<Disconnection>("/disconnection");

    serverScan_ = this->create_subscription<std_msgs::msg::Bool>(
        "/server_scan", qos, std::bind(&MultibotRobot::respond_to_serverScan, this, std::placeholders::_1));

    emergencyStop_ = this->create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop", qos, std::bind(&MultibotRobot::respond_to_emergencyStop, this, std::placeholders::_1));

    killRobot_ = this->create_subscription<std_msgs::msg::Bool>(
        "/" + robotNamespace_ + "/kill", qos,
        std::bind(&MultibotRobot::respond_to_kill, this, std::placeholders::_1));

    modeFromServer_ = this->create_service<ModeSelection>(
        "/" + robotNamespace_ + "/modeFromServer",
        std::bind(&MultibotRobot::change_robot_mode, this, std::placeholders::_1, std::placeholders::_2));

    modeFromRobot_ = this->create_client<ModeSelection>("/" + robotNamespace_ + "/modeFromRobot");

    control_command_ = this->create_service<Path>(
        "/" + robotNamespace_ + "/path",
        std::bind(&MultibotRobot::receivePath, this, std::placeholders::_1, std::placeholders::_2));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + robotNamespace_ + "/odom", qos,
        std::bind(&MultibotRobot::odom_callback, this, std::placeholders::_1));

    tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(*this);

    robotState_pub_ = this->create_publisher<RobotState>("/" + robotNamespace_ + "/state", qos);
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/" + robotNamespace_ + "/cmd_vel", qos);

    update_timer_ = this->create_wall_timer(
        timeStep, std::bind(&MultibotRobot::publish_topics, this));

    RCLCPP_INFO(this->get_logger(), "MultibotRobot has been initialized");
}

MultibotRobot::~MultibotRobot()
{
    request_disconnection();
    RCLCPP_INFO(this->get_logger(), "MultibotRobot has been terminated");
}