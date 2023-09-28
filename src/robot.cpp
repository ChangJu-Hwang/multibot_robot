#include "multibot_robot/robot.hpp"

#include <string>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <QApplication>

using namespace std::chrono_literals;
using namespace Robot;

void MultibotRobot::execRobotPanel(int argc, char *argv[])
{
    QApplication app(argc, argv);

    robotPanel_ = std::make_shared<Panel>(nh_, robot_);
    robotPanel_->show();

    is_pannel_running_ = true;

    app.exec();
}

void MultibotRobot::init_varibales()
{
    nh_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

    is_pannel_running_ = false;
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void MultibotRobot::init_parameters()
{
    this->declare_parameter("name");
    this->declare_parameter("linear_tolerance");
    this->declare_parameter("angular_tolerance");

    this->get_parameter_or("name", robotNamespace_, std::string("ISR_M2"));

    loadRobotInfo();
}

void MultibotRobot::loadRobotInfo()
{
    this->declare_parameter("type");

    this->get_parameter_or("name", robot_.name_, std::string("ISR_M2"));
    this->get_parameter_or("type", robot_.type_, std::string("ISR_M2"));

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

void MultibotRobot::receiveTraj(
    const std::shared_ptr<Traj::Request> _request,
    std::shared_ptr<Traj::Response> _response)
{
    kanayama_controller_->receiveTraj(_request);

    _response->receive_status = true;
    robotPanel_->setModeState(PanelUtil::Mode::AUTO);
}

void MultibotRobot::odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg)
{
    tf2::Quaternion q(
        _odom_msg->pose.pose.orientation.x,
        _odom_msg->pose.pose.orientation.y,
        _odom_msg->pose.pose.orientation.z,
        _odom_msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    robot_.linVel_ = _odom_msg->twist.twist.linear.x;
    robot_.angVel_ = _odom_msg->twist.twist.angular.z;

    if (is_pannel_running_ and
        not(robotPanel_->getModeState() == PanelUtil::Mode::MANUAL))
    {
        robotPanel_->setVelocity(
            robot_.linVel_, robot_.angVel_);
    }
}

void MultibotRobot::robotPoseCalculate()
{
    geometry_msgs::msg::TransformStamped geoTr;

    try
    {
        geoTr = tf_buffer_->lookupTransform(
            "map", (robot_.name_ + "/base_link"), tf2::TimePointZero);
    }
    catch (const tf2::TransformException &ex)
    {
        return;
    }

    tf2::Quaternion q(
        geoTr.transform.rotation.x,
        geoTr.transform.rotation.y,
        geoTr.transform.rotation.z,
        geoTr.transform.rotation.w);
    tf2::Matrix3x3 m(q);

    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    robot_.pose_.component_.x = geoTr.transform.translation.x;
    robot_.pose_.component_.y = geoTr.transform.translation.y;
    robot_.pose_.component_.theta = yaw;
}

void MultibotRobot::run()
{
    robotPoseCalculate();
    report_state();
    
    auto_control();
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

void MultibotRobot::auto_control()
{
    if (not(is_pannel_running_ == true and
            robotPanel_->getModeState() == PanelUtil::AUTO))
        return;

    if (not(kanayama_controller_->control(robot_.pose_)))
    {
        std::cout << "Current Pose: " << robot_.pose_ << std::endl;
        robotPanel_->set_pushButton_Manual_clicked();

        return;
    }
}

MultibotRobot::MultibotRobot()
    : Node("robot")
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    init_varibales();
    init_parameters();

    kanayama_controller_ = std::make_shared<Control::Kanayama_Controller>(nh_, robot_);

    receiveTraj_cmd_ = this->create_service<Traj>(
        "/" + robotNamespace_ + "/traj",
        std::bind(&MultibotRobot::receiveTraj, this, std::placeholders::_1, std::placeholders::_2));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/" + robotNamespace_ + "/odom", qos,
        std::bind(&MultibotRobot::odom_callback, this, std::placeholders::_1));

    robotState_pub_ = this->create_publisher<RobotState>("/" + robotNamespace_ + "/state", qos);

    update_timer_ = this->create_wall_timer(
        10ms, std::bind(&MultibotRobot::run, this));

    RCLCPP_INFO(this->get_logger(), "MultibotRobot has been initialized");
}

MultibotRobot::~MultibotRobot()
{
    RCLCPP_INFO(this->get_logger(), "MultibotRobot has been terminated");
}