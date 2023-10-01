#include "multibot_robot/pid_controller.hpp"

using namespace Control;

bool PID_Controller::control(
    const Position::Pose &_curPose)
{
    geometry_msgs::msg::Twist cmd_vel;

    if (time_ > traj_[localTrajIdx_].arrival_time_ + 1e-8)
        localTrajIdx_++;

    if (localTrajIdx_ >= static_cast<int>(traj_.size()))
    {
        cmd_vel_pub_->publish(cmd_vel);
        return false;
    }

    if (traj_.front().departure_time_ > time_)
    {
        time_ = time_ + timeStep_;

        cmd_vel_pub_->publish(cmd_vel);
        return true;
    }

    Position::Pose reference_pose = PoseComputer(traj_[localTrajIdx_], time_);

    double xError = reference_pose.component_.x - _curPose.component_.x;
    double yError = reference_pose.component_.y - _curPose.component_.y;

    double linearError = std::sqrt(xError * xError + yError * yError);
    double angularError = reference_pose.component_.theta - _curPose.component_.theta;
    while (std::fabs(angularError) - M_PI > 1e-8 or
           angularError + 1e-8 > M_PI)
    {
        int sign = angularError > 0 ? 1 : -1;
        angularError = angularError - sign * 2 * M_PI;
    }

    static double prior_linearError = 0.0;
    static double prior_angularError = 0.0;

    cmd_vel.linear.x = kp_linear_ * linearError + kd_linear_ * (linearError - prior_linearError);
    cmd_vel.angular.z = kp_angular_ * angularError + kd_angular_ * (angularError - prior_angularError);

    prior_linearError = linearError;
    prior_angularError = angularError;

    time_ = time_ + timeStep_;
    cmd_vel_pub_->publish(cmd_vel);

    return true;
}

void PID_Controller::odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg)
{
    current_linear_velocity_ = _odom_msg->twist.twist.linear.x;
    current_angular_velocity_ = _odom_msg->twist.twist.angular.z;
}

void PID_Controller::init_varibales(const AgentInstance::Agent &_robot)
{
    timeStep_ = 0.01;

    max_lin_vel_ = _robot.max_linVel_;
    max_lin_acc_ = _robot.max_linAcc_;
    max_ang_vel_ = _robot.max_angVel_;
    max_ang_acc_ = _robot.max_angAcc_;
}

void PID_Controller::init_parameters(const std::string _type)
{
    nh_->declare_parameter(_type + ".controlParam.PID.kp_linear");
    nh_->declare_parameter(_type + ".controlParam.PID.kp_angular");
    nh_->declare_parameter(_type + ".controlParam.PID.kd_linear");
    nh_->declare_parameter(_type + ".controlParam.PID.kd_angular");

    nh_->get_parameter_or(_type + ".linear.tolerance", linear_tolerance_, 0.10);
    nh_->get_parameter_or(_type + ".angular.tolerance", angular_tolerance_, 0.018);

    nh_->get_parameter_or(_type + ".controlParam.PID.kp_linear", kp_linear_, 1.00);
    nh_->get_parameter_or(_type + ".controlParam.PID.kp_angular", kp_angular_, 1.00);
    nh_->get_parameter_or(_type + ".controlParam.PID.kd_linear", kd_linear_, 0.01);
    nh_->get_parameter_or(_type + ".controlParam.PID.kd_angular", kd_angular_, 0.01);
}

PID_Controller::PID_Controller(
    std::shared_ptr<rclcpp::Node> _nh,
    const AgentInstance::Agent &_robot)
{
    nh_ = _nh;

    init_varibales(_robot);
    init_parameters(_robot.type_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    odom_sub_ = nh_->create_subscription<nav_msgs::msg::Odometry>(
        "/" + _robot.name_ + "/odom", qos,
        std::bind(&PID_Controller::odom_callback, this, std::placeholders::_1));
    cmd_vel_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("/" + _robot.name_ + "/cmd_vel", qos);

    RCLCPP_INFO(nh_->get_logger(), "PID Controller has been initialzied");
}

PID_Controller::~PID_Controller()
{
    RCLCPP_INFO(nh_->get_logger(), "PID Controller has been terminated");
}