#include "multibot_robot/kanayama_controller.hpp"

using namespace Control;

// Kanayama Contoller
// See also: https://edisciplinas.usp.br/pluginfile.php/2333514/mod_resource/content/0/Artigo_Kanayama.pdf
bool Kanayama_Controller::control(
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

    Position::Coordinates cur_unit_vec = Position::Coordinates(
        cos(_curPose.component_.theta), sin(_curPose.component_.theta));

    Position::Coordinates cur_to_ref_vec = Position::Coordinates(
        reference_pose.component_.x - _curPose.component_.x,
        reference_pose.component_.y - _curPose.component_.y);

    double xError = cur_unit_vec * cur_to_ref_vec;
    double yError = Position::crossProduct(cur_unit_vec, cur_to_ref_vec);
    double thetaError = (yError > 0 ? 1 : -1) * Position::getAngleDiff(_curPose, reference_pose);

    double vRef = 0, wRef = 0;
    // Rotate
    if (traj_[localTrajIdx_].rotational_duration_ + 1e-8 > time_ - traj_[localTrajIdx_].departure_time_)
    {
        wRef = (traj_[localTrajIdx_].angle_ > 0 ? 1 : -1) *
               Motion::VelocityComputer(
                   traj_[localTrajIdx_].angle_, max_ang_vel_, max_ang_acc_,
                   time_ - traj_[localTrajIdx_].departure_time_);
    }
    // Move
    else
    {
        vRef = Motion::VelocityComputer(
            traj_[localTrajIdx_].distance_, max_lin_vel_, max_lin_acc_,
            time_ - traj_[localTrajIdx_].departure_time_ - traj_[localTrajIdx_].rotational_duration_);
    }

    cmd_vel.linear.x = vRef * cos(thetaError) + Kx_ * xError;
    cmd_vel.angular.z = wRef + vRef * (Ky_ * yError + Ktheta_ * sin(thetaError));

    time_ = time_ + timeStep_;
    cmd_vel_pub_->publish(cmd_vel);

    return true;
}

void Kanayama_Controller::init_varibales(const AgentInstance::Agent &_robot)
{
    timeStep_ = 0.01;

    max_lin_vel_ = _robot.max_linVel_;
    max_lin_acc_ = _robot.max_linAcc_;
    max_ang_vel_ = _robot.max_angVel_;
    max_ang_acc_ = _robot.max_angAcc_;
}

void Kanayama_Controller::init_parameters(const std::string _type)
{
    nh_->declare_parameter(_type + ".controlParam.kanayama.Kx");
    nh_->declare_parameter(_type + ".controlParam.kanayama.Ky");
    nh_->declare_parameter(_type + ".controlParam.kanayama.Ktheta");

    nh_->get_parameter_or(_type + ".linear.tolerance", linear_tolerance_, 0.10);
    nh_->get_parameter_or(_type + ".angular.tolerance", angular_tolerance_, 0.018);

    nh_->get_parameter_or(_type + ".controlParam.kanayama.Kx", Kx_, 0.25);
    nh_->get_parameter_or(_type + ".controlParam.kanayama.Ky", Ky_, 1.00);
    nh_->get_parameter_or(_type + ".controlParam.kanayama.Ktheta", Ktheta_, 1.27);
}

Kanayama_Controller::Kanayama_Controller(
    std::shared_ptr<rclcpp::Node> _nh,
    const AgentInstance::Agent &_robot)
{
    nh_ = _nh;

    init_varibales(_robot);
    init_parameters(_robot.type_);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    cmd_vel_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("/" + _robot.name_ + "/cmd_vel", qos);

    RCLCPP_INFO(nh_->get_logger(), "Kanayama Controller has been initialzied");
}

Kanayama_Controller::~Kanayama_Controller()
{
    RCLCPP_INFO(nh_->get_logger(), "Kanayama Controller has been terminated");
}