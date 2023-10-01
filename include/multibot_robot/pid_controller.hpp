#pragma once

#include <nav_msgs/msg/odometry.hpp>

#include "multibot_robot/trajectory_follower.hpp"

using namespace Instance;

namespace Control
{
    class PID_Controller : public Trajectory_Follower
    {
    public:
        bool control(
            const Position::Pose &_curPose) override;
        
    private:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg);
    
    private:
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    private:
        void init_varibales(const AgentInstance::Agent &_robot);
        void init_parameters(const std::string _type);

    private:
        double current_linear_velocity_;
        double current_angular_velocity_;

        double loop_rate_ = 10; // Hz

        // PID Controller Parameter
        double kp_linear_, kd_linear_;
        double kp_angular_, kd_angular_;

    public:
        PID_Controller(
            std::shared_ptr<rclcpp::Node> _nh,
            const AgentInstance::Agent &_robot);
        ~PID_Controller();
    }; // class PID_Controller
} // using namespace Control