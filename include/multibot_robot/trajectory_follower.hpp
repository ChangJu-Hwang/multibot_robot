#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "multibot_util/Instance.hpp"
#include "multibot_robot/motion.hpp"

#include "multibot_ros2_interface/srv/traj.hpp"

using namespace Instance;

namespace Control
{
    enum Strategy
    {
        Kanayama,
        PID
    }; // enum Strategy

    class Trajectory_Follower
    {
    public:
        using Traj = multibot_ros2_interface::srv::Traj;

    protected:
        using LocalTraj = multibot_ros2_interface::msg::LocalTraj;

    protected:
        struct TrajSegment
        {
            Position::Pose start_;
            Position::Pose goal_;

            double departure_time_;
            double arrival_time_;

            double distance_;
            double angle_;

            double rotational_duration_;
            double translational_duration_;

            TrajSegment(const LocalTraj &_localTraj);
        };

    protected:
        virtual bool control(
            const Position::Pose &_curPose) = 0;

    public:
        void receiveTraj(
            const std::shared_ptr<Traj::Request> _request);

    protected:
        Position::Pose PoseComputer(
            const TrajSegment &_trajSegment,
            const double _time);

    protected:
        std::shared_ptr<rclcpp::Node> nh_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

    protected:
        int localTrajIdx_;

        double max_lin_vel_, max_lin_acc_;
        double max_ang_vel_, max_ang_acc_;

        // Current: Do no use
        double linear_tolerance_;
        double angular_tolerance_;

        double time_;
        double timeStep_;

        std::vector<TrajSegment> traj_;

    public:
        virtual ~Trajectory_Follower(){};
    }; // class Trajectory_Follower
} // namespace Control