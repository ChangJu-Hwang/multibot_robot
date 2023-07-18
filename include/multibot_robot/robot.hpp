#pragma once

#include <vector>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "multibot_util/Instance.hpp"
#include "multibot_robot/motion_controller.hpp"

#include "multibot_ros2_interface/srv/robot_info.hpp"
#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/srv/path.hpp"

using namespace Instance;

namespace Robot
{
    class MultibotRobot : public rclcpp::Node
    {
    private:
        using RobotInfo = multibot_ros2_interface::srv::RobotInfo;
        using RobotState = multibot_ros2_interface::msg::RobotState;
        using LocalPath = multibot_ros2_interface::msg::LocalPath;
        using Path = multibot_ros2_interface::srv::Path;

    private:
        struct PathSegment
        {
            Position::Pose start_;
            Position::Pose goal_;

            double departure_time_;
            double arrival_time_;

            double distance_;
            double angle_;

            double rotational_duration_;
            double translational_duration_;

            PathSegment(const LocalPath &_localPath);
        };

    private:
        void saveRobotInfo(
            const std::shared_ptr<RobotInfo::Request> _request,
            std::shared_ptr<RobotInfo::Response> _response);

        void receivePath(
            const std::shared_ptr<Path::Request> _request,
            std::shared_ptr<Path::Response> _response);

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg);

        void publish_topics();
        void report_state();
        void control();

    private:
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Service<RobotInfo>::SharedPtr registration_;
        rclcpp::Service<Path>::SharedPtr control_command_;
        rclcpp::Publisher<RobotState>::SharedPtr robotState_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    private:
        Position::Pose PoseComputer(
            const PathSegment _pathSegment,
            const double _time);

    private:
        AgentInstance::Agent robot_;

        std::vector<PathSegment> path_;
        int localPathIdx_;
        bool is_activated_;

        double time_;
        double timeStep_;

        double linear_tolerance_;
        double angular_tolerance_;

    public:
        MultibotRobot();
        ~MultibotRobot();
    }; // class MultibotRobot
} // namespace Robot