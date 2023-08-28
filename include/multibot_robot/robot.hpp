#pragma once

#include <vector>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "multibot_util/Instance.hpp"
#include "multibot_robot/motion_controller.hpp"
#include "multibot_robot/robot_panel.hpp"

#include "multibot_ros2_interface/srv/connection.hpp"
#include "multibot_ros2_interface/srv/disconnection.hpp"
#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/srv/mode_selection.hpp"
#include "multibot_ros2_interface/srv/path.hpp"

using namespace Instance;

namespace Robot
{
    class MultibotRobot : public rclcpp::Node, public Observer::ObserverInterface<PanelUtil::Msg>
    {
    private:
        using Connection = multibot_ros2_interface::srv::Connection;
        using Disconnection = multibot_ros2_interface::srv::Disconnection;
        using RobotState = multibot_ros2_interface::msg::RobotState;
        using ModeSelection = multibot_ros2_interface::srv::ModeSelection;
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

    public:
        void execRobotPanel(int argc, char *argv[]);

    private:
        void init(std::chrono::milliseconds _timeStep);

        void loadRobotInfo();

        bool request_connection();
        bool request_disconnection();
        bool request_modeChange(bool _is_remote);
        void change_robot_mode(
            const std::shared_ptr<ModeSelection::Request> _request,
            std::shared_ptr<ModeSelection::Response> _response);
        void respond_to_serverScan(const std_msgs::msg::Bool::SharedPtr _msg);
        void respond_to_kill(const std_msgs::msg::Bool::SharedPtr _msg);

        void receivePath(
            const std::shared_ptr<Path::Request> _request,
            std::shared_ptr<Path::Response> _response);

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg);

        void publish_topics();
        void report_state();
        void control();

    private:
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Client<Connection>::SharedPtr connection_;
        rclcpp::Client<Disconnection>::SharedPtr disconnection_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr serverScan_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killRobot_;
        rclcpp::Service<ModeSelection>::SharedPtr modeFromServer_;
        rclcpp::Client<ModeSelection>::SharedPtr modeFromRobot_;

        rclcpp::Service<Path>::SharedPtr control_command_;
        rclcpp::Publisher<RobotState>::SharedPtr robotState_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    private:
        Position::Pose PoseComputer(
            const PathSegment _pathSegment,
            const double _time);

    public:
        void update(const PanelUtil::Msg &_msg) override;

    private:
        AgentInstance::Agent robot_;

        std::vector<PathSegment> path_;
        int localPathIdx_;
        bool is_activated_;

        std::string robotNamespace_;

        double time_;
        double timeStep_;

        // Current: Does not use.
        double linear_tolerance_;
        double angular_tolerance_;

        // Kanayama Controller Parameter
        double Kx_, Ky_, Ktheta_;

        std::shared_ptr<Panel> robotPanel_;
        bool is_pannel_running_;

    public:
        MultibotRobot();
        ~MultibotRobot();
    }; // class MultibotRobot
} // namespace Robot