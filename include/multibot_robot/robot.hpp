#pragma once

#include <vector>
#include <tuple>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "multibot_util/Instance.hpp"
#include "multibot_robot/motion_controller.hpp"
#include "multibot_robot/robot_panel.hpp"

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/srv/path.hpp"

using namespace Instance;

namespace Robot
{
    class MultibotRobot : public rclcpp::Node
    {
    private:
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

    public:
        void execRobotPanel(int argc, char *argv[]);

    private:
        void init(std::chrono::milliseconds _timeStep);

        void loadRobotInfo();

        void receivePath(
            const std::shared_ptr<Path::Request> _request,
            std::shared_ptr<Path::Response> _response);

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg);
        void robotPoseCalculate();

        void publish_topics();
        void report_state();
        void control();

    private:
        std::shared_ptr<rclcpp::Node> nh_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Service<Path>::SharedPtr control_command_;
        rclcpp::Publisher<RobotState>::SharedPtr robotState_pub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    private:
        Position::Pose PoseComputer(
            const PathSegment _pathSegment,
            const double _time);

    private:
        AgentInstance::Agent robot_;

        geometry_msgs::msg::PoseStamped::SharedPtr last_amcl_pose_;

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