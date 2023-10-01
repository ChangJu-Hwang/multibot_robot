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
#include "multibot_robot/motion.hpp"
#include "multibot_robot/robot_panel.hpp"
#include "multibot_robot/kanayama_controller.hpp"
#include "multibot_robot/pid_controller.hpp"

#include "multibot_ros2_interface/msg/robot_state.hpp"
#include "multibot_ros2_interface/srv/traj.hpp"

using namespace Instance;

namespace Robot
{
    class MultibotRobot : public rclcpp::Node
    {
    private:
        using RobotState = multibot_ros2_interface::msg::RobotState;
        using Traj = Control::Trajectory_Follower::Traj;

    public:
        void execRobotPanel(int argc, char *argv[]);

    private:
        void init_varibales();
        void init_parameters();
        void loadRobotInfo();

        void receiveTraj(
            const std::shared_ptr<Traj::Request> _request,
            std::shared_ptr<Traj::Response> _response);

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg);
        void robotPoseCalculate();

        void run();
        void report_state();
        void auto_control();

    private:
        std::shared_ptr<rclcpp::Node> nh_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Service<Traj>::SharedPtr receiveTraj_cmd_;
        rclcpp::Publisher<RobotState>::SharedPtr robotState_pub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    private:
        AgentInstance::Agent robot_;

        std::string robotNamespace_;

        std::shared_ptr<Panel> robotPanel_;
        bool is_pannel_running_;

        Control::Strategy control_strategy_;
        std::shared_ptr<Control::Kanayama_Controller> kanayama_controller_;
        std::shared_ptr<Control::PID_Controller> pid_controller_;

    public:
        MultibotRobot();
        ~MultibotRobot();
    }; // class MultibotRobot
} // namespace Robot