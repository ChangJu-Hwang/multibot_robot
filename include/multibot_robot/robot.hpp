#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "multibot_util/Instance.hpp"
#include "multibot_robot/motion_generator.hpp"

#include "multibot_ros2_interface/msg/local_path.hpp"
#include "multibot_ros2_interface/srv/robot_info.hpp"
#include "multibot_ros2_interface/action/path.hpp"

using namespace Instance;

namespace Robot
{
    class MultibotRobot : public rclcpp::Node
    {
    private:
        using RobotInfo         = multibot_ros2_interface::srv::RobotInfo;
        using LocalPath         = multibot_ros2_interface::msg::LocalPath;
        using Path              = multibot_ros2_interface::action::Path;
        using GoalHandlePath    = rclcpp_action::ServerGoalHandle<Path>;
    
    private:
        void saveRobotInfo(
            const std::shared_ptr<RobotInfo::Request>   _request,
            std::shared_ptr<RobotInfo::Response>        _response);

        // rclcpp_action::GoalResponse handle_goal(
        //     const rclcpp_action::GoalUUID &_uuid,
        //     std::shared_ptr<const Path::Goal> _goal_handle);
        // rclcpp_action::CancelResponse handle_cancle(
        //     const std::shared_ptr<GoalHandlePath> _goal_handle);
        // void execute_controller(const std::shared_ptr<GoalHandlePath> _goal_handle);

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg);

    private:
        rclcpp::Service<RobotInfo>::SharedPtr registration_;
        // rclcpp_action::Server<Path>::SharedPtr controller_server_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    private:
        AgentInstance::Agent robot_;
        // nav_msgs::msg::Odometry odom_;
        // std::vector<LocalPath> path_segments_;

        double time_;
        double timeStep_;
    
    public:
        MultibotRobot();
        ~MultibotRobot();
    }; // class MultibotRobot
} // namespace Robot