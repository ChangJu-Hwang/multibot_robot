#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "multibot_util/Instance.hpp"

#include "multibot_ros2_interface/srv/robot_info.hpp"
#include "multibot_ros2_interface/action/path.hpp"

using namespace Instance;

namespace Robot
{
    class MultibotRobot : public rclcpp::Node
    {
    private:
        using RobotInfo         = multibot_ros2_interface::srv::RobotInfo;
        using Path              = multibot_ros2_interface::action::Path;
        using GoalHandlePath    = rclcpp_action::ServerGoalHandle<Path>;
    
    private:
        void saveRobotInfo(
            const std::shared_ptr<RobotInfo::Request>    _request,
            std::shared_ptr<RobotInfo::Response>         _response);
        
        rclcpp_action::GoalResponse handle_goal(
            const rclcpp_action::GoalUUID &_uuid,
            std::shared_ptr<const Path::Goal> _goal_handle);
        rclcpp_action::CancelResponse handle_cancel(
            const std::shared_ptr<GoalHandlePath> _goal_handle);
        void execute_controller(const std::shared_ptr<GoalHandlePath> _goal_handle);

    private:
        rclcpp::Service<RobotInfo>::SharedPtr registration_;
        rclcpp_action::Server<Path>::SharedPtr control_server_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    private:
        AgentInstance::Agent robot_;
    
    public:
        MultibotRobot();
        ~MultibotRobot();
    }; // class MultibotRobot
} // namespace Robot