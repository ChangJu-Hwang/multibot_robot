#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "multibot_util/Instance.hpp"
#include "multibot_robot/motion_generator.hpp"

#include "multibot_ros2_interface/srv/robot_info.hpp"
#include "multibot_ros2_interface/msg/robot_state.hpp"

using namespace Instance;

namespace Robot
{
    class MultibotRobot : public rclcpp::Node
    {
    private:
        using RobotInfo         = multibot_ros2_interface::srv::RobotInfo;
        using RobotState        = multibot_ros2_interface::msg::RobotState;
    
    private:
        void saveRobotInfo(
            const std::shared_ptr<RobotInfo::Request>   _request,
            std::shared_ptr<RobotInfo::Response>        _response);

        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg);
        void report_state();

    private:
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Service<RobotInfo>::SharedPtr registration_;
        rclcpp::Publisher<RobotState>::SharedPtr robotState_pub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    
    private:
        AgentInstance::Agent robot_;
        
        double time_;
        double timeStep_;
    
    public:
        MultibotRobot();
        ~MultibotRobot();
    }; // class MultibotRobot
} // namespace Robot