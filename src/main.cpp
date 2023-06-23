#include <iostream>
#include <memory>

#include "multibot_robot/robot.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<Robot::MultibotRobot>();

    rclcpp::spin(robot);
    rclcpp::shutdown();

    return 0;
}