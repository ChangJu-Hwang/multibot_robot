#include <iostream>
#include <memory>
#include <future>
#include <thread>

#include <QApplication>

#include "multibot_robot/robot.hpp"
#include "multibot_robot/robot_panel.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto robot = std::make_shared<Robot::MultibotRobot>();

    auto spinThread = std::async(
        [robot]()
        {
            rclcpp::spin(robot);
            rclcpp::shutdown();
        });

    auto panelThread = std::async(
        [&argc, &argv]()
        {
            QApplication app(argc, argv);

            RobotPanel robotPanel;
            robotPanel.show();
            app.exec();
        }
    );

    spinThread.get();
    panelThread.get();

    return 0;
}