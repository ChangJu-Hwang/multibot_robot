#include "multibot_robot/robot_panel.hpp"

#include "multibot_robot/ui_robot_panel.h"

RobotPanel::RobotPanel(QWidget *_parent)
    : QWidget(_parent), ui_(new Ui::RobotPanel)
{
    ui_->setupUi(this);
}

RobotPanel::~RobotPanel()
{
    delete ui_;
}