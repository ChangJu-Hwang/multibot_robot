#pragma once

#include <QWidget>
#include <QTimer>

#include "multibot_robot/ui_robot_panel.h"

namespace Ui
{
    class RobotPanel;
}

class RobotPanel : public QWidget
{
    Q_OBJECT
private:
    Ui::RobotPanel *ui_;
    QTimer timer_;

public:
    explicit RobotPanel(QWidget *_parent = nullptr);
    ~RobotPanel();
};