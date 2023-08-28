#pragma once

#include <list>

#include <geometry_msgs/msg/twist.hpp>

#include <QWidget>
#include <QTimer>
#include <QKeyEvent>

#include "multibot_util/Interface/Observer_Interface.hpp"
#include "multibot_util/Instance.hpp"
#include "multibot_robot/ui_robot_panel.h"

using namespace Instance;

namespace PanelUtil
{
    enum Request
    {
        NO_REQUEST,
        CONNECTION_REQUEST,
        DISCONNECTION_REQUEST,
        REMOTE_REQUEST,
        MANUAL_REQUEST
    }; // enum Request

    enum Mode
    {
        REMOTE,
        MANUAL,
        AUTO
    }; // enum Mode

    typedef std::pair<Request, Mode> Msg;
} // namespace PanelUtil

namespace Ui
{
    class RobotPanel;
} // namespace Ui

namespace Robot
{
    class Panel : public QWidget, public Observer::SubjectInterface<PanelUtil::Msg>
    {
    private:
        Q_OBJECT

    public:
        void attach(Observer::ObserverInterface<PanelUtil::Msg> &_observer) override;
        void detach(Observer::ObserverInterface<PanelUtil::Msg> &_observer) override;
        void notify() override;

    private slots:
        void connectionDisp();
        void modeDisp();
        void velocityDisp();

    private slots:
        void on_pushButton_Connect_clicked();
        void on_pushButton_Disconnect_clicked();

        void on_pushButton_Remote_clicked();
        void on_pushButton_Manual_clicked();

        void keyPressEvent(QKeyEvent *_event) override;

    public:
        void setConnectionState(bool _connection_state) { connection_state_ = _connection_state; }
        void setModeState(PanelUtil::Mode _mode_state) { mode_state_ = _mode_state; }
        void setVelocity(double _lin_vel, double _ang_vel);
        void set_pushButton_Connect_clicked();
        void set_pushButton_Manual_clicked();

        void setRobotName(const std::string _robotName);

        PanelUtil::Mode getModeState() { return mode_state_; }
        bool getConnectionState() { return connection_state_; }
        geometry_msgs::msg::Twist get_cmd_vel();

    private:
        void pubManualMsg();

    private:
        Ui::RobotPanel *ui_;
        QTimer *displayTimer_;

    private:
        bool connection_state_;
        PanelUtil::Mode mode_state_;

        PanelUtil::Msg msg_;
        double lin_vel_;
        double ang_vel_;

        std::list<Observer::ObserverInterface<PanelUtil::Msg> *> list_observer_;

    public:
        explicit Panel(QWidget *_parent = nullptr);
        ~Panel();
    }; // class Panel
} // namespace Robot