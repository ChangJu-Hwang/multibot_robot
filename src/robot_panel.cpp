#include "multibot_robot/robot_panel.hpp"

#include <QString>

#include "multibot_robot/ui_robot_panel.h"

void Robot::Panel::attach(Observer::ObserverInterface<PanelUtil::Msg> &_observer)
{
    std::scoped_lock<std::mutex> lock(mtx_);

    if (std::find(list_observer_.begin(), list_observer_.end(), &_observer) == list_observer_.end())
        list_observer_.push_back(&_observer);
}

void Robot::Panel::detach(Observer::ObserverInterface<PanelUtil::Msg> &_observer)
{
    std::scoped_lock<std::mutex> lock(mtx_);

    auto observer_iter = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
    while (observer_iter != list_observer_.end())
    {
        list_observer_.remove(&_observer);
        observer_iter = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
    }
}

void Robot::Panel::notify()
{
    for (auto &observer : list_observer_)
        observer->update(msg_);
}

void Robot::Panel::connectionDisp()
{
    if (connection_state_)
    {
        ui_->label_Connection_State->setText("Connected");
        ui_->label_Connection_State->setStyleSheet(
            "color: rgb(0,255,51);\nborder: none");
    }
    else
    {
        ui_->label_Connection_State->setText("Disconnected");
        ui_->label_Connection_State->setStyleSheet(
            "color: rgb(255,0,110);\nborder: none");
    }
}

void Robot::Panel::modeDisp()
{
    switch (mode_state_)
    {
    case PanelUtil::Mode::REMOTE:
        ui_->label_Mode_State->setText("Remote");
        ui_->label_Mode_State->setStyleSheet(
            "color: rgb(0,213,255);\nborder: none");
        break;

    case PanelUtil::Mode::MANUAL:
        ui_->label_Mode_State->setText("Manual");
        ui_->label_Mode_State->setStyleSheet(
            "color: rgb(255,190,11);\nborder: none");
        break;
    
    case PanelUtil::Mode::AUTO:
        ui_->label_Mode_State->setText("Auto");
        ui_->label_Mode_State->setStyleSheet(
            "color: rgb(230, 19, 237);\nborder: none");

    default:
        break;
    }
}

void Robot::Panel::velocityDisp()
{
    QString lin_vel_qstring = QString::number(std::round(lin_vel_ * 100.0) / 100.0);
    QString ang_vel_qstring = QString::number(std::round(ang_vel_ * 100.0) / 100.0);

    ui_->label_Linear_Velocity->setText(lin_vel_qstring);
    ui_->label_Angular_Velocity->setText(ang_vel_qstring);
}

void Robot::Panel::on_pushButton_Connect_clicked()
{
    msg_.first = PanelUtil::Request::CONNECTION_REQUEST;
    notify();
}

void Robot::Panel::on_pushButton_Disconnect_clicked()
{
    msg_.first = PanelUtil::Request::DISCONNECTION_REQUEST;
    notify();
}

void Robot::Panel::on_pushButton_Remote_clicked()
{
    msg_.first = PanelUtil::Request::REMOTE_REQUEST;

    notify();
}

void Robot::Panel::on_pushButton_Manual_clicked()
{
    msg_.first = PanelUtil::Request::MANUAL_REQUEST;

    notify();
}

void Robot::Panel::keyPressEvent(QKeyEvent *_event)
{
    if (mode_state_ == PanelUtil::Mode::REMOTE)
        return;

    switch (_event->key())
    {
    case Qt::Key_Up:
        lin_vel_ = lin_vel_ + 0.1;
        pubManualMsg();
        break;

    case Qt::Key_Down:
        lin_vel_ = lin_vel_ - 0.1;
        pubManualMsg();
        break;

    case Qt::Key_Left:
        ang_vel_ = ang_vel_ + 0.1;
        pubManualMsg();
        break;

    case Qt::Key_Right:
        ang_vel_ = ang_vel_ - 0.1;
        pubManualMsg();
        break;

    case Qt::Key_Space:
        lin_vel_ = 0.0;
        ang_vel_ = 0.0;
        pubManualMsg();
        break;

    default:
        break;
    }
}

void Robot::Panel::setVelocity(double _lin_vel, double _ang_vel)
{
    lin_vel_ = _lin_vel;
    ang_vel_ = _ang_vel;
}

void Robot::Panel::set_pushButton_Connect_clicked()
{
    ui_->pushButton_Connect->clicked(true);
}

void Robot::Panel::setRobotName(const std::string _robotName)
{
    ui_->label_Robot_Name->setText(QString::fromStdString(_robotName));
}

geometry_msgs::msg::Twist Robot::Panel::get_cmd_vel()
{
    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = lin_vel_;
    cmd_vel.angular.z = ang_vel_;

    return cmd_vel;
}

void Robot::Panel::pubManualMsg()
{
    msg_.first = PanelUtil::Request::NO_REQUEST;
    msg_.second = mode_state_;

    notify();
}

Robot::Panel::Panel(QWidget *_parent)
    : QWidget(_parent), ui_(new Ui::RobotPanel)
{
    ui_->setupUi(this);

    setFocusPolicy(Qt::StrongFocus);

    setWindowFlags(
        Qt::WindowStaysOnTopHint |
        Qt::Window |
        Qt::WindowTitleHint |
        Qt::CustomizeWindowHint |
        Qt::WindowMinimizeButtonHint);

    // init
    connection_state_ = false;
    mode_state_ = PanelUtil::Mode::MANUAL;

    lin_vel_ = 0.0;
    ang_vel_ = 0.0;

    msg_.first = PanelUtil::Request::NO_REQUEST;
    msg_.second = PanelUtil::Mode::MANUAL;

    displayTimer_ = new QTimer(this);
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(connectionDisp()));
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(modeDisp()));
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(velocityDisp()));

    displayTimer_->start(10);
}

Robot::Panel::~Panel()
{
    delete ui_;
}