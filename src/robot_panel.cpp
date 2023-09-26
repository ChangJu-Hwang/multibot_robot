#include "multibot_robot/robot_panel.hpp"

#include <QString>

#include "multibot_robot/ui_robot_panel.h"

using namespace std::chrono_literals;

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
    case Mode::REMOTE:
        ui_->label_Mode_State->setText("Remote");
        ui_->label_Mode_State->setStyleSheet(
            "color: rgb(0,213,255);\nborder: none");
        break;

    case Mode::MANUAL:
        ui_->label_Mode_State->setText("Manual");
        ui_->label_Mode_State->setStyleSheet(
            "color: rgb(255,190,11);\nborder: none");
        break;

    case Mode::AUTO:
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
    if (connection_state_ == false and request_connection())
        connection_state_ = true;
}

void Robot::Panel::on_pushButton_Disconnect_clicked()
{
    if (connection_state_ == true and request_disconnection())
    {
        connection_state_ = false;
        mode_state_ = Mode::MANUAL;

        lin_vel_ = 0.0;
        ang_vel_ = 0.0;
    }
}

void Robot::Panel::on_pushButton_Remote_clicked()
{
    if (connection_state_ == false)
        return;

    if (request_modeChange(Mode::REMOTE))
        mode_state_ = Mode::REMOTE;
}

void Robot::Panel::on_pushButton_Manual_clicked()
{
    if (connection_state_ == false)
        return;

    if (request_modeChange(Mode::MANUAL))
    {
        mode_state_ = Mode::MANUAL;

        lin_vel_ = 0.0;
        ang_vel_ = 0.0;
    }
}

void Robot::Panel::keyPressEvent(QKeyEvent *_event)
{
    if (mode_state_ == Mode::REMOTE)
        return;

    switch (_event->key())
    {
    case Qt::Key_Up:
        lin_vel_ = lin_vel_ + 0.1;
        break;

    case Qt::Key_Down:
        lin_vel_ = lin_vel_ - 0.1;
        break;

    case Qt::Key_Left:
        ang_vel_ = ang_vel_ + 0.1;
        break;

    case Qt::Key_Right:
        ang_vel_ = ang_vel_ - 0.1;
        break;

    case Qt::Key_Space:
        lin_vel_ = 0.0;
        ang_vel_ = 0.0;
        break;

    default:
        break;
    }
}

bool Robot::Panel::request_connection()
{
    while (!connection_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
            return false;
        }
        RCLCPP_ERROR(nh_->get_logger(), "Connection not available, waiting again...");
    }

    auto request = std::make_shared<Connection::Request>();

    request->config.name = robot_.name_;
    request->config.type = robot_.type_;
    request->config.size = robot_.size_;
    request->config.wheel_radius = robot_.wheel_radius_;
    request->config.wheel_seperation = robot_.wheel_seperation_;

    request->config.max_linvel = robot_.max_linVel_;
    request->config.max_linacc = robot_.max_linAcc_;
    request->config.max_angvel = robot_.max_angVel_;
    request->config.max_angacc = robot_.max_angAcc_;

    request->goal = robot_.goal_.component_;

    auto response_received_callback = [this](rclcpp::Client<Connection>::SharedFuture _future)
    {
        auto response = _future.get();
        return;
    };

    auto future_result =
        connection_->async_send_request(request, response_received_callback);

    return future_result.get()->is_connected;
}

bool Robot::Panel::request_disconnection()
{
    while (!disconnection_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
            return false;
        }
        RCLCPP_ERROR(nh_->get_logger(), "Disconnection not available, waiting again...");
    }

    auto request = std::make_shared<Disconnection::Request>();

    request->name = robot_.name_;

    auto response_received_callback = [this](rclcpp::Client<Disconnection>::SharedFuture _future)
    {
        auto response = _future.get();
        return;
    };

    auto future_result =
        disconnection_->async_send_request(request, response_received_callback);

    return future_result.get()->is_disconnected;
}

bool Robot::Panel::request_modeChange(Mode _mode)
{
    while (!modeFromRobot_->wait_for_service(1s))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
            return false;
        }
        RCLCPP_ERROR(nh_->get_logger(), "ModeChange not available, waiting again...");
    }

    auto request = std::make_shared<ModeSelection::Request>();

    request->name = robot_.name_;

    if (_mode == Mode::REMOTE)
        request->is_remote = true;
    else if (_mode == Mode::MANUAL)
        request->is_remote = false;
    else
    {
    }

    auto response_received_callback = [this](rclcpp::Client<ModeSelection>::SharedFuture _future)
    {
        auto response = _future.get();
        return;
    };

    auto future_result =
        modeFromRobot_->async_send_request(request, response_received_callback);

    return future_result.get()->is_complete;
}

void Robot::Panel::respond_to_serverScan(const std_msgs::msg::Bool::SharedPtr _msg)
{
    if (_msg->data == false)
        return;

    ui_->pushButton_Connect->clicked(true);
}

void Robot::Panel::respond_to_kill(const std_msgs::msg::Bool::SharedPtr _msg)
{
    if (_msg->data == false)
        return;

    ui_->pushButton_Disconnect->clicked(true);
}

void Robot::Panel::change_robot_mode(
    const std::shared_ptr<ModeSelection::Request> _request,
    std::shared_ptr<ModeSelection::Response> _response)
{
    if (_request->name != robot_.name_)
        abort();

    if (_request->is_remote == true)
        mode_state_ = Mode::REMOTE;
    else
    {
        mode_state_ = Mode::MANUAL;

        lin_vel_ = 0.0;
        ang_vel_ = 0.0;
    }

    _response->is_complete = true;
}

void Robot::Panel::respond_to_emergencyStop(const std_msgs::msg::Bool::SharedPtr _msg)
{
    if (_msg->data == false)
        return;

    ui_->pushButton_Manual->clicked(true);
}

void Robot::Panel::manual_control()
{
    if (not(mode_state_ == Mode::MANUAL))
        return;
    
    geometry_msgs::msg::Twist manual_cmd_vel;
    {
        manual_cmd_vel.linear.x = lin_vel_;
        manual_cmd_vel.angular.z = ang_vel_;
    }

    manual_cmd_vel_pub_->publish(manual_cmd_vel);
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

void Robot::Panel::set_pushButton_Manual_clicked()
{
    ui_->pushButton_Manual->clicked(true);
}

geometry_msgs::msg::Twist Robot::Panel::get_cmd_vel()
{
    geometry_msgs::msg::Twist cmd_vel;

    cmd_vel.linear.x = lin_vel_;
    cmd_vel.angular.z = ang_vel_;

    return cmd_vel;
}

Robot::Panel::Panel(
    std::shared_ptr<rclcpp::Node> &_nh,
    const AgentInstance::Agent &_robot,
    QWidget *_parent)
    : QWidget(_parent), ui_(new Ui::RobotPanel),
      nh_(_nh), robot_(_robot)
{
    // Panel Setup
    ui_->setupUi(this);
    ui_->label_Robot_Name->setText(QString::fromStdString(robot_.name_));

    setFocusPolicy(Qt::StrongFocus);

    setWindowFlags(
        Qt::WindowStaysOnTopHint |
        Qt::Window |
        Qt::WindowTitleHint |
        Qt::CustomizeWindowHint |
        Qt::WindowMinimizeButtonHint);

    // init Panel Units
    connection_state_ = false;
    mode_state_ = Mode::MANUAL;

    lin_vel_ = 0.0;
    ang_vel_ = 0.0;

    displayTimer_ = new QTimer(this);
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(connectionDisp()));
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(modeDisp()));
    connect(displayTimer_, SIGNAL(timeout()), this, SLOT(velocityDisp()));

    displayTimer_->start(10);

    // init ROS2 Instances
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

    connection_ = nh_->create_client<Connection>("/connection");
    disconnection_ = nh_->create_client<Disconnection>("/disconnection");
    modeFromRobot_ = nh_->create_client<ModeSelection>("/" + robot_.name_ + "/modeFromRobot");

    serverScan_ = nh_->create_subscription<std_msgs::msg::Bool>(
        "/server_scan", qos, std::bind(&Robot::Panel::respond_to_serverScan, this, std::placeholders::_1));
    killRobot_ = nh_->create_subscription<std_msgs::msg::Bool>(
        "/" + robot_.name_ + "/kill", qos,
        std::bind(&Robot::Panel::respond_to_kill, this, std::placeholders::_1));
    modeFromServer_ = nh_->create_service<ModeSelection>(
        "/" + robot_.name_ + "/modeFromServer",
        std::bind(&Robot::Panel::change_robot_mode, this, std::placeholders::_1, std::placeholders::_2));

    emergencyStop_ = nh_->create_subscription<std_msgs::msg::Bool>(
        "/emergency_stop", qos, std::bind(&Robot::Panel::respond_to_emergencyStop, this, std::placeholders::_1));

    manual_cmd_vel_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("/" + robot_.name_ + "/cmd_vel", qos);

    update_timer_ = nh_->create_wall_timer(
        10ms, std::bind(&Robot::Panel::manual_control, this));
}

Robot::Panel::~Panel()
{
    delete ui_;
}