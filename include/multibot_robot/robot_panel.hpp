#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <QWidget>
#include <QTimer>
#include <QKeyEvent>

#include "multibot_util/Instance.hpp"
#include "multibot_robot/ui_robot_panel.h"

#include "multibot_util/Panel_Util.hpp"

using namespace Instance;
using namespace PanelUtil;

namespace Ui
{
    class RobotPanel;
} // namespace Ui

namespace Robot
{
    class Panel : public QWidget
    {
    private:
        Q_OBJECT

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
        void setModeState(Mode _mode_state) { mode_state_ = _mode_state; }
        void setVelocity(double _lin_vel, double _ang_vel);
        void set_pushButton_Connect_clicked();
        void set_pushButton_Manual_clicked();

        Mode getModeState() { return mode_state_; }
        bool getConnectionState() { return connection_state_; }
        geometry_msgs::msg::Twist get_cmd_vel();
    
    private:
        Ui::RobotPanel *ui_;
        QTimer *displayTimer_;

    private:
        bool request_connection();
        bool request_disconnection();
        bool request_modeChange(Mode _mode);

        void respond_to_serverScan(const std_msgs::msg::Bool::SharedPtr _msg);
        void respond_to_kill(const std_msgs::msg::Bool::SharedPtr _msg);
        void change_robot_mode(
            const std::shared_ptr<ModeSelection::Request> _request,
            std::shared_ptr<ModeSelection::Response> _response);

        void respond_to_emergencyStop(const std_msgs::msg::Bool::SharedPtr _msg);

        void manual_control();
    
    private:
        std::shared_ptr<rclcpp::Node> nh_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Client<Connection>::SharedPtr connection_;
        rclcpp::Client<Disconnection>::SharedPtr disconnection_;
        rclcpp::Client<ModeSelection>::SharedPtr modeFromRobot_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr serverScan_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr killRobot_;
        rclcpp::Service<ModeSelection>::SharedPtr modeFromServer_;
        
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergencyStop_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_vel_pub_;

    private:
        AgentInstance::Agent robot_;

        bool connection_state_;
        Mode mode_state_;

        double lin_vel_;
        double ang_vel_;

    public:
        Panel(
            std::shared_ptr<rclcpp::Node> &_nh,
            const AgentInstance::Agent &_robot,
            QWidget *_parent = nullptr);
        ~Panel();
    }; // class Panel
} // namespace Robot