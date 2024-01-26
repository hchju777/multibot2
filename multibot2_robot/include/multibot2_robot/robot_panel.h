#pragma once

#include <memory>

#include <nav2_util/lifecycle_node.hpp>

#include <QWidget>
#include <QTimer>
#include <QKeyEvent>

#include "multibot2_robot/instance_manager.h"

#include "multibot2_util/panel_util.h"

#include "multibot2_robot/ui_robot_panel.h"

#include <std_msgs/msg/bool.hpp>

namespace Ui
{
    class RobotPanel;
}

namespace multibot2_robot
{
    class Panel : public QWidget
    {
    public:
        typedef std::unique_ptr<Panel> UniquePtr;
        typedef std::shared_ptr<Panel> SharedPtr;

    public:
        typedef multibot2_util::PanelUtil::Tab Tab;
        typedef multibot2_util::PanelUtil::Mode Mode;

        typedef multibot2_util::PanelUtil::Connection Connection;
        typedef multibot2_util::PanelUtil::Disconnection Disconnection;
        typedef multibot2_util::PanelUtil::ModeSelection ModeSelection;

    public:
        Panel() {}

        Panel(nav2_util::LifecycleNode::SharedPtr &_nh, Instance_Manager::SharedPtr &_instance_manager,
              QWidget *_parent = nullptr);

        ~Panel() { delete ui_; }

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

    protected:
        void init_ui();

    protected:
        void manual_control();

    protected:
        bool request_connection();

        bool request_disconnection();

        bool request_modeChange(Mode _mode);

        void respond_to_serverScan(const std_msgs::msg::Bool::SharedPtr _msg);

        void respond_to_emergencyStop(const std_msgs::msg::Bool::SharedPtr _msg);

        void respond_to_kill(const std_msgs::msg::Bool::SharedPtr _msg);

        void change_robot_mode(
            const std::shared_ptr<ModeSelection::Request> _request,
            std::shared_ptr<ModeSelection::Response> _response);

    protected:
        Ui::RobotPanel *ui_;
        QTimer *displayTimer_;

    protected:
        nav2_util::LifecycleNode::SharedPtr nh_;

        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Client<Connection>::SharedPtr connection_;
        rclcpp::Client<Disconnection>::SharedPtr disconnection_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr serverScan_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergencyStop_;

        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr kill_robot_cmd_;

        rclcpp::Service<ModeSelection>::SharedPtr modeFromServer_;
        rclcpp::Client<ModeSelection>::SharedPtr modeFromRobot_;

    protected:
        Instance_Manager::SharedPtr instance_manager_;

        bool connection_state_{false};

    }; // class Panel
}