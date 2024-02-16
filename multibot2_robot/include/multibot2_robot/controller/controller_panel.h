#pragma once

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include <QWidget>
#include <QTimer>
#include <QKeyEvent>

#include "multibot2_robot/instance_manager.h"

#include "multibot2_robot/ui_robot_panel.h"

namespace Ui
{
    class RobotPanel;
}

namespace Controller
{
    class ControllerPanel : public QWidget
    {
    public:
        typedef std::unique_ptr<ControllerPanel> UniquePtr;
        typedef std::shared_ptr<ControllerPanel> SharedPtr;

        typedef multibot2_robot::Robot Robot;
        typedef multibot2_robot::Robot_ROS Robot_ROS;

    public:
        ControllerPanel() {}

        ControllerPanel(rclcpp_lifecycle::LifecycleNode::SharedPtr &_nh, std::shared_ptr<Robot_ROS> _robot_ros,
                       QWidget *_parent = nullptr);

        ~ControllerPanel() { delete ui_; }

    private:
        Q_OBJECT

    private slots:
        void connectionDisp();
        void modeDisp();
        void velocityDisp();

    private slots:
        void keyPressEvent(QKeyEvent *_event) override;

    protected:
        void init_ui();

    protected:
        void manual_control();

    protected:
        Ui::RobotPanel *ui_;
        QTimer *displayTimer_;

    protected:
        rclcpp_lifecycle::LifecycleNode::SharedPtr nh_;

        rclcpp::TimerBase::SharedPtr update_timer_;

    protected:
        std::shared_ptr<Robot_ROS> robot_ros_;

    }; // class ControllerPanel
} // namespace Controller