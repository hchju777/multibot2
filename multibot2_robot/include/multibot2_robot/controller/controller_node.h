#pragma once

#include <QApplication>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "multibot2_robot/controller/controller_panel.h"

namespace Controller
{
    class ControllerNode : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        typedef std::unique_ptr<ControllerNode> UniquePtr;
        typedef std::shared_ptr<ControllerNode> SharedPtr;

        typedef ControllerPanel::Robot Robot;
        typedef ControllerPanel::Robot_ROS Robot_ROS;

    public:
        ControllerNode();

        ~ControllerNode();

    public:
        void run_controller_panel(int argc, char *argv[]);

    protected:
        rclcpp_lifecycle::LifecycleNode::SharedPtr nh_;

        ControllerPanel::SharedPtr controller_panel_;

        std::shared_ptr<Robot_ROS> robot_ros_;

    }; // class ControllerNode
} // namespace Controller