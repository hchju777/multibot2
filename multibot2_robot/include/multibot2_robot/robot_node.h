#pragma once

#include <memory>

#include <QApplication>

#include <nav2_util/lifecycle_node.hpp>

#include "multibot2_robot/instance_manager.h"
#include "multibot2_robot/global_planner/navfn_planner.hpp"
#include "multibot2_robot/teb_local_planner/teb_local_planner_ros.h"
#include "multibot2_robot/robot_panel.h"

namespace multibot2_robot
{
    class MultibotRobot : public nav2_util::LifecycleNode
    {
    public:
        typedef std::unique_ptr<MultibotRobot> UniquePtr;
        typedef std::shared_ptr<MultibotRobot> SharedPtr;

    public:
        MultibotRobot();

        ~MultibotRobot();

    public:
        void run_robot_panel(int argc, char *argv[]);

    protected:
        void init_variables();
        void init_parameters();

    protected:
        void auto_control();

    protected:
        nav2_util::LifecycleNode::SharedPtr nh_;
        rclcpp::TimerBase::SharedPtr control_timer_;

    protected:
        Instance_Manager::SharedPtr instance_manager_;

        std::shared_ptr<nav2_navfn_planner::NavfnPlanner> navfn_global_planner_;
        std::shared_ptr<teb_local_planner::TebLocalPlannerROS> teb_local_planner_;

        Panel::SharedPtr robot_panel_;

        bool panel_is_running_{false};
        double goal_tolerance_{0.25};
    }; // class MultibotRobot
} // namespace multibot2_robot