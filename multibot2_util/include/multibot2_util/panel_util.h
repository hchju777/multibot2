#pragma once

#include "multibot2_msgs/srv/connection.hpp"
#include "multibot2_msgs/srv/disconnection.hpp"
#include "multibot2_msgs/srv/mode_selection.hpp"

namespace multibot2_util::PanelUtil
{
    typedef multibot2_msgs::srv::Connection Connection;
    typedef multibot2_msgs::srv::Disconnection Disconnection;
    typedef multibot2_msgs::srv::ModeSelection ModeSelection;

    enum Tab
    {
        DASHBOARD,
        ROBOT
    }; // enum Tab

    enum Request
    {
        NO_REQUEST,
        PLAN_REQUEST,
        START_REQUEST
    }; // enum Request

    enum Mode
    {
        REMOTE,
        MANUAL,
        AUTO,
        STAY
    }; // enum Mode

    enum PlanState
    {
        READY,
        PLANNING,
        SUCCESS,
        FAIL
    }; // enum PlanState

    typedef Request Msg;
} // namespace PanelUtil;