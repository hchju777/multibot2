#include "multibot2_util/robot.h"

namespace multibot2_util
{
    BaseRobotInfo::BaseRobotInfo(const BaseRobotInfo &_baseRobotInfo)
    {
        name_ = _baseRobotInfo.name_;
        type_ = _baseRobotInfo.type_;

        pose_ = _baseRobotInfo.pose_;
        goal_ = _baseRobotInfo.goal_;

        radius_ = _baseRobotInfo.radius_;

        max_vel_x_ = _baseRobotInfo.max_vel_x_;
        max_vel_theta_ = _baseRobotInfo.max_vel_theta_;
        acc_lim_x_ = _baseRobotInfo.acc_lim_x_;
        acc_lim_theta_ = _baseRobotInfo.acc_lim_theta_;

        cur_vel_x_ = _baseRobotInfo.cur_vel_x_;
        cur_vel_theta_ = _baseRobotInfo.cur_vel_theta_;
    }

    BaseRobotInfo &BaseRobotInfo::operator=(const BaseRobotInfo &_rhs)
    {
        if (&_rhs != this)
        {
            name_ = _rhs.name_;
            type_ = _rhs.type_;

            pose_ = _rhs.pose_;
            goal_ = _rhs.goal_;

            radius_ = _rhs.radius_;

            max_vel_x_ = _rhs.max_vel_x_;
            max_vel_theta_ = _rhs.max_vel_theta_;
            acc_lim_x_ = _rhs.acc_lim_x_;
            acc_lim_theta_ = _rhs.acc_lim_theta_;

            cur_vel_x_ = _rhs.cur_vel_x_;
            cur_vel_theta_ = _rhs.cur_vel_theta_;
        }

        return *this;
    }

    void BaseRobotInfo::initBaseRobotInfo()
    {
        name_ = std::string();
        type_ = std::string();

        pose_.setZero();
        goal_.setZero();

        radius_ = std::numeric_limits<double>::quiet_NaN();

        max_vel_x_ = std::numeric_limits<double>::quiet_NaN();
        max_vel_theta_ = std::numeric_limits<double>::quiet_NaN();
        acc_lim_x_ = std::numeric_limits<double>::quiet_NaN();
        acc_lim_theta_ = std::numeric_limits<double>::quiet_NaN();

        cur_vel_x_ = std::numeric_limits<double>::quiet_NaN();
        cur_vel_theta_ = std::numeric_limits<double>::quiet_NaN();
    }
} // namespace multibot2_util