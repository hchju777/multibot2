#pragma once

#include <map>

#include <Eigen/Core>

#include <geometry_msgs/msg/twist.hpp>

#include "multibot2_util/pose.h"

namespace multibot2_util
{
    class BaseRobotInfo
    {
    public:
        BaseRobotInfo() {}

        BaseRobotInfo(const BaseRobotInfo &_baseRobotInfo);

    public:
        inline std::string &name() { return name_; }
        inline const std::string &name() const { return name_; }

        inline std::string &type() { return type_; }
        inline const std::string &type() const { return type_; }

        inline Pose &pose() { return pose_; }
        inline const Pose &pose() const { return pose_; }

        inline Pose &goal() { return goal_; }
        inline const Pose &goal() const { return goal_; }

        inline double &radius() { return radius_; }
        inline const double &radius() const { return radius_; }

        inline double &max_vel_x() { return max_vel_x_; }
        inline const double &max_vel_x() const { return max_vel_x_; }

        inline double &max_vel_theta() { return max_vel_theta_; }
        inline const double &max_vel_theta() const { return max_vel_theta_; }

        inline double &acc_lim_x() { return acc_lim_x_; }
        inline const double &acc_lim_x() const { return acc_lim_x_; }

        inline double &acc_lim_theta() { return acc_lim_theta_; }
        inline const double &acc_lim_theta() const { return acc_lim_theta_; }

        inline double &cur_vel_x() { return cur_vel_x_; }
        inline const double &cur_vel_x() const { return cur_vel_x_; }

        inline double &cur_vel_theta() { return cur_vel_theta_; }
        inline const double &cur_vel_theta() const { return cur_vel_theta_; }

        inline Eigen::Vector2d &velocity()
        {
            Eigen::Vector2d velocity(cur_vel_x_ * std::cos(pose_.theta()), cur_vel_x_ * std::sin(pose_.theta()));
            return velocity;
        }

        inline const Eigen::Vector2d &velocity() const
        {
            Eigen::Vector2d velocity(cur_vel_x_ * std::cos(pose_.theta()), cur_vel_x_ * std::sin(pose_.theta()));
            return velocity;
        }

    public:
        BaseRobotInfo &operator=(const BaseRobotInfo &_rhs);

    public:
        void initBaseRobotInfo();

    public:
        friend std::ostream &operator<<(std::ostream &_os, const BaseRobotInfo &_baseRobotInfo)
        {
            _os << "[" << _baseRobotInfo.name_ << "]"
                << "\t Radius: " << _baseRobotInfo.radius_
                << "\t Pose: " << _baseRobotInfo.pose_
                << "\t Goal: " << _baseRobotInfo.goal_;

            return _os;
        }

    protected:
        std::string name_{std::string("robot")};
        std::string type_{std::string("DiffDrive")};

        Pose pose_;
        Pose goal_;

        double radius_{std::numeric_limits<double>::quiet_NaN()};

        double max_vel_x_{std::numeric_limits<double>::quiet_NaN()};
        double max_vel_theta_{std::numeric_limits<double>::quiet_NaN()};
        double acc_lim_x_{std::numeric_limits<double>::quiet_NaN()};
        double acc_lim_theta_{std::numeric_limits<double>::quiet_NaN()};

        double cur_vel_x_{std::numeric_limits<double>::quiet_NaN()};
        double cur_vel_theta_{std::numeric_limits<double>::quiet_NaN()};

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    }; // class BaseRobotInfo
} // namespace multibot2_util