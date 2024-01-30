#pragma once

#include <memory>
#include <map>

#include "multibot2_server/robot.h"

namespace multibot2_server::SubgoalGenerator
{
    class VelocityObstacle
    {
    public:
        typedef std::unique_ptr<VelocityObstacle> UniquePtr;
        typedef std::shared_ptr<VelocityObstacle> SharedPtr;

    public:
        VelocityObstacle() {}

        VelocityObstacle(const VelocityObstacle &_velocity_obstacle)
        {
            robots_ = _velocity_obstacle.robots_;
            timeHorizon_ = _velocity_obstacle.timeHorizon_;
        }

        ~VelocityObstacle() { clearRobots(); }

    public:
        bool updateVOCones(std::string _name);

    public:
        std::map<std::string, Robot *> &robots() { return robots_; }
        const std::map<std::string, Robot *> &robots() const { return robots_; }

    public:
        VelocityObstacle &operator=(const VelocityObstacle &_rhs)
        {
            if (&_rhs != this)
            {
                robots_ = _rhs.robots_;
                timeHorizon_ = _rhs.timeHorizon_;
            }

            return *this;
        }

    public:
        inline void emplaceRobot(Robot *_robot) { robots_.emplace(_robot->name(), _robot); }

    protected:
        void clearRobots();

    protected:
        std::map<std::string, Robot *> robots_;

        double timeHorizon_{0.05};

    }; // class VelocityObstacle
} // namespace multibot2_server::SubgoalGenerator