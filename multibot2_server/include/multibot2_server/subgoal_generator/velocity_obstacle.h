#pragma once

#include <memory>
#include <map>

#include "multibot2_server/robot.h"
#include "multibot2_server/subgoal_generator/subgoalgen_config.h"

namespace multibot2_server::SubgoalGenerator
{
    class VelocityObstacle
    {
    public:
        typedef std::unique_ptr<VelocityObstacle> UniquePtr;
        typedef std::shared_ptr<VelocityObstacle> SharedPtr;

    public:
        VelocityObstacle() {}

        VelocityObstacle(const Config::SharedPtr &_cfg) : cfg_(_cfg) {}

        VelocityObstacle(const VelocityObstacle &_velocity_obstacle)
        {
            cfg_ = _velocity_obstacle.cfg_;
            robots_ = _velocity_obstacle.robots_;
        }

        ~VelocityObstacle() { clearRobots(); }

    public:
        bool updateVOCones(std::string _name);

    public:
        Robots &robots() { return robots_; }
        const Robots &robots() const { return robots_; }

    public:
        VelocityObstacle &operator=(const VelocityObstacle &_rhs)
        {
            if (&_rhs != this)
            {
                cfg_ = _rhs.cfg_;
                robots_ = _rhs.robots_;
            }

            return *this;
        }

    public:
        inline void emplaceRobot(const Robot &_robot) { robots_.emplace(_robot.name(), _robot); }

    protected:
        void clearRobots();

    protected:
        Config::SharedPtr cfg_;

        Robots robots_;
    }; // class VelocityObstacle
} // namespace multibot2_server::SubgoalGenerator