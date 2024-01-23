#include "multibot2_server/robot.h"

namespace multibot2_server
{
    Robot::Robot(const Robot &_robot)
    : multibot2_util::BaseRobotInfo(_robot)
    {
        subgoal_ = _robot.subgoal_;

        neighbors_ = _robot.neighbors_;
        VOCones_ = _robot.VOCones_;
    }

    Robot &Robot::operator=(const Robot &_rhs)
    {
        if (&_rhs != this)
        {
            multibot2_util::BaseRobotInfo::operator=(_rhs);

            subgoal_ = _rhs.subgoal_;

            neighbors_ = _rhs.neighbors_;
            VOCones_ = _rhs.VOCones_;
        }

        return *this;
    }

    void Robot::init()
    {
        initBaseRobotInfo();

        subgoal_.setZero();

        Neighbors empty_neighbors;
        std::vector<Cone> empty_VOCones;

        neighbors_.swap(empty_neighbors);
        VOCones_.swap(empty_VOCones);
    }
} // namespace multibot2_server