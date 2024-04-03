#include "multibot2_server/robot.h"

namespace multibot2_server
{
    Robot::Robot(const Robot &_robot)
        : multibot2_util::BaseRobotInfo(_robot)
    {
        subgoal_ = _robot.subgoal_;

        neighbors_ = _robot.neighbors_;
        higher_neighbors_ = _robot.higher_neighbors_;
        VOCones_ = _robot.VOCones_;

        front_ = _robot.front_;
        replan_time_ = _robot.replan_time_;

        task_queue_ = _robot.task_queue_;
        goal_queue_ = _robot.goal_queue_;

        local_obstacles_ = _robot.local_obstacles_;
    }

    Robot &Robot::operator=(const Robot &_rhs)
    {
        if (&_rhs != this)
        {
            multibot2_util::BaseRobotInfo::operator=(_rhs);

            subgoal_ = _rhs.subgoal_;

            neighbors_ = _rhs.neighbors_;
            higher_neighbors_ = _rhs.higher_neighbors_;
            VOCones_ = _rhs.VOCones_;

            front_ = _rhs.front_;
            replan_time_ = _rhs.replan_time_;

            task_queue_ = _rhs.task_queue_;
            goal_queue_ = _rhs.goal_queue_;

            local_obstacles_ = _rhs.local_obstacles_;
        }

        return *this;
    }

    void Robot::init()
    {
        initBaseRobotInfo();

        subgoal_.setZero();

        Neighbors empty_neighbors;
        std::set<std::string> empty_higher_neighbors;
        std::vector<Cone> empty_VOCones;

        neighbors_.swap(empty_neighbors);
        higher_neighbors_.swap(empty_higher_neighbors);
        VOCones_.swap(empty_VOCones);

        front_ = std::string();
        replan_time_ = std::chrono::system_clock::now();

        task_queue_ = std::queue<Task>();
        goal_queue_ = std::queue<Task>();

        std::vector<CGAL::Polygon_2<Kernel>> empty_obstacles;
        local_obstacles_.swap(empty_obstacles);
    }

    void Robot::set_local_obstacles(const std::vector<geometry_msgs::msg::Polygon> &_local_obstacles)
    {
        this->local_obstacles_.clear();

        for (const auto &obst : _local_obstacles)
        {
            CGAL::Polygon_2<Kernel> poly;
            for (const auto &p : obst.points)
                poly.push_back(Kernel::Point_2(p.x, p.y));

            this->local_obstacles_.push_back(poly);
        }
    }

} // namespace multibot2_server