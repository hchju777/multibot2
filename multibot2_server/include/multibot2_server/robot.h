#pragma once

#include <memory>
#include <map>
#include <queue>

#include "multibot2_util/base_robot_info.h"

namespace multibot2_server
{
    class Robot : public multibot2_util::BaseRobotInfo
    {
    public:
        typedef std::unique_ptr<Robot> UniquePtr;
        typedef std::shared_ptr<Robot> SharedPtr;

    public:
        typedef std::map<double, Robot, std::greater<double>> Neighbors;

        struct Task
        {
        public:
            Task() {}

            Task(const multibot2_util::Pose &_loc, double _duration) : loc_(_loc), duration_(_duration) {}

            Task(const Task &_task)
            {
                loc_ = _task.loc_;
                duration_ = _task.duration_;
            }

            Task &operator=(const Task &_rhs)
            {
                if (&_rhs != this)
                {
                    loc_ = _rhs.loc_;
                    duration_ = _rhs.duration_;
                }

                return *this;
            }

            inline multibot2_util::Pose &loc() { return loc_; }
            inline const multibot2_util::Pose &loc() const { return loc_; }

            inline double &duration() { return duration_; }
            inline const double &duration() const { return duration_; }

        protected:
            multibot2_util::Pose loc_;
            double duration_;
        }; // struct Task

        struct Cone
        {
            Cone() {}

            Cone(const Cone &_cone)
            {
                neighbor_ = _cone.neighbor_;
                point_ = _cone.point_;
                radius_ = _cone.radius_;
                left_direction_ = _cone.left_direction_;
                right_direction_ = _cone.right_direction_;
            }

            Cone &operator=(const Cone &_rhs)
            {
                if (&_rhs != this)
                {
                    neighbor_ = _rhs.neighbor_;
                    point_ = _rhs.point_;
                    radius_ = _rhs.radius_;
                    left_direction_ = _rhs.left_direction_;
                    right_direction_ = _rhs.right_direction_;
                }

                return *this;
            }

            std::string neighbor_;
            Eigen::Vector2d point_;
            double radius_;
            Eigen::Vector2d left_direction_;
            Eigen::Vector2d right_direction_;
        }; // struct Cone

    public:
        Robot(){};

        Robot(const Robot &_robot);

        ~Robot(){};

    public:
        inline multibot2_util::Pose &subgoal() { return subgoal_; }
        inline const multibot2_util::Pose &subgoal() const { return subgoal_; }

        inline Neighbors &neighbors() { return neighbors_; }
        inline const Neighbors &neighbors() const { return neighbors_; }

        inline std::vector<Cone> &VOCones() { return VOCones_; }
        inline const std::vector<Cone> &VOCones() const { return VOCones_; }

        inline std::queue<Task> &task_queue() { return task_queue_; }
        inline const std::queue<Task> &task_queue() const { return task_queue_; }

        inline std::queue<Task> &goal_queue() { return goal_queue_; }
        inline const std::queue<Task> &goal_queue() const { return goal_queue_; }

    public:
        Robot &operator=(const Robot &_rhs);

    public:
        void init();

    public:
        friend std::ostream &operator<<(std::ostream &_os, const Robot &_robot)
        {
            _os << static_cast<const multibot2_util::BaseRobotInfo &>(_robot)
                << "\t Subgoal: " << _robot.subgoal_;

            return _os;
        }

    protected:
        multibot2_util::Pose subgoal_;

        Neighbors neighbors_;
        std::vector<Cone> VOCones_;

        std::queue<Task> task_queue_;
        std::queue<Task> goal_queue_;

    }; // class Robot

    typedef std::map<std::string, Robot> Robots;

} // namespace multibot2_server