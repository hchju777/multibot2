#pragma once

#include <memory>
#include <map>
#include <queue>

#include <CGAL/Exact_predicates_exact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/create_offset_polygons_2.h>

#include "multibot2_util/base_robot_info.h"

typedef CGAL::Exact_predicates_exact_constructions_kernel Kernel;

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

        inline std::set<std::string> &higher_neighbors() { return higher_neighbors_; }
        inline const std::set<std::string> &higher_neighbors() const { return higher_neighbors_; }

        inline std::vector<Cone> &VOCones() { return VOCones_; }
        inline const std::vector<Cone> &VOCones() const { return VOCones_; }

        inline std::string &front() { return front_; }
        inline const std::string &front() const { return front_; }

        inline std::chrono::system_clock::time_point &replan_time() { return replan_time_; }
        inline const std::chrono::system_clock::time_point &replan_time() const { return replan_time_; }

        inline std::queue<Task> &task_queue() { return task_queue_; }
        inline const std::queue<Task> &task_queue() const { return task_queue_; }

        inline std::queue<Task> &goal_queue() { return goal_queue_; }
        inline const std::queue<Task> &goal_queue() const { return goal_queue_; }

        inline std::vector<CGAL::Polygon_2<Kernel>> local_obstacles() {return local_obstacles_;}
        inline const std::vector<CGAL::Polygon_2<Kernel>> local_obstacles() const {return local_obstacles_;}

    public:
        Robot &operator=(const Robot &_rhs);

    public:
        void init();

        void set_local_obstacles(const std::vector<geometry_msgs::msg::Polygon> &_local_obstacles);

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
        std::set<std::string> higher_neighbors_;
        std::vector<Cone> VOCones_;

        std::string front_;
        std::chrono::system_clock::time_point replan_time_;

        std::queue<Task> task_queue_;
        std::queue<Task> goal_queue_;

        std::vector<CGAL::Polygon_2<Kernel>> local_obstacles_;

    }; // class Robot

    typedef std::map<std::string, Robot> Robots;

} // namespace multibot2_server