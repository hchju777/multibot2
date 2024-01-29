#pragma once

#include <iostream>

#include "multibot2_util/pose.h"

namespace multibot2_server::SubgoalGenerator
{
    class Vertex
    {
    public:
        typedef multibot2_util::Pose Pose;

    public:
        Vertex() {}

        Vertex(std::string _name, const Pose &_current, const Pose &_goal)
            : name_(_name), current_(_current), goal_(_goal) {}

        Vertex(const Vertex &_vertex)
        {
            name_ = _vertex.name_;
            current_ = _vertex.current_;
            goal_ = _vertex.goal_;
        }

        ~Vertex() {}

    public:
        inline std::string &name() { return name_; }
        inline const std::string &name() const { return name_; }

        inline Pose &current() { return current_; }
        inline const Pose &current() const { return current_; }

        inline Pose &goal() { return goal_; }
        inline const Pose &goal() const { return goal_; }

    public:
        inline static double getDistance(const Vertex &_lhs, const Vertex &_rhs)
        {
            Eigen::Vector2d v = _lhs.current().position() - _rhs.current().position();

            return v.norm();
        }

    public:
        Vertex &operator=(const Vertex &_rhs)
        {
            if (&_rhs != this)
            {
                name_ = _rhs.name_;
                current_ = _rhs.current_;
                goal_ = _rhs.goal_;
            }

            return *this;
        }

        friend std::ostream &operator<<(std::ostream &_os, const Vertex &_vertex)
        {
            _os << "[" << _vertex.name_ << "]"
                << "\t Current Pose: " << _vertex.current_
                << "\t Goal Pose: " << _vertex.goal_;

            return _os;
        }

    public:
        inline bool isGoal() const { return goalDistance() < 1e-8; }

        inline double goalDistance() const { return (current_.position() - goal_.position()).norm(); }

    protected:
        std::string name_;
        Pose current_;
        Pose goal_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    }; // class Vertex

    typedef std::map<std::string, Vertex> Vertices;

} // multibot2_server::SubgoalGenerator