#pragma once

#include <Eigen/Core>

#include <tf2/utils.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>

namespace multibot2_util
{
    class Pose
    {
    public:
        Pose() { setZero(); }

        Pose(const Eigen::Ref<const Eigen::Vector2d> &_position, double _theta)
        {
            position_ = _position;
            theta_ = _theta;
        }

        Pose(const Pose &_pose)
        {
            position_ = _pose.position_;
            theta_ = _pose.theta_;
        }

        Pose(double _x, double _y, double _theta)
        {
            position_.coeffRef(0) = _x;
            position_.coeffRef(1) = _y;
            theta_ = _theta;
        }

        Pose(const geometry_msgs::msg::PoseStamped &_pose) : Pose(_pose.pose) {}

        Pose(const geometry_msgs::msg::Pose &_pose)
        {
            position_.coeffRef(0) = _pose.position.x;
            position_.coeffRef(1) = _pose.position.y;
            theta_ = tf2::getYaw(_pose.orientation);
        }

        Pose(const geometry_msgs::msg::Pose2D &_pose2d)
        {
            position_.coeffRef(0) = _pose2d.x;
            position_.coeffRef(1) = _pose2d.y;
            theta_ = _pose2d.theta;
        }

        ~Pose() {}

    public:
        Eigen::Vector2d &position() { return position_; }
        const Eigen::Vector2d &position() const { return position_; }

        double &x() { return position_.coeffRef(0); }
        const double &x() const { return position_.coeffRef(0); }

        double &y() { return position_.coeffRef(1); }
        const double &y() const { return position_.coeffRef(1); }

        double &theta() { return theta_; }
        const double &theta() const { return theta_; }

        void setZero()
        {
            position_.setZero();
            theta_ = 0.0;
        }

        void toPoseMsg(geometry_msgs::msg::Pose &_pose) const
        {
            _pose.position.x = position_.x();
            _pose.position.y = position_.y();
            _pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, theta_);
            _pose.orientation = tf2::toMsg(q);
        }

        void toPoseMsg(geometry_msgs::msg::Pose2D &_pose) const
        {
            _pose.x = position_.x();
            _pose.y = position_.y();
            _pose.theta = theta_;
        }

        Eigen::Vector2d orientationUnitVec() const { return Eigen::Vector2d(std::cos(theta_), std::sin(theta_)); }

public:
    void scale(double _factor)
    {
        position_ *= _factor;
        theta_ = normalize_theta(theta_ * _factor);
    }

    void plus(const double *_pose_as_array)
    {
        position_.coeffRef(0) += _pose_as_array[0];
        position_.coeffRef(1) += _pose_as_array[1];
        theta_ = normalize_theta(theta_ + _pose_as_array[2]);
    }

    void averageInPlace(const Pose &_pose1, const Pose &_pose2)
    {
        position_ = (_pose1.position_ + _pose2.position_) / 2;
        theta_ = average_angle(_pose1.theta_, _pose2.theta_);
    }

    static Pose average(const Pose &_pose1, const Pose &_pose2)
    {
        return Pose((_pose1.position_ + _pose2.position_) / 2, average_angle(_pose1.theta_, _pose2.theta_));
    }

    void rotateGlobal(double _angle, bool _adjust_theta = true)
    {
        double new_x = std::cos(_angle) * position_.x() - std::sin(_angle) * position_.y();
        double new_y = std::sin(_angle) * position_.y() + std::cos(_angle) * position_.y();

        position_.x() = new_x;
        position_.y() = new_y;
        if (_adjust_theta)
            theta_ = normalize_theta(theta_ + _angle);
    }

public:
    Pose &operator=(const Pose &_rhs)
    {
        if (&_rhs != this)
        {
            position_ = _rhs.position_;
            theta_ = _rhs.theta_;
        }

        return *this;
    }

    Pose &operator+=(const Pose &_rhs)
    {
        position_ += _rhs.position_;
        theta_ = normalize_theta(theta_ + _rhs.theta_);

        return *this;
    }

    friend Pose operator+(Pose _lhs, const Pose &_rhs)
    {
        return _lhs += _rhs;
    }

    Pose &operator-=(const Pose &_rhs)
    {
        position_ -= _rhs.position_;
        theta_ = normalize_theta(theta_ - _rhs.theta_);

        return *this;
    }

    friend Pose operator-(Pose _lhs, const Pose &_rhs)
    {
        return _lhs -= _rhs;
    }

    friend Pose operator*(Pose _pose, double _scalar)
    {
        _pose.position_ *= _scalar;
        _pose.theta_ *= _scalar;
        return _pose;
    }

    friend Pose operator*(double _scalar, Pose _pose)
    {
        _pose.position_ *= _scalar;
        _pose.theta_ *= _scalar;
        return _pose;
    }

    friend std::ostream &operator<<(std::ostream &_stream, const Pose &_pose)
    {
        _stream << "x: " << _pose.position_[0] << " y: " << _pose.position_[1] << " theta: " << _pose.theta_;
        return _stream;
    }

public:
    static inline double normalize_theta(double _theta)
    {
        const double result = std::fmod(_theta + M_PI, 2.0 * M_PI);
        if (result <= 0.0)
            return (result + M_PI);
        return (result - M_PI);
    }

    static inline double average_angle(double _theta1, double _theta2)
    {
        double x = std::cos(_theta1) + std::cos(_theta2);
        double y = std::sin(_theta1) + std::sin(_theta2);

        if (x == 0 and y == 0)
            return 0.0;
        else
            return std::atan2(y, x);
    }

    template <typename V1, typename V2>
    static inline double cross2d(const V1 &_pose1, const V2 &_pose2)
    {
        return (_pose1.x() * _pose2.y() - _pose2.x() * _pose1.y());
    }

    protected:
        Eigen::Vector2d position_;
        double theta_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    }; // class Pose

} // namespace multibot2_util