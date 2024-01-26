#pragma once

#include <memory>

#include <nav2_util/lifecycle_node.hpp>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include "multibot2_util/base_robot_info.h"
#include "multibot2_util/panel_util.h"

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include "multibot2_msgs/msg/robot_state.hpp"

namespace multibot2_robot
{
    typedef multibot2_util::BaseRobotInfo Robot;

    struct Robot_ROS
    {
    public:
        typedef multibot2_util::Pose Pose;

        typedef multibot2_msgs::msg::RobotState RobotState;
        typedef multibot2_util::PanelUtil::ModeSelection ModeSelection;
        typedef multibot2_util::PanelUtil::Mode Mode;

    public:
        Robot_ROS() {}

        Robot_ROS(const Robot &_robot) : robot_(_robot) {}

        Robot_ROS(const Robot_ROS &_robot_ros);

        ~Robot_ROS() {}

    public:
        Robot_ROS &operator=(const Robot_ROS &_rhs);

    public:
        inline Robot &robot() { return robot_; }
        inline const Robot &robot() const { return robot_; }

        inline Pose &subgoal() { return subgoal_; }
        inline const Pose &subgoal() const { return subgoal_; }

        inline Mode &mode() { return mode_; }
        inline const Mode &mode() const { return mode_; }

        inline rclcpp_lifecycle::LifecyclePublisher<RobotState>::SharedPtr &state_pub() { return state_pub_; }
        inline const rclcpp_lifecycle::LifecyclePublisher<RobotState>::SharedPtr &state_pub() const { return state_pub_; }

        inline rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub() { return cmd_vel_pub_; }
        inline const rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr &cmd_vel_pub() const { return cmd_vel_pub_; }

        inline rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr &odom_sub() { return odom_sub_; }
        inline const rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr &odom_sub() const { return odom_sub_; }

        inline rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr &goal_sub() { return goal_sub_; }
        inline const rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr &goal_sub() const { return goal_sub_; }

        inline rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr &subgoal_sub() { return subgoal_sub_; }
        inline const rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr &subgoal_sub() const { return subgoal_sub_; }

        inline rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr &rviz_path_pub() { return rviz_path_pub_; }
        inline const rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr &rviz_path_pub() const { return rviz_path_pub_; }

    protected:
        Robot robot_;
        Pose subgoal_;

        Mode mode_{Mode::MANUAL};

    protected:
        rclcpp_lifecycle::LifecyclePublisher<RobotState>::SharedPtr state_pub_;

        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subgoal_sub_;

        rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr rviz_path_pub_;
    }; // struct Robot_ROS

    class Instance_Manager
    {
    public:
        typedef std::unique_ptr<Instance_Manager> UniquePtr;
        typedef std::shared_ptr<Instance_Manager> SharedPtr;

    public:
        Instance_Manager() {}

        Instance_Manager(nav2_util::LifecycleNode::SharedPtr &_nh);

        ~Instance_Manager();

    public:
        inline Robot &robot() { return robot_ros_.robot(); }
        inline const Robot &robot() const { return robot_ros_.robot(); }

        inline Robot_ROS &robot_ros() { return robot_ros_; }
        inline const Robot_ROS &robot_ros() const { return robot_ros_; }

        inline std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &global_costmap_ros() { return global_costmap_ros_; }
        inline const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &global_costmap_ros() const { return global_costmap_ros_; }

        inline std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &local_costmap_ros() { return local_costmap_ros_; }
        inline const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &local_costmap_ros() const { return local_costmap_ros_; }

    public:
        void init_variables();
        void init_parameters();

    protected:
        void update_state()
        {
            update_pose();
            report_state();
        }

        void update_pose();

        void report_state();

    protected:
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg);

        void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr _goal_msg);

        void subgoal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr _subgoal_msg);

    protected:
        nav2_util::LifecycleNode::SharedPtr nh_;
        rclcpp::TimerBase::SharedPtr update_timer_;

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    protected:
        Robot_ROS robot_ros_;

    protected:
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> global_costmap_thread_;

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> local_costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> local_costmap_thread_;

    }; // class Instance_Manager
} // namespace multibot2_robot