#pragma once

#include <memory>
#include <map>

#include <nav2_util/lifecycle_node.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include "costmap_converter/costmap_to_polygons.h"

#include "multibot2_server/robot.h"

#include "multibot2_util/panel_util.h"

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include "multibot2_msgs/msg/robot_state.hpp"

using namespace multibot2_util;

namespace multibot2_server
{
    struct Robot_ROS
    {
        typedef multibot2_msgs::msg::RobotState State;

        Robot robot_;
        int32_t id_;
        PanelUtil::Mode mode_;

        rclcpp::Time last_update_time_;
        rclcpp::Time prior_update_time_;

        rclcpp::Subscription<State>::SharedPtr state_sub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr kill_robot_cmd_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
        rclcpp::Client<PanelUtil::ModeSelection>::SharedPtr modeFromServer_;
        rclcpp::Service<PanelUtil::ModeSelection>::SharedPtr modeFromRobot_;
    }; // struct Robot

    class Instance_Manager
    {
    public:
        typedef std::unique_ptr<Instance_Manager> UniquePtr;
        typedef std::shared_ptr<Instance_Manager> SharedPtr;

    public:
        std::map<std::string, Robot_ROS> &robots() { return robots_; }
        const std::map<std::string, Robot_ROS> &robots() const { return robots_; }

        costmap_converter::PolygonContainerConstPtr &static_polygons() { return static_polygons_; }
        const costmap_converter::PolygonContainerConstPtr &static_polygons() const { return static_polygons_; }

    protected:
        void robotState_callback(const Robot_ROS::State::SharedPtr _state_msg);

    protected:
        nav2_util::LifecycleNode::SharedPtr nh_;
        rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));

    public:
        void insertRobot(const Robot_ROS &_robot);
        void deleteRobot(const std::string _robotName);

        const Robot_ROS &getRobot(const std::string _robotName) const;

        void setGoal(const std::string _robotName, const geometry_msgs::msg::Pose2D _goal);
        void setMode(const std::string _robotName, const PanelUtil::Mode _mode);
        void request_modeChange(const std::string _robotName, const PanelUtil::Mode _mode);
        void request_kill(const std::string _robotName);
        void remote_control(const std::string _robotName, const geometry_msgs::msg::Twist &_remote_cmd_vel);
        void initialpose_pub(const std::string _robotName, const geometry_msgs::msg::PoseWithCovarianceStamped &_initialpose_msg);
        void goal_pose_pub(const std::string _robotName, const geometry_msgs::msg::PoseStamped &_goal_msg);

    protected:
        std::map<std::string, Robot_ROS> robots_;

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> global_costmap_thread_;

        costmap_converter::PolygonContainerConstPtr static_polygons_;

    public:
        Instance_Manager(nav2_util::LifecycleNode::SharedPtr &_nh);
        ~Instance_Manager() {}
    }; // class Instance_Manager
} // namespace multibot2_server