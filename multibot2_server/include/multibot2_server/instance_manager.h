#pragma once

#include <memory>
#include <map>

#include <filesystem>
#include <yaml-cpp/yaml.h>

#include <nav2_util/lifecycle_node.hpp>

#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/cost_values.hpp>

#include "costmap_converter/costmap_to_polygons.h"

#include "multibot2_server/robot.h"
#include "multibot2_server/global_planner/navfn_planner.hpp"

#include "multibot2_util/panel_util.h"

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include "multibot2_msgs/msg/robot_state.hpp"
#include "multibot2_msgs/msg/task.hpp"
#include "multibot2_msgs/msg/neighbors.hpp"
#include "multibot2_msgs/msg/robot_with_trajectory.hpp"
#include "multibot2_msgs/msg/robot_with_trajectory_array.hpp"
#include "multibot2_msgs/srv/queue_rivision.hpp"

using namespace multibot2_util;

namespace multibot2_server
{
    struct Robot_ROS
    {
        typedef multibot2_msgs::msg::RobotState State;
        typedef multibot2_msgs::msg::Task Task;
        typedef multibot2_msgs::msg::Neighbor Neighbor;
        typedef multibot2_msgs::msg::Neighbors Neighbors;
        typedef multibot2_msgs::msg::RobotWithTrajectory RobotWithTrajectory;
        typedef multibot2_msgs::msg::RobotWithTrajectoryArray RobotWithTrajectoryArray;
        typedef multibot2_msgs::srv::QueueRivision QueueRivision;

        Robot robot_;
        int32_t id_;
        PanelUtil::Mode mode_;

        rclcpp::Time last_update_time_;
        rclcpp::Time prior_update_time_;

        RobotWithTrajectory local_trajectory_;

        rclcpp::Subscription<State>::SharedPtr state_sub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr kill_robot_cmd_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
        rclcpp_lifecycle::LifecyclePublisher<Task>::SharedPtr task_pub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr subgoal_pose_pub_;
        rclcpp_lifecycle::LifecyclePublisher<Neighbors>::SharedPtr neighbors_pub_;
        rclcpp::Client<PanelUtil::ModeSelection>::SharedPtr modeFromServer_;
        rclcpp::Service<PanelUtil::ModeSelection>::SharedPtr modeFromRobot_;
        rclcpp::Subscription<RobotWithTrajectory>::SharedPtr local_trajectory_sub_;
        rclcpp_lifecycle::LifecyclePublisher<RobotWithTrajectoryArray>::SharedPtr dynamic_obstacles_pub_;
        rclcpp::Service<QueueRivision>::SharedPtr queue_revision_;
    }; // struct Robot_ROS

    class Instance_Manager
    {
    public:
        typedef std::unique_ptr<Instance_Manager> UniquePtr;
        typedef std::shared_ptr<Instance_Manager> SharedPtr;

    public:
        Instance_Manager(nav2_util::LifecycleNode::SharedPtr &_nh);

        ~Instance_Manager() {}

    public:
        std::map<std::string, Robot_ROS> &robots() { return robots_; }
        const std::map<std::string, Robot_ROS> &robots() const { return robots_; }

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &global_costmap_ros() { return global_costmap_ros_; }
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &global_costmap_ros() const { return global_costmap_ros_; }

        costmap_converter::PolygonContainerConstPtr &static_obstacles() { return static_obstacles_; }
        const costmap_converter::PolygonContainerConstPtr &static_obstacles() const { return static_obstacles_; }

        double &subgoal_generator_duration() { return subgoal_generator_duration_; }
        const double &subgoal_generator_duration() const { return subgoal_generator_duration_; }

    public:
        void init_variables();

        void init_parameters();

        void init_global_planner();

        bool load_tasks();

    public:
        void insertRobot(const Robot_ROS &_robot);

        void deleteRobot(const std::string _robotName);

        const Robot_ROS &getRobot(const std::string _robotName) const;

        void setGoal(const std::string _robotName, const geometry_msgs::msg::Pose2D _goal);

        void setMode(const std::string _robotName, const PanelUtil::Mode _mode);

        void setFront(const std::string _robotName, const std::string _front) { robots_[_robotName].robot_.front() = _front; }

        void setReplanTime(const std::string _robotName, const std::chrono::system_clock::time_point _replan_time) { robots_[_robotName].robot_.replan_time() = _replan_time; }

        void setSubgoal(const std::string _robotName, const Pose &_subgoal) { robots_[_robotName].robot_.subgoal() = _subgoal; }

        void request_modeChange(const std::string _robotName, const PanelUtil::Mode _mode);

        void request_kill(const std::string _robotName);

        void remote_control(const std::string _robotName, const geometry_msgs::msg::Twist &_remote_cmd_vel);

        void initialpose_pub(const std::string _robotName, const geometry_msgs::msg::PoseWithCovarianceStamped &_initialpose_msg);

        void goal_pose_pub(const std::string _robotName, const geometry_msgs::msg::PoseStamped &_goal_msg);

        void subgoal_pose_pub(const std::string _robotName, const geometry_msgs::msg::PoseStamped &_subgoal_msg);

        void queue_revision(const std::shared_ptr<Robot_ROS::QueueRivision::Request> _request,
                            std::shared_ptr<Robot_ROS::QueueRivision::Response> _response);

    protected:
        void convert_map_to_polygons();

        void robotState_callback(const Robot_ROS::State::SharedPtr _state_msg);

        void local_trajectory_callback(const Robot_ROS::RobotWithTrajectory::SharedPtr _trajectory_msg);

        void update()
        {
            update_goals();
            update_neighbors();
        }

        void update_goals();

        void update_neighbors();

    protected:
        nav2_util::LifecycleNode::SharedPtr nh_;
        rclcpp::TimerBase::SharedPtr update_timer_;

    protected:
        std::map<std::string, Robot_ROS> robots_;

        std::map<std::string, std::queue<Robot::Task>> tasks_;
        std::map<std::string, bool> task_loops_;

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> global_costmap_ros_;
        std::unique_ptr<nav2_util::NodeThread> global_costmap_thread_;

        costmap_converter::PolygonContainerConstPtr static_obstacles_;

        std::shared_ptr<nav2_navfn_planner::NavfnPlanner> navfn_global_planner_;
        
        double communication_range_{4.0};
        double lookahead_dist_{5.0};
        double subgoal_generator_duration_{0.05};

    }; // class Instance_Manager
} // namespace multibot2_server