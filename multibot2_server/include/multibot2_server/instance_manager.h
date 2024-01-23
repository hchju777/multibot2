#pragma once

#include <memory>
#include <map>

#include <nav2_util/lifecycle_node.hpp>

#include "multibot2_server/robot.h"

#include "multibot2_util/panel_util.h"

#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>

#include "multibot2_msgs/msg/robot_state.hpp"

namespace multibot2_server
{
    struct Robot_ROS
    {
    public:
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

        inline int32_t &id() { return id_; }
        inline const int32_t &id() const { return id_; }

        inline Mode &mode() { return mode_; }
        inline const Mode &mode() const { return mode_; }

        inline rclcpp::Time &last_update_time() { return last_update_time_; }
        inline const rclcpp::Time &last_update_time() const { return last_update_time_; }

        inline rclcpp::Time &prior_update_time() { return prior_update_time_; }
        inline const rclcpp::Time &prior_update_time() const { return prior_update_time_; }

        inline rclcpp::Subscription<RobotState>::SharedPtr &state_sub() { return state_sub_; }
        inline const rclcpp::Subscription<RobotState>::SharedPtr &state_sub() const { return state_sub_; }

        inline rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub() { return cmd_vel_pub_; }
        inline const rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub() const { return cmd_vel_pub_; }

        inline rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr kill_robot_cmd() { return kill_robot_cmd_; }
        inline const rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr kill_robot_cmd() const { return kill_robot_cmd_; }

        inline rclcpp::Client<ModeSelection>::SharedPtr modeFromServer() { return modeFromServer_; }
        inline const rclcpp::Client<ModeSelection>::SharedPtr modeFromServer() const { return modeFromServer_; }

        inline rclcpp::Service<ModeSelection>::SharedPtr modeFromRobot() { return modeFromRobot_; }
        inline const rclcpp::Service<ModeSelection>::SharedPtr modeFromRobot() const { return modeFromRobot_; }

    protected:
        Robot robot_;

        int32_t id_{std::numeric_limits<int32_t>::quiet_NaN()};
        Mode mode_{Mode::MANUAL};

        rclcpp::Time last_update_time_{rclcpp::Time(0)};
        rclcpp::Time prior_update_time_{rclcpp::Time(0)};

    protected:
        rclcpp::Subscription<RobotState>::SharedPtr state_sub_;

        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;

        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr kill_robot_cmd_;

        rclcpp::Client<ModeSelection>::SharedPtr modeFromServer_;
        rclcpp::Service<ModeSelection>::SharedPtr modeFromRobot_;

    }; // struct Robot_ROS

    class Instance_Manager
    {
    public:
        typedef std::unique_ptr<Instance_Manager> UniquePtr;
        typedef std::shared_ptr<Instance_Manager> SharedPtr;

    public:
        Instance_Manager(){};

        Instance_Manager(nav2_util::LifecycleNode::SharedPtr &_nh);

        Instance_Manager(const Instance_Manager &_instance_manager);

        ~Instance_Manager();

    public:
        inline std::map<std::string, Robot_ROS> robots() { return robots_; }
        inline const std::map<std::string, Robot_ROS> robots() const { return robots_; }

    public:
        Instance_Manager &operator=(const Instance_Manager &_rhs);

    public:
        void init_variables();

    public:
        void emplaceRobot(Robot_ROS &_robot_ros);

        void deleteRobot(const std::string _robotName);

    protected:
        void robotState_callback(const Robot_ROS::RobotState::SharedPtr _state_msg);

    protected:
        nav2_util::LifecycleNode::SharedPtr nh_;

        rclcpp::QoS qos_ = rclcpp::QoS(rclcpp::KeepLast(10));

    protected:
        std::map<std::string, Robot_ROS> robots_;

    }; // class Instance_Manager
} // namespace multibot2_server