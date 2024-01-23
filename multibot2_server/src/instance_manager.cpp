#include "multibot2_server/instance_manager.h"

namespace multibot2_server
{
    Robot_ROS::Robot_ROS(const Robot_ROS &_robot_ros)
    {
        robot_ = _robot_ros.robot_;

        id_ = _robot_ros.id_;
        mode_ = _robot_ros.mode_;

        last_update_time_ = _robot_ros.last_update_time_;
        prior_update_time_ = _robot_ros.prior_update_time_;

        state_sub_ = _robot_ros.state_sub_;

        cmd_vel_pub_ = _robot_ros.cmd_vel_pub_;

        modeFromServer_ = _robot_ros.modeFromServer_;
        modeFromRobot_ = _robot_ros.modeFromRobot_;
    }

    Robot_ROS &Robot_ROS::operator=(const Robot_ROS &_rhs)
    {
        if (&_rhs != this)
        {
            robot_ = _rhs.robot_;

            id_ = _rhs.id_;
            mode_ = _rhs.mode_;

            last_update_time_ = _rhs.last_update_time_;
            prior_update_time_ = _rhs.prior_update_time_;

            state_sub_ = _rhs.state_sub_;

            cmd_vel_pub_ = _rhs.cmd_vel_pub_;

            modeFromServer_ = _rhs.modeFromServer_;
            modeFromRobot_ = _rhs.modeFromRobot_;
        }

        return *this;
    }

    Instance_Manager::Instance_Manager(nav2_util::LifecycleNode::SharedPtr &_nh)
        : nh_(_nh)
    {
        init_variables();

        RCLCPP_INFO(nh_->get_logger(), "Instance Manager has been initialized");
    }

    Instance_Manager::Instance_Manager(const Instance_Manager &_instance_manager)
    {
        nh_ = _instance_manager.nh_;

        robots_ = _instance_manager.robots_;
    }

    Instance_Manager::~Instance_Manager()
    {
        RCLCPP_INFO(nh_->get_logger(), "Instance Manager has been terminated");
    }

    Instance_Manager &Instance_Manager::operator=(const Instance_Manager &_rhs)
    {
        if (&_rhs != this)
        {
            nh_ = _rhs.nh_;

            robots_ = _rhs.robots_;
        }

        return *this;
    }

    void Instance_Manager::init_variables()
    {
        qos_ = rclcpp::QoS(rclcpp::KeepLast(10));

        std::map<std::string, Robot_ROS> empty_robots;
        robots_.swap(empty_robots);
    }

    void Instance_Manager::emplaceRobot(Robot_ROS &_robot_ros)
    {
        if (robots_.contains(_robot_ros.robot().name()))
            return;

        static int32_t robotID = 0;

        const std::string &robotName = _robot_ros.robot().name();

        _robot_ros.id() = ++robotID;

        _robot_ros.state_sub() = nh_->create_subscription<Robot_ROS::RobotState>(
            "/" + robotName + "/state", qos_,
            std::bind(&Instance_Manager::robotState_callback, this, std::placeholders::_1));

        robots_.emplace(robotName, _robot_ros);

        RCLCPP_INFO(nh_->get_logger(), "Instance_Manager::insertRobot()");
        RCLCPP_INFO(nh_->get_logger(), "New Robot " + robotName + " is registered.");
        RCLCPP_INFO(nh_->get_logger(), "Total Robots: " + std::to_string(robots_.size()) + "EA");
        std::cout << static_cast<const multibot2_util::BaseRobotInfo &>(robots_[robotName].robot()) << std::endl;
    }

    void Instance_Manager::deleteRobot(const std::string _robotName)
    {
        if (robots_.contains(_robotName))
        {
            robots_.erase(_robotName);

            RCLCPP_INFO(nh_->get_logger(), "Robot " + _robotName + " is deleted.");
            RCLCPP_INFO(nh_->get_logger(), "Total Robots: " + std::to_string(robots_.size()) + "EA\n");
        }
    }

    void Instance_Manager::robotState_callback(const Robot_ROS::RobotState::SharedPtr _state_msg)
    {
        Robot_ROS &robot_ros = robots_[_state_msg->name];

        robot_ros.prior_update_time() = robot_ros.last_update_time();
        robot_ros.last_update_time() = nh_->now();

        robot_ros.robot().pose().x() = _state_msg->pose.x;
        robot_ros.robot().pose().y() = _state_msg->pose.y;
        robot_ros.robot().pose().theta() = _state_msg->pose.theta;

        robot_ros.robot().cur_vel_x() = _state_msg->lin_vel;
        robot_ros.robot().cur_vel_theta() = _state_msg->ang_vel;
    }

} // namespace multibot2_server