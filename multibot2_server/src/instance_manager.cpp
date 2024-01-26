#include "multibot2_server/instance_manager.h"

using namespace std::chrono_literals;

namespace multibot2_server
{
    void Instance_Manager::robotState_callback(const Robot_ROS::State::SharedPtr _state_msg)
    {
        auto &robot_ros = robots_[_state_msg->name];

        robot_ros.prior_update_time_ = robot_ros.last_update_time_;
        robot_ros.last_update_time_ = nh_->now();

        robot_ros.robot_.pose().x() = _state_msg->pose.x;
        robot_ros.robot_.pose().y() = _state_msg->pose.y;
        robot_ros.robot_.pose().theta() = _state_msg->pose.theta;

        robot_ros.robot_.goal().x() = _state_msg->goal.x;
        robot_ros.robot_.goal().y() = _state_msg->goal.y;
        robot_ros.robot_.goal().theta() = _state_msg->goal.theta;

        robot_ros.robot_.cur_vel_x() = _state_msg->lin_vel;
        robot_ros.robot_.cur_vel_theta() = _state_msg->ang_vel;
    }

    void Instance_Manager::insertRobot(const Robot_ROS &_robot_ros)
    {
        if (robots_.contains(_robot_ros.robot_.name()))
        {
            return;
        }

        std::string robotName = _robot_ros.robot_.name();

        Robot_ROS robot_ros = _robot_ros;
        {
            robot_ros.state_sub_ = nh_->create_subscription<Robot_ROS::State>(
                "/" + robotName + "/state", qos_,
                std::bind(&Instance_Manager::robotState_callback, this, std::placeholders::_1));
            robot_ros.cmd_vel_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>(
                "/" + robotName + "/cmd_vel", qos_);
            robot_ros.cmd_vel_pub_->on_activate();
            robot_ros.kill_robot_cmd_ = nh_->create_publisher<std_msgs::msg::Bool>(
                "/" + robotName + "/kill", qos_);
            robot_ros.kill_robot_cmd_->on_activate();
            robot_ros.initialpose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/" + robotName + "/initialpose", qos_);
            robot_ros.initialpose_pub_->on_activate();
            robot_ros.goal_pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/" + robotName + "/goal_pose", qos_);
            robot_ros.goal_pose_pub_->on_activate();
        }
        robots_.insert(std::make_pair(robotName, robot_ros));

        RCLCPP_INFO(nh_->get_logger(), "Instance_Manager::insertRobot()");
        RCLCPP_INFO(nh_->get_logger(), "New Robot " + robotName + " is registered.");
        RCLCPP_INFO(nh_->get_logger(), "Total Robots: " + std::to_string(robots_.size()) + "EA");
        std::cout << robot_ros.robot_ << std::endl;
    }

    void Instance_Manager::deleteRobot(
        const std::string _robotName)
    {
        if (robots_.contains(_robotName))
        {
            robots_.erase(_robotName);

            RCLCPP_INFO(nh_->get_logger(), "Robot " + _robotName + " is deleted.");
            RCLCPP_INFO(nh_->get_logger(), "Total Robots: " + std::to_string(robots_.size()) + "EA\n");
        }
    }

    const Robot_ROS &Instance_Manager::getRobot(const std::string _robotName) const
    {
        const auto &robot_ros = robots_.find(_robotName)->second;

        return robot_ros;
    }

    void Instance_Manager::setGoal(
        const std::string _robotName, const geometry_msgs::msg::Pose2D _goal)
    {
        if (robots_.contains(_robotName))
        {
            robots_[_robotName].robot_.goal().x() = _goal.x;
            robots_[_robotName].robot_.goal().y() = _goal.y;
            robots_[_robotName].robot_.goal().theta() = _goal.theta;

            std::cout << "Changing the Goal of " << _robotName << ": "
                      << robots_[_robotName].robot_.goal() << std::endl;

            geometry_msgs::msg::PoseStamped goal_pose;
            {
                goal_pose.pose.position.x = _goal.x;
                goal_pose.pose.position.y = _goal.y;

                tf2::Quaternion q;
                q.setRPY(0, 0, _goal.theta);
                goal_pose.pose.orientation = tf2::toMsg(q);
            }

            robots_[_robotName].goal_pose_pub_->publish(goal_pose);
        }
    }

    void Instance_Manager::setMode(
        const std::string _robotName, const PanelUtil::Mode _mode)
    {
        if (not(robots_.contains(_robotName)))
            return;

        robots_[_robotName].mode_ = _mode;

        if (_mode == PanelUtil::Mode::REMOTE)
        {
            geometry_msgs::msg::Twist stop_cmd_vel;
            {
                stop_cmd_vel.linear.x = 0.0;
                stop_cmd_vel.angular.z = 0.0;
            }
            robots_[_robotName].cmd_vel_pub_->publish(stop_cmd_vel);
        }
    }

    void Instance_Manager::request_modeChange(
        const std::string _robotName, const PanelUtil::Mode _mode)
    {
        if (not(robots_.contains(_robotName)))
            return;

        auto &robot_ros = robots_[_robotName];

        while (!robot_ros.modeFromServer_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_ERROR(nh_->get_logger(), "ModeChange not available, waiting again...");
        }

        auto request = std::make_shared<PanelUtil::ModeSelection::Request>();

        request->name = _robotName;

        if (_mode == PanelUtil::Mode::REMOTE)
            request->is_remote = true;
        else if (_mode == PanelUtil::Mode::MANUAL)
            request->is_remote = false;
        else
        {
        }

        auto response_received_callback = [this](rclcpp::Client<PanelUtil::ModeSelection>::SharedFuture _future)
        {
            auto response = _future.get();
            return;
        };

        auto future_result =
            robot_ros.modeFromServer_->async_send_request(request, response_received_callback);

        if (future_result.get()->is_complete)
            robot_ros.mode_ = _mode;

        return;
    }

    void Instance_Manager::request_kill(const std::string _robotName)
    {
        if (not(robots_.contains(_robotName)))
            return;

        std_msgs::msg::Bool killActivated;
        killActivated.data = true;
        robots_[_robotName].kill_robot_cmd_->publish(killActivated);
    }

    void Instance_Manager::remote_control(
        const std::string _robotName, const geometry_msgs::msg::Twist &_remote_cmd_vel)
    {
        if (robots_.contains(_robotName))
            robots_[_robotName].cmd_vel_pub_->publish(_remote_cmd_vel);
    }

    void Instance_Manager::initialpose_pub(
        const std::string _robotName, const geometry_msgs::msg::PoseWithCovarianceStamped &_initialpose_msg)
    {
        if (robots_.contains(_robotName))
            robots_[_robotName].initialpose_pub_->publish(_initialpose_msg);
    }

    void Instance_Manager::goal_pose_pub(
        const std::string _robotName, const geometry_msgs::msg::PoseStamped &_goal_msg)
    {
        if (robots_.contains(_robotName))
            robots_[_robotName].goal_pose_pub_->publish(_goal_msg);
    }

    Instance_Manager::Instance_Manager(nav2_util::LifecycleNode::SharedPtr &_nh)
        : nh_(_nh)
    {
        robots_.clear();

        std::cout << "Instance Manager has been initialzied" << std::endl;
    }

} // namespace multibot2_server