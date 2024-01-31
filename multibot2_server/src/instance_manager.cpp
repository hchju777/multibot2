#include "multibot2_server/instance_manager.h"

using namespace std::chrono_literals;

namespace multibot2_server
{
    Instance_Manager::Instance_Manager(nav2_util::LifecycleNode::SharedPtr &_nh)
        : nh_(_nh)
    {
        init_variables();
        init_parameters();

        global_costmap_ros_->activate();

        convert_map_to_polygons();
        init_global_planner();

        update_timer_ = nh_->create_wall_timer(
            10ms, std::bind(&Instance_Manager::update_neighbors, this));

        std::cout << "Instance Manager has been initialzied" << std::endl;
    }

    void Instance_Manager::init_variables()
    {
        std::map<std::string, Robot_ROS> empty_robots;
        robots_.swap(empty_robots);

        global_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "global_costmap", std::string{nh_->get_namespace()}, "global_costmap");
        global_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(global_costmap_ros_);
    }

    void Instance_Manager::init_parameters()
    {
        nh_->declare_parameter("server.communication_range", communication_range_);
        nh_->declare_parameter("server.subgoal_generator.duration", subgoal_generator_duration_);

        nh_->get_parameter_or("server.communication_range", communication_range_, communication_range_);
        nh_->get_parameter_or("server.subgoal_generator.duration", subgoal_generator_duration_, subgoal_generator_duration_);

        global_costmap_ros_->configure();
    }

    void Instance_Manager::init_global_planner()
    {
        navfn_global_planner_ = std::make_shared<nav2_navfn_planner::NavfnPlanner>();
        navfn_global_planner_->configure(
            nh_, "global_planner",
            global_costmap_ros_->getTfBuffer(), global_costmap_ros_);
        navfn_global_planner_->activate();
    }

    void Instance_Manager::convert_map_to_polygons()
    {
        std::shared_ptr<costmap_converter::CostmapToPolygonsDBSMCCH> costmap_converter = std::make_shared<costmap_converter::CostmapToPolygonsDBSMCCH>();
        costmap_converter->setCostmap2D(global_costmap_ros_->getCostmap());
        costmap_converter->compute();

        static_obstacles_ = costmap_converter->getPolygons();
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
            rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(10));

            robot_ros.state_sub_ = nh_->create_subscription<Robot_ROS::State>(
                "/" + robotName + "/state", qos,
                std::bind(&Instance_Manager::robotState_callback, this, std::placeholders::_1));
            robot_ros.cmd_vel_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>(
                "/" + robotName + "/cmd_vel", qos);
            robot_ros.cmd_vel_pub_->on_activate();
            robot_ros.kill_robot_cmd_ = nh_->create_publisher<std_msgs::msg::Bool>(
                "/" + robotName + "/kill", qos);
            robot_ros.kill_robot_cmd_->on_activate();
            robot_ros.initialpose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/" + robotName + "/initialpose", qos);
            robot_ros.initialpose_pub_->on_activate();
            robot_ros.goal_pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/" + robotName + "/goal_pose", qos);
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

        switch (_mode)
		{
		case PanelUtil::Mode::REMOTE:
			request->mode = "REMOTE";
			break;

		case PanelUtil::Mode::MANUAL:
			request->mode = "MANUAL";
			break;

		case PanelUtil::Mode::AUTO:
			request->mode = "AUTO";
			break;

		case PanelUtil::Mode::STAY:
			request->mode = "STAY";
			break;

		default:
			break;
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

    void Instance_Manager::update_neighbors()
    {
        for (auto &robotPair : robots_)
        {
            Robot &robot = robotPair.second.robot_;

            robot.neighbors().clear();
        }

        if (robots_.size() < 2)
            return;

        for (auto self_it = robots_.begin(); self_it != std::prev(robots_.end()); ++self_it)
        {
            Robot &self = self_it->second.robot_;
            geometry_msgs::msg::PoseStamped self_pose;
            self.pose().toPoseMsg(self_pose.pose);

            for (auto neighbor_it = std::next(self_it); neighbor_it != robots_.end(); ++neighbor_it)
            {
                Robot &neighbor = neighbor_it->second.robot_;

                const Pose relative_pose = neighbor.pose() - self.pose();

                if (relative_pose.position().squaredNorm() > communication_range_ * communication_range_)
                    continue;

                geometry_msgs::msg::PoseStamped neighbor_pose;
                neighbor.pose().toPoseMsg(neighbor_pose.pose);

                const nav_msgs::msg::Path plan = navfn_global_planner_->createPlan(self_pose, neighbor_pose);

                double path_length = 0.0;
                if (not(plan.poses.empty()))
                {
                    for (size_t i = 0; i < plan.poses.size() - 1; ++i)
                    {
                        double diffX = plan.poses[i].pose.position.x - plan.poses[i + 1].pose.position.x;
                        double diffY = plan.poses[i].pose.position.y - plan.poses[i + 1].pose.position.y;

                        path_length += std::sqrt(diffX * diffX + diffY * diffY);
                    }
                }
                else
                    path_length = relative_pose.position().squaredNorm();

                if (path_length > communication_range_)
                    continue;

                self.neighbors().emplace(path_length, neighbor);
                neighbor.neighbors().emplace(path_length, self);
            }
        }
    }
} // namespace multibot2_server