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

        load_tasks();

        update_timer_ = nh_->create_wall_timer(
            10ms, std::bind(&Instance_Manager::update, this));

        RCLCPP_INFO(nh_->get_logger(), "Instance Manager has been initialzied");
    }

    void Instance_Manager::init_variables()
    {
        std::map<std::string, Robot_ROS> empty_robots;
        robots_.swap(empty_robots);

        std::map<std::string, std::queue<Robot::Task>> empty_tasks;
        tasks_.swap(empty_tasks);

        std::map<std::string, bool> empty_task_loops;
        task_loops_.swap(task_loops_);

        global_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "global_costmap", std::string{nh_->get_namespace()}, "global_costmap");
        global_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(global_costmap_ros_);
    }

    void Instance_Manager::init_parameters()
    {
        if (not(nh_->has_parameter("server.record")))
            nh_->declare_parameter("server.record", record_);
        if (not(nh_->has_parameter("server.communication_range")))
            nh_->declare_parameter("server.communication_range", communication_range_);
        if (not(nh_->has_parameter("server.lookahead_dist")))
            nh_->declare_parameter("server.lookahead_dist", lookahead_dist_);
        if (not(nh_->has_parameter("server.subgoal_generator.duration")))
            nh_->declare_parameter("server.subgoal_generator.duration", subgoal_generator_duration_);
        if (not(nh_->has_parameter("server.mode")))
            nh_->declare_parameter("server.mode", mode_);

        nh_->get_parameter_or("server.record", record_, record_);
        nh_->get_parameter_or("server.communication_range", communication_range_, communication_range_);
        nh_->get_parameter_or("server.lookahead_dist", lookahead_dist_, lookahead_dist_);
        nh_->get_parameter_or("server.subgoal_generator.duration", subgoal_generator_duration_, subgoal_generator_duration_);
        nh_->get_parameter_or("server.mode", mode_, mode_);

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

    bool Instance_Manager::load_tasks()
    {
        YAML::Node node;

        try
        {
            std::string maps_fPath = std::filesystem::current_path().generic_string() + "/src/multibot2/multibot2_server/maps/";

            std::string map_server_params_fPath = std::filesystem::current_path().generic_string() + "/src/multibot2/multibot2_server/params/map_server_params.yaml";
            YAML::Node map_server_params = YAML::LoadFile(map_server_params_fPath);

            std::string map_name = map_server_params["map_server"]["ros__parameters"]["map"].as<std::string>();
            std::string task_fPath = maps_fPath + map_name + "/" + "task.yaml";

            node = YAML::LoadFile(task_fPath);
        }
        catch (const YAML::BadFile &_err_BadFile)
        {
            RCLCPP_ERROR(nh_->get_logger(), "Instance_Manager::load_tasks(): %s", _err_BadFile.what());
            return false;
        }
        catch (const YAML::ParserException &_err_ParserException)
        {
            RCLCPP_ERROR(nh_->get_logger(), "Instance_Manager::load_tasks(): %s", _err_ParserException.what());
            return false;
        }

        try
        {
            for (const auto &robot_tasks : node)
            {
                std::string name = robot_tasks["name"].as<std::string>();

                std::queue<Robot::Task> task_queue;
                for (const auto &task : robot_tasks["task_queue"])
                {
                    std::vector<double> location = task["location"].as<std::vector<double>>();
                    double duration = task["duration"].as<double>();

                    task_queue.emplace(Pose(location), duration);
                }

                tasks_.emplace(name, task_queue);

                bool task_loop = robot_tasks["loop"].as<bool>();
                task_loops_.emplace(name, task_loop);
            }

            return true;
        }
        catch (const YAML::ParserException &_err_ParserException)
        {
            std::cerr << "[Error] RoadMap::saveVertices(): " << _err_ParserException.what() << std::endl;
            return false;
        }
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
            robot_ros.task_pub_ = nh_->create_publisher<Robot_ROS::Task>(
                "/" + robotName + "/task", qos);
            robot_ros.task_pub_->on_activate();
            robot_ros.goal_pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/" + robotName + "/goal_pose", qos);
            robot_ros.goal_pose_pub_->on_activate();
            robot_ros.subgoal_pose_pub_ = nh_->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/" + robotName + "/subgoal_pose", qos);
            robot_ros.subgoal_pose_pub_->on_activate();
            robot_ros.neighbors_pub_ = nh_->create_publisher<Robot_ROS::Neighbors>(
                "/" + robotName + "/neighbors", qos);
            robot_ros.neighbors_pub_->on_activate();
            robot_ros.local_trajectory_sub_ = nh_->create_subscription<Robot_ROS::RobotWithTrajectory>(
                "/" + robotName + "/local_trajectory", rclcpp::QoS(rclcpp::KeepLast(100)),
                std::bind(&Instance_Manager::local_trajectory_callback, this, std::placeholders::_1));
            robot_ros.dynamic_obstacles_pub_ = nh_->create_publisher<Robot_ROS::RobotWithTrajectoryArray>(
                "/" + robotName + "/dynamic_obstacles", rclcpp::QoS(rclcpp::KeepLast(100)));
            robot_ros.dynamic_obstacles_pub_->on_activate();
            robot_ros.queue_revision_ = nh_->create_service<Robot_ROS::QueueRivision>(
                "/" + robotName + "/queue_revision",
                std::bind(&Instance_Manager::queue_revision, this, std::placeholders::_1, std::placeholders::_2));
            robot_ros.local_obstacles_sub_ = nh_->create_subscription<Robot_ROS::ObstacleArrayMsg>(
                "/" + robotName + "/local_obstacles", rclcpp::QoS(rclcpp::KeepLast(100)),
                std::bind(&Instance_Manager::local_obstacles_callback, this, std::placeholders::_1));
        }

        if (tasks_.contains(robotName))
        {
            if (not(tasks_[robotName].empty()))
            {
                if (tasks_[robotName].size() > 1 and task_loops_[robotName])
                    tasks_[robotName].push(tasks_[robotName].front());

                robot_ros.robot_.goal() = tasks_[robotName].front().loc();

                tasks_[robotName].pop();
            }

            robot_ros.robot_.task_queue() = tasks_[robotName];
            robot_ros.robot_.goal_queue() = tasks_[robotName];

            Robot_ROS::Task task;
            {
                robot_ros.robot_.goal().toPoseMsg(task.location.pose);
                task.duration = robot_ros.robot_.task_queue().front().duration();
            }

            robot_ros.task_pub_->publish(task);
        }

        robots_.insert(std::make_pair(robotName, robot_ros));

        RCLCPP_INFO(nh_->get_logger(), "Instance_Manager::insertRobot()");
        // RCLCPP_INFO(nh_->get_logger(), "New Robot " + robotName + " is registered.");
        RCLCPP_INFO(nh_->get_logger(), "New Robot %s is registered.", robotName);
        // RCLCPP_INFO(nh_->get_logger(), "Total Robots: " + std::to_string(robots_.size()) + "EA");
        RCLCPP_INFO(nh_->get_logger(), "Total Robots: %dEA", robots_.size());
        std::cout << robot_ros.robot_ << std::endl;
    }

    void Instance_Manager::deleteRobot(
        const std::string _robotName)
    {
        if (robots_.contains(_robotName))
        {
            robots_.erase(_robotName);

            // RCLCPP_INFO(nh_->get_logger(), "Robot " + _robotName + " is deleted.");
            RCLCPP_INFO(nh_->get_logger(), "Robot %s is deleted.", _robotName);
            // RCLCPP_INFO(nh_->get_logger(), "Total Robots: " + std::to_string(robots_.size()) + "EA\n");
            RCLCPP_INFO(nh_->get_logger(), "Total Robots: %dEA", robots_.size());
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

    void Instance_Manager::queue_revision(const std::shared_ptr<Robot_ROS::QueueRivision::Request> _request,
                                          std::shared_ptr<Robot_ROS::QueueRivision::Response> _response)
    {
        _response->is_complete = false;

        std::string robotName = _request->name;
        if (robots_.contains(robotName))
        {
            Robot_ROS &robot_ros = robots_[robotName];

            std::queue<Robot::Task> new_queue = std::queue<Robot::Task>();
            new_queue.emplace(Pose(_request->pose), 0.0);

            tasks_[robot_ros.robot_.name()] = new_queue;
            robot_ros.robot_.task_queue() = new_queue;
            robot_ros.robot_.goal_queue() = new_queue;

            _response->is_complete = true;
        }
    }

    void Instance_Manager::subgoal_pose_pub(
        const std::string _robotName, const geometry_msgs::msg::PoseStamped &_subgoal_msg)
    {
        if (robots_.contains(_robotName))
            robots_[_robotName].subgoal_pose_pub_->publish(_subgoal_msg);
    }

    void Instance_Manager::record_computation_time(const double _sec)
    {
        if (not(record_ and record_flag_))
            return;

        static double total_computation_time = 0.0;
        static int computation_number = 0;

        total_computation_time += _sec;
        computation_number++;

        computation_time_ = total_computation_time / computation_number;
    }

    void Instance_Manager::record_robot_poses()
    {
        if (not(record_ and record_flag_))
            return;

        std::vector<multibot2_util::Pose> robot_poses;
        for (const auto &robotPair : robots_)
            robot_poses.push_back(robotPair.second.robot_.pose());

        pose_table_.push_back(std::make_pair(std::chrono::system_clock::now(), robot_poses));
    }

    void Instance_Manager::export_recording()
    {
        record_flag_ = false;

        std::cout << "=====================================================" << std::endl;
        std::cout << "Stop" << std::endl;

        std::chrono::duration<double> transition_time = pose_table_.back().first - pose_table_.front().first;

        std::cout << "Transition time: " << transition_time.count() << "s" << std::endl;
        std::cout << "Compuation time: " << computation_time_ * 1000 << "ms" << std::endl;

        double total_transition_length = 0.0;
        for (size_t timeStep = 0; timeStep < pose_table_.size(); ++timeStep)
        {
            static std::vector<multibot2_util::Pose> prior_pose_vec = pose_table_[timeStep].second;
            if (timeStep == 0)
                continue;

            for (size_t robotIdx = 0; robotIdx < prior_pose_vec.size(); ++robotIdx)
            {
                auto relative_pose = pose_table_[timeStep].second[robotIdx] - prior_pose_vec[robotIdx];
                total_transition_length += relative_pose.position().norm();
            }

            prior_pose_vec = pose_table_[timeStep].second;
        }
        total_transition_length /= robots_.size();
        std::cout << "Transition length: " << total_transition_length << "m" << std::endl;
    }

    void Instance_Manager::convert_map_to_polygons()
    {
        std::shared_ptr<costmap_converter::CostmapToPolygonsDBSMCCH> costmap_converter = std::make_shared<costmap_converter::CostmapToPolygonsDBSMCCH>();
        costmap_converter->setCostmap2D(global_costmap_ros_->getCostmap());
        costmap_converter->compute();

        static_obstacles_ = costmap_converter->getPolygons();
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

        robot_ros.robot_.arrived() = _state_msg->arrived;
    }

    void Instance_Manager::local_obstacles_callback(const Robot_ROS::ObstacleArrayMsg::SharedPtr _obstacle_array_msg)
    {
        auto &robot = robots_[_obstacle_array_msg->name].robot_;

        robot.set_local_obstacles(_obstacle_array_msg->obstacles);
    }

    void Instance_Manager::local_trajectory_callback(const Robot_ROS::RobotWithTrajectory::SharedPtr _trajectory_msg)
    {
        auto &robot_ros = robots_[_trajectory_msg->name];

        robot_ros.local_trajectory_ = *_trajectory_msg;
    }

    void Instance_Manager::update_goals()
    {
        for (auto &robotPair : robots_)
        {
            Robot &robot = robotPair.second.robot_;

            if (not(robot.arrived()))
                continue;

            double SquaredDistance = (robot.goal() - robot.pose()).position().squaredNorm();
            if (SquaredDistance > 0.25 * 0.25)
                continue;

            if (tasks_[robot.name()].empty() or robot.task_queue().empty() or robot.goal_queue().empty())
                continue;

            if (tasks_[robot.name()].size() > 1 and task_loops_[robot.name()])
            {
                tasks_[robot.name()].push(tasks_[robot.name()].front());
                robot.task_queue().push(robot.task_queue().front());
                robot.goal_queue().push(robot.goal_queue().front());
            }

            robot.goal() = robot.goal_queue().front().loc();

            tasks_[robot.name()].pop();
            robot.task_queue().pop();
            robot.goal_queue().pop();

            Robot_ROS::Task task;
            {
                robot.goal().toPoseMsg(task.location.pose);
                task.duration = robot.goal_queue().front().duration();
            }

            robots_[robot.name()].task_pub_->publish(task);
        }
    }

    void Instance_Manager::update_neighbors()
    {
        if (mode_ == "V-RVO")
            return;

        for (auto &robotPair : robots_)
        {
            Robot &robot = robotPair.second.robot_;

            robot.neighbors().clear();
        }

        if (robots_.size() < 2)
            return;

        for (auto self_it = robots_.begin(); self_it != robots_.end(); ++self_it)
        {
            Robot &self = self_it->second.robot_;
            geometry_msgs::msg::PoseStamped self_pose;
            self.pose().toPoseMsg(self_pose.pose);

            Robot_ROS::Neighbors neighbors_msg;
            Robot_ROS::RobotWithTrajectoryArray dynamic_obstacles_msg;
            for (auto neighbor_it = robots_.begin(); neighbor_it != robots_.end(); ++neighbor_it)
            {
                if (neighbor_it == self_it)
                    continue;

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

                if (path_length < lookahead_dist_)
                {
                    Robot_ROS::Neighbor neighbor_msg;
                    {
                        neighbor_msg.radius = neighbor.radius();
                        neighbor.pose().toPoseMsg(neighbor_msg.pose.pose);
                        neighbor_msg.velocity.linear.x = neighbor.cur_vel_x();
                        neighbor_msg.velocity.angular.z = neighbor.cur_vel_theta();
                    }
                    neighbors_msg.neighbors.push_back(neighbor_msg);

                    if (mode_ == "V-PIBT")
                    {
                        if (self.higher_neighbors().contains(neighbor.name()))
                        {
                            Robot_ROS &neighbor_ros = neighbor_it->second;
                            dynamic_obstacles_msg.robots.push_back(neighbor_ros.local_trajectory_);
                        }
                    }
                    else if (mode_ == "DTEB")
                    {
                        if (self.name().compare(neighbor.name()) > 0)
                        {
                            Robot_ROS &neighbor_ros = neighbor_it->second;
                            dynamic_obstacles_msg.robots.push_back(neighbor_ros.local_trajectory_);
                        }
                    }
                }
            }
            Robot_ROS &self_ros = self_it->second;
            self_ros.neighbors_pub_->publish(neighbors_msg);
            self_ros.dynamic_obstacles_pub_->publish(dynamic_obstacles_msg);
        }
    }
} // namespace multibot2_server