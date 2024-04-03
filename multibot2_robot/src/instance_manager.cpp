#include "multibot2_robot/instance_manager.h"

using namespace std::chrono_literals;

namespace multibot2_robot
{
    Robot_ROS::Robot_ROS(const Robot_ROS &_robot_ros)
    {
        robot_ = _robot_ros.robot_;
        subgoal_ = _robot_ros.subgoal_;

        mode_ = _robot_ros.mode_;

        state_pub_ = _robot_ros.state_pub_;

        cmd_vel_pub_ = _robot_ros.cmd_vel_pub_;

        odom_sub_ = _robot_ros.odom_sub_;

        task_sub_ = _robot_ros.task_sub_;

        goal_sub_ = _robot_ros.goal_sub_;

        subgoal_sub_ = _robot_ros.subgoal_sub_;

        neighbors_sub_ = _robot_ros.neighbors_sub_;

        queue_revision_ = _robot_ros.queue_revision_;

        rviz_path_pub_ = _robot_ros.rviz_path_pub_;
    }

    Robot_ROS &Robot_ROS::operator=(const Robot_ROS &_rhs)
    {
        if (&_rhs != this)
        {
            robot_ = _rhs.robot_;
            subgoal_ = _rhs.subgoal_;

            mode_ = _rhs.mode_;

            state_pub_ = _rhs.state_pub_;

            cmd_vel_pub_ = _rhs.cmd_vel_pub_;

            odom_sub_ = _rhs.odom_sub_;

            task_sub_ = _rhs.task_sub_;

            goal_sub_ = _rhs.goal_sub_;

            subgoal_sub_ = _rhs.subgoal_sub_;

            neighbors_sub_ = _rhs.neighbors_sub_;

            queue_revision_ = _rhs.queue_revision_;

            rviz_path_pub_ = _rhs.rviz_path_pub_;
        }

        return *this;
    }

    Instance_Manager::Instance_Manager(nav2_util::LifecycleNode::SharedPtr &_nh)
        : nh_(_nh)
    {
        init_variables();
        init_parameters();

        robot_ros_.subgoal() = robot_ros_.robot().goal();

        global_costmap_ros_->activate();
        local_costmap_ros_->activate();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        robot_ros_.state_pub() = nh_->create_publisher<Robot_ROS::RobotState>(
            std::string{nh_->get_namespace()} + "/state", rclcpp::QoS(rclcpp::KeepLast(100)));
        robot_ros_.state_pub()->on_activate();

        robot_ros_.cmd_vel_pub() = nh_->create_publisher<geometry_msgs::msg::Twist>(
            std::string{nh_->get_namespace()} + "/cmd_vel", rclcpp::QoS(rclcpp::KeepLast(100)));
        robot_ros_.cmd_vel_pub()->on_activate();

        robot_ros_.odom_sub() = nh_->create_subscription<nav_msgs::msg::Odometry>(
            std::string{nh_->get_namespace()} + "/odom", qos,
            std::bind(&Instance_Manager::odom_callback, this, std::placeholders::_1));

        robot_ros_.task_sub() = nh_->create_subscription<Robot_ROS::Task>(
            std::string{nh_->get_namespace()} + "/task", qos,
            std::bind(&Instance_Manager::task_callback, this, std::placeholders::_1));

        robot_ros_.goal_sub() = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
            std::string{nh_->get_namespace()} + "/goal_pose", qos,
            std::bind(&Instance_Manager::goal_callback, this, std::placeholders::_1));

        robot_ros_.subgoal_sub() = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
            std::string{nh_->get_namespace()} + "/subgoal_pose", qos,
            std::bind(&Instance_Manager::subgoal_callback, this, std::placeholders::_1));

        robot_ros_.neighbors_sub() = nh_->create_subscription<Robot_ROS::Neighbors>(
            std::string{nh_->get_namespace()} + "/neighbors", qos,
            std::bind(&Instance_Manager::neighbors_callback, this, std::placeholders::_1));

        robot_ros_.queue_revision() = nh_->create_client<Robot_ROS::QueueRivision>(
            std::string{nh_->get_namespace()} + "/queue_revision");

        robot_ros_.rviz_path_pub() = nh_->create_publisher<nav_msgs::msg::Path>(
            std::string{nh_->get_namespace()} + "/rviz_traj", qos);
        robot_ros_.rviz_path_pub()->on_activate();

        robot_ros_.local_obstacles_pub() = nh_->create_publisher<Robot_ROS::ObstacleArrayMsg>(
            std::string{nh_->get_namespace()} + "/local_obstacles", rclcpp::QoS(rclcpp::KeepLast(100)));
        robot_ros_.local_obstacles_pub()->on_activate();

        update_timer_ = nh_->create_wall_timer(
            10ms, std::bind(&Instance_Manager::update_state, this));

        local_obstacles_update_timer_ = nh_->create_wall_timer(
            50ms, std::bind(&Instance_Manager::report_local_obstacles, this));

        RCLCPP_INFO(nh_->get_logger(), "Instance Manager has been initialized");
    }

    Instance_Manager::~Instance_Manager()
    {
        RCLCPP_INFO(nh_->get_logger(), "Instance Manager has been terminated");
    }

    void Instance_Manager::init_variables()
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(nh_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        robot_ros_ = Robot_ROS();

        global_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "global_costmap", std::string{nh_->get_namespace()}, "global_costmap");
        global_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(global_costmap_ros_);

        local_costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "local_costmap", std::string{nh_->get_namespace()}, "local_costmap");
        local_costmap_thread_ = std::make_unique<nav2_util::NodeThread>(local_costmap_ros_);
    }

    void Instance_Manager::init_parameters()
    {
        Robot &robot = robot_ros_.robot();
        {
            nh_->declare_parameter("robot.name", robot.name());
            nh_->declare_parameter("robot.type", robot.type());

            nh_->declare_parameter("robot.goal.x", robot.goal().x());
            nh_->declare_parameter("robot.goal.y", robot.goal().y());
            nh_->declare_parameter("robot.goal.theta", robot.goal().theta());

            nh_->declare_parameter("robot.radius", robot.radius());

            nh_->declare_parameter("robot.velocity_profile.max_vel_x", robot.max_vel_x());
            nh_->declare_parameter("robot.velocity_profile.max_vel_theta", robot.max_vel_theta());
            nh_->declare_parameter("robot.velocity_profile.acc_lim_x", robot.acc_lim_x());
            nh_->declare_parameter("robot.velocity_profile.acc_lim_theta", robot.acc_lim_theta());

            nh_->get_parameter_or("robot.name", robot.name(), robot.name());
            nh_->get_parameter_or("robot.type", robot.type(), robot.type());

            nh_->get_parameter_or("robot.goal.x", robot.goal().x(), robot.goal().x());
            nh_->get_parameter_or("robot.goal.y", robot.goal().y(), robot.goal().y());
            nh_->get_parameter_or("robot.goal.theta", robot.goal().theta(), robot.goal().theta());

            nh_->get_parameter_or("robot.radius", robot.radius(), robot.radius());

            nh_->get_parameter_or("robot.velocity_profile.max_vel_x", robot.max_vel_x(), robot.max_vel_x());
            nh_->get_parameter_or("robot.velocity_profile.max_vel_theta", robot.max_vel_theta(), robot.max_vel_theta());
            nh_->get_parameter_or("robot.velocity_profile.acc_lim_x", robot.acc_lim_x(), robot.acc_lim_x());
            nh_->get_parameter_or("robot.velocity_profile.acc_lim_theta", robot.acc_lim_theta(), robot.acc_lim_theta());
        }

        global_costmap_ros_->configure();
        local_costmap_ros_->configure();
    }

    void Instance_Manager::update_pose()
    {
        Robot &robot = robot_ros_.robot();

        geometry_msgs::msg::TransformStamped geoTr;

        try
        {
            geoTr = tf_buffer_->lookupTransform(
                "map", (robot.name() + "/base_link"), tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex)
        {
            return;
        }

        tf2::Quaternion q(
            geoTr.transform.rotation.x,
            geoTr.transform.rotation.y,
            geoTr.transform.rotation.z,
            geoTr.transform.rotation.w);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        robot.pose().x() = geoTr.transform.translation.x;
        robot.pose().y() = geoTr.transform.translation.y;
        robot.pose().theta() = yaw;
    }

    void Instance_Manager::report_state()
    {
        const Robot &robot = robot_ros_.robot();

        Robot_ROS::RobotState state;
        {
            state.name = robot.name();

            state.pose.x = robot.pose().x();
            state.pose.y = robot.pose().y();
            state.pose.theta = robot.pose().theta();

            state.goal.x = robot.goal().x();
            state.goal.y = robot.goal().y();
            state.goal.theta = robot.goal().theta();

            state.lin_vel = robot.cur_vel_x();
            state.ang_vel = robot.cur_vel_theta();

            state.arrived = robot.arrived();
        }

        robot_ros_.state_pub()->publish(state);
    }

    void Instance_Manager::report_local_obstacles()
    {
        Robot_ROS::ObstacleArrayMsg obstacleArray;
        obstacleArray.header.stamp = nh_->now();
        obstacleArray.name = robot().name();

        Eigen::Affine3d obstacle_to_map_eig;
        try
        {
            geometry_msgs::msg::TransformStamped obstacle_to_map = tf_buffer_->lookupTransform(
                global_costmap_ros_->getGlobalFrameID(), tf2::timeFromSec(0),
                local_costmap_ros_->getGlobalFrameID(), tf2::timeFromSec(0),
                local_costmap_ros_->getGlobalFrameID(), tf2::durationFromSec(0.5));
            obstacle_to_map_eig = tf2::transformToEigen(obstacle_to_map);
        }
        catch (const std::exception &ex)
        {
            RCLCPP_ERROR(nh_->get_logger(), "%s", ex.what());
            obstacle_to_map_eig.setIdentity();
        }

        nav2_costmap_2d::Costmap2D fake_local_costmap = get_fake_local_costmap();
        std::shared_ptr<costmap_converter::CostmapToPolygonsDBSMCCH> costmap_converter = std::make_shared<costmap_converter::CostmapToPolygonsDBSMCCH>();

        costmap_converter->setCostmap2D(&fake_local_costmap);
        costmap_converter->compute();

        costmap_converter::PolygonContainerConstPtr obstacles = costmap_converter->getPolygons();

        for (auto obst_iter = obstacles->begin(); obst_iter != obstacles->end(); ++obst_iter)
        {
            if (obst_iter->points.size() < 3)
                continue;

            std::vector<Eigen::Vector2d> vertices;
            for (const auto &vertex : obst_iter->points)
                vertices.push_back(Eigen::Vector2d(vertex.x, vertex.y));

            if (vertices.front().isApprox(vertices.back()))
                vertices.pop_back();

            if (vertices.size() < 3)
                continue;

            geometry_msgs::msg::Polygon poly;
            {
                for (const auto &vertex : vertices)
                {
                    Eigen::Vector3d obstacle_to_map_pos(vertex.x(), vertex.y(), 0.0);
                    obstacle_to_map_pos = obstacle_to_map_eig * obstacle_to_map_pos;

                    geometry_msgs::msg::Point32 point;
                    {
                        point.x = obstacle_to_map_pos.x();
                        point.y = obstacle_to_map_pos.y();
                    }
                    poly.points.push_back(point);
                }
            }

            obstacleArray.obstacles.push_back(poly);
        }

        robot_ros_.local_obstacles_pub()->publish(obstacleArray);
    }

    void Instance_Manager::odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg)
    {
        if (robot_ros_.mode() == Robot_ROS::Mode::MANUAL)
            return;

        Robot &robot = robot_ros_.robot();

        robot.cur_vel_x() = _odom_msg->twist.twist.linear.x;
        robot.cur_vel_theta() = _odom_msg->twist.twist.angular.z;
    }

    void Instance_Manager::task_callback(const Robot_ROS::Task::SharedPtr _task_msg)
    {
        Robot &robot = robot_ros_.robot();

        robot.goal() = multibot2_util::Pose(_task_msg->location);
        robot_ros_.subgoal() = robot.goal();

        robot_ros_.task_duration() = _task_msg->duration;
    }

    void Instance_Manager::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr _goal_msg)
    {
        if (robot_ros_.mode() != Robot_ROS::Mode::AUTO and
            robot_ros_.mode() != Robot_ROS::Mode::STAY)
        {
            return;
        }

        tf2::Quaternion q(
            _goal_msg->pose.orientation.x,
            _goal_msg->pose.orientation.y,
            _goal_msg->pose.orientation.z,
            _goal_msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        Robot &robot = robot_ros_.robot();

        robot.goal() = multibot2_util::Pose(_goal_msg->pose);

        robot_ros_.task_duration() = 0.0;

        while (not(robot_ros_.queue_revision()->wait_for_service(1s)))
        {
            if (not(rclcpp::ok()))
            {
                RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_ERROR(nh_->get_logger(), "Connection not available, waiting again...");
        }

        auto request = std::make_shared<Robot_ROS::QueueRivision::Request>();

        request->name = robot.name();
        request->pose = *_goal_msg;

        auto response_received_callback = [this](rclcpp::Client<Robot_ROS::QueueRivision>::SharedFuture _future)
        {
            auto response = _future.get();
            return;
        };

        auto future_result =
            robot_ros_.queue_revision()->async_send_request(request, response_received_callback);

        return;
    }

    void Instance_Manager::subgoal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr _subgoal_msg)
    {
        tf2::Quaternion q(
            _subgoal_msg->pose.orientation.x,
            _subgoal_msg->pose.orientation.y,
            _subgoal_msg->pose.orientation.z,
            _subgoal_msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        robot_ros_.subgoal() = multibot2_util::Pose(_subgoal_msg->pose);
    }

    const nav2_costmap_2d::Costmap2D Instance_Manager::get_fake_costmap(const nav2_costmap_2d::Costmap2D _original_costmap) const
    {
        nav2_costmap_2d::Costmap2D fake_costmap = _original_costmap;

        for (unsigned int i = 0; i < fake_costmap.getSizeInCellsX() - 1; ++i)
        {
            for (unsigned int j = 0; j < fake_costmap.getSizeInCellsY() - 1; ++j)
            {
                if (fake_costmap.getCost(i, j) == nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE or
                    fake_costmap.getCost(i, j) == nav2_costmap_2d::NO_INFORMATION)
                {
                    fake_costmap.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
                }
            }
        }

        return fake_costmap;
    }
} // namespace multibot2_robot