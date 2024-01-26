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

        goal_sub_ = _robot_ros.goal_sub_;

        subgoal_sub_ = _robot_ros.subgoal_sub_;

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

            goal_sub_ = _rhs.goal_sub_;

            subgoal_sub_ = _rhs.subgoal_sub_;

            rviz_path_pub_ = _rhs.rviz_path_pub_;
        }

        return *this;
    }

    Instance_Manager::Instance_Manager(nav2_util::LifecycleNode::SharedPtr &_nh)
        : nh_(_nh)
    {
        init_variables();
        init_parameters();

        global_costmap_ros_->activate();
        local_costmap_ros_->activate();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        robot_ros_.state_pub() = nh_->create_publisher<Robot_ROS::RobotState>(
            std::string{nh_->get_namespace()} + "/state", rclcpp::QoS(rclcpp::KeepLast(100)));
        robot_ros_.state_pub()->on_activate();

        robot_ros_.cmd_vel_pub() = nh_->create_publisher<geometry_msgs::msg::Twist>(
            std::string{nh_->get_namespace()} + "/cmd_vel", qos);
        robot_ros_.cmd_vel_pub()->on_activate();

        robot_ros_.odom_sub() = nh_->create_subscription<nav_msgs::msg::Odometry>(
            std::string{nh_->get_namespace()} + "/odom", qos,
            std::bind(&Instance_Manager::odom_callback, this, std::placeholders::_1));

        robot_ros_.goal_sub() = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
            std::string{nh_->get_namespace()} + "/goal_pose", qos,
            std::bind(&Instance_Manager::goal_callback, this, std::placeholders::_1));

        robot_ros_.subgoal_sub() = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
            std::string{nh_->get_namespace()} + "/goal_pose", qos,
            std::bind(&Instance_Manager::subgoal_callback, this, std::placeholders::_1));

        robot_ros_.rviz_path_pub() = nh_->create_publisher<nav_msgs::msg::Path>(
            std::string{nh_->get_namespace()} + "/rviz_traj", qos);
        robot_ros_.rviz_path_pub()->on_activate();

        update_timer_ = nh_->create_wall_timer(
            10ms, std::bind(&Instance_Manager::update_state, this));

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
            state.pose.theta = robot.goal().theta();

            state.lin_vel = robot.cur_vel_x();
            state.ang_vel = robot.cur_vel_theta();
        }

        robot_ros_.state_pub()->publish(state);
    }

    void Instance_Manager::odom_callback(const nav_msgs::msg::Odometry::SharedPtr _odom_msg)
    {
        if (robot_ros_.mode() == Robot_ROS::Mode::MANUAL)
            return;

        Robot &robot = robot_ros_.robot();

        robot.cur_vel_x() = _odom_msg->twist.twist.linear.x;
        robot.cur_vel_theta() = _odom_msg->twist.twist.angular.z;
    }

    void Instance_Manager::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr _goal_msg)
    {
        tf2::Quaternion q(
            _goal_msg->pose.orientation.x,
            _goal_msg->pose.orientation.y,
            _goal_msg->pose.orientation.z,
            _goal_msg->pose.orientation.w);
        tf2::Matrix3x3 m(q);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        Robot &robot = robot_ros_.robot();

        robot.goal().x() = _goal_msg->pose.position.x;
        robot.goal().y() = _goal_msg->pose.position.y;
        robot.goal().theta() = yaw;
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

        robot_ros_.subgoal().x() = _subgoal_msg->pose.position.x;
        robot_ros_.subgoal().y() = _subgoal_msg->pose.position.y;
        robot_ros_.subgoal().theta() = yaw;
    }
} // namespace multibot2_robot