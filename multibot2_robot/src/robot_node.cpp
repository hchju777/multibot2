#include "multibot2_robot/robot_node.h"

using namespace std::chrono_literals;

namespace multibot2_robot
{
    MultibotRobot::MultibotRobot()
        : nav2_util::LifecycleNode("robot", "", true)
    {
        init_variables();
        init_parameters();

        navfn_global_planner_->activate();
        teb_local_planner_->activate();

        control_timer_ = this->create_wall_timer(
            50ms, std::bind(&MultibotRobot::auto_control, this));

        RCLCPP_INFO(this->get_logger(), "MultibotRobot has been initialized");
    }

    MultibotRobot::~MultibotRobot()
    {
        RCLCPP_INFO(this->get_logger(), "MultibotRobot has been terminated");
    }

    void MultibotRobot::run_robot_panel(int argc, char *argv[])
    {
        QApplication app(argc, argv);

        robot_panel_ = std::make_shared<Panel>(nh_, instance_manager_);
        robot_panel_->show();

        panel_is_running_ = true;

        app.exec();
    }

    void MultibotRobot::init_variables()
    {
        nh_ = std::shared_ptr<::nav2_util::LifecycleNode>(this, [](::nav2_util::LifecycleNode *) {});

        instance_manager_ = std::make_shared<Instance_Manager>(nh_);

        navfn_global_planner_ = std::make_shared<nav2_navfn_planner::NavfnPlanner>();

        teb_local_planner_ = std::make_shared<teb_local_planner::TebLocalPlannerROS>();

        panel_is_running_ = false;
    }

    void MultibotRobot::init_parameters()
    {
        nh_->declare_parameter("robot.goal_tolerance", goal_tolerance_);

        nh_->get_parameter_or("robot.goal_tolerance", goal_tolerance_, goal_tolerance_);

        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &global_costmap_ros = instance_manager_->global_costmap_ros();
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> &local_costmap_ros = instance_manager_->local_costmap_ros();

        navfn_global_planner_->configure(
            nh_, "global_planner",
            global_costmap_ros->getTfBuffer(), global_costmap_ros);

        teb_local_planner_->configure(
            nh_, "teb_local_planner",
            local_costmap_ros->getTfBuffer(), local_costmap_ros);
    }

    void MultibotRobot::auto_control()
    {
        static std::chrono::steady_clock::time_point time_point = std::chrono::steady_clock::now();

        if (not(panel_is_running_))
            return;

        Robot_ROS &robot_ros = instance_manager_->robot_ros();
        const Robot &robot = instance_manager_->robot();

        multibot2_util::Pose subgoal_difference = robot_ros.subgoal() - robot.pose();
        double squaredSubgoalDistance = subgoal_difference.position().squaredNorm();

        static bool no_task = false;

        if (robot_ros.mode() == Robot_ROS::Mode::STAY)
        {
            auto stay_time = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - time_point).count();

            // if (stay_time < robot_ros.task_duration())
            if (stay_time < 3.0)
                return;

            robot_panel_->emit_mode_signal(Panel::Mode::AUTO);
            no_task = false;

            if (squaredSubgoalDistance < goal_tolerance_ * goal_tolerance_)
            {
                robot_ros.task_duration() = 0.0;
                no_task = true;
                return;
            }
        }

        if (not(robot_ros.mode() == Robot_ROS::Mode::AUTO))
            return;

        geometry_msgs::msg::PoseStamped current_pose;
        robot.pose().toPoseMsg(current_pose.pose);

        if (squaredSubgoalDistance < goal_tolerance_ * goal_tolerance_)
        {
            multibot2_util::Pose goal_difference = robot_ros.subgoal() - robot.goal();
            double squaredDifference = goal_difference.position().squaredNorm();

            if (not(no_task) and squaredDifference < goal_tolerance_ * goal_tolerance_)
            {
                robot_panel_->emit_mode_signal(Panel::Mode::STAY);

                time_point = std::chrono::steady_clock::now();
            }

            robot_ros.robot().cur_vel_x() = 0.0;
            robot_ros.robot().cur_vel_theta() = 0.0;

            geometry_msgs::msg::Twist zero_cmd_vel;
            robot_ros.cmd_vel_pub()->publish(zero_cmd_vel);

            return;
        }

        no_task = false;

        geometry_msgs::msg::Twist current_twist;
        {
            current_twist.linear.x = robot.cur_vel_x();
            current_twist.angular.z = robot.cur_vel_theta();
        }

        geometry_msgs::msg::PoseStamped local_current_pose;
        instance_manager_->local_costmap_ros()->getRobotPose(local_current_pose);

        static geometry_msgs::msg::TwistStamped cmd_vel_2d;

        geometry_msgs::msg::PoseStamped goal_pose;
        robot_ros.subgoal().toPoseMsg(goal_pose.pose);

        teb_local_planner_->setPlan(navfn_global_planner_->createPlan(current_pose, goal_pose));

        try
        {
            cmd_vel_2d = teb_local_planner_->computeVelocityCommands(local_current_pose, current_twist);
        }
        catch (const std::exception &e)
        {
            RCLCPP_WARN(this->get_logger(), "%s", e.what());
            teb_local_planner_->setPlan(navfn_global_planner_->createPlan(current_pose, goal_pose));
        }

        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(cmd_vel_2d.twist);

        robot_ros.cmd_vel_pub()->publish(std::move(cmd_vel));
    }
} // namespace multibot2_robot