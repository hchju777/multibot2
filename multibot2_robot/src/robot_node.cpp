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
        if (not(panel_is_running_))
            return;

        if (not(instance_manager_->robot_ros().mode() == Robot_ROS::Mode::AUTO))
            return;

        Robot_ROS &robot_ros = instance_manager_->robot_ros();
        const Robot &robot = instance_manager_->robot();

        geometry_msgs::msg::PoseStamped current_pose;
        robot.pose().toPoseMsg(current_pose.pose);

        multibot2_util::Pose relativePose = robot_ros.subgoal() - robot.pose();
        double squaredDistance = relativePose.position().squaredNorm();

        double tolerance = 0.25;
        if (squaredDistance < tolerance * tolerance)
        {
            // Change Mode as manual
            geometry_msgs::msg::Twist zero_cmd_vel;
            robot_ros.cmd_vel_pub()->publish(zero_cmd_vel);

            return;
        }

        geometry_msgs::msg::Twist current_twist;
        {
            current_twist.linear.x = robot.cur_vel_x();
            current_twist.angular.z = robot.cur_vel_theta();
        }

        geometry_msgs::msg::PoseStamped local_current_pose;
        instance_manager_->local_costmap_ros()->getRobotPose(local_current_pose);

        static geometry_msgs::msg::TwistStamped cmd_vel_2d;

        try
        {
            cmd_vel_2d = teb_local_planner_->computeVelocityCommands(local_current_pose, current_twist);
        }
        catch (const std::exception &e)
        {
            geometry_msgs::msg::PoseStamped goal_pose;
            robot_ros.subgoal().toPoseMsg(goal_pose.pose);

            teb_local_planner_->setPlan(navfn_global_planner_->createPlan(current_pose, goal_pose));
        }

        auto cmd_vel = std::make_unique<geometry_msgs::msg::Twist>(cmd_vel_2d.twist);

        robot_ros.cmd_vel_pub()->publish(std::move(cmd_vel));
    }
} // namespace multibot2_robot