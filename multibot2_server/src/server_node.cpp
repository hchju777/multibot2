#include "multibot2_server/server_node.h"

using namespace std::chrono_literals;

namespace multibot2_server
{
    MultibotServer::MultibotServer()
        : nav2_util::LifecycleNode("server", "", true)
    {
        init_variables();

        std::chrono::duration<double> duration{instance_manager_->subgoal_generator_duration()};
        subgoal_update_timer_ = nh_->create_wall_timer(
            std::chrono::duration_cast<std::chrono::milliseconds>(duration), std::bind(&MultibotServer::update_subgoals, this));

        RCLCPP_INFO(this->get_logger(), "MultibotServer has been initialized");
    }

    MultibotServer::~MultibotServer()
    {
        RCLCPP_INFO(this->get_logger(), "MultibotServer has been terminated");
    }

    void MultibotServer::run_server_panel(int argc, char *argv[])
    {
        QApplication app(argc, argv);

        serverPanel_ = std::make_unique<Panel>(nh_, instance_manager_);
        serverPanel_->attach(*this);
        serverPanel_->show();

        panel_is_running_ = true;

        app.exec();
    }

    void MultibotServer::update(const multibot2_util::PanelUtil::Msg &_msg)
    {
        if (not(panel_is_running_))
            return;

        switch (_msg)
        {
        case multibot2_util::PanelUtil::Request::PLAN_REQUEST:
        {
            break;
        }

        case multibot2_util::PanelUtil::Request::START_REQUEST:
        {
            break;
        }

        default:
            break;
        }
    }

    void MultibotServer::init_variables()
    {
        nh_ = std::shared_ptr<::nav2_util::LifecycleNode>(this, [](::nav2_util::LifecycleNode *) {});

        instance_manager_ = std::make_shared<Instance_Manager>(nh_);

        subgoalgen_config_ = std::make_shared<SubgoalGenerator::Config>();
        subgoalgen_config_->declareParameters(nh_);
        subgoalgen_config_->loadRosParamFromNodeHandle(nh_);

        subgoal_generator_ = std::make_unique<SubgoalGenerator::Generator>(subgoalgen_config_);

        panel_is_running_ = false;
    }

    void MultibotServer::update_subgoals()
    {
        if (instance_manager_->robots().empty())
            return;

        if (instance_manager_->getMode() == "V-PIBT")
        {
            std::map<std::string, Robot> robots;
            for (const auto &robot_rosPair : instance_manager_->robots())
            {
                const Robot &robot = robot_rosPair.second.robot_;

                robots.emplace(robot.name(), robot);
            }

            // std::chrono::system_clock::time_point start = std::chrono::system_clock::now();
            subgoal_generator_->update_subgoals(robots);
            // std::chrono::duration<double> sec = std::chrono::system_clock::now() - start;

            for (const auto &robotPair : robots)
            {
                std::string robotName = robotPair.first;

                instance_manager_->setFront(robotName, robotPair.second.front());
                instance_manager_->setReplanTime(robotName, robotPair.second.replan_time());
                instance_manager_->setSubgoal(robotName, robotPair.second.subgoal());
                instance_manager_->setHigherNeighbors(robotName, robotPair.second.higher_neighbors());

                geometry_msgs::msg::PoseStamped subgoal_pose;
                robotPair.second.subgoal().toPoseMsg(subgoal_pose.pose);

                instance_manager_->subgoal_pose_pub(robotName, subgoal_pose);
            }
        }
        else if (instance_manager_->getMode() == "DTEB")
        {
            for (const auto &robotPair : instance_manager_->robots())
            {
                std::string robotName = robotPair.first;

                geometry_msgs::msg::PoseStamped goal_pose;
                robotPair.second.robot_.goal().toPoseMsg(goal_pose.pose);

                instance_manager_->goal_pose_pub(robotName, goal_pose);
            }
        }
    }
} // namespace multibot2_server