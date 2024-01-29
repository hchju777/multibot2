#include "multibot2_server/server_node.h"

using namespace std::chrono_literals;

namespace multibot2_server
{
    MultibotServer::MultibotServer()
        : nav2_util::LifecycleNode("server", "", true)
    {
        init_variables();

        subgoal_update_timer_ = nh_->create_wall_timer(
            50ms, std::bind(&MultibotServer::generate_subgoals, this));

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

        panel_is_running_ = false;
    }

    void MultibotServer::generate_subgoals()
    {
        SubgoalGenerator::DynamicGraph::UniquePtr dynamic_graph = std::make_unique<SubgoalGenerator::DynamicGraph>();

        std::map<std::string, Robot> robots;
        for (const auto &robot_rosPair : instance_manager_->robots())
        {
            const Robot &robot = robot_rosPair.second.robot_;

            robots.emplace(robot.name(), robot);
        }

        dynamic_graph->addVertices(robots);
        dynamic_graph->print();

        // dynamic_graph_->addVertices()
    }
} // namespace multibot2_server