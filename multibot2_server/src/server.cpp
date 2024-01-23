#include "multibot2_server/server.h"

namespace multibot2_server
{
    MultibotServer::MultibotServer()
        : nav2_util::LifecycleNode("server", "", true)
    {
        init_variables();

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

        pannel_is_running_ = true;

        app.exec();
    }

    void MultibotServer::update(const multibot2_util::PanelUtil::Msg &_msg)
    {
        if (not(pannel_is_running_))
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

        pannel_is_running_ = false;
    }
} // namespace multibot2_server