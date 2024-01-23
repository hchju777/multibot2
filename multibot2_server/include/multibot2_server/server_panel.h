#pragma once

#include <memory>

#include <QWidget>
#include <QTimer>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QTabBar>
#include <QKeyEvent>

#include <arpa/inet.h>
#include <cerrno>
#include <ifaddrs.h>
#include <net/if.h>
#include <sysexits.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <nav2_util/lifecycle_node.hpp>
#include <tf2/LinearMath/Quaternion.h>

#include "multibot2_server/instance_manager.h"

#include "multibot2_util/panel_util.h"
#include "multibot2_util/Interface/Observer_Interface.h"

#include "multibot2_server/ui_server_panel.h"

#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace Ui
{
    class ServerPanel;
} // namespace Ui

namespace multibot2_server
{
    class Panel : public QWidget, public Observer::SubjectInterface<multibot2_util::PanelUtil::Msg>
    {
    public:
        typedef std::unique_ptr<Panel> UniquePtr;
        typedef std::shared_ptr<Panel> SharedPtr;

    public:
        typedef multibot2_util::PanelUtil::Tab Tab;
        typedef multibot2_util::PanelUtil::Request Request;
        typedef multibot2_util::PanelUtil::Mode Mode;
        typedef multibot2_util::PanelUtil::PlanState PlanState;
        typedef multibot2_util::PanelUtil::Msg Msg;

        typedef multibot2_util::PanelUtil::Connection Connection;
        typedef multibot2_util::PanelUtil::Disconnection Disconnection;
        typedef multibot2_util::PanelUtil::ModeSelection ModeSelection;

    public:
        Panel() {}

        Panel(nav2_util::LifecycleNode::SharedPtr &_nh, Instance_Manager::SharedPtr &_instance_manager,
              QWidget *_parent = nullptr);

        ~Panel() { delete ui_; }

    private:
        Q_OBJECT

    signals:
        void addRobotSignal(QString _robotName);

    public:
        void attach(Observer::ObserverInterface<multibot2_util::PanelUtil::Msg> &_observer) override;
        void detach(Observer::ObserverInterface<multibot2_util::PanelUtil::Msg> &_observer) override;
        void notify() override;

    private slots:
        void handleButton();
        void on_ServerTab_currentChanged(int _tabIndex);
        void on_Start_clicked();
        void on_Stop_clicked();
        void on_Scan_clicked();
        void on_Plan_clicked();
        void on_pushButton_Mode_clicked();
        void on_pushButton_Kill_clicked();

        void keyPressEvent(QKeyEvent *_event) override;

    private slots:
        void robotListDisp();
        void robotNumDisp();
        void robotTabDisp();
        void modeButtonDisp();
        void planButtonDisp();

    private slots:
        void addRobotButton(QString _robotName);
        void deleteRobotButton(QString _robotName);

    protected:
        void init_ui();

        std::string getIPAddress();

    protected:
        void update()
        {
            update_robot_tab();
            update_rviz_poseArray();
        }

        void update_robot_tab();

        void update_rviz_poseArray();

        visualization_msgs::msg::Marker make_robotPoseMarker(const Robot_ROS &_robot_ros);

    protected:
        void register_robot(
            const std::shared_ptr<Connection::Request> _request,
            std::shared_ptr<Connection::Response> _response);

        void expire_robot(
            const std::shared_ptr<Disconnection::Request> _request,
            std::shared_ptr<Disconnection::Response> _response);

        void change_robot_mode(
            const std::shared_ptr<ModeSelection::Request> _request,
            std::shared_ptr<ModeSelection::Response> _response);

    protected:
        Ui::ServerPanel *ui_;
        QTimer *displayTimer_;

    protected:
        nav2_util::LifecycleNode::SharedPtr nh_;

        rclcpp::TimerBase::SharedPtr update_timer_;

        rclcpp::Service<Connection>::SharedPtr connection_;
        rclcpp::Service<Disconnection>::SharedPtr disconnection_;

        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr serverScan_;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr emergencyStop_;

        rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_poses_pub_;

    protected:
        Instance_Manager::SharedPtr instance_manager_;

    protected:
        std::map<std::string, QPushButton *> buttons_;

        std::string activatedRobot_;
        std::string inactivatedRobot_;

        PlanState planState_{PlanState::READY};

    protected:
        multibot2_util::PanelUtil::Msg msg_{Msg::NO_REQUEST};
        std::list<Observer::ObserverInterface<Msg> *> list_observer_;

    protected:
        static constexpr int scrollSpacing_ = 20;
        static constexpr int buttonHeight_ = 50;
        static constexpr int buttonFontSize_ = 13;
        static constexpr int deltaScrollHeight_ = 70;

    }; // class Panel

} // namespace multibot2_server