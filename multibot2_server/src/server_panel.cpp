#include "multibot2_server/server_panel.h"

using namespace std::chrono_literals;

namespace multibot2_server
{
    void Panel::attach(Observer::ObserverInterface<PanelUtil::Msg> &_observer)
    {
        std::scoped_lock<std::mutex> lock(mtx_);

        if (std::find(list_observer_.begin(), list_observer_.end(), &_observer) == list_observer_.end())
            list_observer_.push_back(&_observer);
    }

    void Panel::detach(Observer::ObserverInterface<PanelUtil::Msg> &_observer)
    {
        std::scoped_lock<std::mutex> lock(mtx_);

        auto observer_iter = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
        while (observer_iter != list_observer_.end())
        {
            list_observer_.remove(&_observer);
            observer_iter = std::find(list_observer_.begin(), list_observer_.end(), &_observer);
        }
    }

    void Panel::notify()
    {
        for (auto &observer : list_observer_)
            observer->update(msg_);
    }

    void Panel::handleButton()
    {
        QObject *button = QObject::sender();

        inactivatedRobot_ = activatedRobot_;
        activatedRobot_ = button->objectName().toStdString();

        const auto &robotDB = instance_manager_->getRobot(activatedRobot_);
        activatedRobotGoal_.x = robotDB.robot_.goal().x();
        activatedRobotGoal_.y = robotDB.robot_.goal().y();
        activatedRobotGoal_.theta = robotDB.robot_.goal().theta();
        activatedRobotModeState_ = robotDB.mode_;

        ui_->ServerTab->setTabEnabled(PanelUtil::Tab::ROBOT, true);
        ui_->ServerTab->currentChanged(PanelUtil::Tab::ROBOT);

        ui_->doubleSpinBox_goalX->setValue(activatedRobotGoal_.x);
        ui_->doubleSpinBox_goalY->setValue(activatedRobotGoal_.y);
        ui_->doubleSpinBox_goalYaw->setValue(activatedRobotGoal_.theta);

        switch (activatedRobotModeState_)
        {
        case PanelUtil::Mode::MANUAL:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Manual"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(255,190,11);\nborder: 2px solid rgb(255,190,11);\nborder-radius: 15px");
            break;
        }

        case PanelUtil::Mode::REMOTE:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Remote"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(0,213,255);\nborder: 2px solid rgb(0,213,255);\nborder-radius: 15px");

            activatedRobotLinVel_ = 0.0;
            activatedRobotAngVel_ = 0.0;

            break;
        }

        case PanelUtil::Mode::AUTO:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Auto"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(230, 19, 237);\nborder: 2px solid rgb(230, 19, 237);\nborder-radius: 15px");
            break;
        }

        case PanelUtil::Mode::STAY:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Stay"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(0, 255, 153);\nborder: 2px solid rgb(0, 255, 153);\nborder-radius: 15px");
            break;
        }

        default:
            break;
        }
    }

    void Panel::on_ServerTab_currentChanged(int _tabIndex)
    {
        ui_->ServerTab->setCurrentIndex(_tabIndex);

        switch (_tabIndex)
        {
        case PanelUtil::Tab::DASHBOARD:
        {
            ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::DASHBOARD, QColor(0, 255, 255));
            ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::ROBOT, Qt::white);
            break;
        }

        case PanelUtil::Tab::ROBOT:
        {
            ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::DASHBOARD, Qt::white);
            ui_->ServerTab->tabBar()->setTabTextColor(PanelUtil::Tab::ROBOT, QColor(0, 255, 255));
            break;
        }

        default:
            break;
        }
    }

    void Panel::on_Start_clicked()
    {
        planState_ = PanelUtil::PlanState::READY;

        for (const auto &robotPair : instance_manager_->robots())
        {
            const Robot_ROS &robot_ros = robotPair.second;

            if (robot_ros.mode_ == PanelUtil::Mode::AUTO or
                robot_ros.mode_ == PanelUtil::Mode::STAY)
            {
                continue;
            }
            instance_manager_->request_modeChange(robotPair.first, PanelUtil::Mode::AUTO);
        }

        msg_ = PanelUtil::Request::START_REQUEST;
        notify();
    }

    void Panel::on_Stop_clicked()
    {
        std_msgs::msg::Bool stopActivated;
        stopActivated.data = true;

        emergencyStop_->publish(stopActivated);
    }

    void Panel::on_Scan_clicked()
    {
        std_msgs::msg::Bool scanActivated;
        scanActivated.data = true;

        serverScan_->publish(scanActivated);
    }

    void Panel::on_Plan_clicked()
    {
        if (buttons_.size() == 0)
            return;

        switch (planState_)
        {
        case PanelUtil::PlanState::READY:
        {
            planState_ = PanelUtil::PlanState::PLANNING;

            msg_ = PanelUtil::Request::PLAN_REQUEST;
            notify();
            break;
        }

        case PanelUtil::PlanState::SUCCESS:
        {
            planState_ = PanelUtil::PlanState::READY;
            break;
        }

        case PanelUtil::PlanState::FAIL:
        {
            planState_ = PanelUtil::PlanState::READY;
            break;
        }

        default:
            break;
        }
    }

    void Panel::on_pushButton_Mode_clicked()
    {
        if (activatedRobotModeState_ == PanelUtil::Mode::MANUAL)
            instance_manager_->request_modeChange(activatedRobot_, PanelUtil::Mode::REMOTE);
        else
            instance_manager_->request_modeChange(activatedRobot_, PanelUtil::Mode::MANUAL);
    }

    void Panel::on_pushButton_Kill_clicked()
    {
        instance_manager_->request_kill(activatedRobot_);

        if (buttons_.contains(activatedRobot_))
        {
            instance_manager_->deleteRobot(activatedRobot_);
            deleteRobotButton(QString::fromStdString(activatedRobot_));
        }
    }

    void Panel::keyPressEvent(QKeyEvent *_event)
    {
        if (not(ui_->ServerTab->currentIndex() == PanelUtil::ROBOT))
            return;

        if (_event->key() == Qt::Key_Return)
        {
            geometry_msgs::msg::Pose2D goal;
            {
                goal.x = ui_->doubleSpinBox_goalX->value();
                goal.y = ui_->doubleSpinBox_goalY->value();
                goal.theta = ui_->doubleSpinBox_goalYaw->value();
            }

            instance_manager_->setGoal(activatedRobot_, goal);
        }

        if (activatedRobotModeState_ == PanelUtil::Mode::REMOTE)
        {
            switch (_event->key())
            {
            case Qt::Key_Up:
                activatedRobotLinVel_ = activatedRobotLinVel_ + 0.1;
                break;

            case Qt::Key_Down:
                activatedRobotLinVel_ = activatedRobotLinVel_ - 0.1;
                break;

            case Qt::Key_Left:
                activatedRobotAngVel_ = activatedRobotAngVel_ + 0.1;
                break;

            case Qt::Key_Right:
                activatedRobotAngVel_ = activatedRobotAngVel_ - 0.1;
                break;

            case Qt::Key_Space:
                activatedRobotLinVel_ = 0.0;
                activatedRobotAngVel_ = 0.0;
                break;

            default:
                break;
            }
        }
    }

    void Panel::robotListDisp()
    {
        if (buttons_.contains(inactivatedRobot_))
        {
            buttons_[inactivatedRobot_]->setStyleSheet(
                "color: white;\nborder: 2px solid white;\nborder-radius: 15px;");
        }

        if (buttons_.contains(activatedRobot_))
        {
            buttons_[activatedRobot_]->setStyleSheet(
                "color: rgb(255, 94, 0);\nborder: 3.5px solid rgb(255, 94, 0);\nborder-radius: 15px;");
        }
    }

    void Panel::robotNumDisp()
    {
        if (ui_->ServerTab->currentIndex() == PanelUtil::Tab::DASHBOARD)
            ui_->label_RobotNum->setText(QString::number(buttons_.size()));
    }

    void Panel::robotTabDisp()
    {
        if (ui_->ServerTab->currentIndex() == PanelUtil::Tab::ROBOT)
        {
            ui_->label_activated_robotName->setText(QString::fromStdString(activatedRobot_));

            QString lin_vel_qstring = QString::number(std::round(activatedRobotLinVel_ * 100.0) / 100.0);
            QString ang_vel_qstring = QString::number(std::round(activatedRobotAngVel_ * 100.0) / 100.0);

            auto goal = instance_manager_->getRobot(activatedRobot_).robot_.goal();

            ui_->doubleSpinBox_goalX->setValue(goal.x());
            ui_->doubleSpinBox_goalY->setValue(goal.y());
            ui_->doubleSpinBox_goalYaw->setValue(goal.theta());

            ui_->label_Linear_Velocity->setText(lin_vel_qstring);
            ui_->label_Angular_Velocity->setText(ang_vel_qstring);
        }
    }

    void Panel::modeButtonDisp()
    {
        if (not(ui_->ServerTab->currentIndex() == PanelUtil::Tab::ROBOT))
            return;

        switch (activatedRobotModeState_)
        {
        case PanelUtil::Mode::MANUAL:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Manual"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(255,190,11);\nborder: 2px solid rgb(255,190,11);\nborder-radius: 15px");
            break;
        }

        case PanelUtil::Mode::REMOTE:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Remote"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(0,213,255);\nborder: 2px solid rgb(0,213,255);\nborder-radius: 15px");
            break;
        }

        case PanelUtil::Mode::AUTO:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Auto"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(230, 19, 237);\nborder: 2px solid rgb(230, 19, 237);\nborder-radius: 15px");
            break;
        }

        case PanelUtil::Mode::STAY:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Stay"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(0, 255, 153);\nborder: 2px solid rgb(0, 255, 153);\nborder-radius: 15px");
            break;
        }

        default:
            break;
        }
    }

    void Panel::planButtonDisp()
    {
        switch (planState_)
        {
        case PanelUtil::PlanState::READY:
            ui_->Plan->setText("Plan");
            ui_->Plan->setStyleSheet(
                "color: rgb(58, 134, 255);\nborder: 2px solid rgb(58, 134, 255);\nborder-radius: 15px;");
            break;

        case PanelUtil::PlanState::PLANNING:
            ui_->Plan->setText("Planning");
            ui_->Plan->setStyleSheet(
                "color: rgb(52, 235, 235);\nborder: 2px solid rgb(52, 235, 235);\nborder-radius: 15px;");
            break;

        case PanelUtil::PlanState::SUCCESS:
            ui_->Plan->setText("Success");
            ui_->Plan->setStyleSheet(
                "color: rgb(0, 200, 0);\nborder: 2px solid rgb(0, 200, 0);\nborder-radius: 15px;");
            break;

        case PanelUtil::PlanState::FAIL:
            ui_->Plan->setText("Fail");
            ui_->Plan->setStyleSheet(
                "color: rgb(200, 0, 0);\nborder: 2px solid rgb(200, 0, 0);\nborder-radius: 15px;");
            break;

        default:
            break;
        }
    }

    void Panel::addRobotButton(QString _robotName)
    {
        QPushButton *button = new QPushButton(_robotName);
        button->setObjectName(_robotName);

        ui_->scrollAreaWidgetContents->setFixedHeight(
            ui_->scrollAreaWidgetContents->height() + deltaScrollHeight_);

        button->setMinimumHeight(buttonHeight_);
        button->setFont(QFont("Ubuntu", buttonFontSize_, QFont::Bold));
        button->setStyleSheet(
            "color: white;\nborder: 2px solid white;\nborder-radius: 15px;");

        buttons_.insert(std::make_pair(_robotName.toStdString(), button));
        connect(button, SIGNAL(clicked()), this, SLOT(handleButton()));

        ui_->scrollArea_Robot_List->widget()->layout()->addWidget(button);
    }

    void Panel::deleteRobotButton(QString _robotName)
    {
        auto button = buttons_[_robotName.toStdString()];

        buttons_.erase(_robotName.toStdString());

        ui_->scrollAreaWidgetContents->setFixedHeight(
            ui_->scrollAreaWidgetContents->height() - deltaScrollHeight_);

        if (_robotName.toStdString() == activatedRobot_)
        {
            activatedRobot_ = std::string();
            ui_->ServerTab->setTabEnabled(PanelUtil::Tab::ROBOT, false);

            if (ui_->ServerTab->currentIndex() == PanelUtil::Tab::ROBOT)
                ui_->ServerTab->setCurrentIndex(PanelUtil::Tab::DASHBOARD);
        }

        delete button;
    }

    void Panel::setPlanState(PanelUtil::PlanState _planState)
    {
        if (not(_planState == PanelUtil::PlanState::SUCCESS or
                _planState == PanelUtil::PlanState::FAIL))
            return;

        planState_ = _planState;
    }

    // See: https://dev.to/fmtweisszwerg/cc-how-to-get-all-interface-addresses-on-the-local-device-3pki
    std::string Panel::getIPAddress()
    {
        struct ifaddrs *ptr_ifaddrs = nullptr;

        auto result = getifaddrs(&ptr_ifaddrs);

        // getifaddrs() failed.
        if (result != 0)
            return "000.000.000.000";

        std::string ipaddress_human_readable_form;

        for (
            struct ifaddrs *ptr_entry = ptr_ifaddrs;
            ptr_entry != nullptr;
            ptr_entry = ptr_entry->ifa_next)
        {

            std::string interface_name = std::string(ptr_entry->ifa_name);
            sa_family_t address_family = ptr_entry->ifa_addr->sa_family;

            if (address_family == AF_INET)
            {
                // IPv4

                // Be aware that the `ifa_addr`, `ifa_netmask` and `ifa_data` fields might contain nullptr.
                // Dereferencing nullptr causes "Undefined behavior" problems.
                // So it is need to check these fields before dereferencing.
                if (ptr_entry->ifa_addr != nullptr)
                {
                    char buffer[INET_ADDRSTRLEN] = {
                        0,
                    };
                    inet_ntop(
                        address_family,
                        &((struct sockaddr_in *)(ptr_entry->ifa_addr))->sin_addr,
                        buffer,
                        INET_ADDRSTRLEN);

                    ipaddress_human_readable_form = std::string(buffer);
                }

                if (interface_name != "lo")
                    break;
            }
            else if (address_family == AF_INET6)
            {
                // IPv6
                if (ptr_entry->ifa_addr != nullptr)
                {
                    char buffer[INET6_ADDRSTRLEN] = {
                        0,
                    };
                    inet_ntop(
                        address_family,
                        &((struct sockaddr_in6 *)(ptr_entry->ifa_addr))->sin6_addr,
                        buffer,
                        INET6_ADDRSTRLEN);

                    ipaddress_human_readable_form = std::string(buffer);
                }

                if (interface_name != "lo")
                    break;
            }
            else
            {
                // AF_UNIX, AF_UNSPEC, AF_PACKET etc.
                // If ignored, delete this section.
            }
        }

        freeifaddrs(ptr_ifaddrs);

        return ipaddress_human_readable_form;
    }

    void Panel::register_robot(
        const std::shared_ptr<PanelUtil::Connection::Request> _request,
        std::shared_ptr<PanelUtil::Connection::Response> _response)
    {
        if (buttons_.contains(_request->config.name))
        {
            _response->is_connected = true;
            return;
        }

        static int32_t robotID = 0;

        Robot robot;
        {
            robot.name() = _request->config.name;
            robot.type() = _request->config.type;

            robot.radius() = _request->config.size;

            robot.max_vel_x() = _request->config.max_linvel;
            robot.acc_lim_x() = _request->config.max_linacc;
            robot.max_vel_theta() = _request->config.max_angvel;
            robot.acc_lim_theta() = _request->config.max_angacc;

            robot.goal().x() = _request->goal.x;
            robot.goal().y() = _request->goal.y;
            robot.goal().theta() = _request->goal.theta;
        }

        Robot_ROS robot_ros;
        {
            robot_ros.robot_ = robot;
            robot_ros.id_ = robotID;
            robot_ros.mode_ = PanelUtil::Mode::MANUAL;

            robot_ros.prior_update_time_ = nh_->now();
            robot_ros.last_update_time_ = nh_->now();

            robot_ros.modeFromRobot_ = nh_->create_service<PanelUtil::ModeSelection>(
                "/" + robot.name() + "/modeFromRobot",
                std::bind(&Panel::change_robot_mode, this, std::placeholders::_1, std::placeholders::_2));
            robot_ros.modeFromServer_ = nh_->create_client<PanelUtil::ModeSelection>("/" + robot.name() + "/modeFromServer");
        }

        instance_manager_->insertRobot(robot_ros);
        emit addRobotSignal(QString::fromStdString(_request->config.name));

        _response->is_connected = true;
    }

    void Panel::expire_robot(
        const std::shared_ptr<PanelUtil::Disconnection::Request> _request,
        std::shared_ptr<PanelUtil::Disconnection::Response> _response)
    {
        if (buttons_.contains(_request->name))
        {
            instance_manager_->deleteRobot(_request->name);
            deleteRobotButton(QString::fromStdString(_request->name));
        }
        _response->is_disconnected = true;
    }

    void Panel::change_robot_mode(
        const std::shared_ptr<PanelUtil::ModeSelection::Request> _request,
        std::shared_ptr<PanelUtil::ModeSelection::Response> _response)
    {
        std::map<std::string, PanelUtil::Mode> mode_hashmap = {
            {"REMOTE", PanelUtil::Mode::REMOTE},
            {"MANUAL", PanelUtil::Mode::MANUAL},
            {"AUTO", PanelUtil::Mode::AUTO},
            {"STAY", PanelUtil::Mode::STAY}};

        instance_manager_->setMode(_request->name, mode_hashmap[_request->mode]);

        _response->is_complete = true;
    }

    void Panel::rviz_initialpose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr _initialpose_msg)
    {
        if (buttons_.contains(activatedRobot_))
            instance_manager_->initialpose_pub(activatedRobot_, *_initialpose_msg);
    }

    void Panel::rviz_goal_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr _goal_pose_msg)
    {
        if (buttons_.contains(activatedRobot_))
            instance_manager_->goal_pose_pub(activatedRobot_, *_goal_pose_msg);
    }

    void Panel::update()
    {
        update_robot_tab();
        update_rviz_poseArray();
    }

    void Panel::update_robot_tab()
    {
        if (not(ui_->ServerTab->currentIndex() == PanelUtil::Tab::ROBOT))
            return;

        auto robot_ros = instance_manager_->getRobot(activatedRobot_);

        activatedRobotModeState_ = robot_ros.mode_;
        if (activatedRobotModeState_ == PanelUtil::Mode::REMOTE)
        {
            geometry_msgs::msg::Twist remote_cmd_vel;
            {
                remote_cmd_vel.linear.x = activatedRobotLinVel_;
                remote_cmd_vel.angular.z = activatedRobotAngVel_;
            }
            instance_manager_->remote_control(activatedRobot_, remote_cmd_vel);
        }
        else
        {
            activatedRobotLinVel_ = robot_ros.robot_.cur_vel_x();
            activatedRobotAngVel_ = robot_ros.robot_.cur_vel_theta();
        }
    }

    void Panel::update_rviz_poseArray()
    {
        if (buttons_.empty())
            return;
        visualization_msgs::msg::MarkerArray robotPoseMarkerArray;
        {
            robotPoseMarkerArray.markers.clear();

            for (const auto &buttonPair : buttons_)
            {
                std::string robotName = buttonPair.first;

                auto &robot = instance_manager_->getRobot(robotName);

                if (robot.prior_update_time_.seconds() < 1e-8)
                    continue;

                auto robotPoseMarker = make_robotPoseMarker(robot);

                if (robotPoseMarker.ns == "")
                    continue;

                robotPoseMarkerArray.markers.push_back(robotPoseMarker);
            }
        }

        rviz_poses_pub_->publish(robotPoseMarkerArray);
    }

    visualization_msgs::msg::Marker Panel::make_robotPoseMarker(const Robot_ROS &_robot_ros)
    {
        auto robotPoseMarker = visualization_msgs::msg::Marker();

        robotPoseMarker.header.frame_id = "map";
        robotPoseMarker.header.stamp = _robot_ros.last_update_time_;
        robotPoseMarker.ns = _robot_ros.robot_.name();
        robotPoseMarker.id = _robot_ros.id_;
        robotPoseMarker.type = visualization_msgs::msg::Marker::ARROW;
        robotPoseMarker.action = visualization_msgs::msg::Marker::ADD;

        robotPoseMarker.scale.x = 0.5;
        robotPoseMarker.scale.y = 0.125;
        robotPoseMarker.scale.z = 0.125;

        if (activatedRobot_ == _robot_ros.robot_.name())
        {
            robotPoseMarker.color.r = 1.0;
            robotPoseMarker.color.g = 0.5;
            robotPoseMarker.color.b = 0.5;
            robotPoseMarker.color.a = 1.0;
        }
        else
        {
            robotPoseMarker.color.r = 0.5;
            robotPoseMarker.color.g = 0.5;
            robotPoseMarker.color.b = 1.0;
            robotPoseMarker.color.a = 1.0;
        }

        robotPoseMarker.lifetime = _robot_ros.last_update_time_ - _robot_ros.prior_update_time_;

        robotPoseMarker.pose.position.x = _robot_ros.robot_.pose().x();
        robotPoseMarker.pose.position.y = _robot_ros.robot_.pose().y();
        robotPoseMarker.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, _robot_ros.robot_.pose().theta());
        robotPoseMarker.pose.orientation.x = q.x();
        robotPoseMarker.pose.orientation.y = q.y();
        robotPoseMarker.pose.orientation.z = q.z();
        robotPoseMarker.pose.orientation.w = q.w();

        return robotPoseMarker;
    }

    Panel::Panel(
        nav2_util::LifecycleNode::SharedPtr &_nh,
        std::shared_ptr<Instance_Manager> _instance_manager,
        QWidget *_parent)
        : QWidget(_parent), ui_(new Ui::ServerPanel),
          nh_(_nh), instance_manager_(_instance_manager)
    {
        ui_->setupUi(this);

        setFocusPolicy(Qt::StrongFocus);

        setWindowFlags(
            Qt::WindowStaysOnTopHint |
            Qt::Window |
            Qt::WindowTitleHint |
            Qt::CustomizeWindowHint |
            Qt::WindowMinimizeButtonHint);

        msg_ = PanelUtil::Request::NO_REQUEST;

        ui_->label_IPAddress->setText(QString::fromStdString(getIPAddress()));
        planState_ = PanelUtil::PlanState::READY;

        buttons_.clear();
        ui_->scrollAreaWidgetContents->setFixedHeight(0);
        ui_->scrollArea_Robot_List->widget()->setLayout(new QVBoxLayout());
        ui_->scrollArea_Robot_List->widget()->layout()->setSpacing(scrollSpacing_);
        ui_->scrollArea_Robot_List->setWidgetResizable(true);

        ui_->ServerTab->currentChanged(PanelUtil::Tab::DASHBOARD);
        ui_->ServerTab->setTabEnabled(PanelUtil::Tab::ROBOT, false);

        displayTimer_ = new QTimer(this);
        connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotListDisp()));
        connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotNumDisp()));
        connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotTabDisp()));
        connect(displayTimer_, SIGNAL(timeout()), this, SLOT(modeButtonDisp()));
        connect(displayTimer_, SIGNAL(timeout()), this, SLOT(planButtonDisp()));

        connect(this, SIGNAL(addRobotSignal(QString)), this, SLOT(addRobotButton(QString)));

        displayTimer_->start(10);

        // init ROS2 Instances
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        connection_ = nh_->create_service<PanelUtil::Connection>(
            "/server/connection",
            std::bind(&Panel::register_robot, this, std::placeholders::_1, std::placeholders::_2));
        disconnection_ = nh_->create_service<PanelUtil::Disconnection>(
            "/server/disconnection",
            std::bind(&Panel::expire_robot, this, std::placeholders::_1, std::placeholders::_2));

        serverScan_ = nh_->create_publisher<std_msgs::msg::Bool>("/server/server_scan", qos);
        serverScan_->on_activate();
        emergencyStop_ = nh_->create_publisher<std_msgs::msg::Bool>("/server/emergency_stop", qos);
        emergencyStop_->on_activate();

        rviz_initialpose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/server/initialpose", qos,
            std::bind(&Panel::rviz_initialpose_callback, this, std::placeholders::_1));

        rviz_goal_pose_sub_ = nh_->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/server/goal_pose", qos,
            std::bind(&Panel::rviz_goal_pose_callback, this, std::placeholders::_1));

        rviz_poses_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("/server/robot_list", qos);
        rviz_poses_pub_->on_activate();

        update_timer_ = nh_->create_wall_timer(
            10ms, std::bind(&Panel::update, this));
    }

    Panel::~Panel()
    {
        delete ui_;
    }

} // namespace multibot2_server