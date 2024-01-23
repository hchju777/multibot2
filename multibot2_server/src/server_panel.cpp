#include "multibot2_server/server_panel.h"

using namespace std::chrono_literals;

namespace multibot2_server
{
    Panel::Panel(nav2_util::LifecycleNode::SharedPtr &_nh, Instance_Manager::SharedPtr &_instance_manager,
                 QWidget *_parent)
        : QWidget(_parent), ui_(new Ui::ServerPanel),
          nh_(_nh), instance_manager_(_instance_manager)
    {
        init_ui();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        connection_ = nh_->create_service<Connection>(
            "/server/connection",
            std::bind(&Panel::register_robot, this, std::placeholders::_1, std::placeholders::_2));

        disconnection_ = nh_->create_service<Disconnection>(
            "/server/disconnection",
            std::bind(&Panel::expire_robot, this, std::placeholders::_1, std::placeholders::_2));

        serverScan_ = nh_->create_publisher<std_msgs::msg::Bool>("/server/scan", qos);
        serverScan_->on_activate();

        emergencyStop_ = nh_->create_publisher<std_msgs::msg::Bool>("/server/emergency_stop", qos);
        emergencyStop_->on_activate();

        rviz_poses_pub_ = nh_->create_publisher<visualization_msgs::msg::MarkerArray>("/server/robot_list", qos);

        update_timer_ = nh_->create_wall_timer(10ms, std::bind(&Panel::update, this));
    }

    void Panel::attach(Observer::ObserverInterface<Msg> &_observer)
    {
        std::scoped_lock<std::mutex> lock(mtx_);

        if (std::find(list_observer_.begin(), list_observer_.end(), &_observer) == list_observer_.end())
            list_observer_.push_back(&_observer);
    }

    void Panel::detach(Observer::ObserverInterface<Msg> &_observer)
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

        if (not(instance_manager_->robots().contains(activatedRobot_)))
            return;

        Robot_ROS &robot_ros = instance_manager_->robots()[activatedRobot_];
        Robot &robot = robot_ros.robot();

        ui_->ServerTab->setTabEnabled(Tab::ROBOT, true);
        ui_->ServerTab->currentChanged(Tab::ROBOT);

        ui_->doubleSpinBox_goalX->setValue(robot.goal().x());
        ui_->doubleSpinBox_goalY->setValue(robot.goal().y());
        ui_->doubleSpinBox_goalYaw->setValue(robot.goal().theta());

        switch (robot_ros.mode())
        {
        case Mode::MANUAL:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Manual"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(255,190,11);\nborder: 2px solid rgb(255,190,11);\nborder-radius: 15px");
            break;
        }

        case Mode::REMOTE:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Remote"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(0,213,255);\nborder: 2px solid rgb(0,213,255);\nborder-radius: 15px");

            robot.cur_vel_x() = 0.0;
            robot.cur_vel_theta() = 0.0;

            break;
        }

        case Mode::AUTO:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Auto"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(230, 19, 237);\nborder: 2px solid rgb(230, 19, 237);\nborder-radius: 15px");
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
        case Tab::DASHBOARD:
        {
            ui_->ServerTab->tabBar()->setTabTextColor(Tab::DASHBOARD, QColor(0, 255, 255));
            ui_->ServerTab->tabBar()->setTabTextColor(Tab::ROBOT, Qt::white);
            break;
        }

        case Tab::ROBOT:
        {
            ui_->ServerTab->tabBar()->setTabTextColor(Tab::DASHBOARD, Qt::white);
            ui_->ServerTab->tabBar()->setTabTextColor(Tab::ROBOT, QColor(0, 255, 255));
            break;
        }

        default:
            break;
        }
    }

    void Panel::on_Start_clicked()
    {
        if (not(planState_ == PlanState::SUCCESS))
            return;

        planState_ = PlanState::READY;

        msg_ = Request::START_REQUEST;
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
        case PlanState::READY:
        {
            planState_ = PlanState::PLANNING;

            msg_ = Request::PLAN_REQUEST;
            notify();
            break;
        }

        case PlanState::SUCCESS:
        {
            planState_ = PlanState::READY;
            break;
        }

        case PlanState::FAIL:
        {
            planState_ = PlanState::READY;
            break;
        }

        default:
            break;
        }
    }

    void Panel::on_pushButton_Mode_clicked()
    {
        if (not(instance_manager_->robots().contains(activatedRobot_)))
            return;

        Robot_ROS &robot_ros = instance_manager_->robots()[activatedRobot_];

        while (not(robot_ros.modeFromServer()->wait_for_service(1s)))
        {
            if (not(rclcpp::ok()))
            {
                RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_ERROR(nh_->get_logger(), "ModeChange not available, waiting again...");
        }

        ModeSelection::Request::SharedPtr request = std::make_shared<ModeSelection::Request>();
        {
            request->name = robot_ros.robot().name();

            if (robot_ros.mode() == Mode::REMOTE)
                request->is_remote = false;
            else if (robot_ros.mode() == Mode::MANUAL)
                request->is_remote = true;
        }

        auto response_received_callback = [this](rclcpp::Client<ModeSelection>::SharedFuture _future)
        {
            auto response = _future.get();
            return;
        };

        auto future_result =
            robot_ros.modeFromServer()->async_send_request(request, response_received_callback);

        if (not(future_result.get()->is_complete))
            return;

        Mode prev_mode = robot_ros.mode();

        robot_ros.mode() = Mode::MANUAL;
        if (prev_mode == robot_ros.mode())
            robot_ros.mode() = Mode::REMOTE;

        assert(prev_mode != robot_ros.mode());
    }

    void Panel::on_pushButton_Kill_clicked()
    {
        if (buttons_.contains(activatedRobot_))
        {
            instance_manager_->deleteRobot(activatedRobot_);
            deleteRobotButton(QString::fromStdString(activatedRobot_));
        }
    }

    void Panel::keyPressEvent(QKeyEvent *_event)
    {
        if (not(ui_->ServerTab->currentIndex() == Tab::ROBOT))
            return;

        if (not(instance_manager_->robots().contains(activatedRobot_)))
            return;

        Robot_ROS &robot_ros = instance_manager_->robots()[activatedRobot_];
        Robot &robot = robot_ros.robot();

        if (_event->key() == Qt::Key_Return)
        {
            robot.goal().x() = ui_->doubleSpinBox_goalX->value();
            robot.goal().y() = ui_->doubleSpinBox_goalY->value();
            robot.goal().theta() = ui_->doubleSpinBox_goalYaw->value();
        }

        if (robot_ros.mode() == Mode::REMOTE)
        {
            switch (_event->key())
            {
            case Qt::Key_Up:
                robot.cur_vel_x() += 0.1;
                break;

            case Qt::Key_Down:
                robot.cur_vel_x() -= 0.1;
                break;

            case Qt::Key_Left:
                robot.cur_vel_theta() += 0.1;
                break;

            case Qt::Key_Right:
                robot.cur_vel_x() -= 0.1;
                break;

            case Qt::Key_Space:
                robot.cur_vel_x() = 0.0;
                robot.cur_vel_theta() = 0.0;
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
        if (ui_->ServerTab->currentIndex() == Tab::DASHBOARD)
            ui_->label_RobotNum->setText(QString::number(buttons_.size()));
    }

    void Panel::robotTabDisp()
    {
        if (not(ui_->ServerTab->currentIndex() == Tab::ROBOT))
            return;

        if (not(instance_manager_->robots().contains(activatedRobot_)))
            return;

        const Robot &robot = instance_manager_->robots()[activatedRobot_].robot();

        ui_->label_activated_robotName->setText(QString::fromStdString(activatedRobot_));

        QString lin_vel_qstring = QString::number(std::round(robot.cur_vel_x() * 100.0) / 100.0);
        QString ang_vel_qstring = QString::number(std::round(robot.cur_vel_theta() * 100.0) / 100.0);

        ui_->label_Linear_Velocity->setText(lin_vel_qstring);
        ui_->label_Angular_Velocity->setText(ang_vel_qstring);
    }

    void Panel::modeButtonDisp()
    {
        if (not(ui_->ServerTab->currentIndex() == Tab::ROBOT))
            return;

        if (not(instance_manager_->robots().contains(activatedRobot_)))
            return;

        const Robot_ROS &robot_ros = instance_manager_->robots()[activatedRobot_];

        switch (robot_ros.mode())
        {
        case Mode::MANUAL:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Manual"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(255,190,11);\nborder: 2px solid rgb(255,190,11);\nborder-radius: 15px");
            break;
        }

        case Mode::REMOTE:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Remote"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(0,213,255);\nborder: 2px solid rgb(0,213,255);\nborder-radius: 15px");
            break;
        }

        case Mode::AUTO:
        {
            ui_->pushButton_Mode->setText(QString::fromStdString("Auto"));
            ui_->pushButton_Mode->setStyleSheet(
                "color: rgb(230, 19, 237);\nborder: 2px solid rgb(230, 19, 237);\nborder-radius: 15px");
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
        case PlanState::READY:
            ui_->Plan->setText("Plan");
            ui_->Plan->setStyleSheet(
                "color: rgb(58, 134, 255);\nborder: 2px solid rgb(58, 134, 255);\nborder-radius: 15px;");
            break;

        case PlanState::PLANNING:
            ui_->Plan->setText("Planning");
            ui_->Plan->setStyleSheet(
                "color: rgb(52, 235, 235);\nborder: 2px solid rgb(52, 235, 235);\nborder-radius: 15px;");
            break;

        case PlanState::SUCCESS:
            ui_->Plan->setText("Success");
            ui_->Plan->setStyleSheet(
                "color: rgb(0, 200, 0);\nborder: 2px solid rgb(0, 200, 0);\nborder-radius: 15px;");
            break;

        case PlanState::FAIL:
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
            ui_->ServerTab->setTabEnabled(Tab::ROBOT, false);

            if (ui_->ServerTab->currentIndex() == Tab::ROBOT)
                ui_->ServerTab->setCurrentIndex(Tab::DASHBOARD);
        }

        delete button;
    }

    void Panel::init_ui()
    {
        ui_->setupUi(this);

        setFocusPolicy(Qt::StrongFocus);

        setWindowFlags(
            Qt::WindowStaysOnTopHint |
            Qt::Window |
            Qt::WindowTitleHint |
            Qt::CustomizeWindowHint |
            Qt::WindowMinimizeButtonHint);

        ui_->label_IPAddress->setText(QString::fromStdString(getIPAddress()));

        buttons_.clear();
        ui_->scrollAreaWidgetContents->setFixedHeight(0);
        ui_->scrollArea_Robot_List->widget()->setLayout(new QVBoxLayout());
        ui_->scrollArea_Robot_List->widget()->layout()->setSpacing(scrollSpacing_);
        ui_->scrollArea_Robot_List->setWidgetResizable(true);

        ui_->ServerTab->currentChanged(Tab::DASHBOARD);
        ui_->ServerTab->setTabEnabled(Tab::ROBOT, false);

        displayTimer_ = new QTimer(this);
        connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotListDisp()));
        connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotNumDisp()));
        connect(displayTimer_, SIGNAL(timeout()), this, SLOT(robotTabDisp()));
        connect(displayTimer_, SIGNAL(timeout()), this, SLOT(modeButtonDisp()));
        connect(displayTimer_, SIGNAL(timeout()), this, SLOT(planButtonDisp()));

        connect(this, SIGNAL(addRobotSignal(QString)), this, SLOT(addRobotButton(QString)));

        displayTimer_->start(10);
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

    void Panel::update_robot_tab()
    {
        if (not(ui_->ServerTab->currentIndex() == Tab::ROBOT))
            return;

        if (not(instance_manager_->robots().contains(activatedRobot_)))
            return;

        Robot_ROS &robot_ros = instance_manager_->robots()[activatedRobot_];

        if (robot_ros.mode() == Mode::REMOTE)
        {
            geometry_msgs::msg::Twist remote_cmd_vel;
            {
                remote_cmd_vel.linear.x = robot_ros.robot().cur_vel_x();
                remote_cmd_vel.angular.z = robot_ros.robot().cur_vel_theta();
            }
            robot_ros.cmd_vel_pub()->publish(remote_cmd_vel);
        }
    }

    void Panel::update_rviz_poseArray()
    {
        if (buttons_.empty())
            return;

        visualization_msgs::msg::MarkerArray robotPoseMarkerArray;
        {
            robotPoseMarkerArray.markers.clear();

            for (const auto &robot_rosPair : instance_manager_->robots())
            {
                const Robot_ROS &robot_ros = robot_rosPair.second;

                if (robot_ros.prior_update_time().seconds() < 1e-8)
                    continue;

                auto robotPoseMarker = make_robotPoseMarker(robot_ros);

                if (robotPoseMarker.ns == "")
                    continue;

                robotPoseMarkerArray.markers.push_back(robotPoseMarker);
            }
        }

        rviz_poses_pub_->publish(robotPoseMarkerArray);
    }

    visualization_msgs::msg::Marker Panel::make_robotPoseMarker(const Robot_ROS &_robot_ros)
    {
        visualization_msgs::msg::Marker robotPoseMarker;

        robotPoseMarker.header.frame_id = "map";
        robotPoseMarker.header.stamp = _robot_ros.last_update_time();
        robotPoseMarker.ns = _robot_ros.robot().name();
        robotPoseMarker.id = _robot_ros.id();
        robotPoseMarker.type = visualization_msgs::msg::Marker::ARROW;
        robotPoseMarker.action = visualization_msgs::msg::Marker::ADD;

        robotPoseMarker.scale.x = 0.5;
        robotPoseMarker.scale.y = 0.125;
        robotPoseMarker.scale.z = 0.125;

        if (activatedRobot_ == _robot_ros.robot().name())
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

        robotPoseMarker.lifetime = _robot_ros.last_update_time() - _robot_ros.prior_update_time();

        const Robot &robot = _robot_ros.robot();

        robotPoseMarker.pose.position.x = robot.pose().x();
        robotPoseMarker.pose.position.y = robot.pose().y();
        robotPoseMarker.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, robot.pose().theta());
        robotPoseMarker.pose.orientation.x = q.x();
        robotPoseMarker.pose.orientation.y = q.y();
        robotPoseMarker.pose.orientation.z = q.z();
        robotPoseMarker.pose.orientation.w = q.w();

        return robotPoseMarker;
    }

    void Panel::register_robot(
        const std::shared_ptr<Connection::Request> _request,
        std::shared_ptr<Connection::Response> _response)
    {
        if (buttons_.contains(_request->config.name))
        {
            _response->is_connected = true;
            return;
        }

        Robot robot;
        {
            robot.name() = _request->config.name;
            robot.type() = _request->config.type;

            robot.goal().x() = _request->goal.x;
            robot.goal().y() = _request->goal.y;
            robot.goal().theta() = _request->goal.theta;

            robot.radius() = _request->config.size;

            robot.max_vel_x() = _request->config.max_linvel;
            robot.max_vel_theta() = _request->config.max_angvel;
            robot.acc_lim_x() = _request->config.max_linacc;
            robot.acc_lim_theta() = _request->config.max_angacc;
        }

        Robot_ROS robot_ros(robot);
        {
            robot_ros.mode() = Mode::MANUAL;

            robot_ros.last_update_time() = nh_->now();
            robot_ros.prior_update_time() = nh_->now();

            auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

            robot_ros.cmd_vel_pub() = nh_->create_publisher<geometry_msgs::msg::Twist>(
                "/" + robot.name() + "/cmd_vel", qos);
            robot_ros.cmd_vel_pub()->on_activate();

            robot_ros.kill_robot_cmd() = nh_->create_publisher<std_msgs::msg::Bool>(
                "/" + robot.name() + "/kill", qos);
            robot_ros.kill_robot_cmd()->on_activate();

            robot_ros.modeFromServer() = nh_->create_client<ModeSelection>(
                "/" + robot.name() + "/modeFromServer");

            robot_ros.modeFromRobot() = nh_->create_service<ModeSelection>(
                "/" + robot.name() + "/modeFromRobot",
                std::bind(&Panel::change_robot_mode, this, std::placeholders::_1, std::placeholders::_2));
        }

        instance_manager_->emplaceRobot(robot_ros);
        emit addRobotSignal(QString::fromStdString(_request->config.name));

        _response->is_connected = true;
    }

    void Panel::expire_robot(
        const std::shared_ptr<Disconnection::Request> _request,
        std::shared_ptr<Disconnection::Response> _response)
    {
        if (buttons_.contains(_request->name))
        {
            instance_manager_->deleteRobot(_request->name);
            deleteRobotButton(QString::fromStdString(_request->name));
        }

        _response->is_disconnected = true;
    }

    void Panel::change_robot_mode(
        const std::shared_ptr<ModeSelection::Request> _request,
        std::shared_ptr<ModeSelection::Response> _response)
    {
        if (not(instance_manager_->robots().contains(_request->name)))
        {
            _response->is_complete = false;
            return;
        }

        Robot_ROS &robot_ros = instance_manager_->robots()[_request->name];

        robot_ros.mode() = Mode::MANUAL;

        if (_request->is_remote == true)
            robot_ros.mode() = Mode::REMOTE;
        else if (_request->is_remote == false)
            robot_ros.mode() = Mode::MANUAL;

        _response->is_complete = true;
    }

} // namespace multibot2_server