#include "multibot2_robot/robot_panel.h"

using namespace std::chrono_literals;

namespace multibot2_robot
{
	Panel::Panel(nav2_util::LifecycleNode::SharedPtr &_nh, Instance_Manager::SharedPtr &_instance_manager,
				 QWidget *_parent)
		: QWidget(_parent), ui_(new Ui::RobotPanel),
		  nh_(_nh), instance_manager_(_instance_manager)
	{
		init_ui();

		Robot &robot = instance_manager_->robot();
		robot.cur_vel_x() = 0.0;
		robot.cur_vel_theta() = 0.0;

		auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

		connection_ = nh_->create_client<Connection>(
			"/server/connection");

		disconnection_ = nh_->create_client<Disconnection>(
			"/server/disconnection");

		serverScan_ = nh_->create_subscription<std_msgs::msg::Bool>(
			"/server/scan", qos,
			std::bind(&Panel::respond_to_serverScan, this, std::placeholders::_1));

		emergencyStop_ = nh_->create_subscription<std_msgs::msg::Bool>(
			"/server/emergency_stop", qos,
			std::bind(&Panel::respond_to_emergencyStop, this, std::placeholders::_1));

		kill_robot_cmd_ = nh_->create_subscription<std_msgs::msg::Bool>(
			std::string{nh_->get_namespace()} + "/kill", qos,
			std::bind(&Panel::respond_to_kill, this, std::placeholders::_1));

		modeFromServer_ = nh_->create_service<ModeSelection>(
			std::string{nh_->get_namespace()} + "/modeFromServer",
			std::bind(&Panel::change_robot_mode, this, std::placeholders::_1, std::placeholders::_2));

		modeFromRobot_ = nh_->create_client<ModeSelection>(
			std::string{nh_->get_namespace()} + "/modeFromRobot");

		update_timer_ = nh_->create_wall_timer(
			10ms, std::bind(&Panel::manual_control, this));
	}

	void Panel::connectionDisp()
	{
		if (connection_state_)
		{
			ui_->label_Connection_State->setText("Connected");
			ui_->label_Connection_State->setStyleSheet(
				"color: rgb(0,255,51);\nborder: none");
		}
		else
		{
			ui_->label_Connection_State->setText("Disconnected");
			ui_->label_Connection_State->setStyleSheet(
				"color: rgb(255,0,110);\nborder: none");
		}
	}

	void Panel::modeDisp()
	{
		const Robot_ROS &robot_ros = instance_manager_->robot_ros();

		switch (robot_ros.mode())
		{
		case Mode::REMOTE:
			ui_->label_Mode_State->setText("Remote");
			ui_->label_Mode_State->setStyleSheet(
				"color: rgb(0,213,255);\nborder: none");
			break;

		case Mode::MANUAL:
			ui_->label_Mode_State->setText("Manual");
			ui_->label_Mode_State->setStyleSheet(
				"color: rgb(255,190,11);\nborder: none");
			break;

		case Mode::AUTO:
			ui_->label_Mode_State->setText("Auto");
			ui_->label_Mode_State->setStyleSheet(
				"color: rgb(230, 19, 237);\nborder: none");

		default:
			break;
		}
	}

	void Panel::velocityDisp()
	{
		const Robot &robot = instance_manager_->robot();

		QString lin_vel_qstring = QString::number(std::round(robot.cur_vel_x() * 100.0) / 100.0);
		QString ang_vel_qstring = QString::number(std::round(robot.cur_vel_theta() * 100.0) / 100.0);

		ui_->label_Linear_Velocity->setText(lin_vel_qstring);
		ui_->label_Angular_Velocity->setText(ang_vel_qstring);
	}

	void Panel::on_pushButton_Connect_clicked()
	{
		if (connection_state_ == false and request_connection())
			connection_state_ = true;
	}

	void Panel::on_pushButton_Disconnect_clicked()
	{
		if (connection_state_ == true and request_disconnection())
		{
			connection_state_ = false;

			Robot_ROS &robot_ros = instance_manager_->robot_ros();

			robot_ros.mode() = Mode::MANUAL;

			robot_ros.robot().cur_vel_x() = 0.0;
			robot_ros.robot().cur_vel_theta() = 0.0;
		}
	}

	void Panel::on_pushButton_Remote_clicked()
	{
		if (connection_state_ == false)
			return;

		if (request_modeChange(Mode::REMOTE))
			instance_manager_->robot_ros().mode() = Mode::REMOTE;
	}

	void Panel::on_pushButton_Manual_clicked()
	{
		if (connection_state_ == false)
			return;

		if (request_modeChange(Mode::MANUAL))
		{
			Robot_ROS &robot_ros = instance_manager_->robot_ros();

			robot_ros.mode() = Mode::MANUAL;

			robot_ros.robot().cur_vel_x() = 0.0;
			robot_ros.robot().cur_vel_theta() = 0.0;
		}
	}

	void Panel::keyPressEvent(QKeyEvent *_event)
	{
		if (not(instance_manager_->robot_ros().mode() == Mode::MANUAL))
			return;

		Robot &robot = instance_manager_->robot();

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
			robot.cur_vel_theta() -= 0.1;
			break;

		case Qt::Key_Space:
			robot.cur_vel_x() = 0.0;
			robot.cur_vel_theta() = 0.0;
			break;

		default:
			break;
		}
	}

	void Panel::init_ui()
	{
		Robot &robot = instance_manager_->robot();

		ui_->setupUi(this);
		ui_->label_Robot_Name->setText(QString::fromStdString(robot.name()));

		setFocusPolicy(Qt::StrongFocus);

		setWindowFlags(
			Qt::WindowStaysOnTopHint |
			Qt::Window |
			Qt::WindowTitleHint |
			Qt::CustomizeWindowHint |
			Qt::WindowMinimizeButtonHint);

		displayTimer_ = new QTimer(this);
		connect(displayTimer_, SIGNAL(timeout()), this, SLOT(connectionDisp()));
		connect(displayTimer_, SIGNAL(timeout()), this, SLOT(modeDisp()));
		connect(displayTimer_, SIGNAL(timeout()), this, SLOT(velocityDisp()));

		displayTimer_->start(10);
	}

	void Panel::manual_control()
	{
		Robot_ROS &robot_ros = instance_manager_->robot_ros();

		if (not(robot_ros.mode() == Mode::MANUAL))
			return;

		geometry_msgs::msg::Twist manual_cmd_vel;
		{
			manual_cmd_vel.linear.x = robot_ros.robot().cur_vel_x();
			manual_cmd_vel.angular.z = robot_ros.robot().cur_vel_theta();
		}

		robot_ros.cmd_vel_pub()->publish(manual_cmd_vel);
	}

	bool Panel::request_connection()
	{
		while (not(connection_->wait_for_service(1s)))
		{
			if (not(rclcpp::ok()))
			{
				RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
				return false;
			}
			RCLCPP_ERROR(nh_->get_logger(), "Connection not available, waiting again...");
		}

		auto request = std::make_shared<Connection::Request>();

		const Robot &robot = instance_manager_->robot();

		request->config.name = robot.name();
		request->config.type = robot.type();

		request->goal.x = robot.goal().x();
		request->goal.y = robot.goal().y();
		request->goal.theta = robot.goal().theta();

		request->config.size = robot.radius();

		request->config.max_linvel = robot.max_vel_x();
		request->config.max_angvel = robot.max_vel_theta();
		request->config.max_linacc = robot.acc_lim_x();
		request->config.max_angacc = robot.acc_lim_theta();

		auto response_received_callback = [this](rclcpp::Client<Connection>::SharedFuture _future)
		{
			auto response = _future.get();
			return;
		};

		auto future_result =
			connection_->async_send_request(request, response_received_callback);

		return future_result.get()->is_connected;
	}

	bool Panel::request_disconnection()
	{
		while (not(disconnection_->wait_for_service(1s)))
		{
			if (not(rclcpp::ok()))
			{
				RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
				return false;
			}
			RCLCPP_ERROR(nh_->get_logger(), "Disconnection not available, waiting again...");
		}

		auto request = std::make_shared<Disconnection::Request>();

		request->name = instance_manager_->robot().name();

		auto response_received_callback = [this](rclcpp::Client<Disconnection>::SharedFuture _future)
		{
			auto response = _future.get();
			return;
		};

		auto future_result =
			disconnection_->async_send_request(request, response_received_callback);

		return future_result.get()->is_disconnected;
	}

	bool Panel::request_modeChange(Mode _mode)
	{
		while (not(modeFromRobot_->wait_for_service(1s)))
		{
			if (not(rclcpp::ok()))
			{
				RCLCPP_ERROR(nh_->get_logger(), "Interrupted while waiting for the service.");
				return false;
			}
			RCLCPP_ERROR(nh_->get_logger(), "ModeChange not available, waiting again...");
		}

		auto request = std::make_shared<ModeSelection::Request>();

		request->name = instance_manager_->robot().name();

		if (_mode == Mode::REMOTE)
			request->is_remote = true;
		else if (_mode == Mode::MANUAL)
			request->is_remote = false;

		auto response_received_callback = [this](rclcpp::Client<ModeSelection>::SharedFuture _future)
		{
			auto response = _future.get();
			return;
		};

		auto future_result =
			modeFromRobot_->async_send_request(request, response_received_callback);

		return future_result.get()->is_complete;
	}

	void Panel::respond_to_serverScan(const std_msgs::msg::Bool::SharedPtr _msg)
	{
		if (_msg->data == false)
			return;

		ui_->pushButton_Connect->clicked(true);
	}

	void Panel::respond_to_emergencyStop(const std_msgs::msg::Bool::SharedPtr _msg)
	{
		if (_msg->data == false)
			return;

		ui_->pushButton_Manual->clicked(true);
	}

	void Panel::respond_to_kill(const std_msgs::msg::Bool::SharedPtr _msg)
	{
		if (_msg->data == false)
			return;

		ui_->pushButton_Disconnect->clicked(true);
	}

	void Panel::change_robot_mode(
		const std::shared_ptr<ModeSelection::Request> _request,
		std::shared_ptr<ModeSelection::Response> _response)
	{
		Robot_ROS &robot_ros = instance_manager_->robot_ros();

		if (_request->name != robot_ros.robot().name())
			abort();
		
		if (_request->is_remote == true)
		{
			robot_ros.mode() = Mode::REMOTE;
		}
		else
		{
			robot_ros.mode() = Mode::MANUAL;

			robot_ros.robot().cur_vel_x() = 0.0;
			robot_ros.robot().cur_vel_theta() = 0.0;
		}

		_response->is_complete = true;
	}
} // namespace multibot2_robot