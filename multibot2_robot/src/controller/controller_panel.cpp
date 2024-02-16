#include "multibot2_robot/controller/controller_panel.h"

using namespace std::chrono_literals;

namespace Controller
{
	ControllerPanel::ControllerPanel(rclcpp_lifecycle::LifecycleNode::SharedPtr &_nh, std::shared_ptr<Robot_ROS> _robot_ros,
									 QWidget *_parent)
		: QWidget(_parent), ui_(new Ui::RobotPanel),
		  nh_(_nh), robot_ros_(_robot_ros)
	{
		init_ui();

		auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

		robot_ros_->cmd_vel_pub() = nh_->create_publisher<geometry_msgs::msg::Twist>(
			std::string{nh_->get_namespace()} + "/cmd_vel", qos);
		robot_ros_->cmd_vel_pub()->on_activate();

		update_timer_ = nh_->create_wall_timer(
			10ms, std::bind(&ControllerPanel::manual_control, this));
	}

	void ControllerPanel::connectionDisp()
	{
		ui_->label_Connection_State->setText("Disconnected");
		ui_->label_Connection_State->setStyleSheet(
			"color: rgb(255,0,110);\nborder: none");
	}

	void ControllerPanel::modeDisp()
	{
		ui_->label_Mode_State->setText("Manual");
		ui_->label_Mode_State->setStyleSheet(
			"color: rgb(255,190,11);\nborder: none");
	}

	void ControllerPanel::velocityDisp()
	{
		const Robot &robot = robot_ros_->robot();

		QString lin_vel_qstring = QString::number(std::round(robot.cur_vel_x() * 100.0) / 100.0);
		QString ang_vel_qstring = QString::number(std::round(robot.cur_vel_theta() * 100.0) / 100.0);

		ui_->label_Linear_Velocity->setText(lin_vel_qstring);
		ui_->label_Angular_Velocity->setText(ang_vel_qstring);
	}

	void ControllerPanel::keyPressEvent(QKeyEvent *_event)
	{
		Robot &robot = robot_ros_->robot();

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

	void ControllerPanel::init_ui()
	{
		Robot &robot = robot_ros_->robot();

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

	void ControllerPanel::manual_control()
	{
		geometry_msgs::msg::Twist manual_cmd_vel;
		{
			manual_cmd_vel.linear.x = robot_ros_->robot().cur_vel_x();
			manual_cmd_vel.angular.z = robot_ros_->robot().cur_vel_theta();
		}

		robot_ros_->cmd_vel_pub()->publish(manual_cmd_vel);
	}
} // namespace multibot2_robot