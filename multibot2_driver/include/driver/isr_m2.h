#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "multibot2_msgs/msg/driver_status_stamped.hpp"
#include "multibot2_msgs/srv/driver_command.hpp"

#include <boost/asio.hpp>
#include <boost/array.hpp>

#define WHEEL_RADIUS_M 0.155 // Unit:m
#define WHEEL_BASE_M 0.531	 // Unit:m
#define WHEEL_WIDTH_M 0.102	 // Unit:m
#define ENCODER_PPR 6400.0	 // Unit: pulse/rev
#define GEAR_RATIO 31.778	 // Gearhead reduction ratio: 26 (26:1), Spurgear reduction ratio: 1.22 (44:36)
#define MPS2RPM 61.608		 // same as (60 /(2 * M_PI * WHEEL_RADIUS_M))
#define MAX_RPM 4650.0

using namespace std::placeholders;
using namespace std::chrono_literals;

namespace multibot2_driver::isr_m2_driver
{
	class ISR_M2 : public rclcpp::Node
	{
	public:
		using RobotStatusStamped = multibot2_msgs::msg::DriverStatusStamped;
		using RobotCommand = multibot2_msgs::srv::DriverCommand;

	public:
		const ISR_M2 &operator=(const ISR_M2 &) = delete;
		ISR_M2(const ISR_M2 &) = delete;

		static std::shared_ptr<ISR_M2> create()
		{
			auto isr_m2_shared_ptr = std::shared_ptr<ISR_M2>(new ISR_M2);
			isr_m2_shared_ptr->weak_self_ = isr_m2_shared_ptr;
			return isr_m2_shared_ptr;
		}

		auto createQuaternionMsgFromYaw(double yaw)
		{
			tf2::Quaternion q;
			q.setRPY(0, 0, yaw);
			return tf2::toMsg(q);
		}

		bool ConnectRobot(const std::string &port, const uint32_t baudrate);
		void DisconnectRobot(void);

		bool Initialize(void);

		bool EnableMotors(void);
		bool DisableMotors(void);
		bool StopMotors(void);
		bool ResumeMotors(void);

		bool SetVelocity(double leftWheelVel_MPS, double rightWheelVel_MPS); // m/s, m/s
		bool SetVelocityVW(double linearVel_MPS, double angularVel_RPS);	 // m/s, rad/s
		bool ReadVelocity_ADCApprox();

		bool ReadEncoder(void);
		bool ResetRobotPos(void);

		bool ReadRobotStatus(uint8_t &motorEnableStatus, uint8_t &motorStopStatus, uint8_t &emergencyButtonPressed);

	public:
		rclcpp::Time prev_encoder_time_; // ms
		rclcpp::Time cur_encoder_time_;	 // ms

		long left_encoder_;	 // A encoder value of left wheel (Unit: pulse)
		long right_encoder_; // A encoder value of right wheel (Unit: pulse)

		double del_dist_left_m_;  // left_encoder_ - prev_leftEncoder
		double del_dist_right_m_; // right_encoder_ - prev_rightEncoder

		double left_wheel_vel_endoer_mps_;
		double right_wheel_vel_encoder_mps_;

		bool left_wheel_direction_reading_;
		bool right_wheel_direction_reading_;
		double left_wheel_vel_reading_mps_;
		double right_wheel_vel_reading_mps_;

		struct Position
		{
			double x;
			double y;
			double theta;
		} position_; // Robot pose calculated by dead-reckoning (Unit: m, m, rad)

		struct Velocity
		{
			double v;
			double w;
		} velocity_; // Robot velocity calculated by dead-reckoning (Unit: m, m, rad)u

	private:
		std::weak_ptr<ISR_M2> weak_self_;
		ISR_M2()
			: Node("isr2_m2_driver_node"), serial_(io)
		{
			// Declare and acquire serial port parameter
			this->declare_parameter<std::string>("port", "/dev/ttyACM0");
			this->declare_parameter<int>("baudrate", 115200);

			this->declare_parameter("odom_frame", odom_frame_);
			this->declare_parameter("base_frame", base_frame_);

			this->get_parameter_or("odom_frame", odom_frame_, odom_frame_);
			this->get_parameter_or("base_frame", base_frame_, base_frame_);

			// Initialize cmd_vel msg subscription
			cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 10, std::bind(&ISR_M2::cmd_vel_callback, this, _1));
			cmd_vel_msg_ = std::make_shared<geometry_msgs::msg::Twist>();
			cmd_vel_msg_->linear.x;
			cmd_vel_msg_->angular.z;

			// Initialize odometry transform broadcaster
			odom_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

			// Initialize odometry msg publisher
			odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

			// Initialize robot status msg publisher
			robot_status_pub_ = this->create_publisher<RobotStatusStamped>("robot_status", 10);
			robot_status_msg_.header.stamp = this->now();
			robot_status_msg_old_.header.stamp = this->now();

			// Initialize robot command service server
			robot_cmd_srv_ = this->create_service<RobotCommand>(
				"robot_cmd", std::bind(&ISR_M2::robot_cmd_callback, this, _1, _2));
		}

		void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
		{
			cmd_vel_msg_ = msg;
		}

		void robot_cmd_callback(const std::shared_ptr<RobotCommand::Request> request,
								std::shared_ptr<RobotCommand::Response> response)
		{
			while (rclcpp::ok())
			{
				if (this->serial_io_mut_.try_lock())
				{
					response->result = RobotCommand::Response::RESULT_FAIL;
					switch (request->command)
					{
					case RobotCommand::Request::COMMAND_INITIALIZE:
						RCLCPP_INFO(this->get_logger(), "Initialize motor");
						if (Initialize())
							response->result = RobotCommand::Response::RESULT_SUCCESS;
						else
							RCLCPP_ERROR(this->get_logger(), "Initialize motor failed");
						break;
					case RobotCommand::Request::COMMAND_ENABLE_MOTOR:
						if (request->val == 0)
						{
							RCLCPP_INFO(this->get_logger(), "Enable motor");
							if (EnableMotors())
								response->result = RobotCommand::Response::RESULT_SUCCESS;
							else
								RCLCPP_ERROR(this->get_logger(), "Enable motor failed");
						}
						else
						{
							RCLCPP_INFO(this->get_logger(), "Disable motor");
							if (DisableMotors())
								response->result = RobotCommand::Response::RESULT_SUCCESS;
							else
								RCLCPP_ERROR(this->get_logger(), "Disable motor failed");
						}
						break;
					case RobotCommand::Request::COMMAND_STOP_MOTOR:
						if (request->val == 0)
						{
							RCLCPP_INFO(this->get_logger(), "Stop motor");
							if (StopMotors())
								response->result = RobotCommand::Response::RESULT_SUCCESS;
							else
								RCLCPP_ERROR(this->get_logger(), "Stop motor failed");
						}
						else
						{
							RCLCPP_INFO(this->get_logger(), "Resume motor");
							if (ResumeMotors())
								response->result = RobotCommand::Response::RESULT_SUCCESS;
							else
								RCLCPP_ERROR(this->get_logger(), "Resume motor failed");
						}
						break;
					default:
						break;
					}

					this->serial_io_mut_.unlock();
					break;
				}
			}
		}

		// serial io
		boost::asio::io_service io;
		boost::asio::serial_port serial_; /// @brief Actual serial port object for reading/writing to robot

		// serial io thread
		rclcpp::TimerBase::SharedPtr timer_;
		std::mutex serial_io_mut_;

		// cmd_vel subscription
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
		geometry_msgs::msg::Twist::SharedPtr cmd_vel_msg_;

		// odometry transform broadcaster
		std::unique_ptr<tf2_ros::TransformBroadcaster> odom_tf_broadcaster_;

		// odometry publisher
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

		// robot status publisher
		rclcpp::Publisher<RobotStatusStamped>::SharedPtr robot_status_pub_;
		RobotStatusStamped robot_status_msg_;
		RobotStatusStamped robot_status_msg_old_;

		// robot command service server
		rclcpp::Service<RobotCommand>::SharedPtr robot_cmd_srv_;

		// robot frame_id
		std::string odom_frame_{std::string("odom")};
		std::string base_frame_{std::string("base_link")};

		/**
		 * @brief DeadReckoning
		 * Do not need to use this func. 'SetVel' func calls this function.
		 */
		void DeadReckoning(long dl, long dr); // Do not need to use this func. 'SetVel' func calls this function.

		bool SendData(uint8_t command, uint8_t numparam, uint8_t *params);
		std::vector<uint8_t> ReceiveData(uint8_t &command);

		std::vector<uint8_t> Word2Bytes(int16_t dat);
		std::vector<uint8_t> Long2Bytes(int32_t dat);
		int16_t Bytes2Word(uint8_t *data);
		int32_t Bytes2Long(uint8_t *data);
	}; // class ISR_M2

} // namespace multibot2_driver::isr_m2_driver