#include "fake_driver/fake_odom.h"

#include <chrono>

#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

namespace multibot2_driver::DiffDrive
{
    FakeOdom::FakeOdom()
        : Node("FakeOdom")
    {
        init_parameters();
        init_varibales();

        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));

        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/" + namespace_ + "/odom", qos);
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/" + namespace_ + "/joint_states", qos);
        tf_pub_ = this->create_publisher<tf2_msgs::msg::TFMessage>("/tf", qos);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/" + namespace_ + "/cmd_vel", qos,
            std::bind(&FakeOdom::command_velocity_callback, this, std::placeholders::_1));

        update_timer_ = this->create_wall_timer(
            10ms, std::bind(&FakeOdom::update_callback, this));

        RCLCPP_INFO(this->get_logger(), "DiffDrive Fake Odometry has been initialized");
    }

    FakeOdom::~FakeOdom()
    {
        RCLCPP_INFO(this->get_logger(), "DiffDrive Fake Odometry has been terminated");
    }

    void FakeOdom::command_velocity_callback(
        const geometry_msgs::msg::Twist::SharedPtr _cmd_vel_msg)
    {
        prev_cmd_vel_time_ = this->now();

        linear_velocity_ = _cmd_vel_msg->linear.x;
        angular_velocity_ = _cmd_vel_msg->angular.z;

        wheel_speed_[WHEEL::LEFT] = linear_velocity_ - (angular_velocity_ * wheel_seperation_ / 2);
        wheel_speed_[WHEEL::RIGHT] = linear_velocity_ + (angular_velocity_ * wheel_seperation_ / 2);
    }

    void FakeOdom::update_callback()
    {
        static rclcpp::Time update_time = this->now();
        // rclcpp::Duration duration(this->now().nanoseconds() - update_time.nanoseconds());
        rclcpp::Duration duration = rclcpp::Duration::from_nanoseconds(this->now().nanoseconds() - update_time.nanoseconds());

        if ((update_time.nanoseconds() - prev_cmd_vel_time_.nanoseconds()) * 1e-9 > cmd_vel_timeout_)
        {
            wheel_speed_[WHEEL::LEFT] = 0.0;
            wheel_speed_[WHEEL::RIGHT] = 0.0;
        }

        calculate_odometry(duration);
        publish(update_time);

        update_time = this->now();
    }

    void FakeOdom::init_parameters()
    {
        if (not(this->has_parameter("robot.wheels.seperation")))
            this->declare_parameter("robot.wheels.seperation", wheel_seperation_);
        if (not(this->has_parameter("robot.wheels.radius")))
            this->declare_parameter("robot.wheels.radius", wheel_radius_);

        if (not(this->has_parameter("namespace")))
            this->declare_parameter("namespace", namespace_);
        if (not(this->has_parameter("robot.odometry.frame_id")))
            this->declare_parameter("robot.odometry.frame_id", frame_id_of_odometry_);
        if (not(this->has_parameter("robot.odometry.child_frame_id")))
            this->declare_parameter("robot.odometry.child_frame_id", child_frame_id_of_odometry_);

        if (not(this->has_parameter("use_gazebo_odom")))
            this->declare_parameter("use_gazebo_odom", use_gazebo_odom_);

        this->get_parameter_or("robot.wheels.seperation", wheel_seperation_, wheel_seperation_);
        this->get_parameter_or("robot.wheels.radius", wheel_radius_, wheel_radius_);

        this->get_parameter_or("namespace", namespace_, namespace_);
        this->get_parameter_or("robot.odometry.frame_id", frame_id_of_odometry_, frame_id_of_odometry_);
        this->get_parameter_or("robot.odometry.child_frame_id", child_frame_id_of_odometry_, child_frame_id_of_odometry_);

        this->get_parameter_or("use_gazebo_odom", use_gazebo_odom_, use_gazebo_odom_);
    }

    void FakeOdom::init_varibales()
    {
        frame_id_of_odometry_ = namespace_ + "/" + frame_id_of_odometry_;
        child_frame_id_of_odometry_ = namespace_ + "/" + child_frame_id_of_odometry_;

        robot_pose_ = {0.0, 0.0, 0.0};
        robot_vel_ = {0.0, 0.0, 0.0};

        wheel_speed_[WHEEL::LEFT] = 0.0;
        wheel_speed_[WHEEL::RIGHT] = 0.0;

        last_velocity_[WHEEL::LEFT] = 0.0;
        last_velocity_[WHEEL::RIGHT] = 0.0;

        last_position_[WHEEL::LEFT] = 0.0;
        last_position_[WHEEL::RIGHT] = 0.0;

        cmd_vel_timeout_ = 1.0;
    }

    void FakeOdom::calculate_odometry(const rclcpp::Duration &_duration)
    {
        std::array<double, 2> w = {0.0, 0.0};
        {
            w[WHEEL::LEFT] = wheel_speed_[WHEEL::LEFT] / wheel_radius_;
            w[WHEEL::RIGHT] = wheel_speed_[WHEEL::RIGHT] / wheel_radius_;
        }

        last_velocity_[WHEEL::LEFT] = w[WHEEL::LEFT];
        last_velocity_[WHEEL::RIGHT] = w[WHEEL::RIGHT];

        double step_time = _duration.nanoseconds() * 1e-9;
        double wheel_l = w[WHEEL::LEFT] * step_time;
        double wheel_r = w[WHEEL::RIGHT] * step_time;

        if (std::isnan(wheel_l))
            wheel_l = 0.0;

        if (std::isnan(wheel_r))
            wheel_r = 0.0;

        last_position_[WHEEL::LEFT] += wheel_l;
        last_position_[WHEEL::RIGHT] += wheel_r;

        double delta_s = wheel_radius_ * (wheel_r + wheel_l) / 2.0;
        double delta_theta = wheel_radius_ * (wheel_r - wheel_l) / wheel_seperation_;

        robot_pose_[0] += delta_s * cos(robot_pose_[2] + (delta_theta / 2.0));
        robot_pose_[1] += delta_s * sin(robot_pose_[2] + (delta_theta / 2.0));
        robot_pose_[2] += delta_theta;

        if (std::fabs(step_time) > 1e-8)
        {
            robot_vel_[0] = delta_s / step_time;
            robot_vel_[1] = 0.0;
            robot_vel_[2] = delta_theta / step_time;
        }
    }

    void FakeOdom::publish(const rclcpp::Time &_now)
    {
        auto odom_msg = std::make_unique<nav_msgs::msg::Odometry>();
        {
            odom_msg->header.frame_id = frame_id_of_odometry_;
            odom_msg->child_frame_id = child_frame_id_of_odometry_;
            odom_msg->header.stamp = _now;

            odom_msg->pose.pose.position.x = robot_pose_[0];
            odom_msg->pose.pose.position.y = robot_pose_[1];
            odom_msg->pose.pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, robot_pose_[2]);

            odom_msg->pose.pose.orientation.x = q.getX();
            odom_msg->pose.pose.orientation.y = q.getY();
            odom_msg->pose.pose.orientation.z = q.getZ();
            odom_msg->pose.pose.orientation.w = q.getW();

            odom_msg->twist.twist.linear.x = robot_vel_[0];
            odom_msg->twist.twist.angular.z = robot_vel_[2];
        }

        auto joint_states_msg = std::make_unique<sensor_msgs::msg::JointState>();
        {
            joint_states_msg->header.frame_id = child_frame_id_of_odometry_;
            joint_states_msg->header.stamp = _now;

            joint_states_msg->name.push_back("left_wheel_joint");
            joint_states_msg->name.push_back("right_wheel_joint");

            joint_states_msg->position.resize(2, 0.0);
            joint_states_msg->velocity.resize(2, 0.0);
            joint_states_msg->effort.resize(2, 0.0);

            joint_states_msg->position[WHEEL::LEFT] = last_position_[WHEEL::LEFT];
            joint_states_msg->position[WHEEL::RIGHT] = last_position_[WHEEL::RIGHT];

            joint_states_msg->velocity[WHEEL::LEFT] = last_velocity_[WHEEL::LEFT];
            joint_states_msg->velocity[WHEEL::RIGHT] = last_velocity_[WHEEL::RIGHT];
        }

        auto odom_tf_msg = std::make_unique<tf2_msgs::msg::TFMessage>();
        {
            geometry_msgs::msg::TransformStamped odom_tf;

            odom_tf.header = odom_msg->header;
            odom_tf.child_frame_id = child_frame_id_of_odometry_;

            odom_tf.transform.translation.x = odom_msg->pose.pose.position.x;
            odom_tf.transform.translation.y = odom_msg->pose.pose.position.y;
            odom_tf.transform.translation.z = odom_msg->pose.pose.position.z;

            odom_tf.transform.rotation = odom_msg->pose.pose.orientation;

            odom_tf_msg->transforms.push_back(odom_tf);
        }

        if (not(use_gazebo_odom_))
            odom_pub_->publish(std::move(odom_msg));

        joint_states_pub_->publish(std::move(joint_states_msg));
        tf_pub_->publish(std::move(odom_tf_msg));
    }
}