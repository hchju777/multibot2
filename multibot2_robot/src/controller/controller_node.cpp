#include "multibot2_robot/controller/controller_node.h"

namespace Controller
{
    ControllerNode::ControllerNode()
        : rclcpp_lifecycle::LifecycleNode("keyboard_controller")
    {
        robot_ros_ = std::make_shared<Robot_ROS>();
        Robot &robot = robot_ros_->robot();

        robot.name() = "Controller";
        robot.cur_vel_x() = 0.0;
        robot.cur_vel_theta() = 0.0;

        RCLCPP_INFO(this->get_logger(), "Keyboard controller has been initialized");
    }

    ControllerNode::~ControllerNode()
    {
        RCLCPP_INFO(this->get_logger(), "Keyboard controller has been terminated");
    }

    void ControllerNode::run_controller_panel(int argc, char *argv[])
    {
        nh_ = std::shared_ptr<::rclcpp_lifecycle::LifecycleNode>(this, [](::rclcpp_lifecycle::LifecycleNode *) {});

        QApplication app(argc, argv);

        controller_panel_ = std::make_shared<ControllerPanel>(nh_, robot_ros_);
        controller_panel_->show();

        app.exec();
    }
} // namespace Controller