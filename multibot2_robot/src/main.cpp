#include "multibot2_robot/robot_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto robot = std::make_shared<multibot2_robot::MultibotRobot>();
    
    auto spinThread = std::async(
        [robot]()
        {
            rclcpp::spin(robot->get_node_base_interface());
            rclcpp::shutdown();
        });

    auto panelThread = std::async(
        [&argc, &argv, robot]()
        {
            robot->run_robot_panel(argc, argv);
        });

    spinThread.get();
    panelThread.get();

    return 0;
}