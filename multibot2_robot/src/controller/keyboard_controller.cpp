#include "multibot2_robot/controller/controller_node.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto controller = std::make_shared<Controller::ControllerNode>();

    auto spinThread = std::async(
        [controller]()
        {
            rclcpp::spin(controller->get_node_base_interface());
            rclcpp::shutdown();
        });

    auto panelThread = std::async(
        [&argc, &argv, controller]()
        {
            controller->run_controller_panel(argc, argv);
        });

    spinThread.get();
    panelThread.get();

    return 0;
}