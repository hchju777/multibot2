#include <iostream>

#include "multibot2_server/server_node.h"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto server = std::make_shared<multibot2_server::MultibotServer>();
    
    auto spinThread = std::async(
        [server]()
        {
            rclcpp::spin(server->get_node_base_interface());
            rclcpp::shutdown();
        });

    auto panelThread = std::async(
        [&argc, &argv, server]()
        {
            server->run_server_panel(argc, argv);
        });

    spinThread.get();
    panelThread.get();

    return 0;
}