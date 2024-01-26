#include "driver/isr_m2.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<multibot2_driver::isr_m2_driver::ISR_M2> isr_m2 = multibot2_driver::isr_m2_driver::ISR_M2::create();

    std::string port = isr_m2->get_parameter("port").get_parameter_value().get<std::string>();
    uint32_t baudrate = isr_m2->get_parameter("baudrate").get_parameter_value().get<uint32_t>();

    if (!isr_m2->ConnectRobot(port, baudrate))
        return 1;

    std::cout << "OK Ready" << std::endl;

    rclcpp::spin(isr_m2);
    rclcpp::shutdown();

    return 0;
}