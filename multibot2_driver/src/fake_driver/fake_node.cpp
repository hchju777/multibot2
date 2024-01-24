#include "fake_driver/fake_odom.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto fake_odometry = std::make_shared<multibot2_driver::DiffDrive::FakeOdom>();
    
    rclcpp::spin(fake_odometry);
    rclcpp::shutdown();

    return 0;
}