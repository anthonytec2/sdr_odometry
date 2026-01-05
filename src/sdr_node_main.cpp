#include "rclcpp/rclcpp.hpp"

#include "sdr_odometry/sdr_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDRNode>(rclcpp::NodeOptions()));
    rclcpp::shutdown();
    return 0;
}
