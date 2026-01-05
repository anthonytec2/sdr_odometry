#pragma once

#include <rtl-sdr.h>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sdr_odometry/msg/sdr_packet.hpp"

class SDRNode : public rclcpp::Node
{
public:
    SDRNode();
    explicit SDRNode(const rclcpp::NodeOptions &options);
    ~SDRNode();

private:
    void init();
    void read_and_publish();

    rtlsdr_dev_t *dev_{nullptr};
    rclcpp::Publisher<sdr_odometry::msg::SdrPacket>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int buffer_size_{0};
};
