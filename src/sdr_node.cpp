#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sdr_odometry/sdr_node.hpp"

using namespace std::chrono_literals;

SDRNode::SDRNode() : Node("sdr_node")
{
    init();
}

SDRNode::SDRNode(const rclcpp::NodeOptions &options) : Node("sdr_node", options)
{
    init();
}

void SDRNode::init()
{
    this->declare_parameter("center_freq", 878000000); // 878 MHz
    this->declare_parameter("gain", 229);              // 22.9 dB
    this->declare_parameter("sample_rate", 2048000);   // 2.048 MSps

    int freq = this->get_parameter("center_freq").as_int();
    int gain = this->get_parameter("gain").as_int();
    int sample_rate = this->get_parameter("sample_rate").as_int();

    int dev_index = 0;
    if (rtlsdr_open(&dev_, dev_index) < 0)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open RTL-SDR device! Is it plugged in?");
        rclcpp::shutdown();
        return;
    }

    rtlsdr_set_sample_rate(dev_, sample_rate);
    rtlsdr_set_center_freq(dev_, freq);
    rtlsdr_set_tuner_gain_mode(dev_, 1); // Manual Gain (Critical for odometry)
    rtlsdr_set_tuner_gain(dev_, gain);
    rtlsdr_reset_buffer(dev_);

    RCLCPP_INFO(this->get_logger(), "SDR Started: %d Hz, Gain: %d", freq, gain);

    publisher_ = this->create_publisher<sdr_odometry::msg::SdrPacket>(
        "sdr/raw_iq",
        rclcpp::SensorDataQoS());

    buffer_size_ = 32 * 1024;

    timer_ = this->create_timer(1ms, std::bind(&SDRNode::read_and_publish, this));
}

SDRNode::~SDRNode()
{
    if (dev_ != nullptr)
    {
        rtlsdr_close(dev_);
        dev_ = nullptr;
    }
}

void SDRNode::read_and_publish()
{
    if (dev_ == nullptr)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "SDR device not initialized");
        return;
    }

    auto msg = std::make_unique<sdr_odometry::msg::SdrPacket>();

    msg->data.resize(buffer_size_);

    int n_read;
    int r = rtlsdr_read_sync(dev_, msg->data.data(), buffer_size_, &n_read);

    msg->header.stamp = this->now();
    msg->header.frame_id = "sdr_antenna_link";

    if (r < 0)
    {
        RCLCPP_WARN(this->get_logger(), "SDR Read failed");
    }
    else
    {
        if (n_read < buffer_size_)
        {
            msg->data.resize(n_read);
        }
        publisher_->publish(std::move(msg));
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(SDRNode)