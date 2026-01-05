#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include <rtl-sdr.h>

#include "rclcpp/rclcpp.hpp"
#include "sdr_odometry/msg/sdr_packet.hpp"

using namespace std::chrono_literals;

class SDRNode : public rclcpp::Node
{
public:
    SDRNode() : Node("sdr_node")
    {
        // --- CONFIGURATION ---
        this->declare_parameter("center_freq", 878000000); // 878 MHz
        this->declare_parameter("gain", 229);              // 22.9 dB
        this->declare_parameter("sample_rate", 2048000);   // 2.048 MSps

        int freq = this->get_parameter("center_freq").as_int();
        int gain = this->get_parameter("gain").as_int();
        int sample_rate = this->get_parameter("sample_rate").as_int();

        // --- SDR HARDWARE SETUP ---
        int dev_index = 0;
        if (rtlsdr_open(&dev, dev_index) < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to open RTL-SDR device! Is it plugged in?");
            rclcpp::shutdown();
            return;
        }

        rtlsdr_set_sample_rate(dev, sample_rate);
        rtlsdr_set_center_freq(dev, freq);
        rtlsdr_set_tuner_gain_mode(dev, 1); // Manual Gain (Critical for odometry)
        rtlsdr_set_tuner_gain(dev, gain);
        rtlsdr_reset_buffer(dev);

        RCLCPP_INFO(this->get_logger(), "SDR Started: %d Hz, Gain: %d", freq, gain);

        // --- PUBLISHER ---
        // Best Effort QoS to avoid queueing old data
        publisher_ = this->create_publisher<sdr_odometry::msg::SdrPacket>(
            "sdr/raw_iq",
            rclcpp::SensorDataQoS());

        // --- BUFFER OPTIMIZATION ---
        // 32 KB = ~8ms of data at 2.048 MSps.
        // This keeps latency low while avoiding USB choke.
        buffer_size_ = 32 * 1024;

        // Run the read loop as fast as possible (1ms timer triggers checking)
        timer_ = this->create_timer(1ms, std::bind(&SDRNode::read_and_publish, this));
    }

    ~SDRNode()
    {
        rtlsdr_close(dev);
    }

private:
    void read_and_publish()
    {
        // 1. Create unique_ptr for Zero Copy transport
        auto msg = std::make_unique<sdr_odometry::msg::SdrPacket>();

        // 2. Pre-allocate memory directly in the message
        msg->data.resize(buffer_size_);

        int n_read;

        // 3. READ DIRECTLY FROM DRIVER TO MESSAGE MEMORY
        // This blocks until 32KB is filled (~8ms)
        int r = rtlsdr_read_sync(dev, msg->data.data(), buffer_size_, &n_read);

        // 4. TIMESTAMP IMMEDIATELY
        // This time represents the END of the capture block.
        msg->header.stamp = this->now();
        msg->header.frame_id = "sdr_antenna_link";

        if (r < 0)
        {
            RCLCPP_WARN(this->get_logger(), "SDR Read failed");
        }
        else
        {
            // Handle partial reads (rare)
            if (n_read < buffer_size_)
            {
                msg->data.resize(n_read);
            }
            // 5. Publish
            publisher_->publish(std::move(msg));
        }
    }

    rtlsdr_dev_t *dev;
    rclcpp::Publisher<sdr_odometry::msg::SdrPacket>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int buffer_size_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SDRNode>());
    rclcpp::shutdown();
    return 0;
}