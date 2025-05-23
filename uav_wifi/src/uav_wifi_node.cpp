#include <chrono>
#include <cstring>
#include <iostream>
#include <array>
#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "uav_comms_msgs/msg/wifi_info_stamped.hpp"

using namespace std::chrono_literals;

class UavWifiNode : public rclcpp::Node
{
public:
    UavWifiNode() : Node("uav_wifi_node")
    {
        this->declare_parameter<std::string>("interface", "wlP1p1s0");
        interface_name_ = this->get_parameter("interface").as_string();

        publisher_ = this->create_publisher<uav_comms_msgs::msg::WifiInfoStamped>("wifi_info", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&UavWifiNode::scan_wifi, this));
    }

private:
    std::string execute_cmd(const std::string& cmd) {
        FILE *pipe = popen(cmd.c_str(), "r");
        if (!pipe)
        {
            RCLCPP_ERROR(this->get_logger(), "popen() failed!");
            return "";
        }

        char buffer[128];
        std::string result = "";
        while (!feof(pipe))
        {
            if (fgets(buffer, 128, pipe) != NULL)
            {
                result += buffer;
            }
        }
        pclose(pipe);
        return result;
    }

    void scan_wifi()
    {
        std::string cmd_link = "iw dev " + interface_name_ + " link";
        std::string result_link = execute_cmd(cmd_link);
        std::string cmd_info = "iw dev " + interface_name_ + " info";
        std::string result_info = execute_cmd(cmd_info);
        
        int signal_strength = -255; // Default to very low values
        float rx_rate = 0.00;
        float tx_rate = 0.00;
        float tx_power = 0.00;

        // Check if not connected
        if (result_link.find("Not connected.") != std::string::npos)
        {
            RCLCPP_WARN(this->get_logger(), "WiFi connection not found");
        }
        // Parse signal strength if connected
        else
        {
            size_t pos_signal = result_link.find("signal:");
            if (pos_signal != std::string::npos)
            {
                int parsed_signal;
                if (sscanf(result_link.c_str() + pos_signal, "signal: %d", &parsed_signal) == 1)
                {
                    signal_strength = parsed_signal;
                }
            }

            size_t pos_rx = result_link.find("rx bitrate:");
            if (pos_rx != std::string::npos){
                float parsed_rx_rate;
                if (sscanf(result_link.c_str() + pos_rx, "rx bitrate: %f", &parsed_rx_rate) == 1){
                    rx_rate = parsed_rx_rate;
                }
            }

            size_t pos_tx = result_link.find("tx bitrate:");
            if (pos_tx != std::string::npos){
                float parsed_tx_rate;
                if (sscanf(result_link.c_str() + pos_tx, "tx bitrate: %f", &parsed_tx_rate) == 1){
                    tx_rate = parsed_tx_rate;
                }
            }
            
            size_t pos_pwr = result_info.find("txpower");
            if (pos_pwr != std::string::npos){
                float parsed_tx_pwr;
                if (sscanf(result_info.c_str() + pos_pwr, "txpower %f", &parsed_tx_pwr)==1){
                    tx_power = parsed_tx_pwr;
                }
            }
        }

        // Create and publish the message
        auto message = uav_comms_msgs::msg::WifiInfoStamped();
        message.header.stamp = this->now();
        message.header.frame_id = "wifi"; 
        message.rssi_signal = signal_strength;
        message.rx_rate = rx_rate;
        message.tx_rate = tx_rate;
        message.tx_power = tx_power;
        publisher_->publish(message);

        RCLCPP_DEBUG(this->get_logger(), "Published WiFi info: signal strength: %d dBm, rx bitrate: %f MBit/s, tx bitrate: %f MBit/s, txpower: %f dBm", 
                     signal_strength, rx_rate, tx_rate, tx_power);
    }

    std::string interface_name_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<uav_comms_msgs::msg::WifiInfoStamped>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UavWifiNode>());
    rclcpp::shutdown();
    return 0;
}