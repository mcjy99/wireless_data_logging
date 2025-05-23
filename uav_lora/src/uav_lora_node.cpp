#include "rclcpp/rclcpp.hpp"
#include "uav_comms_msgs/msg/lora_info_stamped.hpp"
#include "uav_comms_msgs/msg/lora_params_info_stamped.hpp"
#include <stdio.h>
#include <string>
#include <iostream>
#include <unistd.h>  // write(), read(), close()
#include <fcntl.h>   // Contains file controls like O_RDWR
#include <errno.h>   // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <cstring>
#include <regex>     // Added for regex functionality
#include <thread>    // Added for sleep_for

using namespace std::chrono_literals;

class UavLoraNode : public rclcpp::Node
{
public:
    UavLoraNode() : Node("uav_lora_node"), serial_port(-1), actual_sf_(0), actual_tx_power_(0)
    {
        this->declare_parameter<std::string>("port", "/dev/ttyTHS1");
        port_ = this->get_parameter("port").as_string();
        this->declare_parameter<std::string>("sf", "7");
        sf_ = this->get_parameter("sf").as_string();
        this->declare_parameter<std::string>("tp", "14");
        tp_ = this->get_parameter("tp").as_string();

        params_publisher_ = this->create_publisher<uav_comms_msgs::msg::LoraParamsInfoStamped>("lora_params", 10);
        data_publisher_ = this->create_publisher<uav_comms_msgs::msg::LoraInfoStamped>("lora_info", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&UavLoraNode::lora_receive, this));
        params_timer_ = this->create_wall_timer(10s, std::bind(&UavLoraNode::publish_params, this));
        
        if (!setup_serial())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialise serial port");
            return;
        }
    }

    void publish_params()
    {
        // Only publish if we have valid configuration data
        if (actual_freq_ > 0 && actual_bw_ > 0 && actual_sf_ > 0 && actual_tx_power_ > 0)
        {
            auto config_message = uav_comms_msgs::msg::LoraParamsInfoStamped();
            config_message.header.stamp = this->now();
            config_message.header.frame_id = "lora_config";
            config_message.frequency = actual_freq_;
            config_message.bandwidth = actual_bw_;
            config_message.spreading_factor = actual_sf_;
            config_message.tx_power = actual_tx_power_;
            
            params_publisher_->publish(config_message);
            RCLCPP_DEBUG(this->get_logger(), "Published params - F:%u Hz, BW:%u kHz, SF:%u, TX Power:%u dBm", 
                       actual_freq_, actual_bw_, actual_sf_, actual_tx_power_);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Cannot publish params - configuration not available");
        }
        config_lora();
        RCLCPP_INFO(this->get_logger(), "LoRa logging node initialised");
    }

    ~UavLoraNode()
    {
        // close serial port
        if (serial_port >= 0)
        {
            close(serial_port);
        }
    }

private:
    std::string port_;
    std::string sf_;
    std::string tp_;
    int serial_port;
    // Class member variables for actual config values
    uint32_t actual_freq_;
    uint8_t actual_bw_;
    uint8_t actual_sf_;
    uint8_t actual_tx_power_;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr params_timer_;
    rclcpp::Publisher<uav_comms_msgs::msg::LoraParamsInfoStamped>::SharedPtr params_publisher_;
    rclcpp::Publisher<uav_comms_msgs::msg::LoraInfoStamped>::SharedPtr data_publisher_;

    bool setup_serial() // setup serial port
    {
        serial_port = open(port_.c_str(), O_RDWR);
        
        if (serial_port < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
            return false;
        }
        
        struct termios tty;
        memset(&tty, 0, sizeof tty);
        if (tcgetattr(serial_port, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcgetattr: %s\n", errno, strerror(errno));
            return false;
        }

        cfsetospeed(&tty, B9600); // in/out baud rate
        cfsetispeed(&tty, B9600);
        tty.c_cflag &= ~PARENB;                                                      // disable parity
        tty.c_cflag &= ~CSTOPB;                                                      // 1 stop bit
        tty.c_cflag &= ~CSIZE;                                                       // clear all size bits then set below
        tty.c_cflag |= CS8;                                                          // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;                                                     // disable hardware flow control
        tty.c_cflag |= CREAD | CLOCAL;                                               // turn on receiver
        tty.c_lflag &= ~ICANON;                                                      // Disable canonical mode
        tty.c_lflag &= ~ECHO;                                                        // Disable echo
        tty.c_lflag &= ~ECHOE;                                                       // Disable erasure
        tty.c_lflag &= ~ECHONL;                                                      // Disable newline echo
        tty.c_lflag &= ~ISIG;                                                        // Disable interpretation of INTR, QUIT and SUSP
        tty.c_iflag &= ~(IXON | IXOFF | IXANY);                                      // disable software flow control
        tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL); // Disable any special handling of received bytes
        tty.c_oflag &= ~OPOST;                                                       // prevent special interpretation of output bytes
        tty.c_oflag &= ~ONLCR;                                                       // Prevent conversion of newline to carriage return/line feed
        tty.c_cc[VMIN] = 0;                                                          // no minimum characters to read
        tty.c_cc[VTIME] = 10;                                                        // wait up to 1s (10 deciseconds) for data

        if (tcsetattr(serial_port, TCSANOW, &tty) != 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error %i from tcsetattr: %s\n", errno, strerror(errno));
            return false;
        }
        return true;
    }

    void config_lora()
    {
        // send initial commands
        serial_write("AT+MODE=TEST");
        std::string params = "AT+TEST=RFCFG,923,SF" + sf_ + ",125,8,8," + tp_ + ",ON,OFF,OFF";
        serial_write(params);
        
        // Send AT+TEST command and read the configuration response
        write(serial_port, "AT+TEST", 7);
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        
        char response[512];
        memset(response, 0, sizeof(response));
        int n = read(serial_port, response, sizeof(response));
        
        if (n > 0)
        {
            std::string config_response(response);
            RCLCPP_INFO(this->get_logger(), "%s", response);
            
            // Fixed regex patterns to match actual output format
            std::regex freq_pattern("F:(\\d+)");  // This will capture the full frequency
            std::smatch freq_matches;

            std::regex bandwidth_pattern("BW(\\d+)K");
            std::smatch bandwidth_matches;

            std::regex sf_pattern("SF(\\d+)");
            std::smatch sf_matches;
            
            std::regex pow_pattern("POW:(\\d+)dBm");
            std::smatch pow_matches;
            
            // Parse frequency (will be in Hz, e.g., 923000000)
            if (std::regex_search(config_response, freq_matches, freq_pattern) && freq_matches.size() >= 2)
            {
                actual_freq_ = std::stoul(freq_matches[1].str());
            }

            // Parse bandwidth (will be in kHz, e.g., 125)
            if (std::regex_search(config_response, bandwidth_matches, bandwidth_pattern) && bandwidth_matches.size() >= 2)
            {
                actual_bw_ = std::stoi(bandwidth_matches[1].str());
            }
            
            // Parse spreading factor
            if (std::regex_search(config_response, sf_matches, sf_pattern) && sf_matches.size() >= 2)
            {
                actual_sf_ = std::stoi(sf_matches[1].str());
            }
            
            // Parse TX power
            if (std::regex_search(config_response, pow_matches, pow_pattern) && pow_matches.size() >= 2)
            {
                actual_tx_power_ = std::stoi(pow_matches[1].str());
            }
            
            // Publish configuration message once
            if (actual_freq_ > 0 && actual_bw_ > 0 && actual_sf_ > 0 && actual_tx_power_ > 0)
            {
                RCLCPP_INFO(this->get_logger(), "LoRa configuration parsed successfully - F:%u Hz, BW:%u kHz, SF:%u, TX Power:%u dBm", 
                           actual_freq_, actual_bw_, actual_sf_, actual_tx_power_);
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Failed to parse LoRa params. F:%u, BW:%u, SF:%u, POW:%u", 
                           actual_freq_, actual_bw_, actual_sf_, actual_tx_power_);
            }
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "No response received from AT+TEST command");
        }
        
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        serial_write("AT+TEST=RXLRPKT");
    }

    void lora_receive()
    {
        std::string rx_data;
        uint16_t rx_length;
        int rssi = -255; //default to a very low value
        int snr = -255; 
        
        char buffer[256];
        memset(buffer, 0, sizeof(buffer));
        int n = read(serial_port, buffer, sizeof(buffer));
        if (n > 0)
        {
            std::string response(buffer);
            std::regex rssi_snr_pattern("\\+TEST: LEN:(\\d+), RSSI:(-?\\d+), SNR:(-?\\d+)");
            std::smatch matches;
            std::regex data_pattern("\\+TEST: RX \"([0-9A-Fa-f]+)\"");
            std::smatch data_match;

            if (std::regex_search(response, matches, rssi_snr_pattern) && matches.size() >= 4 && std::regex_search(response, data_match, data_pattern))
            {
                // Parse values
                try
                {   
                    rx_data = data_match[1].str();
                    rx_length = std::stoi(matches[1].str());
                    rssi = std::stoi(matches[2].str());
                    snr = std::stoi(matches[3].str());

                    auto message = uav_comms_msgs::msg::LoraInfoStamped();
                    message.header.stamp = this->now();
                    message.header.frame_id = "lora_data"; 
                    message.data = rx_data;
                    message.length = rx_length;
                    message.rssi = rssi;
                    message.snr = snr;
                    
                    data_publisher_->publish(message);
                    RCLCPP_INFO(this->get_logger(), "Data: %s, LEN: %d, RSSI: %d dBm, SNR: %d dB", 
                               rx_data.c_str(), rx_length, rssi, snr);
                }
                catch (const std::exception& e)
                {
                    RCLCPP_ERROR(this->get_logger(), "Error parsing values: %s", e.what());
                }
            }
        }
    }

    void serial_write(const std::string &serial_cmd)
    {
        write(serial_port, serial_cmd.c_str(), serial_cmd.length());
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        char response[256]; // read and print response
        memset(response, 0, sizeof(response));
        int n = read(serial_port, response, sizeof(response));
        if (n > 0)
        {
            RCLCPP_INFO(this->get_logger(), "%s", response);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Error reading from serial port");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UavLoraNode>());
    rclcpp::shutdown();
    return 0;
}