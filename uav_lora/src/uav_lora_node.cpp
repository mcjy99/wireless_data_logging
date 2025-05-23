#include "rclcpp/rclcpp.hpp"
#include "uav_comms_msgs/msg/lora_info_stamped.hpp"
//#include "uav_comms_msgs/msg/lora_params_info_stamped.hpp"
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
    UavLoraNode() : Node("uav_lora_node")
    {
        this->declare_parameter<std::string>("port", "/dev/ttyTHS1"); //serial port
        port_ = this->get_parameter("port").as_string();
        this->declare_parameter<std::string>("sf", "7"); //spreading factor
        sf_ = this->get_parameter("sf").as_string();
        this->declare_parameter<std::string>("bw","125"); //bandwidth
        bw_ = this->get_parameter("bw").as_string();
        this->declare_parameter<std::string>("tp", "14"); //tx power
        tp_ = this->get_parameter("tp").as_string();

        data_publisher_ = this->create_publisher<uav_comms_msgs::msg::LoraInfoStamped>("lora_info", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&UavLoraNode::lora_receive, this));
        
        if (!setup_serial())
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to initialise serial port");
            return;
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

    bool setup_serial() // setup serial port
    {
        serial_port = open(port_.c_str(), O_RDWR);
        
        if (serial_port < 0)
        {
            RCLCPP_ERROR(this->get_logger(), "Error %i from open: %s", errno, strerror(errno));
            return false;
        }
        int flags = fcntl(serial_port,F_GETFL,0);
        fcntl(serial_port,F_SETFL,flags | O_NONBLOCK);
        
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

    void config_lora() // send initial commands to set params before receiving
    {
        serial_write("AT+MODE=TEST");
        std::string params = "AT+TEST=RFCFG,923,SF" + sf_ + "," + bw_ + ",8,8," + tp_ + ",ON,OFF,OFF";
        serial_write(params); //AT+TEST=RFCFG,923,SF12,125,8,8,14,ON,OFF,OFF
        //serial_write("AT+TEST"); //check params
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
    std::string port_;
    int serial_port;
    std::string sf_;
    std::string bw_;
    std::string tp_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<uav_comms_msgs::msg::LoraInfoStamped>::SharedPtr data_publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UavLoraNode>());
    rclcpp::shutdown();
    return 0;
}
