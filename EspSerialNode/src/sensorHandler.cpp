#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>

#include <sstream>
#include <string>
#include <vector>

#include "cerberus_msgs/msg/sensor_data.hpp"
#include "cerberus_msgs/msg/esp_commands.hpp"

class SensorReaderNode : public rclcpp::Node {
public:
    SensorReaderNode() : Node("sensor_reader_node") {
        // Declare parameters
        this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);

        this->get_parameter("serial_port", port_);
        this->get_parameter("baudrate", baudrate_);

        // Test serial connection
        try {
            serial_.setPort(port_);
            serial_.setBaudrate(baudrate_);

            serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
            serial_.setTimeout(timeout);

            serial_.open();
        } catch (serial::IOException& e) {
            RCLCPP_ERROR(this->get_logger(), "Unable to open serial port: %s", port_.c_str());
            rclcpp::shutdown();
            return;
        }

        if (serial_.isOpen()) {
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully", port_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            rclcpp::shutdown();
            return;
        }

        sensor_publisher_ = this->create_publisher<cerberus_msgs::msg::SensorData>("sensors_data", 10);
        command_publisher_ = this->create_publisher<cerberus_msgs::msg::EspCommands>("esp_commands", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&SensorReaderNode::readSerial, this)
        );
    }

    private:
    std::string serial_buffer_;

    void readSerial() {
        if (serial_.available()) {
            std::string new_data = serial_.read(serial_.available());
            serial_buffer_ += new_data;

            size_t newline_pos;
            // Find out if we have one full line of data in the buffer
            while ((newline_pos = serial_buffer_.find('\n')) != std::string::npos) {
                std::string line = serial_buffer_.substr(0, newline_pos);
                serial_buffer_.erase(0, newline_pos + 1);

                if (line.empty()) {
                    continue;
                }

                std::vector<float> parsed_data;
                std::istringstream ss(line);
                std::string token;

                while (std::getline(ss, token, ',')) {
                    if (token.empty()) continue;
                    try {
                        parsed_data.push_back(std::stof(token));
                    } catch (const std::exception& e) {
                        RCLCPP_WARN(this->get_logger(), "Invalid float received: '%s', skipping line", token.c_str());
                        return;
                    }
                }

                if (parsed_data.size() >= 13) {
                    cerberus_msgs::msg::SensorData msg;
                    msg.accel_x = parsed_data[0];
                    msg.accel_y = parsed_data[1];
                    msg.accel_z = parsed_data[2];
                    msg.gyro_x = parsed_data[3];
                    msg.gyro_y = parsed_data[4];
                    msg.gyro_z = parsed_data[5];
                    msg.heading = parsed_data[6];
                    msg.latitude = parsed_data[7];
                    msg.longitude = parsed_data[8];
                    msg.altitude = parsed_data[9];

                    sensor_publisher_->publish(msg);

                    cerberus_msgs::msg::EspCommands command_msg;
                    command_msg.calibrate = parsed_data[10];
                    command_msg.startup = parsed_data[11];
                    command_msg.shutdown = parsed_data[12];

                    command_publisher_->publish(command_msg);
                } else {
                    RCLCPP_WARN(this->get_logger(), "Incomplete data received (%ld values)", parsed_data.size());
                }
            }
        }
    }

    std::string port_;
    int baudrate_;
    serial::Serial serial_;
    rclcpp::Publisher<cerberus_msgs::msg::SensorData>::SharedPtr sensor_publisher_;
    rclcpp::Publisher<cerberus_msgs::msg::EspCommands>::SharedPtr command_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
  
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorReaderNode>());
    rclcpp::shutdown();
    return 0;
}