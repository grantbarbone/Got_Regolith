#include <functional>
#include <memory>
//Ros 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include <cmath>

#define DEFAULT_TIMEOUT 15
using namespace std::chrono_literals;
using std::placeholders::_1;

class Serial_Protocal {
public:
    Serial_Protocal();
    void protocal_write(float angular_position, float angular_direction);
    uint32_t protocal_read();
private:
    boost::asio::io_service io_service;
    boost::asio::serial_port serial;
};

Serial_Protocal::Serial_Protocal(): serial(io_service) {
    try {
        std::cout << "Attempting to open serial port..." << std::endl;
        serial.open("/dev/ttyACM0");
        if (!serial.is_open()) {
            std::cerr << "Failed to open serial port." << std::endl;
            std::exit(EXIT_FAILURE);
        }
        std::cout << "Serial port opened successfully." << std::endl;

        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

void Serial_Protocal::protocal_write(float angular_position, float angular_direction) {
    try {
        char data[sizeof(float)* 2];
        std::memcpy(data , &angular_position, sizeof(float));
        std::memcpy(data + sizeof(float), &angular_direction, sizeof(float));
        
        boost::asio::write(serial, boost::asio::buffer(data, sizeof(data)));

        std::cout << "Data written to serial port." << std::endl;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error writing to serial port: " << e.what() << std::endl;
    }
}

uint32_t Serial_Protocal::protocal_read() {
    uint32_t data = 0;
    uint8_t bytes[4];

    std::cout << "Waiting for the data..." << std::endl;
    try {
        boost::asio::read(serial, boost::asio::buffer(bytes, 4));

        std::cout << "Data received from serial port." << std::endl;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error reading from serial port: " << e.what() << std::endl;
    }
    return data;
}

/**
 * @brief class for creating the ROS2 node for subscriber
 *
 */
class Motor_Controls : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Minimal Subscriber object
     *
     */
    Motor_Controls() : Node("Motor_Controls_PubSub"), count_(0), x_coordinate(0), y_coordinate(0) {
        publisher_ = this->create_publisher<std_msgs::msg::Bool>("topic_Motor_Controls_to_Main", 10);
        subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "topic_Main_to_Motor_Controls", 10, std::bind(&Motor_Controls::topic_callback, this, std::placeholders::_1));
    }

    void start_publishing() {
        motor_controls_timer_ = this->create_wall_timer(
            5ms, std::bind(&Motor_Controls::timer_callback, this)); // Restart the timer
    }

    void stop_publishing() {
        motor_controls_timer_->cancel(); // Stop the timer
    }

private:
    uint16_t float_to_uint16(float float_value, int decimal_length) {
        return static_cast<uint16_t>((static_cast<uint16_t>(float_value) << decimal_length) | 
                                    static_cast<uint16_t>((float_value - static_cast<uint16_t>(float_value)) * pow(10, decimal_length)));
    }

    void timer_callback() {
        auto message = std_msgs::msg::Bool();
        message.data = true;

       // RCLCPP_INFO(this->get_logger(), "Publishing to Node Graph_Algorithm: '%s'", message.data ? "true" : "false");
        publisher_->publish(message);
    }

    void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        std::cout << msg->data[0] << "x " << msg->data[1] <<std::endl;
        float delta_x = msg->data[0] - x_coordinate;
        float delta_y = msg->data[1] - y_coordinate;

        std::cout << delta_x << "delta x " << delta_y <<std::endl;

        x_coordinate = msg->data[0];
        y_coordinate = msg->data[1];

        std::cout << x_coordinate << "set x " << y_coordinate <<std::endl;
        float angular_position = static_cast<uint16_t>(std::sqrt(std::pow(delta_x, 2) + std::pow(delta_y, 2)));

        std::cout << angular_position <<" new angular position"<<std::endl;
        float angle_radians = std::atan2(delta_y, delta_x);
        std::cout << angle_radians << "new angular radians"<<std::endl;

      if (angular_position<2){
        float new_angular_direction = float (angle_radians * (180.0 / M_PI));
        std::cout << new_angular_direction << "new angular direction" << std::endl;
        if (delta_x < 0 ) {
            new_angular_direction += (delta_y < 0) ? 180 : 90;
        
        } else if (delta_y < 0) {
            new_angular_direction += 270;
        }
 
        RCLCPP_INFO(this->get_logger(), "sent motor control: %.2f d %.2f s",new_angular_direction, angular_position);
        Serial.protocal_write(new_angular_direction, angular_position);
      }
      else {
        if (delta_x < 0 ) {
            angle_radians += (delta_y < 0) ? M_PI : M_PI / 2.0;
        
        } else if (delta_y < 0) {
            angle_radians += (3*M_PI) / 2.0;
        }

        RCLCPP_INFO(this->get_logger(), "sent motor control: %.2f r %.2f s",angle_radians, angular_position);
        Serial.protocal_write(angle_radians, angular_position);
      }
       // Serial.protocal_write(angular_position, new_angular_direction);
        uint32_t received_data = Serial.protocal_read();
        uint16_t angular_position_received = received_data >> 16;
        uint16_t angular_direction_received = received_data & 0xFFFF;

        auto start_time = std::chrono::high_resolution_clock::now();
        start_publishing();
        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end_time - start_time;
        //RCLCPP_INFO(this->get_logger(), "Publishing time: %f seconds", elapsed.count());
        stop_publishing();


    }

    rclcpp::TimerBase::SharedPtr motor_controls_timer_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
    size_t count_;
    bool is_publishing_;
    uint16_t x_coordinate;
    uint16_t y_coordinate;
    Serial_Protocal Serial;
};

/**
 * @brief Initiate the subscriber
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Motor_Controls>();

    // Start publishing
    node->start_publishing();

    rclcpp::spin(node);
    node->stop_publishing(); // Stop publishing
    rclcpp::shutdown();
    return 0;
}