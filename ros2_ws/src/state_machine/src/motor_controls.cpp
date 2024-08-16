// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file subscriber_member_function.cpp
 * @author Jay Prajapati (jayp@umd.edu)
 * @brief create a minimal subscriber for ROS2
 * @version 0.1
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <functional>
#include <memory>
//Ros 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#include <cmath>
// Your code here


#define DEFAULT_TIMEOUT 15
using namespace std::chrono_literals;
using std::placeholders::_1;

class Serial_Protocal {
public:
    Serial_Protocal();
    void protocal_write(uint16_t angular_position, uint16_t angular_direction);
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

void Serial_Protocal::protocal_write(uint16_t angular_position, uint16_t angular_direction) {
uint32_t data = ((uint32_t)angular_position << 16) | angular_direction;
    uint8_t bytes[4];
    bytes[0] = (data >> 24) & 0xFF;
    bytes[1] = (data >> 16) & 0xFF;
    bytes[2] = (data >> 8) & 0xFF;
    bytes[3] = data & 0xFF;

    try {
        boost::asio::write(serial, boost::asio::buffer(bytes, 4));
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
        data = ((uint32_t)bytes[1] << 24) |
               ((uint32_t)bytes[0] << 16) |
               ((uint32_t)bytes[3] << 8) |
               bytes[2];
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
  
  Motor_Controls() : Node("Motor_Controls_PubSub"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("topic_Motor_Controls_to_Main", 10);
    subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
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
  /**
   * @brief callback function for the topic
   *
   * @param msg
   */


  Serial_Protocal Serial;
  void timer_callback() {
    auto message = std_msgs::msg::Bool();
    message.data = true;

    RCLCPP_INFO(this->get_logger(), "Publishing to Node Graph_Algorithm: '%s'", message.data ? "true" : "false");
    publisher_->publish(message);
  }

  void topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    Serial.protocal_write(msg->data[0], msg->data[1]);
    // uint32_t received_data = Serial.protocal_read();
    // uint16_t angular_position=received_data >>16;
    // uint16_t angular_direction=received_data >>32;
    // std::cout << "Received angular_position: " << std::hex << angular_position << std::endl;
    // std::cout << "Received angular_direction: " << std::hex << angular_direction << std::endl;
    uint16_t next_x_coordinate= msg->data[0];
    uint16_t next_y_coordinate= msg->data[1];
    x_coordinate=next_x_coordinate-x_coordinate;
    y_coordinate=next_y_coordinate-y_coordinate;
    uint16_t angular_position=std::sqrt(std::pow(x_coordinate,2)+std::pow(y_coordinate,2));
    
    double angle_radians = atan2(y_coordinate, x_coordinate);
      
    std::cout<< angle_radians <<std::endl;
    // Convert the angle to degrees
    uint16_t angular_directoin = angle_radians * (180.0 / M_PI);
    if (x_coordinate<0  &&  !(y_coordinate<0)){
      angular_directoin= angular_directoin+90;
    }
    else if (x_coordinate<0 && y_coordinate<0 ){
      angular_directoin= angular_directoin+180;
    }
    else if (!(x_coordinate<0) && y_coordinate<0){
      angular_directoin= angular_directoin+270;
    }
    RCLCPP_INFO(this->get_logger(), "Received angular_position and angular_direction: 0x%04X 0x%04X", angular_position, angular_directoin);
    auto start_time = std::chrono::high_resolution_clock::now();
    start_publishing();
    auto end_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end_time - start_time;
    RCLCPP_INFO(this->get_logger(), "Publishing time: %f seconds", elapsed.count());  
    stop_publishing();
  }
  rclcpp::TimerBase::SharedPtr motor_controls_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscription_;
  size_t count_;
  bool is_publishing_;
  uint16_t x_coordinate=0;
  uint16_t y_coordinate=0;
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

  // Example of toggling publishing
  //node->start_publishing(); // Start publishing

  rclcpp::spin(std::make_shared<Motor_Controls>());
  //node->stop_publishing(); // Stop publishing
  rclcpp::shutdown();
  return 0;
}