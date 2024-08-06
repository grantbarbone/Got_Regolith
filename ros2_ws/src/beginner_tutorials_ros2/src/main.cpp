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
 * @file publisher_member_function.cpp
 * @author Jay Prajapati (jayp@umd.edu)
 * @brief create a minimal publisher for ROS2
 * @version 0.1
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/**
 * @brief class for creating the ROS2 node for publisher
 *
 */
class Main : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   */
  Main() : Node("Main_PubSub"), count_(0) {
    zed_publisher_ = this->create_publisher<std_msgs::msg::Bool>("topic_Main_to_Zed", 10);
    zed_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::zed_timer_callback, this));

    zed_subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "topic_Zed_to_Main", 10, std::bind(&Main::zed_topic_callback, this, std::placeholders::_1));
    
    lidar_publisher_ = this->create_publisher<std_msgs::msg::Bool>("topic_Main_to_Lidar", 10);
    
    lidar_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::lidar_timer_callback, this));
    
    lidar_subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "topic_Lidar_to_Main", 10, std::bind(&Main::lidar_topic_callback, this, std::placeholders::_1));  

    radar_publisher_ = this->create_publisher<std_msgs::msg::Bool>("topic_Main_to_Radar", 10);
    
    radar_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::radar_timer_callback, this));
    
    radar_subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "topic_Radar_to_Main", 10, std::bind(&Main::radar_topic_callback, this, std::placeholders::_1)); 

    graph_algorithm_publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("topic_Main_to_Graph_Algirthm", 10);
    graph_algorithm_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::graph_algorithm_timer_callback, this));
    graph_algorithm_subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "topic_Graph_Algorithm_to_Main", 10, std::bind(&Main::graph_algorithm_topic_callback, this, std::placeholders::_1));  
    
    motor_controls_publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("topic_Main_to_Motor_Controls", 10);
    
    motor_controls_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::motor_controls_timer_callback, this));
    
    motor_controls_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "topic_Motor_Controls_to_Main", 10, std::bind(&Main::motor_controls_topic_callback, this, std::placeholders::_1));

    
    battery_publisher_ = this->create_publisher<std_msgs::msg::String>("topic_Main_to_Battery", 10);
    
    battery_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::battery_timer_callback, this));
    
    battery_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic_Battery_to_Main", 10, std::bind(&Main::battery_topic_callback, this, std::placeholders::_1));     
  }

 private:
  /**
   * @brief callback function
   *
   */

  std::string data;
  void set_data(const std::string& input_data) {
     data=input_data;// or perform some processing
  }
  std::string get_data() {
    return data; // or perform some processing
  }

  void zed_timer_callback() {
    auto message = std_msgs::msg::Bool();
    message.data = true;
    RCLCPP_INFO(this->get_logger(), "Publishing to Node Zed: '%s'", message.data ? "true" : "false");
    zed_publisher_->publish(message);
  }

  void lidar_timer_callback() {
    auto message = std_msgs::msg::Bool();
    message.data = true;
    RCLCPP_INFO(this->get_logger(), "Publishing to Node Lidar: '%s'", message.data ? "true" : "false");
    lidar_publisher_->publish(message);
  }

  void radar_timer_callback() {
    auto message = std_msgs::msg::Bool();
    message.data = true;
    RCLCPP_INFO(this->get_logger(), "Publishing to Node Radar: '%s'", message.data ? "true" : "false");
    radar_publisher_->publish(message);
  }

  void graph_algorithm_timer_callback() {
    auto message = std_msgs::msg::UInt16MultiArray();
    message.data = {1, 2, 3, 4, 5};
    RCLCPP_INFO(this->get_logger(), "Publishing to Node Graph_Algorithm: [%s]", std::to_string(message.data.size()).c_str());
    graph_algorithm_publisher_->publish(message);
  }
  void motor_controls_timer_callback() {
    auto message = std_msgs::msg::UInt16MultiArray();
    message.data = {1, 2};
    RCLCPP_INFO(this->get_logger(), "Publishing to Node Motor_Controls: [%s]", std::to_string(message.data.size()).c_str());
    motor_controls_publisher_->publish(message);
  }

  void battery_timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Message to Node Battery: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    battery_publisher_->publish(message);
  }
  
  void zed_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: [%s]", std::to_string(msg->data.size()).c_str());
  }
  
  void lidar_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: [%s]", std::to_string(msg->data.size()).c_str());
  }

  void radar_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: [%s]", std::to_string(msg->data.size()).c_str());
  }

  void graph_algorithm_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: [%s]", std::to_string(msg->data.size()).c_str());
  }
  void motor_controls_topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Publishing to Node Zed: '%s'", msg->data ? "true" : "false");
  }

  void battery_topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received from Node Battery: '%s'", msg->data.c_str() );
  }
  rclcpp::TimerBase::SharedPtr zed_timer_;
  rclcpp::TimerBase::SharedPtr lidar_timer_;
  rclcpp::TimerBase::SharedPtr radar_timer_;
  rclcpp::TimerBase::SharedPtr graph_algorithm_timer_;
  rclcpp::TimerBase::SharedPtr motor_controls_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zed_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr zed_subscription_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr lidar_subscription_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr radar_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr radar_subscription_;

  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr graph_algorithm_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr graph_algorithm_subscription_;

  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr motor_controls_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motor_controls_subscription_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr battery_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr battery_subscription_;


  size_t count_;
};

/**
 * @brief Initiate the publisher
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Main>());
  rclcpp::shutdown();
  return 0;
}
