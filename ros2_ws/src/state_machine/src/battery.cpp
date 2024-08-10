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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief class for creating the ROS2 node for subscriber
 *
 */
class Battery : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   *
   */
  
  Battery() : Node("Battery_PubSub"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic_battery_to_main", 10);
    //publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("topic", 10);
    battery_timer_ = this->create_wall_timer(
        500ms, std::bind(&Battery::timer_callback, this));
        
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic_main_to_battery", 10, std::bind(&Battery::topic_callback, this, std::placeholders::_1));
  }

 private:
  /**
   * @brief callback function for the topic
   *
   * @param msg
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Message from Node Battery: " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  void topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received from Node Main: '%s'", msg->data.c_str());
  }
  rclcpp::TimerBase::SharedPtr battery_timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t count_;
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
  rclcpp::spin(std::make_shared<Battery>());
  rclcpp::shutdown();
  return 0;
}
