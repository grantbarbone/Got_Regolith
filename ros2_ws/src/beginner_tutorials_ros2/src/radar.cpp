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
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

/**
 * @brief class for creating the ROS2 node for subscriber
 *
 */
class Radar : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   *
   */
  
  Radar() : Node("Radar_PubSub"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("topic_radar_to_main", 10);
    radar_timer_ = this->create_wall_timer(
        500ms, std::bind(&Radar::timer_callback, this));
        
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "topic_main_to_radar", 10, std::bind(&Radar::topic_callback, this, std::placeholders::_1));
  }

 private:
  /**
   * @brief callback function for the topic
   *
   * @param msg
   */
  void timer_callback() {
    auto message = std_msgs::msg::UInt16MultiArray();
    message.data = {1, 2, 3, 4, 5};
    RCLCPP_INFO(this->get_logger(), "Publishing: [%s]", std::to_string(message.data.size()).c_str());
    publisher_->publish(message);
  }

  void topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data ? "true" : "false");
  }
  rclcpp::TimerBase::SharedPtr radar_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
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
  rclcpp::spin(std::make_shared<Radar>());
  rclcpp::shutdown();
  return 0;
}
