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

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class TestPublisher : public rclcpp::Node
{
public:
  TestPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TestPublisher::timer_callback, this));
    subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "topic", 10, std::bind(&TestPublisher::topic_callback, this, _1));
  }

private:
void timer_callback()
{
  auto message = std_msgs::msg::UInt16MultiArray();
  message.data.push_back(static_cast<uint16_t>(16)); // Example value for angular_position
  message.data.push_back(static_cast<uint16_t>(17)); // Example value for angular_direction

  RCLCPP_INFO(this->get_logger(), "Publishing angular_position: %u", message.data[0]);
  RCLCPP_INFO(this->get_logger(), "Publishing angular_direction: %u", message.data[1]);

  publisher_->publish(message);
}

void topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) const
  {
    if (msg->data.size() < 2) {
      RCLCPP_WARN(this->get_logger(), "Received data does not have enough elements");
      return;
    }

    uint16_t angular_position = msg->data[0];
    uint16_t angular_direction = msg->data[1];

    RCLCPP_INFO(this->get_logger(), "Received angular_position: %u", angular_position);
    RCLCPP_INFO(this->get_logger(), "Received angular_direction: %u", angular_direction);
}
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestPublisher>());
  rclcpp::shutdown();
  return 0;
}
