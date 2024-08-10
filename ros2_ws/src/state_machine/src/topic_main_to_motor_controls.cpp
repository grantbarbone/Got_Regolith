

#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;
class TestSubscriber : public rclcpp::Node
{
public:
  TestSubscriber()
  : Node("minimal_subscriber")
  {
    publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&TestSubscriber::timer_callback, this));
    subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "topic", 10, std::bind(&TestSubscriber::topic_callback, this, _1));
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

  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TestSubscriber>());
  rclcpp::shutdown();
  return 0;
}
