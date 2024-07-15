#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/lidar_data.hpp>

class LidarPublisherSubscriber : public rclcpp::Node {
public:
    LidarPublisherSubscriber()
        : Node("lidar_pubsub")
    {
        // Create a publisher for lidar data
        lidar_publisher_ = this->create_publisher<custom_msgs::msg::LidarData>("lidar_data", 10);

        // Create a subscriber for lidar data
        lidar_subscriber_ = this->create_subscription<custom_msgs::msg::LidarData>(
            "lidar_data",
            10,
            std::bind(&LidarPublisherSubscriber::lidar_callback, this, std::placeholders::_1)
        );

        // Set logging level
        this->get_logger().set_level(rclcpp::Logger::Level::Info);
    }

private:
    void lidar_callback(const custom_msgs::msg::LidarData::SharedPtr msg) {
        // Process lidar data
        bool found = false;

        for (float value : msg->ranges) {
            if (std::fabs(value - 20.0) < 0.1) {
                found = true;
                break;
            }
        }

        custom_msgs::msg::LidarData response_msg;
        if (found) {
            // Echo back the received range
            response_msg.ranges = msg->ranges;
        } else {
            // Echo a default range if not found
            response_msg.ranges.push_back(10.0);
        }

        // Publish the response message
        lidar_publisher_->publish(response_msg);

        // Log the echoed range(s)
        if (!response_msg.ranges.empty()) {
            RCLCPP_INFO(this->get_logger(), "Echoed lidar range: %.2f", response_msg.ranges[0]);
        } else {
            RCLCPP_INFO(this->get_logger(), "No valid range found to echo.");
        }
    }

    rclcpp::Publisher<custom_msgs::msg::LidarData>::SharedPtr lidar_publisher_;
    rclcpp::Subscription<custom_msgs::msg::LidarData>::SharedPtr lidar_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarPublisherSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
