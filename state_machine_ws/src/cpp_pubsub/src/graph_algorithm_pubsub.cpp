#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/graph_algorithm_data.hpp>

class GraphAlgorithmPubSub : public rclcpp::Node {
public:
    GraphAlgorithmPubSub()
        : Node("graph_algorithm_pubsub")
    {
        // Set log level for this node
        this->get_logger().set_level(rclcpp::Logger::Level::Info);
        // Create a publisher for graph_algorithm data
        graph_algorithm_publisher_ = this->create_publisher<custom_msgs::msg::GraphAlgorithmData>("graph_algorithm_data", 10);

        // Create a subscriber for graph_algorithm data
        graph_algorithm_subscriber_ = this->create_subscription<custom_msgs::msg::GraphAlgorithmData>(
            "graph_algorithm_data",
            10,
            std::bind(&GraphAlgorithmPubSub::graph_algorithm_callback, this, std::placeholders::_1)
        );

        // Set logging level
        RCLCPP_INFO(this->get_logger(), "GraphAlgorithmPubSub node has been initialized.");
    }

private:
    void graph_algorithm_callback(const custom_msgs::msg::GraphAlgorithmData::SharedPtr msg) {
        custom_msgs::msg::GraphAlgorithmData response_msg;
        if (msg->x_coordinate == 5.0 && msg->y_coordinate == 5.0) {
            response_msg.x_coordinate = msg->x_coordinate;
            response_msg.y_coordinate = msg->y_coordinate;
        } else {
            response_msg.x_coordinate = 1.0;
            response_msg.y_coordinate = 1.0;
        }

        graph_algorithm_publisher_->publish(response_msg);

        RCLCPP_INFO(this->get_logger(), "Echoed graph_algorithm X_coordinate: %.2f", response_msg.x_coordinate);
        RCLCPP_INFO(this->get_logger(), "Echoed graph_algorithm Y_coordinate: %.2f", response_msg.y_coordinate);
    }

    rclcpp::Publisher<custom_msgs::msg::GraphAlgorithmData>::SharedPtr graph_algorithm_publisher_;
    rclcpp::Subscription<custom_msgs::msg::GraphAlgorithmData>::SharedPtr graph_algorithm_subscriber_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphAlgorithmPubSub>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
