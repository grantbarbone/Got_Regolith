#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class GraphAlgorithmNode : public rclcpp::Node {
public:
    GraphAlgorithmNode() : Node("graph_algorithm_node") {
        RCLCPP_INFO(this->get_logger(), "GraphAlgorithmNode Node has been started.");
        // Your node logic here
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GraphAlgorithmNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
