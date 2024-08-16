#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class SimpleRviz2Node : public rclcpp::Node
{
public:
    SimpleRviz2Node() : Node("simple_rviz2_node")
    {
        // Create a publisher for the marker
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("visualization_marker", 10);

        // Timer to call the function periodically
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&SimpleRviz2Node::publish_marker, this)
        );
    }

private:
    void publish_marker()
    {
        // Create a marker message
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->get_clock()->now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;  // Cube shape
        marker.action = visualization_msgs::msg::Marker::ADD;

        // Set the pose of the marker
        marker.pose.position.x = 1.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;

        // Set the color
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        // Publish the marker
        marker_pub_->publish(marker);

        RCLCPP_INFO(this->get_logger(), "Published marker to RViz2.");
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleRviz2Node>());
    rclcpp::shutdown();
    return 0;
}