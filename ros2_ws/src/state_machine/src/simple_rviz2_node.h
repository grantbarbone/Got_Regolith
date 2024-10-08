#ifndef SIMPLE_RVIZ2_NODE_H
#define SIMPLE_RVIZ2_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

class SimpleRviz2Node : public rclcpp::Node{
    public:
        SimpleRviz2Node(std::string Node);

    private:
        void publish_marker();
}

#endif