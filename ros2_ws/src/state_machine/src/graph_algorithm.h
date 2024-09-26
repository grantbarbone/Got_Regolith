#ifndef GRAPH_ALGORITHM_H
#define GRAPH_ALGORITHM_H

#include "rclcpp/rclcpp.hpp"

class GraphAlgorithm : public rclcpp::Node{
    public:
        GraphAlgorithm(std::string Node, int count_);

    private:
        void timer_callback();
        void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
        }

#endif