#ifndef GRAPH_ALGORITHM_H
#define GRAPH_ALGORITHM_H

#include "rclcpp/rclcpp.hpp"

class RoboticPathPlanning{
    public:
        std::vector<std::vector<Vertex>> fillGridwithVertices(const std::vector<std::vector<float>>& grid);
        float euclidean_calculation(const Vertex& a,const Vertex& b);
        std::priority_queue<Vertex*, std::vector<Vertex*>, CompareVertex> getNeighbors(Vertex* node, Vertex* goal, std::vector<std::vector<Vertex>>& grid);
        std::vector<std::pair<float, float>> a_star_algorithm(Vertex* start, Vertex* goal, std::vector<std::vector<Vertex>> grid);
}

class GraphAlgorithm : public rclcpp::Node{
    public:
        GraphAlgorithm(std::string Node, int count_);

    private:
        void timer_callback();
        void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
}

#endif