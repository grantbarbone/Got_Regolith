#ifndef MAIN_H
#define MAIN_H

#include "std_msgs/msg/u_int16_multi_array.hpp"
#include <vector>

class Main : public rclcpp::Node{
    public:
        Main(std::string Node);
        void start_publishing_zed();
        void stop_publishing_zed();
        void start_publishing_lidar();
        void stop_publishing_lidar();
        void start_publishing_motor_controls();
        void stop_publishing_motor_controls();
        void start_publishing_graph_algorithm();
        void stop_publishing_graph_algorithm();
        void set_zed_connection(bool Switch);
        bool get_zed_connection();
        void set_lidar_connection(bool Switch);
        bool get_lidar_connection();
        void set_zed_data(std::vector<float> zed_data);
        std::vector<float> get_zed_data();

    private:
        void zed_timer_callback();
        void lidar_timer_callback();
        void graph_algorithm_timer_callback();
        void motor_controls_timer_callback();
        void zed_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
        void lidar_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
        void graph_algorithm_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
        void motor_controls_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);
}
#endif