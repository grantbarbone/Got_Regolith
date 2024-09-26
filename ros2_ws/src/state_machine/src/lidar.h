#ifndef NEW_LIDAR_H
#define NEW_LIDAR_H

#include "rclcpp/rclcpp.hpp"

class Lidar : public rclcpp::Node{
    public:
        Lidar(std::string Node)

    private:
        void timer_callback();
        void topic_callback(const std_msgs::msg::Bool::SharedPtr msg);
}


#endif