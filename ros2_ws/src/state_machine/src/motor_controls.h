#ifndef MOTOR_CONTROLS_H
#define MOTOR_CONTROLS_H

#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"

class Serial_Protocal
{
    public:
        Serial_Protocal();
        void protocal_write(float angular_position, float angular_direction);
        uint32_t protocal_read(); 

    private:
        boost::asio::io_service io_service;
        boost::asio::serial_port serial;
}

class Motor_Controls : public rclcpp::Node {
    public:
        Motor_Controls(std::string Node, int count_, int x_coordinate, int y_coordinate);
        void start_publishing();
        void stop_publishing();

    private:    
        uint16_t float_to_uint16(float, int);
        void timer_callback();
        void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg);
}

#endif