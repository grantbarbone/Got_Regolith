#ifndef MOTOR_CONTROLS_H
#define MOTOR_CONTROLS_H

#include <boost/asio.hpp>

class Serial_Protocal
{

    public:
        Serial_Protocal();
        void protocal_write(float angular_position, float angular_direction);
        float protocal_read(); 
    private:
        boost::asio::io_service io_service;
        boost::asio::serial_port serial;
}
#endif