#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <functional>
#include <memory>
#include <random>
#include <sl/Camera.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib> // For std::rand and RAND_MAX
#include <ctime>   // For std::time
#include <chrono>
#include <string>
#include <queue>
#include <limits>
#include <unordered_map>
#include <unordered_set>

class Serial_Protocal {
public:
    Serial_Protocal();
    void protocal_write(float angular_position, float angular_direction);
    std::string  protocal_read();
private:
    boost::asio::io_service io_service;
    boost::asio::serial_port serial;
};

Serial_Protocal::Serial_Protocal(): serial(io_service) {
    try {
        std::cout << "Attempting to open serial port..." << std::endl;
        serial.open("/dev/ttyACM0");
        if (!serial.is_open()) {
            std::cerr << "Failed to open serial port." << std::endl;
            std::exit(EXIT_FAILURE);
        }
        std::cout << "Serial port opened successfully." << std::endl;

        serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
        serial.set_option(boost::asio::serial_port_base::character_size(8));
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

        std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        std::exit(EXIT_FAILURE);
    }
}

void Serial_Protocal::protocal_write(float angular_position, float angular_direction) {
    try {
        char data[sizeof(float)* 2];
        std::memcpy(data , &angular_position, sizeof(float));
        std::memcpy(data + sizeof(float), &angular_direction, sizeof(float));
        
        boost::asio::write(serial, boost::asio::buffer(data, sizeof(data)));

        std::cout << "Data written to serial port." << std::endl;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error writing to serial port: " << e.what() << std::endl;
    }
}

std::string Serial_Protocal::protocal_read() {
   #define BUFSIZE 1

    boost::asio::io_service io;
    // Open serial port
    boost::asio::serial_port serial(io, "/dev/ttyACM0");

    // Configure basic serial port parameters: 115.2kBaud, 8N1
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200));
    serial.set_option(boost::asio::serial_port_base::character_size(8 /* data bits */));
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));

    // Read data in a loop and copy to stdout
    
    char data[BUFSIZE];
    size_t n = serial.read_some(boost::asio::buffer(data, BUFSIZE));
 
    // Write data to stdout
    std::cout.write(data, n);
    return std::string(data,n);
}

#endif