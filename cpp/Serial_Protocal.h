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

class Serial_Protocal { //class name
public: //public
    Serial_Protocal(); //constructor
    void protocal_write(float angular_position, float angular_direction); //prototype
    std::string  protocal_read(); //prototype
private: //private
    boost::asio::io_service io_service; //io_service
    boost::asio::serial_port serial;    //serial
}; //endclass

Serial_Protocal::Serial_Protocal(): serial(io_service) { // class constructor
    try { //try
        std::cout << "Attempting to open serial port..." << std::endl;
        serial.open("/dev/ttyACM0"); //create the serial open
        if (!serial.is_open()) { //open serial
            std::cerr << "Failed to open serial port." << std::endl;
            std::exit(EXIT_FAILURE); //exit the failure
        } //endif
        std::cout << "Serial port opened successfully." << std::endl;

        serial.set_option(boost::asio::serial_port_base::baud_rate(115200)); //serial set the baud rate
        serial.set_option(boost::asio::serial_port_base::character_size(8)); //character size 
        serial.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none)); //set the flow control
        serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none)); //set the parity
        serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one)); //set the stopping bit

        std::this_thread::sleep_for(std::chrono::milliseconds(3000)); //sleep for 3000
    } catch (const boost::system::system_error& e) { //set the catch
        std::cerr << "Error: " << e.what() << std::endl; //set the cerr.
        std::exit(EXIT_FAILURE); //set the exit
    } //endcatch
} //endconstructor

void Serial_Protocal::protocal_write(float angular_position, float angular_direction) { //set the angular position, and angular direction
    try { //try
        char data[sizeof(float)* 2]; //data
        std::memcpy(data , &angular_position, sizeof(float)); //copy the angular position into data.
        std::memcpy(data + sizeof(float), &angular_direction, sizeof(float)); //copy the angular direction into the data.
        boost::asio::write(serial, boost::asio::buffer(data, sizeof(data))); //write to serial
        std::cout << "Data written to serial port." << std::endl; //write the data.
    } catch (const boost::system::system_error& e) { //catch
        std::cerr << "Error writing to serial port: " << e.what() << std::endl; //end the error writing
    } //endcatch
} //endfunc

std::string Serial_Protocal::protocal_read() { //start protocol read
   #define BUFSIZE 1 //set the buffer size

    boost::asio::io_service io; //io
    // Open serial port
    boost::asio::serial_port serial(io, "/dev/ttyACM0"); //set the serial port

    // Configure basic serial port parameters: 115.2kBaud, 8N1
    serial.set_option(boost::asio::serial_port_base::baud_rate(115200)); //set the baud rate
    serial.set_option(boost::asio::serial_port_base::character_size(8 /* data bits */)); //set teh character size
    serial.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none)); //set the parity
    serial.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one)); //set the stopping bit

    // Read data in a loop and copy to stdout
    
    char data[BUFSIZE]; //set the data bufsize
    size_t n = serial.read_some(boost::asio::buffer(data, BUFSIZE)); //read the data to n
 
    // Write data to stdout
    std::cout.write(data, n); //print the data, and n.
    return std::string(data,n); //return the string data, n
} //endfunc

#endif