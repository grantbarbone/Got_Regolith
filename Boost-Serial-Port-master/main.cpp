
#include <iostream>
#include <thread>
#include <chrono>
#include <boost/asio.hpp>
#define DEFAULT_TIMEOUT 15

class Serial_Protocal {
public:
    Serial_Protocal();
    void protocal_write(uint16_t angular_position, uint16_t angular_direction);
    uint32_t protocal_read();
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

void Serial_Protocal::protocal_write(uint16_t angular_position, uint16_t angular_direction) {
uint32_t data = ((uint32_t)angular_position << 16) | angular_direction;
    uint8_t bytes[4];
    bytes[0] = (data >> 24) & 0xFF;
    bytes[1] = (data >> 16) & 0xFF;
    bytes[2] = (data >> 8) & 0xFF;
    bytes[3] = data & 0xFF;

    try {
        boost::asio::write(serial, boost::asio::buffer(bytes, 4));
        std::cout << "Data written to serial port." << std::endl;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error writing to serial port: " << e.what() << std::endl;
    }
}

uint32_t Serial_Protocal::protocal_read() {
    uint32_t data = 0;
    uint8_t bytes[4];

    std::cout << "Waiting for the data..." << std::endl;
    try {
        boost::asio::read(serial, boost::asio::buffer(bytes, 4));
        data = ((uint32_t)bytes[1] << 24) |
               ((uint32_t)bytes[0] << 16) |
               ((uint32_t)bytes[3] << 8) |
               bytes[2];
        std::cout << "Data received from serial port." << std::endl;
    } catch (const boost::system::system_error& e) {
        std::cerr << "Error reading from serial port: " << e.what() << std::endl;
    }
    return data;
}

int main() {
    Serial_Protocal Serial;
    uint16_t angular_position = 0xABCD;
    uint16_t angular_direction = 0x1234;
    Serial.protocal_write(angular_position, angular_direction);
    uint32_t received_data = Serial.protocal_read();
    angular_position=received_data >>16;
    angular_direction=received_data >>32;
    std::cout << "Received angular_position: " << std::hex << angular_position << std::endl;
    std::cout << "Received angular_direction: " << std::hex << angular_direction << std::endl;
    return 0;
}
