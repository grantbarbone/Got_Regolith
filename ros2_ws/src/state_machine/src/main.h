#ifndef MAIN_H
#define MAIN_H

#include "std_msgs/msg/u_int16_multi_array.hpp"
#include <vector>

/**
 * @brief 
 * 
 */
void start_publishing_zed();

/**
 * @brief 
 * 
 */
void stop_publishing_zed();

/**
 * @brief 
 * 
 */
void start_publishing_lidar();

/**
 * @brief 
 * 
 */
void stop_publishing_lidar();

/**
 * @brief 
 * 
 */
void start_publishing_motor_controls();

/**
 * @brief 
 * 
 */
void stop_publishing_motor_controls();

/**
 * @brief 
 * 
 */
void start_publishing_graph_algorithm();

/**
 * @brief 
 * 
 */
void stop_publishing_graph_algorithm();

/**
 * @brief Set the zed connection object
 * 
 * @param Switch 
 */
void set_zed_connection(bool Switch);

/**
 * @brief Get the zed connection object
 * 
 * @return true 
 * @return false 
 */
bool get_zed_connection();

/**
 * @brief Set the lidar connection object
 * 
 * @param Switch 
 */
void set_lidar_connection(bool Switch);

/**
 * @brief Get the lidar connection object
 * 
 * @return true 
 * @return false 
 */
bool get_lidar_connection();

/**
 * @brief Set the zed data object
 * 
 * @param zed_data 
 */
void set_zed_data(std::vector<float> zed_data);

/**
 * @brief Get the zed data object
 * 
 * @return std::vector<float> 
 */
std::vector<float> get_zed_data();

/**
 * @brief 
 * 
 */
void zed_timer_callback();

/**
 * @brief 
 * 
 */
void lidar_timer_callback();

/**
 * @brief 
 * 
 */
void graph_algorithm_timer_callback();

/**
 * @brief 
 * 
 */
void motor_controls_timer_callback();

/**
 * @brief 
 * 
 * @param msg 
 */
void zed_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

/**
 * @brief 
 * 
 * @param msg 
 */
void lidar_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

/**
 * @brief 
 * 
 * @param msg 
 */
void graph_algorithm_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

/**
 * @brief 
 * 
 * @param msg 
 */
void motor_controls_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg);

/**
 * @brief 
 * 
 * @param argc 
 * @param argv 
 * @return int 
 */
int main(int argc, char* argv[]);
#endif