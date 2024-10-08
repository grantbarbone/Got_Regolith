#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <fstream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

/**
 * @brief class for creating the ROS2 node for publisher
 *
 */
class Main : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   */
  Main() : Node("Main_PubSub") {
    zed_publisher_ = this->create_publisher<std_msgs::msg::Bool>("topic_Main_to_Zed", 10);

    zed_subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "topic_Zed_to_Main", 10, std::bind(&Main::zed_topic_callback, this, std::placeholders::_1));
    
    lidar_publisher_ = this->create_publisher<std_msgs::msg::Bool>("topic_Main_to_Lidar", 10);
    
    lidar_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::lidar_timer_callback, this));
    
    lidar_subscription_ = this->create_subscription<std_msgs::msg::UInt16MultiArray>(
      "topic_Lidar_to_Main", 10, std::bind(&Main::lidar_topic_callback, this, std::placeholders::_1));  

    radar_publisher_ = this->create_publisher<std_msgs::msg::Bool>("topic_Main_to_Radar", 10);
    
    radar_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::radar_timer_callback, this));
    
    radar_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "topic_Radar_to_Main", 10, std::bind(&Main::radar_topic_callback, this, std::placeholders::_1)); 

    graph_algorithm_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_Main_to_Graph_Algorithm", 10);
    graph_algorithm_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::graph_algorithm_timer_callback, this));
    graph_algorithm_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
     "topic_Graph_Algorithm_to_Main", 10, std::bind(&Main::graph_algorithm_topic_callback, this, std::placeholders::_1));  
  
    motor_controls_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_Main_to_Motor_Controls", 10);
    
    motor_controls_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::motor_controls_timer_callback, this));
    
    motor_controls_subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "topic_Motor_Controls_to_Main", 10, std::bind(&Main::motor_controls_topic_callback, this, std::placeholders::_1));
    
    battery_publisher_ = this->create_publisher<std_msgs::msg::String>("topic_Main_to_Battery", 10);
    
    battery_timer_ = this->create_wall_timer(
        500ms, std::bind(&Main::battery_timer_callback, this));
    
    battery_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic_Battery_to_Main", 10, std::bind(&Main::battery_topic_callback, this, std::placeholders::_1));     

    
  }

  void start_publishing_zed() { 
    zed_timer_ = this->create_wall_timer(
      5ms, std::bind(&Main::zed_timer_callback, this)); 
  }  

  void stop_publishing_zed() {
    zed_timer_->cancel(); // Stop the timer
  }

    void start_publishing_radar() { 
    radar_timer_ = this->create_wall_timer(
      5ms, std::bind(&Main::radar_timer_callback, this)); 
  }  

  void stop_publishing_radar() {
    radar_timer_->cancel(); // Stop the timer
  }

  void start_publishing_lidar() { 
    lidar_timer_ = this->create_wall_timer(
      5ms, std::bind(&Main::lidar_timer_callback, this)); 
  }  

  void stop_publishing_lidar() {
    lidar_timer_->cancel(); // Stop the timer
  }

    void start_publishing_motor_controls() { 
    motor_controls_timer_ = this->create_wall_timer(
      5ms, std::bind(&Main::motor_controls_timer_callback, this)); 
  }  

  void stop_publishing_motor_controls() {
    motor_controls_timer_->cancel(); // Stop the timer
  }

    void start_publishing_battery() { 
    battery_timer_ = this->create_wall_timer(
      5ms, std::bind(&Main::battery_timer_callback, this)); 
  }  

  void stop_publishing_battery() {
    battery_timer_->cancel(); // Stop the timer
  }

  void start_publishing_graph_algorithm() { 
    graph_algorithm_timer_ = this->create_wall_timer(
      5ms, std::bind(&Main::graph_algorithm_timer_callback, this)); 
  }  

  void stop_publishing_graph_algorithm() {
    graph_algorithm_timer_->cancel(); // Stop the timer
  }
  void set_zed_connection(bool Switch){
    zed_connection=Switch;
  }
  bool get_zed_connection(){
    return zed_connection;
  }
    void set_lidar_connection(bool Switch){
    lidar_connection=Switch;
  }
  bool get_lidar_connection(){
    return lidar_connection;
  }
  void set_radar_connection(bool Switch){
    radar_connection=Switch;
  }
  bool get_radar_connection(){
    return radar_connection;
  }

  //Make a function void set_zed_data and void get_zed_data
  void set_zed_data(std::vector<float> zed_data)
  {
    zed_data_to_ros = zed_data;
  }

  std::vector<float> get_zed_data()
  {
    return zed_data_to_ros;
  }


 private:
  /**
   * @brief callback function
   *
   */




  void zed_timer_callback() {
    auto message = std_msgs::msg::Bool();
    message.data = true;
    //RCLCPP_INFO(this->get_logger(), "Publishing to Node Zed: '%s'", message.data ? "true" : "false");
    zed_publisher_->publish(message);
    stop_publishing_zed(); 
  }

  void lidar_timer_callback() {
    auto message = std_msgs::msg::Bool();
    message.data = true;
    //RCLCPP_INFO(this->get_logger(), "Publishing to Node Lidar: '%s'", message.data ? "true" : "false");
    lidar_publisher_->publish(message);
    stop_publishing_lidar(); 
  }

  void radar_timer_callback() {
    auto message = std_msgs::msg::Bool();
    message.data = true;
    //RCLCPP_INFO(this->get_logger(), "Publishing to Node Radar: '%s'", message.data ? "true" : "false");
    radar_publisher_->publish(message);
    stop_publishing_radar(); 
  }

void graph_algorithm_timer_callback() {
    auto message = std_msgs::msg::Float32MultiArray();
    // Populate the data array with example float values
    message.data = get_zed_data(); // Example data
    
    // Log each element of the array
    std::string data_str;
    for (const auto &value : message.data) {
        data_str += std::to_string(value) + " ";
    }

    RCLCPP_INFO(this->get_logger(), "Publishing to Node Graph_Algorithm: [%s]", data_str.c_str());
    graph_algorithm_publisher_->publish(message);
    stop_publishing_graph_algorithm(); 
}

  void motor_controls_timer_callback() {
    auto message = std_msgs::msg::Float32MultiArray();
    float x_coordinate = 1;
    float y_coordinate = 1;
    message.data = {x_coordinate, y_coordinate};
    RCLCPP_INFO(this->get_logger(), "Publishing to Node Motor_Controls: [%s]", std::to_string(message.data.size()).c_str());
    motor_controls_publisher_->publish(message);
    stop_publishing_motor_controls(); 
  }

  void battery_timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Message to Node Battery: " + std::to_string(1);
    //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    battery_publisher_->publish(message);
    stop_publishing_battery(); 
  }
  
  void zed_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Received: [%s]", std::to_string(msg->data.size()).c_str());
    set_zed_connection(true);
  }
  
  void lidar_topic_callback(const std_msgs::msg::UInt16MultiArray::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Received: [%s]", std::to_string(msg->data.size()).c_str());
    set_lidar_connection(false);
  }

  void radar_topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Received Radar array of size: %zu", msg->data.size());
    //for (size_t i = 0; i < msg->data.size(); ++i) {
    //    RCLCPP_INFO(this->get_logger(), "Element %zu: %f", i, msg->data[i]);
    //}
    set_radar_connection(false);
  }

void graph_algorithm_topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received Graph Algorithm array of size: %zu", msg->data.size());
    for (size_t i = 0; i < msg->data.size(); ++i) {
        RCLCPP_INFO(this->get_logger(), "Coord %zu: %f", i, msg->data[i]);
    }
}

  void motor_controls_topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data ? "true" : "false");

  }

  void battery_topic_callback(const std_msgs::msg::String::SharedPtr msg) {
    //RCLCPP_INFO(this->get_logger(), "Received from Node Battery: '%s'", msg->data.c_str() );
  }
  rclcpp::TimerBase::SharedPtr zed_timer_;
  rclcpp::TimerBase::SharedPtr lidar_timer_;
  rclcpp::TimerBase::SharedPtr radar_timer_;
  rclcpp::TimerBase::SharedPtr graph_algorithm_timer_;
  rclcpp::TimerBase::SharedPtr motor_controls_timer_;
  rclcpp::TimerBase::SharedPtr battery_timer_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr zed_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr zed_subscription_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr lidar_publisher_;
  rclcpp::Subscription<std_msgs::msg::UInt16MultiArray>::SharedPtr lidar_subscription_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr radar_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr radar_subscription_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr graph_algorithm_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr graph_algorithm_subscription_;

  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr motor_controls_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr motor_controls_subscription_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr battery_publisher_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr battery_subscription_;

  bool zed_connection;
  bool lidar_connection;
  bool radar_connection;
  std::vector<float> zed_data_to_ros;
};

/**
 * @brief Initiate the publisher
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Main>();

  enum State_Machine
  {
      Sensor_State, Lidar_State, Radar_State, Zed_State, Battery_State, Graph_Algorithm_State, Motor_Controls_State
  };

  State_Machine Initial_state= Sensor_State;

//toggling publishing

  while (rclcpp::ok()) {
    switch (Initial_state) { 
      case Sensor_State: 
      {
        //std::cout<< "In Sensor_State" <<std::endl;
        
        node->set_zed_connection(true);
        node->set_lidar_connection(false);
        node->set_radar_connection(false);
        node->start_publishing_zed(); 
        std::this_thread::sleep_for(std::chrono::milliseconds(100));      
        node->start_publishing_lidar(); 
        std::this_thread::sleep_for(std::chrono::milliseconds(100));      
        node->start_publishing_radar();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));      
        rclcpp::spin_some(node);
        if (node->get_zed_connection()){
          //std::cout<< "Change to Zed" <<std::endl;
          
          Initial_state=Zed_State;
        }
        else if (node->get_lidar_connection()){
          //std::cout<< "Change to Lidar" <<std::endl;
          Initial_state=Lidar_State;
        }
        else if(node->get_radar_connection()){
          //std::cout<< "Change to Radar" <<std::endl;
          Initial_state=Radar_State;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));      
        //std::cout<< "End of Sensor_State" <<std::endl;
        //else{
          //rclcpp::shutdown(); 
        //}
        continue; 
      }
      case Zed_State:
      {
        //node->start_publishing_zed();
        //rclcpp::spin_some(node); 
        //std::this_thread::sleep_for(std::chrono::milliseconds(1000));      
        system("cd /home/rover/Downloads/zed-sdk/tutorials/tutorial\\ 1\\ -\\ hello\\ ZED/cpp/build && ./ZED_Tutorial_1");
        while(true){
          //std::cout<< "Data is stuck" <<std::endl; 
          std::ifstream zed_stream_in("/home/rover/ros2_ws/src/state_machine/temp.txt");
          //std::this_thread::sleep_for(std::chrono::milliseconds(1000)); 
          if (!zed_stream_in.is_open()) {
            std::cerr << "Error: Could not open the file." << std::endl;
            return 1;
          } else {
          std::vector<float> zed_data;
          float zed_data_value;
          // Read and store the floating-point numbers from the file
          while (zed_stream_in >> zed_data_value ) {
              zed_data.push_back(zed_data_value);
          }
          if (zed_data.size()!=0){
            node->set_zed_data(zed_data);
            break;
          }
          zed_stream_in.close(); // Close the file stream
        }
    }
        Initial_state=Graph_Algorithm_State;
        //std::cout<< "End of Zed_State" <<std::endl;
        continue; 
      }
      case Lidar_State: 
      {
        //std::cout<< "In Lidar_State" <<std::endl;
        node->start_publishing_lidar();
        rclcpp::spin_some(node); 
        //TODO
        //
        //
        Initial_state=Graph_Algorithm_State;
        //std::cout<< "In Graph_Algorithm_State" <<std::endl;
        continue;
      }
      case Radar_State: 
      {
        node->start_publishing_lidar();
        rclcpp::spin_some(node); 
        node->get_zed_data();
        Initial_state=Graph_Algorithm_State;
        continue;
      }
      case Graph_Algorithm_State: 
      {
        node->start_publishing_graph_algorithm();
        rclcpp::spin_some(node); 
        Initial_state=Battery_State;
        continue;
      }
      case Battery_State: 
      {
        //std::cout<< "In Battery_State" <<std::endl;
        uint16_t next_distance = 10;
        uint16_t calculated_distance = 20;
        
        node->start_publishing_battery();
        rclcpp::spin_some(node); 
        if (calculated_distance > next_distance) {
          Initial_state = Motor_Controls_State;
        } else {
          Initial_state = Graph_Algorithm_State;
        }
        //std::cout<< "End of Battery_State" <<std::endl;
        continue;
      }
      case Motor_Controls_State: 
      {
        //std::cout<< "In Motor_Controls_State" <<std::endl;
        node->start_publishing_motor_controls();
        rclcpp::spin_some(node);
        //TODO
        //
        //
        Initial_state=Sensor_State;
        //std::cout<< "End of Motor_Controls_State" <<std::endl;
        continue;
      }
      default:
        Initial_state=Sensor_State;
    } 

  //Allow ROS to process any incoming messages and handle callbacks
  rclcpp::spin_some(node);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
// Shutdown ROS gracefully
  rclcpp::shutdown();
  return 0;
}
