// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file subscriber_member_function.cpp
 * @author Jay Prajapati (jayp@umd.edu)
 * @brief create a minimal subscriber for ROS2
 * @version 0.1
 * @date 2023-11-06
 *
 * @copyright Copyright (c) 2023
 *
 */

#include <functional>
#include <memory>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "std_msgs/msg/u_int16_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

//#include <sl/Camera.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib> // For std::rand and RAND_MAX
#include <ctime>   // For std::time

using namespace std;
//using namespace sl;

using namespace std::chrono_literals;
using std::placeholders::_1;

// class ZedClass {
// public:
//     	ZedClass();
//     	bool zed_open_camera();
//     	void zed_depth_sense();
//     	bool zed_object_detection();
//     	void zed_close();
//       uint16_t floatToUInt16();
// private:
// 	Camera zed;
// 	InitParameters init_parameters;
// 	ERROR_CODE returned_state;
 
// };

// ZedClass::ZedClass(){
	
//     // Set configuration parameters
//     init_parameters.depth_mode = DEPTH_MODE::ULTRA; // Use ULTRA depth mode
//     init_parameters.coordinate_units = UNIT::MILLIMETER; // Use millimeter units (for depth measurements)
//     }

// bool ZedClass::zed_open_camera(){
// 	    // Open the camera
//     auto returned_state = zed.open(init_parameters);
//     if (returned_state != ERROR_CODE::SUCCESS) {
//         cout << "Error " << returned_state << ", exit program." << endl;
//         return false;
//     }
//     return true;

// }


// uint16_t ZedClass::floatToUInt16(float value, float min_value, float max_value) {
//     float clamped_value = std::clamp(value, min_value, max_value);
//     return static_cast<uint16_t>((clamped_value - min_value) / (max_value - min_value) * UINT16_MAX);
// }


// std::vector<uint16_t> ZedClass::zed_depth_sense() {
//         std::vector<uint16_t> data;

//         sl::Mat image, depth, point_cloud;

//         // A new image is available if grab() returns ERROR_CODE::SUCCESS
//         if (zed.grab() == ERROR_CODE::SUCCESS) {
//             // Retrieve left image
//             zed.retrieveImage(image, VIEW::LEFT);
//             // Retrieve depth map. Depth is aligned on the left image
//             zed.retrieveMeasure(depth, MEASURE::DEPTH);
//             // Retrieve colored point cloud. Point cloud is aligned on the left image.
//             zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);

//             // Get and process distance value at the center of the image
//             int x = image.getWidth() / 2;
//             int y = image.getHeight() / 2;
//             sl::float4 point_cloud_value;
//             point_cloud.getValue(x, y, &point_cloud_value);
//             const float min_value = 0.0f;
//             const float max_value = 100.0f; // Example range; adjust as needed

//             // Convert x, y, and z to uint16_t
//             uint16_t x_uint16 = floatToUInt16(point_cloud_value.x,min_value,max_value);
//             uint16_t y_uint16 = floatToUInt16(point_cloud_value.y,min_value,max_value);
//             uint16_t z_uint16 = floatToUInt16(point_cloud_value.z,min_value,max_value);

//             // Store in a vector
//             data.push_back(x_uint16);
//             data.push_back(y_uint16);
//             data.push_back(z_uint16);
//         }
//         // Return the vector ready for publishing
//         return data;
//     }



// bool ZedClass::zed_object_detection(){
//         // Define the Objects detection module parameters
//     ObjectDetectionParameters detection_parameters;
//     // run detection for every Camera grab
//     detection_parameters.image_sync = true;
//     // track detects object accross time and space
//     detection_parameters.enable_tracking = true;
//     // compute a binary mask for each object aligned on the left image
//     detection_parameters.enable_segmentation = true; // designed to give person pixel mask

//     // If you want to have object tracking you need to enable positional tracking first
//     if (detection_parameters.enable_tracking)
//         zed.enablePositionalTracking();    
        
//     returned_state = zed.enableObjectDetection(detection_parameters);
//     if (returned_state != ERROR_CODE::SUCCESS) {
//         cout << "Error " << returned_state << ", exit program.\n";
//         zed.close();
//         return false;
//     }
//     // detection runtime parameters
//     ObjectDetectionRuntimeParameters detection_parameters_rt;
//     // detection output
//     Objects objects;
//     cout << setprecision(3);

//     int nb_detection = 0;
//     while (nb_detection < 100) {

//         if(zed.grab() == ERROR_CODE::SUCCESS){
//            zed.retrieveObjects(objects, detection_parameters_rt);

//             if (objects.is_new) {}
//                 if (!objects.object_list.empty()) {

//                     auto first_object = objects.object_list.front();

//                     // cout << " Label '" << first_object.label << "' (conf. "
//                     //     << first_object.confidence << "/100)\n";

//                     // if (detection_parameters.enable_tracking)
//                     //     cout << " Tracking ID: " << first_object.id << " tracking state: " <<
//                     //     first_object.tracking_state << " / " << first_object.action_state << "\n";

//                     cout << " 3D position: " << first_object.position <<
//                         " Velocity: " << first_object.velocity << "\n";

//                     cout << " 3D dimensions: " << first_object.dimensions << "\n";

//                     if (first_object.mask.isInit())
//                         cout << " 2D mask available\n";

//                     cout << " Bounding Box 2D \n";
//                     for (auto it : first_object.bounding_box_2d)
//                         cout << "    " << it<<"\n";

//                     cout << " Bounding Box 3D \n";
//                     for (auto it : first_object.bounding_box)
//                         cout << "    " << it << "\n";

//                     cout << "\nPress 'Enter' to continue...\n";
//                     cin.ignore();
//                 }
//                 nb_detection++;
//             }
//         }   
// }

// std::vector<uint16_t> IMU() {
//     std::vector<uint16_t> data;

//     SensorsData sensors_data;
//     if (zed.grab() == ERROR_CODE::SUCCESS) {
//         zed.getSensorsData(sensors_data, TIME_REFERENCE::CURRENT);
//     }

//     TimestampHandler ts;
//     if (ts.isNew(sensors_data.imu)) {
//         // Define min and max ranges for conversion
//         const float orientation_min = -180.0f; // Example min, adjust as needed
//         const float orientation_max = 180.0f; // Example max, adjust as needed
//         const float acceleration_min = -10.0f; // Example min, adjust as needed
//         const float acceleration_max = 10.0f; // Example max, adjust as needed
//         const float angular_velocity_min = -500.0f; // Example min, adjust as needed
//         const float angular_velocity_max = 500.0f; // Example max, adjust as needed
//         //    cout << "IMU Orientation: {" << sensors_data.imu.pose.getOrientation() << "}";
//         //    cout << "IMU Linear Acceleration: {" << sensors_data.imu.linear_acceleration << "} [m/sec^2]";
//         //    cout << "IMU Angular Velocity: {" << sensors_data.imu.angular_velocity << "} [deg/sec]";
//         // Convert IMU data to uint16_t
//         uint16_t orientation_x = floatToUInt16(sensors_data.imu.pose.getOrientation().x, orientation_min, orientation_max);
//         uint16_t orientation_y = floatToUInt16(sensors_data.imu.pose.getOrientation().y, orientation_min, orientation_max);
//         uint16_t orientation_z = floatToUInt16(sensors_data.imu.pose.getOrientation().z, orientation_min, orientation_max);
//         uint16_t orientation_w = floatToUInt16(sensors_data.imu.pose.getOrientation().w, orientation_min, orientation_max);

//         uint16_t linear_accel_x = floatToUInt16(sensors_data.imu.linear_acceleration.x, acceleration_min, acceleration_max);
//         uint16_t linear_accel_y = floatToUInt16(sensors_data.imu.linear_acceleration.y, acceleration_min, acceleration_max);
//         uint16_t linear_accel_z = floatToUInt16(sensors_data.imu.linear_acceleration.z, acceleration_min, acceleration_max);

//         uint16_t angular_vel_x = floatToUInt16(sensors_data.imu.angular_velocity.x, angular_velocity_min, angular_velocity_max);
//         uint16_t angular_vel_y = floatToUInt16(sensors_data.imu.angular_velocity.y, angular_velocity_min, angular_velocity_max);
//         uint16_t angular_vel_z = floatToUInt16(sensors_data.imu.angular_velocity.z, angular_velocity_min, angular_velocity_max);

//         // Store in a vector
//         data.push_back(orientation_x);
//         data.push_back(orientation_y);
//         data.push_back(orientation_z);
//         data.push_back(orientation_w);

//         data.push_back(linear_accel_x);
//         data.push_back(linear_accel_y);
//         data.push_back(linear_accel_z);

//         data.push_back(angular_vel_x);
//         data.push_back(angular_vel_y);
//         data.push_back(angular_vel_z);
//     }
//     // Check if Magnetometer data has been updated
//     // if (ts.isNew(sensors_data.magnetometer)) {
//     //     cout << " Magnetometer Magnetic Field: {" << sensors_data.magnetometer.magnetic_field_calibrated << "} [uT]";
//     // }

//     // // Check if Barometer data has been updated
//     // if (ts.isNew(sensors_data.barometer)) {
//     //     cout << " Barometer Atmospheric pressure:" << sensors_data.barometer.pressure << " [hPa]";
//     // }

//     // Return the vector ready for publishing
//     return data;
// }

// void ZedClass::zed_close(){
//     zed.close();
// }


/**
 * @brief class for creating the ROS2 node for subscriber
 *
 */
class Zed : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   *
   */
  
  Zed() : Node("Zed_PubSub") {
    publisher_ = this->create_publisher<std_msgs::msg::UInt16MultiArray>("topic_zed_to_main", 10);
        
    subscription_ = this->create_subscription<std_msgs::msg::Bool>(
      "topic_main_to_zed", 10, std::bind(&Zed::topic_callback, this, std::placeholders::_1));
  }
    void start_publishing() {
    zed_timer_ = this->create_wall_timer(
      5ms, std::bind(&Zed::timer_callback, this)); // Restart the timer
  }

  void stop_publishing() {
    zed_timer_->cancel(); // Stop the timer
  }
  // bool set_combine_data(
  //   const std::vector<uint16_t>& depth_data,
  //   const std::vector<uint16_t>& object_detection_data,
  //   const std::vector<uint16_t>& IMU_data)
  //   {
  //   combined_data.reserve(depth_data.size() + object_detection_data.size() + IMU_data.size());

  //   combined_data.insert(combined_data.end(), depth_data.begin(), depth_data.end());
  //   combined_data.insert(combined_data.end(), object_detection_data.begin(), object_detection_data.end());
  //   combined_data.insert(combined_data.end(), IMU_data.begin(), IMU_data.end());
  //   if (combined_data.empty()){
  //       return false;
  //   }
  //   return true;
  //   }
 private:
  /**
   * @brief callback function for the topic
   *
   * @param msg
   */

  std::vector<uint16_t> combined_data;

  void timer_callback() {
    auto message = std_msgs::msg::UInt16MultiArray();
    message.data = combined_data;
    RCLCPP_INFO(this->get_logger(), "Publishing: [%s]", std::to_string(message.data.size()).c_str());
    publisher_->publish(message);
  }


  void topic_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received: '%s'", msg->data ? "true" : "false");
    stop_publishing();
  }

  rclcpp::TimerBase::SharedPtr zed_timer_;
  rclcpp::Publisher<std_msgs::msg::UInt16MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
  size_t count_;
  //Float32MultiArray
};

std::vector<uint16_t> generate_random_uint16(size_t size) {
    std::vector<uint16_t> data(size);
    
    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<uint16_t> dis(0, UINT16_MAX);

    // Generate random values
    for (size_t i = 0; i < size; ++i) {
        data[i] = dis(gen);
    }

    return data;
}
/**
 * @brief Initiate the subscriber
 *
 * @param argc
 * @param argv
 * @return int
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Zed>();
    // Assuming Zed is an instance of a class that provides these methods
    //ZedClass Zed;
    bool open_camera_bool = true; // Example flag
    //std::vector<uint16_t> combined_data;
    
    //size_t depth_data_size = 3;  // x, y, z
    //size_t object_detection_data_size = 8;  // 4 points, each with x, y
    //size_t IMU_data_size = 10;  // 3 linear_acceleration, 3 angular_velocity, 4 orientation

    rclcpp::spin_some(node);
    while (rclcpp::ok() && open_camera_bool) {
        
                // Retrieve data
            // std::vector<uint16_t> depth_data = Zed.zed_depth_sense();
            // std::vector<uint16_t> object_detection_data = Zed.zed_object_detection();
            // std::vector<uint16_t> IMU_data = Zed.zed_IMU(); // Fixed method call

            // Sizes of each array

            // Generate random data
      //      std::vector<uint16_t> depth_data = generate_random_uint16(depth_data_size);
      //      std::vector<uint16_t> object_detection_data = generate_random_uint16(object_detection_data_size);
      //      std::vector<uint16_t> IMU_data = generate_random_uint16(IMU_data_size);

            //Check if combined_data is empty
 //           if (!node->set_combine_data(depth_data, object_detection_data, IMU_data)) {
   //             std::cout << "Combined data is empty." << std::endl;
                // Additional logic if combined_data is empty
                //Zed.zed_close();
     //           rclcpp::shutdown();
       //         return EXIT_FAILURE;
         //   }
            
            node->start_publishing();
            rclcpp::spin_some(node);
            }
        
    // Cleanup
    //Zed.zed_close();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}