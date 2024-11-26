#ifndef ZEDCLASS_H
#define ZEDCLASS_H

#include <functional>
#include <memory>
#include <random>
#include <sl/Camera.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib> // For std::rand and RAND_MAX
#include <ctime>   // For std::time
#include <chrono>
#include <string>
#include <iostream>
#include <queue>
#include <limits>
#include <unordered_map>
#include <unordered_set>

using namespace std;
using namespace sl;

using namespace std::chrono_literals;
using std::placeholders::_1;

class ZedClass {
public:
    	ZedClass();
    	bool zed_open_camera();
    	std::vector<float> zed_depth_sense();
    	std::vector<std::vector<float>> zed_object_detection(float object_detected);
        std::vector<float> IMU();
    	void zed_close();

private:
    float alpha_orientation =0.1;
    float alpha_acceleration =0.1;
    float alpha_velocity =0.1;
    std::vector<float> prev_filtered_orientation;
    std::vector<float> prev_filtered_acceleration;
    std::vector<float> prev_filtered_velocity;
    
	Camera zed;
	InitParameters init_parameters;
	ERROR_CODE returned_state;
    ObjectDetectionParameters detection_parameters;
// Basic structure to compare timestamps of a sensor. Determines if a specific sensor data has been updated or not.
    struct TimestampHandler {

        // Compare the new timestamp to the last valid one. If it is higher, save it as new reference.
        inline bool isNew(Timestamp& ts_curr, Timestamp& ts_ref) {
            bool new_ = ts_curr > ts_ref;
            if (new_) ts_ref = ts_curr;
            return new_;
        }
        // Specific function for IMUData.
        inline bool isNew(SensorsData::IMUData& imu_data) {
            return isNew(imu_data.timestamp, ts_imu);
        }
        // Specific function for MagnetometerData.
        inline bool isNew(SensorsData::MagnetometerData& mag_data) {
            return isNew(mag_data.timestamp, ts_mag);
        }
        // Specific function for BarometerData.
        inline bool isNew(SensorsData::BarometerData& baro_data) {
            return isNew(baro_data.timestamp, ts_baro);
        }

        Timestamp ts_imu = 0, ts_baro = 0, ts_mag = 0; // Initial values
    };
};

ZedClass::ZedClass(){

    // Set configuration parameters
    init_parameters.depth_mode = DEPTH_MODE::ULTRA; // Use ULTRA depth mode
    init_parameters.coordinate_units = UNIT::INCH; // Use inches units (for depth measurements)
    init_parameters.camera_resolution = RESOLUTION::AUTO; 
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP;
    init_parameters.sensors_required= true;
    init_parameters.sdk_verbose = true;

}

bool ZedClass::zed_open_camera(){
	    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Error " << returned_state << ", exit program." << endl;
        return false;
    }
    PositionalTrackingParameters tracking_parameters;
    returned_state =zed.enablePositionalTracking(tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Error " << returned_state << ", exit program.\n";
        return EXIT_FAILURE;
    }

    return true;

}



std::vector<float> ZedClass::zed_depth_sense() {
        std::vector<float> data;
        sl::Mat image, depth, point_cloud;
         while (true){
        // A new image is available if grab() returns ERROR_CODE::SUCCESS
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            // Retrieve left image
                zed.retrieveImage(image, VIEW::LEFT);
                // Retrieve depth map. Depth is aligned on the left image
                zed.retrieveMeasure(depth, MEASURE::DEPTH);
                // Retrieve colored point cloud. Point cloud is aligned on the left image.
                zed.retrieveMeasure(point_cloud, MEASURE::XYZRGBA);
            // Get and process distance value at the center of the image
            
                int x = image.getWidth() / 2;
                int y = image.getHeight() / 2;
                sl::float4 point_cloud_value;
                point_cloud.getValue(x, y, &point_cloud_value);
                if (std::isnan(point_cloud_value.x) || std::isnan(point_cloud_value.y) || std::isnan(point_cloud_value.z)) {
                    //std::cout << point_cloud_value.x << std::endl;
                    continue;
                }
                data.push_back(point_cloud_value.x);
                data.push_back(point_cloud_value.y);
                data.push_back(point_cloud_value.z);
                break;
            }
        }
        // Return the vector ready for publishing
        return data;
    }


//float object_depth
std::vector<std::vector<float>> ZedClass::zed_object_detection(float object_detected){
        // Define the Objects detection module parameters

    // run detection for every Camera grab
    //detection_parameters.image_sync = true;
    // track detects object accross time and space
    detection_parameters.enable_tracking = true;
    // compute a binary mask for each object aligned on the left image
    detection_parameters.enable_segmentation = true; // designed to give person pixel mask
    
    // If you want to have object tracking you need to enable positional tracking first
    if (detection_parameters.enable_tracking)
         
        zed.enablePositionalTracking();    
    
    returned_state = zed.enableObjectDetection(detection_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        
        cout << "Error " << returned_state << ", exit program.\n";
        zed.close();
    }
    // detection runtime parameters
    ObjectDetectionRuntimeParameters detection_parameters_rt;
    // detection output
    Objects objects;
    cout << setprecision(3);
    
    std::vector<std::vector<float>> object_detect_data;
    zed.retrieveObjects(objects, detection_parameters_rt);
    
    bool object_identified=false;
    float current_time=1;
    std::chrono::high_resolution_clock::time_point start;
    start = std::chrono::high_resolution_clock::now();
    float counter=0;
    float side_1=0;
    float side_2=0;
    float max_z=0;
    std::cout << object_detected <<std::endl;

    while (!object_identified && current_time<=2 && object_detected > -30.0 ){
        
        if(zed.grab() == ERROR_CODE::SUCCESS){
            zed.retrieveObjects(objects, detection_parameters_rt);
            if (objects.is_new) {
               // cout << objects.object_list.size() << " Object(s) detected\n\n";
                if (!objects.object_list.empty()) {
                    counter=0;
                    auto first_object = objects.object_list.front();
                    for (auto it : first_object.bounding_box) {
                        object_identified=true; 
                        std::cout << it.x << "x box "<<  it.y << "y box " <<  it.z << "z box "<< std::endl;   
                        object_detect_data.push_back({it.x,it.y,it.z, counter++});
                        if (std::abs(object_detected)+12.0 >=std::abs(it.z)  && std::abs(object_detected) <= std::abs(it.z) ){ 
                            side_1++;
                        }
                        if (it.x>=0.0 && it.x < 12.0){
                            side_2++;
                        }
                    }
                    for (auto point = std::begin(object_detect_data); point != std::end(object_detect_data); point++){
                        max_z=std::max(max_z, std::abs((*point)[2]));
                    }
                    if (side_1<=2){
                        if (side_2<=1){
                            if (max_z<= std::abs(object_detected) || max_z>= std::abs(object_detected)+12.0) {
                            object_detect_data.clear();
                            continue;
                            }
                        }
                    }
                    std::cout <<side_1 << std::endl;
                    side_1=0;
                    side_2=0;
                }
            }   
        }
        current_time = std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::high_resolution_clock::now() - start).count();
    }
    return object_detect_data;
}

std::vector<float> quaternionToEuler(const sl::Orientation&  orientation){
    std::vector<float> vector_orientations;
    #define RAD2DEG 57.2958f;
    float ox= orientation.ox;
    float oy= orientation.oy;
    float oz= orientation.oz;
    float ow= orientation.ow;
    
    float roll=atan2(2.0f * (ow*ox+ oy*oz), 1.0f-2.0f * (ow*ox+ oy*oz));
    float pitch=asin(2.0f * (ow * oy - oz * ox));
    float yaw= atan2(2.0f * (ow* oz + ox * oy), 1.0f - 20.f * (oy* oy+ oz*oz));
    roll*=RAD2DEG;
    roll=fmod(roll+360.0f,360.0f);
    std::cout << "roll: " << roll <<std::endl;
    vector_orientations.push_back(roll);
    pitch*=RAD2DEG;
    pitch=fmod(pitch+360.0f,360.0f);
    std::cout << "pitch: " << pitch <<std::endl;
    vector_orientations.push_back(pitch);
    yaw*=RAD2DEG;
    yaw=fmod(yaw+360.0f,360.0f);
    std::cout << "yaw: " << yaw <<std::endl;
    vector_orientations.push_back(yaw);
    return vector_orientations;
}

std::vector<float> ZedClass::IMU() {
    std::vector<float> data;
    sl::Pose zed_pose;
    bool zed_has_imu = zed.getCameraInformation().sensors_configuration.isSensorAvailable(sl::SENSOR_TYPE::GYROSCOPE);
    SensorsData sensor_data;
    // Ensure positional tracking is enabled
    sl::PositionalTrackingParameters tracking_params;
    int i=0;
    float x_max=0;
    float y_max=0;
    float z_max=0;
    float x_current=0;
    float y_current=0;
    float z_current=0;
    float range_min_z=0.01;
    float range_max_z=0.1;
    float range_min_x=0.01;
    float range_max_x=0.1;
    while(i<20){
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);

            // printf("Translation: Tx: %.3f, Ty: %.3f, Tz: %.3f, Timestamp: %llu\n",
            //     zed_pose.getTranslation().tx, zed_pose.getTranslation().ty,
            //     zed_pose.getTranslation().tz, zed_pose.timestamp);

            zed_pose.getTranslation().tx;
            zed_pose.getTranslation().ty;
            zed_pose.getTranslation().tz;
 
            // printf("Orientation: Ox: %.3f, Oy: %.3f, Oz: %.3f, Ow: %.3f\n\n",
            //     zed_pose.getOrientation().ox, zed_pose.getOrientation().oy,
            //     zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);

        //    zed_pose.getOrientation().ox;
        //    zed_pose.getOrientation().oy;
        //    zed_pose.getOrientation().oz;
        //    zed_pose.getOrientation().ow; // Add w component if needed
        }
        i++;
    }
    i=0;
     //   while(i<20){
       // if (zed.grab() == ERROR_CODE::SUCCESS) {
         //   zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);

            // printf("Translation: Tx: %.3f, Ty: %.3f, Tz: %.3f, Timestamp: %llu\n",
            //     zed_pose.getTranslation().tx, zed_pose.getTranslation().ty,
            //     zed_pose.getTranslation().tz, zed_pose.timestamp);
            x_current=zed_pose.getTranslation().tx;
            y_current=zed_pose.getTranslation().ty;
            z_current=zed_pose.getTranslation().tz;
            std::cout << x_current << " the current x is " << y_current << " the current y is " << z_current << " the current z is." << std::endl;
        //    if (z_current> range_min_z){
          //      if (z_current< range_max_z ){
            //        if (z_max< z_current){
              //          z_max=z_current;
           //         }
          //      }
          //  }
           // std::cout << z_current << "This is the z_current value."<< std::endl;
          //  if (x_current> range_min_x){
          //      if ( x_current< range_max_x ){
            //        if (x_max< x_current){
              //          x_max=x_current;
               //     }
             //   }
          //  }
            // printf("Orientation: Ox: %.3f, Oy: %.3f, Oz: %.3f, Ow: %.3f\n\n",
            //     zed_pose.getOrientation().ox, zed_pose.getOrientation().oy,
            //     zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);

        //    zed_pose.getOrientation().ox;
        //    zed_pose.getOrientation().oy;
        //    zed_pose.getOrientation().oz;
        //    zed_pose.getOrientation().ow; // Add w component if needed
      //  }
      //  i++;
   // }
  
    data.push_back(zed_pose.getTranslation().tx);
    data.push_back(zed_pose.getTranslation().ty);
    data.push_back(zed_pose.getTranslation().tz);
    //std::vector<float> orientation=quaternionToEuler(zed_pose.getOrientation());
   // data.insert(data.end(),orientation.begin(), orientation.end());
    return data;
}

void ZedClass::zed_close(){
    zed.disablePositionalTracking();
    zed.close();
}

#endif
