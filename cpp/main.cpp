/**
 * @file main.cpp
 * @author Grant Barbone
 *
*/

#include "Serial_Protocal.h"
#include "RoboticPathPlanning.h"
#include "ZedClass.h"
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
//#include <unordered_s::_1;

enum Robot_Pathing_State{
      Update_State, Recieve_State, Setup_State, Camera_State, IMU_State, Object_Depth_State, Error_Kinematic_State, Check_Point_State, Kinematic_Adjustment_State, Smooth_Out_State, Send_State
};
void write_data(float x, float y) {
    int fd = open("store_coord.txt", O_RDWR| O_CREAT |O_TRUNC, 0666);
    if (fd == -1) {
        perror("Error opening file");
    }
    std::string message_s =  std::to_string(x) + " " + std::to_string(y) + "\n";
    const char * message= message_s.c_str();
    ssize_t bytes_written= write(fd, message, strlen(message));
    if (bytes_written==-1){
     //   std::cerr << "Error writing to file." << std::endl;
        close(fd);
    }
}
std::vector<float> read_data(){ 
   int fd=open("store_coord.txt", O_RDONLY);
    if (fd==-1){
       // std::cerr << "Error opening file for reading." << std::endl;
    }
    char buffer[46];
    std::vector<float> data;
    ssize_t bytes_read=read(fd, buffer, sizeof(buffer) -1);
    if (bytes_read==-1) {
     //   std::cerr <<  "Error reading from file." << std::endl;
        close(fd);
    }
    std::string str(buffer);
    
    float point1, point2;
    std::istringstream iss(buffer);
    if (iss>> point1 >> point2){
      // std::cout << buffer << " This is the data read from the file!" << std::endl;
        std::vector<float> data{point1, point2};
        return data;
    }
    return data;
}

int main(int argc, char* argv[]) {

    ZedClass Zed;
    RoboticPathPlanning robotic;
    //Serial_Protocal Serial;
    std::vector<float> imu_data;
    float imu_X=0;
    float imu_Y=0;
    float imu_Z=0;
    std::vector<std::vector<float>> object_detection_data;
    std::vector<float> depth_data;
    bool start_rover=false;
    float second_pass=0;
    float x_end;
    float y_end;
    float imu_x=0;
    float imu_y=0;
    float imu_prev_x=0;
    float imu_prev_y=0;
    float imu_delta_x;
    float imu_delta_y;
    float current_x;
    float current_y;
    float current_time;
    float next_velocity;
    float next_acceleration;
    float forsight_delta_x;
    float forsight_delta_y;
    float angle_x;
    float angle_y;
    float angle_radians;
    float counter=0;
    std::pair<float,float>last_point;
    std::pair<float, float> foresight_point;
    std::pair<float, float> temp_path;
    std::pair<float, float> check_check_point;
    float hypotunuse;
    float coord_x_to_send;
    float coord_y_to_send;
    float direction;
    float object_y_resize_bottom_param;
    float object_y_resize_top_param;
    float object_x_resize_bottom_param;
    float object_x_resize_top_param;
    float r=1;
    float s=1;
    float prev_x=0;
    float prev_y=0;
    std::chrono::time_point<std::chrono::high_resolution_clock>  end;
    std::chrono::duration<double> elapsed;
    std::chrono::high_resolution_clock::time_point start;
    std::vector<std::vector<float>> grid = {
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,},
        {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,}
    };

    std::vector<std::vector<Vertex>> grid_vertex = robotic.fillGridWithVertices(grid);
    Vertex object_detected=grid_vertex[0][0];
    std::vector<std::pair<float, float>> path;
    std::vector<std::pair<float, float>> set_points = {{10, 7}, {13, 9}, {6, 12}, {5, 4}, {4, 4}}; 
    std::vector<std::pair<float, float>> circular_points= {{0,0},{0.3129, 0.3129}, { 0.618, 0.618}, { 0.908, 0.908 }, {1.1756, 1.1756}, {1.414, 1.414}, { 1.618, 1.618}, { 1.782, 1.782}, { 1.902, 1.902}, {2.0,2.0}};  
    Vertex current_location=grid_vertex[0][0];
   
    Robot_Pathing_State Initial_state= Setup_State;

    while(true){
        switch (Initial_state) { 
            case Setup_State: {
                std::cout << "Setup State: " << std::endl;
                
                std::vector <float>data_stored=read_data();
                if (data_stored[0]==0.0 && data_stored[1]==0.0){
                    last_point=set_points.back();
                    x_end=last_point.first;
                    y_end=last_point.second;
                    set_points.pop_back();
                    Vertex start(0, 0);
                    Vertex goal(x_end, y_end);
                    std::cout << "Entering path planning. \n" << std::endl;
                    path = robotic.a_star_algorithm(&start, &goal, grid_vertex);
                    std::cout << "Exiting path planning. \n" << std::endl;
                }
                else{
                    for (int i =set_points.size()-1 ; i > 1; i--){
                        std::cout << data_stored[0] << std::endl;
                      
                        std::cout << set_points[i-1].first << std::endl;
                        if ( data_stored[0]< set_points[i-1].first && data_stored[0]< set_points[i].first ) {
                
                            if ( data_stored[1]< set_points[i-1].second && data_stored[0]< set_points[i].second ) {
                         
                                Vertex start(data_stored[0], data_stored[1]);
                                x_end=set_points[i].first;
                                y_end=set_points[i].second;
                                Vertex goal( x_end, y_end);
                                for (int i =0; i < counter; i++){
                                    set_points.pop_back();
                                }
                                std::cout << x_end << " the end for x." <<std::endl;
                                std::cout << y_end << " the end for y."<<std::endl;
                                std::cout << "Entering path planning. \n" << std::endl;
                                path = robotic.a_star_algorithm(&start, &goal, grid_vertex);
                                std::cout << "Exiting path planning. \n" << std::endl;
                                break;
                            }
                        }
                        counter++;
                    }
                }


                std::cout << "Path size: " << path.size() << std::endl;
                for (const auto& point : path) {    
                    std::cout << " x: "<< point.first<< " y: " << point.second << std::endl;
                }
                
                current_x=x_end;
                current_y=y_end;

                Initial_state = Camera_State;
                break; // Added break statement
            }
            case Check_Point_State: {
                std::cout << "Check Point State:" << std::endl;

                if (!set_points.empty()){
                    last_point=set_points.back();
                    x_end=last_point.first;
                    y_end=last_point.second;
                    write_data(current_x, current_y);
                }
                
                std::cout << "Check point x "<< x_end << " check point y " << y_end << "\n" << std::endl;
                set_points.pop_back();
                Vertex start(current_x, current_y);
                Vertex goal(x_end, y_end);
                std::cout << "Entering path planning. \n" << std::endl;
                path = robotic.a_star_algorithm(&start, &goal, grid_vertex);
                std::cout << "Exit path planning. \n" << std::endl;
                for (const auto& point : path) {    
                    std::cout << " x: "<< point.first << " y: \n\r" << point.second<< std::endl;
                }
                current_x=x_end;
                current_y=y_end;
                
                Initial_state = IMU_State;
                break; // Added break statement
            }
            case Camera_State: {
                std::cout << "Camera State: " << std::endl;
        
                Zed.zed_open_camera();
                
                depth_data =Zed.zed_depth_sense();
           
                imu_data =Zed.IMU();
              
                object_detection_data=Zed.zed_object_detection(depth_data[2]);
             
                temp_path=path[1];
                imu_X=temp_path.first*12;
                imu_Z=temp_path.second*12;
                imu_Y=0;
                Zed.zed_close();
                std::cout << "\n----------------------------------------\n" << std::endl;

                std::cout << imu_X<< " Current X in inches point."<< std::endl;
		        std::cout << imu_X/12 << " Current x in feet point" << std::endl;
                std::cout << imu_Y << " Current Y in inches point"<< std::endl;
		        std::cout << imu_Y/12 << "Current Y in feet point" << std::endl;
                std::cout << imu_Z*22.56 << " Current Z in inches point" <<std::endl;
                std::cout << imu_Z << " Current Z in inches point without the factor." <<std::endl;
		        std::cout << imu_Z*22.56/12 << " Current Z in feet point.\n" << std::endl;

              //  std::cout << "\n----------------------------------------\n" << std::endl;
                if (object_detection_data.size()==8){
                    for (int i=0; i < object_detection_data.size(); i++){
                        std::cout << object_detection_data[i][0]  << " X point ";
                        std::cout << object_detection_data[i][1] << " Y point ";
                        std::cout << object_detection_data[i][2] << " Z point "<< object_detection_data[i][3] << " Is the current point.\n" <<std::endl;
                     }
                }
                
                std::cout << "\n----------------------------------------\n" << std::endl;
                std::cout << depth_data[0] << " Depth data x" << std::endl;
                std::cout << depth_data[1] << " Depth data y" << std::endl;
                std::cout << depth_data[2] << " Depth data z" << std::endl;
                std::cout << "\n----------------------------------------\n" << std::endl;
                
                Initial_state = IMU_State;
                break; // Added break statement
            }
            case IMU_State: {

                std::cout << "IMU State:" << std::endl;
                
                imu_delta_x=imu_X/12;
               
                imu_delta_y=imu_Z/12;
                 
                if (second_pass>=2) {
                    imu_x=imu_prev_x+imu_delta_x;
                    imu_y=imu_prev_y+imu_delta_y;
             
                    Vertex current_location = grid_vertex[imu_x][imu_y];  
                    current_location.x=imu_x;
                    current_location.y=imu_y;
                   
                    write_data(current_location.x, current_location.y);
                    //current_location.current_velocity=std::sqrt(std::pow((2*current_location.x)/current_time,2)+std::pow((2*current_location.y)/current_time,2)); 
                    //current_location.current_acceleration=std::sqrt((2* current_location.x) /(std::pow(current_time,2)) + (2* current_location.x)/(std::pow(current_time,2)));
                    std::cout << "\n----------------------------------------\n" << std::endl;
                    std::cout << current_location.x << " Current Grid Point x."<< std::endl;
                    std::cout << current_location.y << " Current Grid Point y."<< std::endl;
               //     std::cout << current_location.current_velocity << " Current Grid velocity." << std::endl;
               //     std::cout << current_location.current_acceleration << " Current Grid Point acceleration." <<std::endl;
               //     std::cout << "\n----------------------------------------\n" << std::endl;
                }
                if (second_pass>2) {
                    //current_location.initial_acceleration=next_acceleration;   
                    //current_location.initial_velocity=next_velocity;
                    //current_location.delta_velocity=current_location.current_velocity-current_location.initial_velocity;
                    //current_location.delta_acceleration=current_location.current_acceleration-current_location.initial_acceleration;
                    //next_velocity= current_location.current_velocity;
                    //next_acceleration= current_location.current_velocity;
                    current_location.robot_location=true;
                  //  std::cout << "\n----------------------------------------\n" << std::endl;
                  //  std::cout << current_location.initial_acceleration << "Initial acceleration." << std::endl;
                  //  std::cout << current_location.initial_velocity << "Initial velocity." << std::endl;
                   // std::cout << current_location.delta_velocity << "delta velocity." << std::endl;
                  //  std::cout << "\n----------------------------------------\n" << std::endl;
                }
                if (second_pass>=3){
                  //  Vertex old_location=grid_vertex[imu_prev_x][imu_prev_y];
                  //  imu_prev_x=imu_x;
                  //  imu_prev_y=imu_y;
                  //  grid_vertex[imu_prev_x][imu_prev_y]=old_location;
                }
           
                Initial_state = Update_State;
                break; // Added break statement
            }
            case Object_Depth_State: {
                std::cout << "Object Depth State " << std::endl;
                last_point=path[0];

                float x_start=last_point.first;
                float y_start=last_point.second;
     
                if (object_detection_data.size()==8){
    
                foresight_point=path[1];
                forsight_delta_x=foresight_point.first-x_start;
                //std::cout << "\n----------------------------------------\n" << std::endl;
                //std::cout << forsight_delta_x << " projected point for x."<<std::endl;
                forsight_delta_y=foresight_point.second-y_start;
                //std::cout << forsight_delta_y << " projected point for y."<<std::endl;
                hypotunuse=std::sqrt(std::pow(forsight_delta_x,2)+std::pow(forsight_delta_y,2));
                //std::cout << hypotunuse << " The hypotunuse vector to the next point."<<std::endl;
                std::cout << "\n----------------------------------------\n" << std::endl;
                if (std::abs(forsight_delta_x) > std::abs(forsight_delta_y)){

                   // std::cout << "Check by x for the first case." << std::endl;
                    Vertex object_detected=grid_vertex[std::floor(x_start+static_cast<int>(std::abs(depth_data[2]/12)))][std::floor(y_start)];
                    object_detected.obstacle=true;
                    grid_vertex[std::floor(x_start+static_cast<int>(std::abs( depth_data[2]/12)))][std::floor(y_start)]=object_detected;
                }
                if (std::abs(forsight_delta_x)<std::abs(forsight_delta_y)){
                    //std::cout << "Check for the second case for y." << std::endl;
                    Vertex object_detected=grid_vertex[y_start + static_cast<int>(std::abs(depth_data[2]/12))][y_start];
                    object_detected.obstacle=true;
                    grid_vertex[y_start + static_cast<int>(std::abs(depth_data[2]/12))][x_start]=object_detected;                    
                }
                }
                else if(std::abs(depth_data[1])<=30) {
                    std::cout << "Third case" << std::endl;
                    foresight_point=path[1];
                    
                    forsight_delta_x=foresight_point.first-x_start;
                    std::cout << "\n----------------------------------------\n" << std::endl;

                    std::cout << forsight_delta_x << " projected point for x."<<std::endl;
                    forsight_delta_y=foresight_point.second-y_start;
                    std::cout << forsight_delta_y << " projected point for y."<<std::endl;
                    hypotunuse=std::sqrt(std::pow(forsight_delta_x,2)+std::pow(forsight_delta_y,2));
                    if (std::abs(forsight_delta_x) > std::abs(forsight_delta_y)){
                    std::cout << "check by x." << std::endl;
                    float direction=forsight_delta_x/std::abs(forsight_delta_x);
                    Vertex object_detected=grid_vertex[x_start+direction*static_cast<int>(std::abs(depth_data[1])/12)][y_start];
                    object_detected.obstacle=true;
                    grid_vertex[object_detected.x][object_detected.y]=object_detected;
                    }
                    if (std::abs(forsight_delta_x)<std::abs(forsight_delta_y)){
                        std::cout << "check by y." << std::endl;
                        float direction=forsight_delta_y/std::abs(forsight_delta_y);
                        std::cout << forsight_delta_y  << std::endl;
                        std::cout << forsight_delta_x  << std::endl;
                        Vertex object_detected = grid_vertex[x_start][y_start+direction*static_cast<int>(std::abs(depth_data[1])/12)];
                        object_detected.obstacle=true;
                        grid_vertex[object_detected.x][object_detected.y]=object_detected;
                    }
                }
                Vertex start(x_start, y_start);
                Vertex goal(x_end, y_end);
                path = robotic.a_star_algorithm(&start, &goal, grid_vertex);
                std::cout << "Exit path planning. \n" << std::endl;
                last_point=path.front();
                write_data(last_point.first, last_point.second);
                for (const auto& point : path) {    
                    std::cout << " x: "<< point.first << " y: " << point.second << std::endl;
                }   
                 //  float direction=forsight_delta_y/std::abs(forsight_delta_y);
                //   Vertex object_detected=grid_vertex[current_location.x][current_location.y+direction*static_cast<int>(depth_data[1]/12)];
                 //  object_detected.obstacle=true;
                //   grid_vertex[object_detected.x][object_detected.y]=object_detected;
                
                Initial_state = Send_State;
                break; // Added break statement
            }
            case Update_State: {
 
                std::cout << "Update State:" << std::endl;
                std::cout << imu_X << std::endl;
                std::cout << imu_Z << std::endl;
                Vertex current_location = grid_vertex[imu_X/12][imu_Z/12];  
                
                float x_start=imu_X/12;
                float y_start=imu_Z/12;
                Vertex start(x_start, y_start);
                Vertex goal(x_end, y_end);
 
                std::cout << x_start<< " x start." << std::endl;
                std::cout << y_start<< " y start." << std::endl;
                std::cout << x_end << " x end." << std::endl;
                std::cout << y_end << " y end." << std::endl;
                std::cout << "Entering path planning. \n" << std::endl;
                path = robotic.a_star_algorithm(&start, &goal, grid_vertex);
                std::cout << "Exit path planning. \n" << std::endl;
                last_point=path.front();
                write_data(last_point.first, last_point.second);
                for (const auto& point : path) {    
                    std::cout << " x: "<< point.first << " y: " << point.second << std::endl;
                }          
                Initial_state=Error_Kinematic_State;
                break;
            }
            case Error_Kinematic_State: {
                //std::cout << "Error Adjustment State:" << std::endl;
                Initial_state = Object_Depth_State;
                break; // Added break statement
            }
            case Send_State: {
                std::cout << "Send State:" << std::endl;
                if (start_rover==false){
                    last_point=path[1];
                }
                //last_point=path.front();
              
                float x_coordinate=last_point.first;
                float y_coordinate=last_point.second;
                write_data(x_coordinate, y_coordinate);
                float angular_position = std::sqrt(std::pow(x_coordinate, 2) + std::pow(y_coordinate, 2));
                float angle_radians = 0;
                std::cout<<"Send the coords: " << x_coordinate<< " x " << y_coordinate <<" y are the coords sent to serial."<< std::endl;
                
                forsight_delta_x=x_coordinate-current_location.x;
                std::cout << forsight_delta_x<< " Projected x change." << std::endl;
                forsight_delta_y=y_coordinate-current_location.y;
                hypotunuse=std::sqrt(std::pow(forsight_delta_x,2)+std::pow(forsight_delta_y,2));
                std::cout << forsight_delta_y<< " Projected y change." << std::endl;
                std::cout << "Previous x " <<  prev_x << std::endl;
                std::cout << "Previous y " <<  prev_y << std::endl;
                if (std::abs(forsight_delta_x)>std::abs(forsight_delta_y) && ((x_coordinate!=prev_x) || (y_coordinate!=prev_y))){
                    direction=forsight_delta_x/std::abs(forsight_delta_x);  
                    std::cout << "Turning via the x coord." << std::endl;  
                   // coord_x_to_send = direction*circular_points[i].first+current_location.x;
                    //direction=forsight_delta_y/std::abs(forsight_delta_y);
                  
                   // coord_y_to_send = direction*circular_points[i].second+current_location.y;
                   // std::cout << coord_y_to_send << "The coord to send to y."<< std::endl;    
                   if (direction==1){
                     r=0;
                     s=1;
                   }
                   else if (direction==-1){
                      r=1;
                      s=0;
                   }
                    //angular_position = std::sqrt(std::pow(coord_x_to_send,2)+std::pow(coord_y_to_send,2));
                    //Serial.protocal_write( s, r);
                            // while (true){
                            //  std::string received_data = Serial.protocal_read();
                            //  size_t n=256;
                            //  if (received_data=="d"){
                            //     std::cout << received_data<< " Recieved the data." <<std::endl;
                            //     break;
                            //     }
                            // }
                       }                       
                
                else if (std::abs(forsight_delta_y)>std::abs(forsight_delta_x) && ((x_coordinate!=prev_x) || (y_coordinate!=prev_y))){
                    //coord_y_to_send = direction*circular_points[i].second+current_location.y;
                    direction=forsight_delta_y/std::abs(forsight_delta_y);
                   std::cout << "Turning via the y coord." << std::endl;
                   // angular_position = std::sqrt(std::pow(coord_x_to_send,2)+std::pow(coord_y_to_send,2));
                        if (direction==1){
                            float r=0;
                            float s=1;
                        }
                        else if (direction==-1){
                            float r=1;
                            float s=0;
                        }
                           //Serial.protocal_write(s,r);
                            // while (true){
                            //  std::string received_data = Serial.protocal_read();
                            //  size_t n=256;
                            //  if (received_data=="d"){
                            //      std::cout << received_data<< " Recieved the data." <<std::endl;
                            //     break;
                            //     }
                            // }
                    }  
                
                else if (std::abs(forsight_delta_x)>std::abs(forsight_delta_y)){
                    std::cout << "Move straight for the x coord." << std::endl;
                    float direction=forsight_delta_x/std::abs(forsight_delta_x);
                    
                    if (direction==1){
                        r=1;
                        s=1;
                       
                    }

                    //Serial.protocal_write( s, r);
                    // while (true){
                    //  std::string received_data = Serial.protocal_read();
                    //  size_t n=256;
                    //  if (received_data=="d"){
                    //     std::cout << received_data<< " Recieved the data." <<std::endl;
                    //     break;
                    //     }
                    // }
                }

                else if (std::abs(forsight_delta_x)<std::abs(forsight_delta_y)){
                    std::cout << "Move forward for the y coord." << std::endl;
                    float direction=forsight_delta_y/std::abs(forsight_delta_y);
                    if (direction==1){
                        r=1;
                        s=1;
                    }
                    //else if (direction==-1){
                    //    r=1;
                    //    s=1;
                    //}
                    //Serial.protocal_write(s, r);
                   //std::cout << angle_radians << " the current angle y\n"<<std::endl;
                    // while (true){
                    //     std::string received_data = Serial.protocal_read();
                    //     size_t n=256;
                    //     if (received_data=="d"){
                    //         std::cout << received_data<< " Recieved the data." <<std::endl;
                    //         break;
                    //     }
                    // } 
                }

                if (start_rover==true){
                    current_time = std::chrono::duration_cast<std::chrono::duration<float>>(std::chrono::high_resolution_clock::now() - start).count();
                    std::cout << "The current time: " << current_time <<std::endl;
                }
             
                start = std::chrono::high_resolution_clock::now();
                start_rover=true;
                           
                second_pass++; 
                check_check_point=path[0];
                if (set_points.empty()&& path.size()==1){
                    std::cout << "Congraulations you have autominously traversed the course" << std::endl;
                    exit(1);
                }
                if (path.size()==1){

                    Initial_state = Check_Point_State;
                }
                else {
                    Initial_state = Camera_State;
                }
                prev_x=x_coordinate;
                prev_y=y_coordinate;
                break; // Added break statement
            }
            default:
                Initial_state = IMU_State; // Default case to handle any undefined state
                break; // Added break statement
            }
    
    }
    return EXIT_SUCCESS;
}