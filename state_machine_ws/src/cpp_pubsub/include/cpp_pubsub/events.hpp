#ifndef EVENTS_HPP
#define EVENTS_HPP

#pragma once
#include <boost/statechart/event.hpp>
#include <custom_msgs/msg/lidar_data.hpp>
#include <custom_msgs/msg/stereo_camera_data.hpp>
#include <custom_msgs/msg/radar_data.hpp>
#include <custom_msgs/msg/graph_algorithm_data.hpp>
#include <custom_msgs/msg/motor_controls.hpp>

struct EventLidarData : boost::statechart::event<EventLidarData> {
    EventLidarData(custom_msgs::msg::LidarData::SharedPtr lidar_data)
        : lidar_data(lidar_data) {}
    custom_msgs::msg::LidarData::SharedPtr lidar_data;
};

/*struct EventStereoCameraData : boost::statechart::event<EventStereoCameraData> {
    explicit EventStereoCameraData(custom_msgs::msg::StereoCameraData_<std::allocator<void>>::SharedPtr msg)
        : msg_(msg) {}
    
    custom_msgs::msg::StereoCameraData_<std::allocator<void>>::SharedPtr msg_;
};*/
/*struct EventRadarData : boost::statechart::event<EventRadarData> {
    explicit EventRadarData(custom_msgs::msg::RadarData_<std::allocator<void>>::SharedPtr msg)
        : msg_(msg) {}
    
    custom_msgs::msg::RadarData_<std::allocator<void>>::SharedPtr msg_;
};*/

struct EventGraphAlgorithmData : boost::statechart::event<EventGraphAlgorithmData> {
    EventGraphAlgorithmData(custom_msgs::msg::GraphAlgorithmData::SharedPtr graph_algorithm_data)
        : graph_algorithm_data(graph_algorithm_data) {}
    custom_msgs::msg::GraphAlgorithmData::SharedPtr graph_algorithm_data;
};

#endif // EVENTS_HPP
