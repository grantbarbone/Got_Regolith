#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/lidar_data.hpp>
//#include <custom_msgs/msg/stereo_camera_data.hpp>
//#include <custom_msgs/msg/radar_data.hpp>
#include <custom_msgs/msg/graph_algorithm_data.hpp>
//#include <custom_msgs/msg/motor_controls.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include "cpp_pubsub/events.hpp"

namespace sc = boost::statechart;

// Forward declaration of states
struct SensorState;
struct LidarSensorState;
//struct StereoCameraState;
//struct RadarSensorState;
struct GraphAlgorithmState;

class SensorStateMachine : public sc::state_machine<SensorStateMachine, SensorState>, public rclcpp::Node {
public:
	SensorStateMachine()
    	: Node("sensor_state_machine")
	{
    	// Publishers
    	lidar_publisher_ = this->create_publisher<custom_msgs::msg::LidarData>("lidar_data", rclcpp::QoS(10));
    	//stereo_camera_publisher_ = this->create_publisher<custom_msgs::msg::StereoCameraData>("stereo_camera_data", rclcpp::QoS(10));
    	//radar_publisher_ = this->create_publisher<custom_msgs::msg::RadarData>("radar_data", rclcpp::QoS(10));
    	graph_algorithm_publisher_ = this->create_publisher<custom_msgs::msg::GraphAlgorithmData>("graph_algorithm_data", rclcpp::QoS(10));

    	// Subscribers
    	lidar_subscriber_ = this->create_subscription<custom_msgs::msg::LidarData>("lidar_data", rclcpp::QoS(10),
        	std::bind(&SensorStateMachine::lidar_callback, this, std::placeholders::_1));
    	//stereo_camera_subscriber_ = this->create_subscription<custom_msgs::msg::StereoCameraData>("stereo_camera_data", rclcpp::QoS(10),
    	//	std::bind(&SensorStateMachine::stereo_camera_callback, this, std::placeholders::_1));
    	//radar_subscriber_ = this->create_subscription<custom_msgs::msg::RadarData>("radar_data", rclcpp::QoS(10),
    	//	std::bind(&SensorStateMachine::radar_callback, this, std::placeholders::_1));
    	graph_algorithm_subscriber_ = this->create_subscription<custom_msgs::msg::GraphAlgorithmData>("graph_algorithm_data", rclcpp::QoS(10),
        	std::bind(&SensorStateMachine::graph_algorithm_callback, this, std::placeholders::_1));
	}

	// Accessor method for graph_algorithm_publisher_
	rclcpp::Publisher<custom_msgs::msg::GraphAlgorithmData>::SharedPtr getGraphAlgorithmPublisher() const {
    	return graph_algorithm_publisher_;
	}
	rclcpp::Publisher<custom_msgs::msg::LidarData>::SharedPtr getLidarSensorStatePublisher() const {
        return lidar_publisher_;
    }


private:
	void lidar_callback(const custom_msgs::msg::LidarData::SharedPtr msg) {
    	this->process_event(EventLidarData(msg));
	}

	//void stereo_camera_callback(const custom_msgs::msg::StereoCameraData::SharedPtr msg) {
	//	this->process_event(EventStereoCameraData(msg));
	//}

	//void radar_callback(const custom_msgs::msg::RadarData::SharedPtr msg) {
	//	this->process_event(EventRadarData(msg));
	//}

	void graph_algorithm_callback(const custom_msgs::msg::GraphAlgorithmData::SharedPtr msg) {
    	this->process_event(EventGraphAlgorithmData(msg));
	}

	rclcpp::Publisher<custom_msgs::msg::LidarData>::SharedPtr lidar_publisher_;
	//rclcpp::Publisher<custom_msgs::msg::StereoCameraData>::SharedPtr stereo_camera_publisher_;
	//rclcpp::Publisher<custom_msgs::msg::RadarData>::SharedPtr radar_publisher_;
	rclcpp::Publisher<custom_msgs::msg::GraphAlgorithmData>::SharedPtr graph_algorithm_publisher_;

	rclcpp::Subscription<custom_msgs::msg::LidarData>::SharedPtr lidar_subscriber_;
	//rclcpp::Subscription<custom_msgs::msg::StereoCameraData>::SharedPtr stereo_camera_subscriber_;
	//rclcpp::Subscription<custom_msgs::msg::RadarData>::SharedPtr radar_subscriber_;
	rclcpp::Subscription<custom_msgs::msg::GraphAlgorithmData>::SharedPtr graph_algorithm_subscriber_;
};

// SensorState
struct SensorState : sc::simple_state<SensorState, SensorStateMachine> {
	using reactions = sc::custom_reaction<EventLidarData>;

    sc::result react(const EventLidarData &event) {
        auto lidar_data = event.lidar_data;
        // Process lidar_data if needed
        return transit<LidarSensorState>();
    
	//sc::result react(const EventStereoCameraData &) {
	//	return transit<StereoCameraState>();
	//}

	//sc::result react(const EventRadarData &) {
	//	return transit<RadarSensorState>();
	//}

	// Mark unused parameters explicitly to suppress warnings
};


// LidarSensorState
struct LidarSensorState : sc::simple_state<LidarSensorState, SensorStateMachine> {
    using reactions = sc::custom_reaction<EventLidarData>;

    sc::result react(const EventLidarData &event) {
        auto lidar_data = event.lidar_data;

        custom_msgs::msg::GraphAlgorithmData graph_algorithm_data;

        
		if ( graph_algorithm_data.x_coordinate== 5.0) {
			lidar_data->ranges.clear();
			lidar_data->ranges.push_back(20.0);

		} else {
			lidar_data->ranges.clear();
			lidar_data->ranges.push_back(10.0);
		}
        
        // Publish graph_algorithm_data using the state machine's publisher
        outermost_context().getGraphAlgorithmPublisher()->publish(graph_algorithm_data);

        return transit<GraphAlgorithmState>(); // Transition to GraphAlgorithmState after processing event
    }
};

};

// StereoCameraState
//struct StereoCameraState : sc::simple_state<StereoCameraState, SensorStateMachine> {
//	typedef sc::custom_reaction<EventStereoCameraData> reactions;
//
//	sc::result react(const EventStereoCameraData &) {
//    	custom_msgs::msg::GraphAlgorithmData graph_data;
//    	// graph_data.data = calculate_graph_data(event.data);  // Assuming event.data is available and correct
//    	outermost_context().getGraphAlgorithmPublisher()->publish(graph_data);
//    	return transit<GraphAlgorithmState>();
//	}
//
//	// Mark unused parameters explicitly to suppress warnings
//	sc::result react(const EventLidarData &) {
//    	return discard_event();
//	}
//
//	sc::result react(const EventRadarData &) {
//    	return discard_event();
//	}
//
//	sc::result react(const EventGraphAlgorithmData &) {
//    	return discard_event();
//	}
//};

// RadarSensorState
//struct RadarSensorState : sc::simple_state<RadarSensorState, SensorStateMachine> {
//	typedef sc::custom_reaction<EventRadarData> reactions;
//
//	sc::result react(const EventRadarData &event) {
//    	auto graph_algorithm_msg = calculate_graph_data(event.data);  // Function call to calculate graph data
//    	outermost_context().getGraphAlgorithmPublisher()->publish(*graph_algorithm_msg);
//    	return transit<GraphAlgorithmState>();  // Transition to GraphAlgorithmState after processing event
//	}
//
//	// Mark unused parameters explicitly to suppress warnings
//	sc::result react(const EventLidarData &) {
//    	return discard_event();
//	}
//
//	sc::result react(const EventStereoCameraData &) {
//    	return discard_event();
//	}
//
//	sc::result react(const EventGraphAlgorithmData &) {
//    	return discard_event();
//	}
//};

// GraphAlgorithmState
struct GraphAlgorithmState : sc::simple_state<GraphAlgorithmState, SensorStateMachine> {
 using reactions = sc::custom_reaction<EventGraphAlgorithmData>;

    sc::result react(const EventGraphAlgorithmData &event) {
        auto graph_algorithm_data = event.graph_algorithm_data;
        
        
        custom_msgs::msg::LidarData lidar_data;

        custom_msgs::msg::GraphAlgorithmData graph_data;

        if (!lidar_data.ranges.empty()) {
            // Use the first range value as an example
            float range = lidar_data.ranges[0];

            if (range == 5.0) {
                graph_data.x_coordinate = 5.0;
                graph_data.y_coordinate = 5.0;
            } else {
                graph_data.x_coordinate = 1.0;
                graph_data.y_coordinate = 1.0;
            }
        } else {
            // Handle case where ranges is empty (or adjust as needed)
            graph_data.x_coordinate = 0.0;
            graph_data.y_coordinate = 0.0;
        }

        // Publish graph_data using the state machine's publisher
        outermost_context().getGraphAlgorithmPublisher()->publish(graph_data);

        return transit<SensorState>();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorStateMachine>());
    rclcpp::shutdown();
    return 0;
}
