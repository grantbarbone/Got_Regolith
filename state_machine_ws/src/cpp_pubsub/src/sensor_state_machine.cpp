#include <rclcpp/rclcpp.hpp>
#include <custom_msgs/msg/lidar_data.hpp>
#include <custom_msgs/msg/graph_algorithm_data.hpp>
#include <boost/statechart/state_machine.hpp>
#include <boost/statechart/simple_state.hpp>
#include <boost/statechart/transition.hpp>
#include <boost/statechart/custom_reaction.hpp>
#include "cpp_pubsub/events.hpp"

namespace sc = boost::statechart;

// Forward declaration of states
struct SensorState;
struct LidarSensorState;
struct GraphAlgorithmState;

class SensorStateMachine : public sc::state_machine<SensorStateMachine, SensorState>, public rclcpp::Node {
public:
    SensorStateMachine()
        : Node("sensor_state_machine")
    {

       // Initialize logging system
        RCLCPP_INFO(this->get_logger(), "Sensor State Machine initialized");
        // Publishers
        lidar_publisher_ = this->create_publisher<custom_msgs::msg::LidarData>("lidar_data", rclcpp::QoS(10));
        graph_algorithm_publisher_ = this->create_publisher<custom_msgs::msg::GraphAlgorithmData>("graph_algorithm_data", rclcpp::QoS(10));
        
        // Subscribers
        lidar_subscriber_ = this->create_subscription<custom_msgs::msg::LidarData>("lidar_data", rclcpp::QoS(10),
            std::bind(&SensorStateMachine::lidar_callback, this, std::placeholders::_1));
        graph_algorithm_subscriber_ = this->create_subscription<custom_msgs::msg::GraphAlgorithmData>("graph_algorithm_data", rclcpp::QoS(10),
            std::bind(&SensorStateMachine::graph_algorithm_callback, this, std::placeholders::_1));

        initiate();  // Start the state machine
    }

    rclcpp::Publisher<custom_msgs::msg::GraphAlgorithmData>::SharedPtr getGraphAlgorithmPublisher() const {
        return graph_algorithm_publisher_;
    }

    rclcpp::Publisher<custom_msgs::msg::LidarData>::SharedPtr getLidarSensorStatePublisher() const {
        return lidar_publisher_;
    }

private:
    void lidar_callback(const custom_msgs::msg::LidarData::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received LidarData message");


        // Check if LidarData is 20.0
        bool is_lidar_20 = false;
        for (float value : msg->ranges) {
            if (std::fabs(value - 20.0) < 0.1) {
                is_lidar_20 = true;
                break;
            }
        }

        custom_msgs::msg::GraphAlgorithmData graph_data;
        graph_data.x_coordinate = 5.0;
        graph_data.y_coordinate = 5.0;

        if (is_lidar_20) {
            RCLCPP_INFO(this->get_logger(), "LidarData is 20.0; Publishing GraphAlgorithmData");
            graph_algorithm_publisher_->publish(graph_data);
        }
    }

    void graph_algorithm_callback(const custom_msgs::msg::GraphAlgorithmData::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Received GraphAlgorithmData message");

        // Check if GraphAlgorithmData x_coordinate and y_coordinate are 5.0
        if (msg->x_coordinate == 5.0 && msg->y_coordinate == 5.0) {
            RCLCPP_INFO(this->get_logger(), "GraphAlgorithmData is (5.0, 5.0); Publishing LidarData");

            custom_msgs::msg::LidarData lidar_data;
            lidar_data.ranges.push_back(20.0); // Echo 20.0 for LidarData
            lidar_publisher_->publish(lidar_data);
        }
    }

    rclcpp::Publisher<custom_msgs::msg::LidarData>::SharedPtr lidar_publisher_;
    rclcpp::Publisher<custom_msgs::msg::GraphAlgorithmData>::SharedPtr graph_algorithm_publisher_;
    rclcpp::Subscription<custom_msgs::msg::LidarData>::SharedPtr lidar_subscriber_;
    rclcpp::Subscription<custom_msgs::msg::GraphAlgorithmData>::SharedPtr graph_algorithm_subscriber_;
};

// SensorState
struct SensorState : sc::simple_state<SensorState, SensorStateMachine> {
    using reactions = sc::custom_reaction<EventLidarData>;

    sc::result react(const EventLidarData &event) {
        RCLCPP_INFO(this->context<SensorStateMachine>().get_logger(), "Processing EventLidarData");
        (void)event;
        return transit<LidarSensorState>();
    }
};

// LidarSensorState
struct LidarSensorState : sc::simple_state<LidarSensorState, SensorStateMachine> {
    using reactions = sc::custom_reaction<EventLidarData>;

    sc::result react(const EventLidarData &event) {
        RCLCPP_INFO(this->context<SensorStateMachine>().get_logger(), "Processing EventLidarData");
        (void)event;

        custom_msgs::msg::GraphAlgorithmData graph_data;
        graph_data.x_coordinate = 5.0;
        graph_data.y_coordinate = 5.0;
        this->context<SensorStateMachine>().getGraphAlgorithmPublisher()->publish(graph_data);

        return transit<GraphAlgorithmState>();
    }
};

// GraphAlgorithmState
struct GraphAlgorithmState : sc::simple_state<GraphAlgorithmState, SensorStateMachine> {
    using reactions = sc::custom_reaction<EventGraphAlgorithmData>;

    sc::result react(const EventGraphAlgorithmData &event) {
        RCLCPP_INFO(this->context<SensorStateMachine>().get_logger(), "Processing EventGraphAlgorithmData");
        (void)event;

        custom_msgs::msg::LidarData lidar_data;
        lidar_data.ranges.push_back(20.0); // Echo 20.0 for LidarData
        this->context<SensorStateMachine>().getLidarSensorStatePublisher()->publish(lidar_data);

        return transit<SensorState>();
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    // Set log level to DEBUG for all nodes
    auto logger = rclcpp::get_logger("sensor_state_machine");
    logger.set_level(rclcpp::Logger::Level::Debug);
 

    rclcpp::spin(std::make_shared<SensorStateMachine>());
    rclcpp::shutdown();
    return 0;
}
