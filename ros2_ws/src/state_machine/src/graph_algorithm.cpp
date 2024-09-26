#include <ctime>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <iostream>
#include <queue>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <visualization_msgs/msg/marker.hpp>

using namespace std::chrono_literals; //namespace itme package
using std::placeholders::_1; //name place ros2

  enum Robot_Pathing_State
  {
      Recieve_State, Setup_State, IMU_State, Object_Depth_State, Error_Kinematic_State, Initial_Path_Planning_State, Kinematic_Adjustment_State, Smooth_Out_State, Send_State
  };

struct Vertex { //Vertex struct.
    float x, y; //x and y.
    float path_cost; //path cost.
    float euclidean_cost; //euclidean cost.
    float final_cost; //final cost.
    Vertex* parent; //parent pointer.
    bool open, closed, obstacle; //open, closed and obstacle booleans.
    bool robot_found;
    Vertex(float x, float y, bool obstacle = false) //Vertex instance call
        : x(x), y(y), path_cost(std::numeric_limits<float>::infinity()), //x, y, path cost function creations.
          euclidean_cost(0.0), final_cost(path_cost + euclidean_cost), //euclidean cost, final cost euclidean cost function creation.
          parent(nullptr), open(false), closed(false), obstacle(obstacle), robot_found(false) {} //parent pointer, open, closed and obstacle boolean. 

    bool operator<(const Vertex& other) const { //operator overloader for priority queue.
        return (path_cost != other.path_cost) ? (path_cost > other.path_cost) : //path cost is not equal to path cost.
               (euclidean_cost != other.euclidean_cost) ? (euclidean_cost > other.euclidean_cost) : //euclidean cost is equal to other euclidiean cost.
               (final_cost > other.final_cost); //final cost is not equal to other final cost.
    } //operator overloader end

    bool operator==(const Vertex& other) const { //operator overloader. 
        return x == other.x && y == other.y; //compare x and y.
    } //endopoverloader
}; //endstruct

class RoboticPathPlanning { //Robotic PathPlaning
public: //public
    struct CompareVertex { //Compare Vertex
        bool operator()(Vertex* const& v1, Vertex* const& v2) { //operator 
            return v1->final_cost > v2->final_cost; //compare final cost
        } // endopoverloader
    }; //endstruct

    std::vector<std::vector<Vertex>> fillGridWithVertices(const std::vector<std::vector<float>>& grid); //prototype

    float euclidean_calculation(const Vertex& a, const Vertex& b); //prototype

    std::priority_queue<Vertex*, std::vector<Vertex*>, CompareVertex> getNeighbors(Vertex* node, Vertex* goal, std::vector<std::vector<Vertex>>& grid); //prototype

    std::vector<std::pair<float, float>> a_star_algorithm(Vertex* start, Vertex* goal, std::vector<std::vector<Vertex>> grid); //prototype
}; //endclass

/**
 * Create 2D vertex grid.
 * @param grid 2D vector, with floats.
 * @return 2D vector of Vertexes
 */

std::vector<std::vector<Vertex>> RoboticPathPlanning::fillGridWithVertices(const std::vector<std::vector<float>>& grid) { //function call
    std::vector<std::vector<Vertex>> vertices_grid; //2D vector of Vertexes
    vertices_grid.reserve(grid.size()); //vertex_grid resize

    for (size_t i = 0; i < grid.size(); ++i) { //for loop to grid size
        std::vector<Vertex> vertex_row; //create row
        for (size_t j = 0; j < grid[i].size(); ++j) { //for loop to grid size
            bool is_obstacle = (grid[i][j] == 1); //create obstacle boolean
            Vertex vertex_instance(i, j, is_obstacle); //create vertex 
            vertex_row.push_back(vertex_instance); //add new vertex
        } //endloop 2
        vertices_grid.push_back(vertex_row); //add vertex row
    }//endloop 1
    return vertices_grid; //return vertex grid
}  //endfunc

/**
 * Caluclation eauclidean distance.
 * @param current_location create vertex .
 * @param goal 2D vector, with floats.
 * @return 2D vector of Vertexes
 */

float RoboticPathPlanning::euclidean_calculation(const Vertex& current_location, const Vertex& goal) { //function call
    return abs(current_location.x - goal.x) + abs(current_location.y - goal.y); //return path distance
} //endfunc

/**
 * get neighbor vertexes.
 * @param node Current Location.
 * @param goal end goal vertex.
 * @param grid 2D vertex of grid points.
 * @return return priority queue with 2D vertex and compare vetex operator overloader.
 */

std::priority_queue<Vertex*, std::vector<Vertex*>, RoboticPathPlanning::CompareVertex> 

RoboticPathPlanning::getNeighbors(Vertex* node, Vertex* goal, std::vector<std::vector<Vertex>>& grid) { //function call
    std::priority_queue<Vertex*, std::vector<Vertex*>, CompareVertex> neighbors;
    
    float dx[] = {-1, 1, 0, 0}; //dx points 
    float dy[] = {0, 0, -1, 1}; //dy points

    float current_path_cost = std::isinf(node->path_cost) ? 1.0f : node->path_cost + 1.0f; //current path cost

    for (int i = 0; i < 4; ++i) { //for loop all 4 directions
        float x = node->x + dx[i]; //add dx change
        float y = node->y + dy[i]; //add dy change

        if (x < 0 || x >= grid[0].size() || y < 0 || y >= grid.size()) { //check grid sides
            continue;
        }//endif
        Vertex* neighbor = &grid[x][y]; //create neightbors vertexes
        if (!neighbor->obstacle && !neighbor->open && !neighbor->closed) { //check all three flags
            neighbor->euclidean_cost = RoboticPathPlanning::euclidean_calculation(*neighbor, *goal); //cacluate euclidean distance
            neighbor->path_cost = current_path_cost; //calculate the path
            neighbor->final_cost = neighbor->path_cost + neighbor->euclidean_cost; //calculate the final cost
            neighbor->open = true; // set priority queue bool to true
            neighbors.push(neighbor); //push the neighbor to neighbors
        }//endif
    }//endif
    return neighbors; //return neighbors list 
} //endif

/**
 * get neighbor vertexes.
 * @param start start Location.
 * @param goal end goal vertex.
 * @param grid 2D vertex of grid points.
 * @return return path of x by y locations.
 */

std::vector<std::pair<float, float>> RoboticPathPlanning::a_star_algorithm(Vertex* start, Vertex* goal, std::vector<std::vector<Vertex>> grid) { //function call
    std::priority_queue<Vertex*, std::vector<Vertex*>, CompareVertex> open_list; //create the open list
    start->path_cost = 0; //set the path cost to zero
    start->euclidean_cost = RoboticPathPlanning::euclidean_calculation(*start, *goal); //calculate the initial euclidean cost
    open_list.push(start); //

    while (!open_list.empty()) {
        Vertex* current = open_list.top();
        open_list.pop();

        if (current->x == goal->x && current->y == goal->y) {
            std::vector<std::pair<float, float>> path;
            while (current) {
                path.push_back({current->x, current->y});
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        current->closed = true;
        std::priority_queue<Vertex*, std::vector<Vertex*>, CompareVertex> neighbors = getNeighbors(current, goal, grid);
        while (!neighbors.empty()) {
            Vertex* neighbor = neighbors.top();
            neighbors.pop();

            neighbor->parent = current;
            open_list.push(neighbor);
        }
    }

    return std::vector<std::pair<float, float>>();
}

class GraphAlgorithm : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   *
   */

  GraphAlgorithm() : Node("Graph_Algorithm_Pubsub"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("topic_Graph_Algorithm_to_Main", 10);
    subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "topic_Main_to_Graph_Algorithm", 10, std::bind(&GraphAlgorithm::topic_callback, this, std::placeholders::_1));
  }
  void start_publishing() {
    graph_algorithm_timer_ = this->create_wall_timer(
      5ms, std::bind(&GraphAlgorithm::timer_callback, this)); // Restart the timer
  }
  void stop_publishing() {
    graph_algorithm_timer_->cancel(); // Stop the timer
  }
  void set_path(std::vector<std::pair<float, float>> path_setter){
    path=path_setter;
  }
  std::vector<std::pair<float, float>> get_path(){
    return path;
  }

  void set_camera_data(std::vector<float> input_data){
    camera_data=input_data;
  }
  
  std::vector<float> get_camera_data(){
    return camera_data;
  }
  bool is_data_available() {
    return data_available_;
  }

  void reset_data_flag() {
    data_available_ = false;
  }
  void set_setup_flag(bool setup){
    setup_flag=setup;
  }

  bool get_setup_flag(){
    return setup_flag;
  }

  
 private:
  /**
   * @brief callback function
   *
   */

void timer_callback() {
    auto message = std_msgs::msg::Float32MultiArray();

    // Call get_path() to retrieve the path data
    std::vector<std::pair<float, float>> path = get_path();

    // Reserve space for the data
    message.data.reserve(path.size() * 2); // Each point has x and y

    // Populate the message with path data
    for (const auto& point : path) {
        message.data.push_back(point.first);  // x coordinate
        message.data.push_back(point.second); // y coordinate
    }

    // Create a string to hold the data values for logging
    std::string data_str;
    for (const auto &value : message.data) {
        data_str += std::to_string(value) + " ";
    }

    
    // Log the data values
    //RCLCPP_INFO(this->get_logger(), "Publishing message data to Main: [%s]", data_str.c_str());

    // Publish the message
    publisher_->publish(message);
}

  void topic_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    std::vector<float> recieve_data;
    for (size_t i = 0; i < msg->data.size(); ++i) {
        recieve_data.push_back(msg->data[i]);
        //RCLCPP_INFO(this->get_logger(), "Recieving Element %zu: %d", i, msg->data[i]); 
    }
    set_camera_data(recieve_data);
    data_available_ = true; // Set the flag when data is received
    
  }


    //stop_publishing();
  

  rclcpp::TimerBase::SharedPtr graph_algorithm_timer_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscription_;
  size_t count_;
  std::vector<std::pair<float, float>> path;
  std::vector<float> camera_data;
  bool data_available_;
  bool setup_flag=true;
};

  Robot_Pathing_State Initial_state= Recieve_State;
//  Recieve_State, IMU_State, Object_Depth_State, Error_Kinematic_State, Initial_Path_Planning_State, Kinematic_Adjustment_State, Smooth_Out_State, Send_State

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    // Create an instance of the Graph_Algorithm ROS 2 node
    auto node = std::make_shared<GraphAlgorithm>();
    // Initialize ROS 2
    RoboticPathPlanning robotic;
    std::vector<float> data;
   
    std::vector<std::vector<float>> grid = {
                    {0, 0, 0, 0, 0},
                    {0, 1, 1, 1, 0},
                    {0, 0, 0, 0, 0},
                    {0, 1, 1, 1, 0},
                    {0, 0, 0, 0, 0}
                };
    Vertex start(0, 0);
    Vertex goal(4, 4);
    while (rclcpp::ok()) {
        switch (Initial_state) { 
            case Recieve_State: {
                std::vector<float>camera_data=node->get_camera_data();
                node->reset_data_flag(); // Reset the flag
                while (!node->is_data_available()) {
                    rclcpp::spin_some(node); // Process callbacks to check for data
                    std::vector<float>camera_data=node->get_camera_data();
                }
                if (node->get_setup_flag()){
                    Initial_state=Setup_State;
                    node->set_setup_flag(false);
                }
                else {
                     Initial_state=IMU_State;
                } 
            }
            case Setup_State:
            {
            // Define a grid for path planning
                
            // Define start and goal vertices
            
            // Convert grid to vertices for the path planning algorithm
            std::vector<std::vector<Vertex>> grid_vertex = robotic.fillGridWithVertices(grid);
            // Run the A* algorithm and get the path
            std::vector<std::pair<float, float>> path = robotic.a_star_algorithm(&start, &goal, grid_vertex);
            node->set_path(path);
            std::cout<< "Setup state is working" <<std::endl;
            Initial_state=IMU_State;
            }
            case IMU_State:
            {

                Initial_state=Object_Depth_State;
            }
            case Object_Depth_State:
            {

                Initial_state=Error_Kinematic_State;
            }
            case Error_Kinematic_State:
            {

                Initial_state=Initial_Path_Planning_State;
            }
            case Initial_Path_Planning_State:
            {
                // Define a grid for path planning
                std::vector<std::vector<float>> grid = {
                    {0, 0, 0, 0, 0},
                    {0, 1, 1, 1, 0},
                    {0, 0, 0, 0, 0},
                    {0, 1, 1, 1, 0},
                    {0, 0, 0, 0, 0}
                };
                // Define start and goal vertices
                Vertex start(0, 0);
                Vertex goal(4, 4);
                // Convert grid to vertices for the path planning algorithm
                std::vector<std::vector<Vertex>> grid_vertex = robotic.fillGridWithVertices(grid);
                // Run the A* algorithm and get the path
                std::vector<std::pair<float, float>> path = robotic.a_star_algorithm(&start, &goal, grid_vertex);
                node->set_path(path);
                Initial_state=Kinematic_Adjustment_State;
            }
            case Kinematic_Adjustment_State:
            {

                Initial_state=Smooth_Out_State;
            }
            case Smooth_Out_State:
            {

                Initial_state=Send_State;
            }
            case Send_State:
            {
                node->start_publishing();
                // Spin the node to keep it alive and responding to callbacks
                rclcpp::spin(node);
                Initial_state=Recieve_State;
            }
            default:
                Initial_state=Recieve_State;
        }
    }
    // Shutdown ROS gracefully
    rclcpp::shutdown();
    return 0;
}


