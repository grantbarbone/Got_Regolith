#ifndef ROBOTICPATHPLANNING_H
#define ROBOTICPATHPLANNING_H

#include <functional>
#include <memory>
#include <random>
#include <sl/Camera.hpp>
#include <boost/asio.hpp>
#include <iostream>
#include <fstream>
#include <cstdlib> // For std::rand and RAND_MAX
#include <ctime>   // For std::time
#include <chrono>
#include <string>
#include <queue>
#include <limits>
#include <unordered_map>
#include <unordered_set>

struct Vertex { //Vertex struct.
    float x, y; //x and y.
    float current_velocity; //current velocity.
    float delta_velocity; //change in velocity.
    float initial_velocity; //previous velocity.
    float current_acceleration; //current acceleration.
    float delta_acceleration; //change in acceleration.
    float initial_acceleration; //previous acceleration.
    float path_cost; //path cost.
    float euclidean_cost; //euclidean cost.
    float final_cost; //final cost.
    Vertex* parent; //parent pointer.
    bool open, closed, obstacle; //open, closed and obstacle booleans.
    bool robot_location; //robot is at this location.
    bool charging_station_location; //charging station is at this location.
    Vertex(float x, float y, bool obstacle = false) //Vertex instance call
        : x(x), y(y), path_cost(std::numeric_limits<float>::infinity()), //x, y, path cost function creations.
          euclidean_cost(0.0), final_cost(path_cost + euclidean_cost), //euclidean cost, final cost euclidean cost function creation.
          parent(nullptr), open(false), closed(false), obstacle(obstacle), robot_location(false), //parent pointer, open, closed and obstacle boolean. 
          current_velocity(0.0), delta_velocity(0.0), initial_velocity(0.0), //current velocity, delta velocity and initial velocity.
          current_acceleration(0.0),  delta_acceleration(0.0), initial_acceleration(0.0) {} //current acceleration, delta acceleration and initial acceleration.

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

    void update_imu_controls( std::vector<float> imu_data, std::vector<std::vector<Vertex>> grid); // prototype
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
   // std::cout << node->x<<" Current x " <<node->y << " Current y"<< std::endl;
    for (int i = 0; i < 4; ++i) { //for loop all 4 directions
        
        float x = node->x + dx[i]; //add dx change 
        float y = node->y + dy[i]; //add dy change

       // std::cout << x << " Updated x is it greater then 18." << y << " Updated y is it greater then 15." << std::endl; 
         //grid size x is 18 and the grid size y is 15 
        if (x < 0 || x >= grid[0].size() || y < 0 || y >= grid.size()) { //check grid sides 
            continue;
        }//endif
        Vertex* neighbor = &grid[x][y]; //create neightbors vertexes
        //std::cout << neighbor->obstacle << "\ncheck obstacle " << neighbor->open << " neighbor is open " << neighbor->closed << " neighbor is closed" << std::endl;
        
        if (!neighbor->obstacle && !neighbor->open && !neighbor->closed) { //check all three flags
          //  std::cout<< neighbor->euclidean_cost<< " Euclidean Cost" <<std::endl;  
          //  std::cout<< neighbor->path_cost << " Path cost."  <<std::endl; 
          //  std::cout<< neighbor->final_cost<< " Final cost." <<std::endl; 
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
    open_list.push(start); 
    
    while (!open_list.empty()) {
        
        Vertex* current = open_list.top();
        
        open_list.pop();
       
        if (current->x == goal->x && current->y == goal->y) {
            
            std::vector<std::pair<float, float>> path;
            while (current) {
             //   std::cout << current->x << std::endl;
               //  std::cout << current->y << std::endl;
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
            //std::cout << current->x << " This is the x: " <<current->y << " This is the y: "<< std::endl;
        }
       
    }

    return std::vector<std::pair<float, float>>();
}

#endif 
