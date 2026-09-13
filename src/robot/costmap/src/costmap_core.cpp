#include "costmap_core.hpp"

#include <vector> 
#include <cmath> 
#include <cstdint> 
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot

{
// Constructor for the CostmapCore class. It fulfills the class arguments + variable members. 
CostmapCore::CostmapCore(const rclcpp::Logger& logger) : logger_(logger) {
    
    // create height rows, each containing width cells. zero set 
    costmap_grid_.assign(height, std::vector<int>(width, 0));
}

void CostmapCore::initializeCostmap() { 
    // Reset all cells to 0 
    // reuse the existing vectors, ovewrite the values - no memory allocation. 
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            costmap_grid_[i][j] = 0;
        }
    }
}

void CostmapCore::laserToGridCord(double range, double angle, int& x_grid, int& y_grid) {
    
    // compute cartesion coordinates 
    double x = range * cos(angle); 
    double y = range * sin(angle); 

    // convert to grid cell index 
    x_grid = static_cast<int>((x - origin_x) / resolution);
    y_grid = static_cast<int>((y - origin_y) / resolution);

}


void CostmapCore::markObstacles(int x_grid, int y_grid){
    // ignore any cords outside the grid 
    if (x_grid >= 0 && x_grid < width && y_grid >= 0 && y_grid < height){
        costmap_grid_[y_grid][x_grid] = 100; // mark as obstacle
    }

}

void CostmapCore::inflateObstacles(){
    
    // copy the grid and write to inflated 
    // prevents already inflated cells from being re-inflated.
    auto inflated_grid = costmap_grid_;

    // Marks obstacles and inflates costs around them using a linear scale with a defined radius. 
    for (int y=0; y < height; ++y){
        for (int x=0; x < width; ++x){
            if (costmap_grid_[y][x] == 100){
                // inflate around the obstacle 

                int inflation_cells = inflation_radius / resolution; // 10 cells
                // calculated inflated region
                for (int dy = -inflation_cells; dy <= inflation_cells; ++dy){ // Across and down 
                    for (int dx = -inflation_cells; dx <= inflation_cells; ++dx){
                        int nx = x + dx;
                        int ny = y + dy;

                        // out of bounds check + already inflated -> skip 
                        if (nx < 0 || nx >= width || ny < 0 || ny >= height) continue; 
                        if (inflated_grid[ny][nx] == 100) continue; 
                        
                        // calculate euclidian distance from the obstacle cell * resolution (convert to meters)
                        double dist = std::sqrt(dx*dx + dy*dy) * resolution;
                        if (dist > inflation_radius) continue; // if dist > 10 blocks from obstacle (meters) away skip 

                        // cost = 99 x (1 - dist / inflation_radius)
                        // only assign if higher than whats already there 
                        int cost = max_cost * (1 - dist / inflation_radius);
                        inflated_grid[ny][nx] = std::max(inflated_grid[ny][nx], cost); // only inflate that cell 
                        
                    }
                }


            }
        }
    }
    costmap_grid_ = inflated_grid; 
}

nav_msgs::msg::OccupancyGrid CostmapCore::getCostmapMsg(){

    /// info: describes grid shape 
    nav_msgs::msg::OccupancyGrid msg; 
    msg.info.resolution = resolution;
    msg.info.width      = width;
    msg.info.height     = height;
    msg.info.origin.position.x = origin_x;
    msg.info.origin.position.y = origin_y;
    msg.info.origin.orientation.w = 1.0; // no rotation 
    msg.header.frame_id = "robot/chassis/lidar";

    // flatten 2d to 1d for message
    std::vector<int8_t> flatten; 

    for (int y = 0; y < height; ++y)
        for (int x = 0; x < width; ++x)
            flatten.push_back(costmap_grid_[y][x]);


    msg.data = flatten; 
    return msg;
}
}