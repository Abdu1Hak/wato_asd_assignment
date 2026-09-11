#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:

    // Constructors for Core 
    void initializeCostmap(); // reset all cells to 0 
    void laserToGridCord(double range, double angle, int& x_grid, int& y_grid); // convert laser scan to grid coordinates
    void markObstacles(int x_grid, int y_grid); // mark obstacles in the costmap. 
    void inflateObstacles(); // inflate obstacles and mark high cost w/100
    void publishCostmap(): 
    
    
    // Constructor + logger arg  
    explicit CostmapCore(const rclcpp::Logger& logger);
    
  private:

    // Member variable
    rclcpp::Logger logger_; 
    double resolution = 0.1; // meters per cell 
    int height = 200; // 20 meters tall 
    int width = 200; // 20 meters wide
    double origin_x = -10.0; // middle of grid 
    double origin_y = -10.0; // middle of grid
    double inflation_radius = 1.0 
    int max_cost = 99; 
  
    std::vector<std::vector<int>> costmap_grid_; // 2D array to represent occupancy grid 
};

}  

#endif  