#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot
{

class CostmapCore {
  public:

    // Defines Constructors for Core 
    void initializeCostmap(); // reset all cells to 0 
    void laserToGridCord(double range, double angle, int& x_grid, int& y_grid); // convert laser scan to grid coordinates
    void markObstacles(int x_grid, int y_grid); // mark obstacles in the costmap. 
    void inflateObstacles(); // inflate obstacles and mark high cost w/100
    nav_msgs::msg::OccupancyGrid getCostmapMsg(); // calls this constructor function to return OccupancyGrid Message

    // Constructor + logger arg  
    explicit CostmapCore(const rclcpp::Logger& logger);
    

  private:

    // Member variable
    rclcpp::Logger logger_; 
    double resolution = 0.1; // meters per cell 
    int height = 300; // 20 meters tall 
    int width = 300; // 20 meters wide
    double origin_x = -15.0; // bottom left 
    double origin_y = -15.0; // bottom left 
    double inflation_radius = 1.3;
    int max_cost = 99; 
  
    std::vector<std::vector<int>> costmap_grid_; // 2D array to represent occupancy grid 
};

}  

#endif  