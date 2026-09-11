#include <chrono>
#include <memory>
 
#include "costmap_node.hpp" 


CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) 
{
  lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", // topic name
    10, // queue size 
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10)
}


void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg){
  // Use costmapcore class methods - better practice: 


  // Step 1: Initialize costmap
  costmap_.initializeCostmap(); // reset all cells to 0


  // Step 2: Convert LaserScan to grid and mark obstacles
  for (size_t i = 0; i < scan_msg->ranges.size(); ++i) {
    
    double angle = scan_msg->angle_min + i * scan_msg->angle_increment;
    double range = scan_msg->ranges[i];

    // skip readings outside valid sensor range (nosie)
    if (range < scan_msg->range_min || range > scan_msg->range_max) continue; 

    int x_grid, y_grid; 
    costmap_.laserToGridCord(scan_msg->ranges[i], angle, x_grid, y_grid);
    costmap_.markObstacles(x_grid, y_grid);
  }

  // Step 3: Inflate obstacles
  costmap_.inflateObstacles();

  // Step 4: Publish Costmap 
  costmap_.publishCostmap() 
    


  
}


 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}

// Current Workflow: 
// 1. Main() calls CostmapNode to create an object 
// 2. CostmapNode constructor runs, which subscribes to /lidar topic and sets up the callback function laserCallback.
// 3. LaserCallback resets the costmap
// 4. LaserCallback converts the laser scan data to grid coordinates. 
// 5. LaserCallback marks the obstacles in the costmap based on the grid coordinates - helper function for now 


