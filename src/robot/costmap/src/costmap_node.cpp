#include <chrono>
#include <memory>
 
#include "costmap_node.hpp" 


CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) 
{
  // from the header, use those two objects here:
  lidar_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", // topic name
    10, // queue size 
    std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  // declare which topic to publish
  costmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
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
  auto msg = costmap_.getCostmapMsg(); 
  msg.header.stamp = this->now(); 
  costmap_pub->publish(msg); 
  
    
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);                      // 1. Initializes ROS2 Middleware 
  rclcpp::spin(std::make_shared<CostmapNode>()); // 2. Block and listen for callback
  rclcpp::shutdown();                            // 3. Cleanup on exit() 
  return 0; 
}


