#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp" // brings in all the ROS2 C++ library types 
#include "costmap_core.hpp"

// Add includes for message types. 
#include "sensor_msgs/msg/laser_scan.hpp" // for subscribing to Lidar data


class CostmapNode : public rclcpp::Node {
  public:

    // Constructor 
    CostmapNode();
 
  private:

    // Add all the callback functions
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);


    // Member variables 
    robot::CostmapCore costmap_; 


    // Subscribe to /lidar  + Publisher to /costmap
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub; 

};
 
#endif 