#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "map_memory_core.hpp"

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;

    //callbacks 
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback(); 

    // member variables
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    rclcpp::TimerBase::SharedPtr timer_; 
    
    bool costmap_updated_ = false;
    bool should_update_map_ = false; 
    bool first_update_done_ = false; 
    // True once a real odometry message has been received. Without this, the
    // first integration could run with the default pose (0,0,yaw 0) and stamp
    // obstacles into the global map at the wrong world position.
    bool have_odom_ = false; 
    double current_x_ = 0.0; 
    double current_y_ = 0.0; 
    double current_yaw_ = 0.0; // heading angle in radians 
    double last_update_x_ = 0.0; 
    double last_update_y_ = 0.0; // position at last map update
    // Distance the robot must travel before the global map is updated again.
    // 1.5 m was far too coarse: obstacles the lidar could already see were not
    // in /map yet, so the planner routed straight through them.
    const double distance_threshold_ = 0.3; 

    // subscribers 
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub; 
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub; 
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub; 

};

#endif 
