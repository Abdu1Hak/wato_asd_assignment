#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp" // brings in all the ROS2 C++ library types 
 
#include "costmap_core.hpp"


class CostmapNode : public rclcpp::Node {
  public:

    // Constructor 
    CostmapNode();
 
  private:

    // Member variables 
    robot::CostmapCore costmap_; 

};
 
#endif 