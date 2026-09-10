#include <chrono>
#include <memory>
 
#include "costmap_node.hpp" 

// Costmap Node constructor. 
// Populate Node name 
// populate costmap_ member variable with a costmapcore object.
// this ->get_logger() is a method of the Node class that returns the logger associated with the node. 
CostmapNode::CostmapNode() : Node("costmap"), costmap_(robot::CostmapCore(this->get_logger())) {
}


// main() 
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}