#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    
    // Constructor + logger arg  
    explicit CostmapCore(const rclcpp::Logger& logger);
    

  private:

    // Member variabl
    rclcpp::Logger logger_; 
    

};

}  

#endif  