#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger); 
    // Runs A* 
    nav_msgs::msg::Path planPath(const nav_msgs::msg::OccupancyGrid& grid, const geometry_msgs::msg::Pose& start, const geometry_msgs::msg::Pose& goal);

  private:
    struct Node {
      int x, y;
      double g,h; // cost to reach the node + cost to reach the goal
      Node* parent; 
      double f() const {return g + h;}
    }; 
    
    double heuristic(int x1, int y1, int x2, int y2) const; 
    bool isTraversable(const nav_msgs::msg::OccupancyGrid& grid, int x, int y) const; 
    int cellCost(const nav_msgs::msg::OccupancyGrid& grid, int x, int y) const; 
    bool canMoveBetween(const nav_msgs::msg::OccupancyGrid& grid, int fx, int fy, int tx, int ty) const;

    rclcpp::Logger logger_;
    // weight applied to a cell's occupancy value in the cost function.  
    double cost_weight_ = 0.8; // strongly penalize pink regions
    int occupancy_threshold_ = 65;
    // Cells at or above this cost are impassable. Inflation runs 99 down to 0
    // across a 1.0 m radius, so 65 hard-blocks only the innermost ~0.34 m and
    // leaves the rest of the inflation band drivable-but-expensive. That lets
    // the planner squeeze past an obstacle when there is no better option,
    // instead of the old 30 which walled off ~0.7 m and could make tight
    // gaps unsolvable.
};

}  

#endif  
