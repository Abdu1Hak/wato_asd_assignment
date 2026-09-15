#include "map_memory_core.hpp"

namespace robot
{

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger) : logger_(logger) {

  // create the global grid 
  global_grid_.assign(height_, std::vector<int8_t>(width_, -1));
}


void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& costmap, double robot_x, double robot_y, double robot_yaw){

  double cos_yaw = std::cos(robot_yaw); 
  double sin_yaw = std::sin(robot_yaw); 
  double cm_ox = costmap.info.origin.position.x; 
  double cm_oy = costmap.info.origin.position.y;
  double cm_res = costmap.info.resolution;
  size_t data_size = costmap.data.size();   

  // let cy and cx be the pixels across and down costmap
  for (int cy = 0; cy < 200; cy++){
    for (int cx = 0; cx < 200; cx++){
      int8_t value = costmap.data[cy * 200 + cx]; // convert 2d cords to 1d index to extract data value 
      
      if (value == -1) continue;
      uint8_t cost = static_cast<uint8_t>(value); 
      // Transformations 
      // 1. Grid Cell Index -> Local Mapping
      
      double local_x = cm_ox + (cx + 0.5) * cm_res; 
      double local_y = cm_oy + (cy + 0.5) * cm_res;

      // 2. Consider Rotation and Translation onto the global map 
      
      double world_x = robot_x + local_x * cos_yaw - local_y * sin_yaw; 
      double world_y = robot_y + local_x * sin_yaw + local_y * cos_yaw; 

      // 3. World Refernece to Global Grid Cell 

      int global_x = static_cast<int>((world_x - origin_x_) / resolution_);  
      int global_y = static_cast<int>((world_y - origin_y_) / resolution_);  

      if (global_x < 0 || global_x >= width_ || global_y < 0 || global_y >= height_) continue; 
      
      int8_t current_val = global_grid_[global_y][global_x]; 
      if (current_val == -1){
        global_grid_[global_y][global_x] = static_cast<int8_t>(cost); 
      } else {
        uint8_t cur_cost = static_cast<uint8_t>(current_val); 
        global_grid_[global_y][global_x] = static_cast<int8_t>(std::max(cur_cost, cost)); 
      }

    }
  }

} 

nav_msgs::msg::OccupancyGrid MapMemoryCore::getGlobalMap() const {
  nav_msgs::msg::OccupancyGrid msg;
  msg.info.resolution = resolution_;
  msg.info.width      = width_;
  msg.info.height     = height_;
  msg.info.origin.position.x    = origin_x_;
  msg.info.origin.position.y    = origin_y_;
  msg.info.origin.orientation.w = 1.0;
  msg.header.frame_id = "sim_world";  // fixed world frame, NOT robot frame

  msg.data.reserve(width_ * height_);
  for (int y = 0; y < height_; ++y)
      for (int x = 0; x < width_; ++x)
          msg.data.push_back(global_grid_[y][x]);
  return msg;
}

}