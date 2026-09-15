#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(robot::MapMemoryCore(this->get_logger())) {
  
  costmap_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap",
    10, 
    std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

  odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered",
    10, 
    std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

  // Publisher for the fused global map. This was missing, which left map_pub
  // null and crashed the node the first time timerCallback tried to publish.
  map_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

  
  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&MapMemoryNode::timerCallback, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){

  latest_costmap_ = *msg; // copy the entire costmap msg 
  costmap_updated_ = true; 

}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){ 
  current_x_ = msg->pose.pose.position.x; 
  current_y_ = msg->pose.pose.position.y; 
  have_odom_ = true; 

  // ROS stores rotation as quaternion (x,y,z,w)
  auto q = msg->pose.pose.orientation; 
  // quaternion-to-Euler angle conversion
  current_yaw_ = std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)); 

  // Euclidian Distance from last map update
  double dx = current_x_ - last_update_x_; 
  double dy = current_y_ - last_update_y_; 
  double distance = std::sqrt(dx * dx + dy*dy);

  if (distance >= distance_threshold_){
    should_update_map_ = true; 
  }

    
}

void MapMemoryNode::timerCallback(){
  
  // Do not integrate until we know where the robot actually is, otherwise the
  // costmap gets stamped into the global map at the default pose and, because
  // cells are merged with std::max, that bad data can never be cleared.
  if (!have_odom_ || !costmap_updated_ || latest_costmap_.data.empty() || latest_costmap_.info.width == 0){
    return; 
  }

  if (!should_update_map_ && first_update_done_) {
    return; 
  }

  map_memory_.integrateCostmap(latest_costmap_, current_x_, current_y_, current_yaw_); 
  last_update_x_ = current_x_; 
  last_update_y_ = current_y_; 
  should_update_map_ = false; 
  first_update_done_ = true;

  auto global_map = map_memory_.getGlobalMap(); 
  global_map.header.stamp = this->now(); 
  map_pub->publish(global_map); 

}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
