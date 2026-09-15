#include "planner_node.hpp"

PlannerNode::PlannerNode() : Node("planner"), planner_(robot::PlannerCore(this->get_logger())) {

  // Subs 
  map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", 10, std::bind(&PlannerNode::mapCallback, this, std::placeholders::_1));
  
  goal_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, std::bind(&PlannerNode::goalCallback, this, std::placeholders::_1));

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&PlannerNode::odomCallback, this, std::placeholders::_1));

  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/path", 10); 

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&PlannerNode::timerCallback, this)); 
}


void PlannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
  current_map_ = *msg;
  have_map_ = true;
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL) {
    planPath();
  }
}

void PlannerNode::goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
  goal_ = *msg;
  have_goal_ = true;
  state_ = State::WAITING_FOR_ROBOT_TO_REACH_GOAL;
  planPath();
}

void PlannerNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
  robot_pose_ = msg->pose.pose;
  have_odom_ = true;
}

void PlannerNode::planPath(){ 
  if (!have_map_ || !have_odom_ || !have_goal_ || current_map_.data.empty()){
    return; 
  }
  geometry_msgs::msg::Pose goal_pose; 
  goal_pose.position = goal_.point; 

  nav_msgs::msg::Path path = planner_.planPath(current_map_, robot_pose_, goal_pose); 
  path.header.stamp = this->get_clock()->now(); 
  path_pub_->publish(path); 
}


void PlannerNode::timerCallback(){
  if (state_ == State::WAITING_FOR_ROBOT_TO_REACH_GOAL){ 
    if (goalReached()){ 
      RCLCPP_INFO(this->get_logger(), "Goal Reached!");
      state_ = State::WAITING_FOR_GOAL; 
    } else {
      RCLCPP_INFO(this->get_logger(), "Replanning..."); 
      planPath(); 
    }
  }
}

bool PlannerNode::goalReached(){
  double dx = goal_.point.x - robot_pose_.position.x; 
  double dy = goal_.point.y - robot_pose_.position.y; 
  return std::sqrt(dx * dx + dy * dy) < 0.5; 
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
