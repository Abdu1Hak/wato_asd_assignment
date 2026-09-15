#include "control_node.hpp"

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/path", 10, std::bind(&ControlNode::pathCallback, this, std::placeholders::_1)
  ); 
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", 10, std::bind(&ControlNode::odomCallback, this, std::placeholders::_1)
  ); 
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10); 

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&ControlNode::controlLoop, this)
  );

}

void ControlNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg ){
  current_path_ = *msg; 
  have_path_ = true; 
}

void ControlNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg){
  current_odom_ = *msg; 
  have_odom_ = true; 
}

void ControlNode::controlLoop(){
  // only push when odom / path are updated
  if (!have_odom_ || !have_path_ || current_path_.poses.empty()){
    return; 
  }
  geometry_msgs::msg::Twist cmd_vel = control_.computeVelocity(current_path_, current_odom_); 
  cmd_vel_pub_->publish(cmd_vel); 
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
