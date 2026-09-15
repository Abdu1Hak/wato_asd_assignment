#include "control_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot
{

ControlCore::ControlCore(const rclcpp::Logger& logger) 
  : logger_(logger) {}
  

double ControlCore::computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
  return std::hypot(b.x - a.x, b.y - a.y);
}

double ControlCore::extractYaw(const geometry_msgs::msg::Quaternion& q) {
  // Quaternion-to-Euler yaw (radians)
  // Seen this before in Map Memory 
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

std::optional<geometry_msgs::msg::Point> ControlCore::findLookaheadPoint(
  const nav_msgs::msg::Path& path,
  const geometry_msgs::msg::Point& robot_pos)
{
  if (path.poses.empty()) return std::nullopt;

  // Search for the first waypoint ahead >= lookahead_distance_
  for (const auto& pose_stamped : path.poses) {
    double dist = computeDistance(robot_pos, pose_stamped.pose.position);
    if (dist >= lookahead_distance_) {
      return pose_stamped.pose.position;
    }
  }

  // If all remaining points are closer than L, target the final destination
  // *imp for the last stretch
  return path.poses.back().pose.position;
}


geometry_msgs::msg::Twist ControlCore::computeVelocity(
  const nav_msgs::msg::Path& path,
  const nav_msgs::msg::Odometry& odom)
{
  // Default safe state with zero velocity and angular speed
  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = 0.0;
  cmd.angular.z = 0.0;

  // if the planner have an empty list of way points, return 
  // *imp for when no goal defined
  if (path.poses.empty()) return cmd; 

  // get the robots position and angle extracted 
  const auto& robot_pos = odom.pose.pose.position; 
  double robot_yaw = extractYaw(odom.pose.pose.orientation); 

  // goal arrival check 
  const auto& final_goal = path.poses.back().pose.position;
  if (computeDistance(robot_pos, final_goal) <= goal_tolerance_){
    return cmd; 
  }

  // 1. Select a Lookahead point
  auto lookahead_opt = findLookaheadPoint(path, robot_pos); 
  if (!lookahead_opt) return cmd; 

  const auto& target = *lookahead_opt;

  // 2. Get the Target to robots frame
  double dx = target.x - robot_pos.x; 
  double dy = target.y - robot_pos.y; 

  double x_local = dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
  double y_local = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);

  // dist reached 
  double target_dist = std::hypot(x_local, y_local); 
  if (target_dist < 1e-4) return cmd; 

  // 3. Pure Pursuit Curvature Formula Implementation
  // k = 2 * ylocal / L^2 
  // If Ylocal > 0, curvature is position turn left. 
  // If Ylocal < 0, curvative is negative turn right. 
  // If Ylocal = 0, go straight 
  double curvature = (2.0 * y_local) / (target_dist * target_dist); 

  // 4. Update Velocity and Steering Location
  // Since Angular velocity relates to linear velocity and curvature as w = v . k 
  // clamp the values then
  cmd.linear.x = linear_speed_; 
  cmd.angular.z = std::clamp(cmd.linear.x * curvature, -max_angular_speed_, max_angular_speed_); 

  return cmd; 
} 
}
