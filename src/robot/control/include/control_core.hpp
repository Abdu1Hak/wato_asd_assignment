#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include <optional>

namespace robot
{

class ControlCore {
  public:
    explicit ControlCore(const rclcpp::Logger& logger);

    // Compute Velocity + Find Look Ahead Point 
    geometry_msgs::msg::Twist computeVelocity(
      const nav_msgs::msg::Path& path,
      const nav_msgs::msg::Odometry& odom);

    std::optional<geometry_msgs::msg::Point> findLookaheadPoint(
      const nav_msgs::msg::Path& path,
      const geometry_msgs::msg::Point& robot_pos);
    
    // Get angle + distance
    double extractYaw(const geometry_msgs::msg::Quaternion& q);
    double computeDistance(const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b);

  private:
    rclcpp::Logger logger_;
    double lookahead_distance_ = 1.2; // Lookahead distance L (meters)
    double goal_tolerance_ = 0.35;    // Arrival threshold (meters)
    double linear_speed_ = 1.5;       // Forward velocity (m/s)
    double max_angular_speed_ = 2.5;  // Angular velocity limit (rad/s)
};

}

#endif