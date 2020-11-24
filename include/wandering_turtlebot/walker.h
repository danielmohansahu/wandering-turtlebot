#pragma once

#include <atomic>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

namespace walker {

class Walker {
 public:
  /* @brief Constructor */
  Walker();

  /* @brief Core execution method; this should be called regularly by the external thread. */
  void execute();

 private:
  /* @brief Publish a forward commanded velocity to continue exploring. */
  void forward();

  /* @brief Spin in place to find a safe route. */
  void spin();

  /* @brief Boolean indicating whether or not we're almost in collision. */
  std::atomic<bool> near_obstacle_;

  /* @brief Core ROS subscriber to laser sensor for collision detection. */
  ros::Subscriber laser_sub_;

  /* @brief Core ROS publisher to set command velocity. */
  ros::Publisher cmd_pub_;

  // commanded speeds
  double linear_velocity_, angular_velocity_;
};

} //namespace walker
