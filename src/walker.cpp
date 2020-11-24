/* @file walker.cpp
 * @brief Implementation of the Walker class.
 *
 * @copyright [2020] Daniel Sahu
 */

#include <wandering_turtlebot/walker.h>

namespace walker {

Walker::Walker() : near_obstacle_(false),
                   linear_velocity_(0.22),
                   angular_velocity_(2.84),
                   obstacle_range_(0.25) {

  // set / get ros params
  ros::NodeHandle nh;
  nh.getParam("linear_velocity", linear_velocity_, linear_velocity_);
  nh.getParam("angular_velocity", angular_velocity_, angular_velocity_);
  nh.getParam("obstacle_range", obstacle_range_, obstacle_range_);

  // handy method for check if the sensor is angle is in front of the robot
  auto within_range = [](const double& angle)->bool {
    return (angle < 0.785) || (angle > 5.498);
  };

  // initialize core command publisher
  cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // initialize core sensor feedback and callback
  laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(
    "scan",
    1,
    [this, within_range](const auto& msg) {
      // check if we're close to being in collision and set our state appropriately
      auto angle = msg->angle_min;
      bool close_hit = false;
      for (auto r : msg->ranges) {
        // we only care about the angles directly in front of the robot
        if (within_range(angle) && r <= obstacle_range_) {
          close_hit = true;
          break;
        }

        // increment ranges
        angle += msg->angle_increment;
      }

      if (close_hit)
        near_obstacle_ = true;      
      else
        near_obstacle_ = false;
    }
  );
}

void Walker::execute() {
  // check if we're near an obstacle and spin or explort
  if (near_obstacle_)
    spin();
  else
    forward();
}

void Walker::forward() {
  // publish a simple linear velocity command
  geometry_msgs::Twist cmd;
  cmd.linear.x = linear_velocity_;
  cmd_pub_.publish(cmd);
}

void Walker::spin() {
  // publish a simple angular velocity command
  geometry_msgs::Twist cmd;
  cmd.angular.z = angular_velocity_;
  cmd_pub_.publish(cmd);
}

} //namespace walker
