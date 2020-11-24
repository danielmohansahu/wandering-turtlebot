#include <wandering_turtlebot/walker.h>

namespace walker {

Walker::Walker() : near_obstacle_(false) {
  // @TODO set / get ros params
  ros::NodeHandle nh;

  linear_velocity_ = 0.22;
  angular_velocity_ = 0.1;

  // initialize core command publisher
  cmd_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

  // initialize core sensor feedback and callback
  laser_sub_ = nh.subscribe<sensor_msgs::LaserScan>(
    "scan",
    1,
    [this](const auto& msg) {
      // check if we're close to being in collision and set our state appropriately
      if (true) // @TODO
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
