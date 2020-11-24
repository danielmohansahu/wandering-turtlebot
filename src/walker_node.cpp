/* @file walker_node.cpp
 * @brief ROS node executable for the Walker class.
 *
 * @copyright [2020] Daniel Sahu
 */

#include <ros/ros.h>
#include <wandering_turtlebot/walker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "walker");

  walker::Walker w;

  // explore until we're shut down
  ros::Rate r(10);
  while (ros::ok()) {
    // process callbacks
    ros::spinOnce();

    // execute based on latest feedback
    w.execute();

    // sleep
    r.sleep();
  }

  return 0;
}
