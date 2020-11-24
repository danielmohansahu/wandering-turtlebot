# wandering-turtlebot

This class instruments a simple robotic behavior for traversing a simulated environment. The robot will simply travel forward until it senses an obstacle (via LaserScan), at which point it will spin until a clear path is detected. A launch file is provided that will launch this behavior in a Gazebo world and explore until shut down.

## Assumptions

This project was written and tested using ROS Melodic and Gazebo9 on Ubuntu 18.04. In addition we rely on the `turtlebot3` package for simulation. Gazebo and `turtlebot3` are not required for the core node; they are runtime dependencies.

## Build

This package requires an existing catkin workspace to build / run. If you have an existing workspace, clone into the `src` folder. Otherwise, for a fresh repositoriy:

```bash
mkdir -p catkin_ws/src/ && cd catkin_ws/src
git clone https://github.com/danielmohansahu/wandering-turtlebot
cd ..
catkin_make
```

## Run

To run the full simulation (including Gazebo), call the core launch file. Note that the `TURTLEBOT3_MODEL` environmental variable is required by the core `turtlebot3` package.

```bash
TURTLEBOT3_MODEL=burger roslaunch wandering_turtlebot walker.launch
```

By default this will record a rosbag in your `ROS_ROOT` (usually `~/.ros`). This can be disabled via command line arg, e.g.:

```bash
TURTLEBOT3_MODEL=burger roslaunch wandering_turtlebot walker.launch bag:=false
```

## Bag Inspection

To analyze a saved bag you can use the `rosbag` API, e.g.:

```bash
rosbag info {PATH_TO_BAG}
```

Output will be similar to the following:

```
path:         results/record.bag
version:      2.0
duration:     20.6s
start:        Dec 31 1969 19:00:00.14 (0.14)
end:          Dec 31 1969 19:00:20.78 (20.78)
size:         8.5 MB
messages:     67781
compression:  bz2 [104/104 chunks; 9.72%]
uncompressed: 78.0 MB @   3.8 MB/s
compressed:    7.6 MB @ 375.8 KB/s (9.72%)
types:        dynamic_reconfigure/Config            [958f16a05573709014982821e6822580]
              dynamic_reconfigure/ConfigDescription [757ce9d44ba8ddd801bb30bc456f946f]
              gazebo_msgs/LinkStates                [48c080191eb15c41858319b4d8a609c2]
              gazebo_msgs/ModelStates               [48c080191eb15c41858319b4d8a609c2]
              geometry_msgs/Twist                   [9f195f881246fdfa2798d1d3eebca84a]
              nav_msgs/Odometry                     [cd5e73d190d741a2f92e81eda573aca7]
              rosgraph_msgs/Clock                   [a9c97c1d230cfc112e270351a944ee47]
              rosgraph_msgs/Log                     [acffd30cd6b6de30f120938c17c593fb]
              sensor_msgs/Imu                       [6a62c6daae103f4ff57a132d6f95cec2]
              sensor_msgs/JointState                [3066dcd76a6cfaef579bd0f34173e9fd]
              sensor_msgs/LaserScan                 [90c7ef2dc6895d81024acba2ac42f369]
              tf2_msgs/TFMessage                    [94810edda583a504dfda3829e70d7eec]
topics:       /clock                           20653 msgs    : rosgraph_msgs/Clock                  
              /cmd_vel                           207 msgs    : geometry_msgs/Twist                  
              /gazebo/link_states              20560 msgs    : gazebo_msgs/LinkStates               
              /gazebo/model_states             20569 msgs    : gazebo_msgs/ModelStates              
              /gazebo/parameter_descriptions       1 msg     : dynamic_reconfigure/ConfigDescription
              /gazebo/parameter_updates            1 msg     : dynamic_reconfigure/Config           
              /imu                              3897 msgs    : sensor_msgs/Imu                      
              /joint_states                      585 msgs    : sensor_msgs/JointState               
              /odom                              584 msgs    : nav_msgs/Odometry                    
              /rosout                             26 msgs    : rosgraph_msgs/Log                     (4 connections)
              /rosout_agg                         16 msgs    : rosgraph_msgs/Log                    
              /scan                               97 msgs    : sensor_msgs/LaserScan                
              /tf                                585 msgs    : tf2_msgs/TFMessage
```

One can play back these bag files against the core robot node. Note that this should be done without Gazebo running to prevent double-publishing.

```bash
# terminal #1: roscore
roscore
# terminal #2: run ros node
source devel/setup.bash
rosrun wandering_turtlebot wandering_turtlebot
# terminal #3: bag playback
rosbag play {PATH_TO_BAG}
```