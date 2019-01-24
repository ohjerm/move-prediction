# move-prediction
Predicting intent of a user based on inputs to a robotic arm.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

[ROS](http://wiki.ros.org/ROS/Installation)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;: This project is tested on melodic but should work fine on some previous versions, likely >Groovy.

[UR](https://github.com/ros-industrial/universal_robot)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;: The project uses UR-industrial, UR-modern-driver and MoveIt to connect to and move the robot arm. 

[Realsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)&nbsp;: We have tested on a D415 using Realsense2_camera v2.1.1. 

[PCL](http://www.pointclouds.org/downloads/)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;: The project uses PCL to perform many pointcloud operations.

Many nodes can work without individual packages but all are necessary for everything to work together

### Installation

1. Clone the repo into your preferred environment. Since the project uses ROS, this will likely be ~/{catkin_workspace}/src
2. Make sure all the prerequisites are fulfilled
3. ????
4. You're done

You may need to `catkin_make`. If this command returns errors, there may be a mismatch in versions or something broken has been pushed. Don't worry, this project is ongoing until at least mid-2019.

As a demonstration, do:

```
roscore
roslaunch realsense2_camera rs_aligned_depth.launch filters:=pointcloud
rosrun move_prediction kd_distance_filter
rosrun rviz rviz
```

Now locate the filtered points in the rostopic /camera/depth/color/filtered_points, show, and if you only see points in a 1m sphere around point 0,0,0, it is working.

