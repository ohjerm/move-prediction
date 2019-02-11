# move-prediction
Predicting intent of a user based on inputs to a robotic arm.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

### Prerequisites

[ROS](http://wiki.ros.org/ROS/Installation)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;: This project is tested on melodic but should work fine on some previous versions, likely >Groovy.

[UR](https://github.com/ros-industrial/universal_robot)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;: The project uses UR-industrial, UR-modern-driver and MoveIt to connect to and move the robot arm. 

[Realsense](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)&nbsp;: We have tested on a D415 using Realsense2_camera v2.1.1. 

[PCL](http://www.pointclouds.org/downloads/)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;: The project uses PCL to perform many pointcloud operations.

[BlueZ](https://github.com/padelt/bluez)&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;: The project uses BLueZ to connect to the GSR sensor via Bluetooth.

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

### Bluetooth

Make sure bluetooth is correctly installed:
```
sudo apt-get install bluetooth bluez bluez-tools rfkill
```

Next perform a hciconfig:
```
example@example:~$ hciconfig
hci0:	Type: Primary  Bus: USB
	BD Address: 9C:2A:70:2A:87:AA  ACL MTU: 1022:8  SCO MTU: 183:5
	UP RUNNING PSCAN ISCAN 
	RX bytes:3180 acl:0 sco:0 events:86 errors:0
	TX bytes:5434 acl:0 sco:0 commands:65 errors:0
```
If issues arise here, a firmware update or upgrade check for bluetooth might be needed.

Next perform a hcitool scan:
```
example@example:~$ hcitool scan
Scanning ...
	10:4A:7D:44:9F:D9	DAHL-PC
	B0:35:9F:A0:D5:4C	oliver-laptop
	90:32:4B:9A:89:E2	FW-85BZ35F
  00:06:66:66:8E:F1 Shimmer3
```
If the correct device does not show up, ensure that the device is on and the bluetooth is working

Using the rfcomm bind command, we connect the device to the computer via bluetooth on comm0:
```
sudo rfcomm bind 0 00:06:66:66:8E:F1
```
The device should now be connected to /dev/rfcomm0

## Deployment

To deploy this to a live system, ensure all the following nodes are running:

### Camera Nodes

`realsense2_camera->rs_aligned_depth.launch filters:=pointcloud` + `kd_distance_filter` + `clustering`

### Keyboard Input Nodes

`sudo -s`

`keyboard_input.py` + `trajectories.py`

... Additional nodes go here when development has caught up

## Built With

ROS, PCL, UR-industrial, MoveIt!

## Contributing

The repo is currently private as it is a student project.

## Versioning

We are currently at version 0.0 and will continue to be here until the project is completed. A versioning scheme will be decided then.

## Authors

* **Frederik Falk** - [sebfrede](https://github.com/sebfrede)
* **Oliver Hjermitslev** - [ChaiKnight](https://github.com/ChaiKnight)

See also [contributors](github.com/ChaiKnight/move-prediction/contributors) for a list of contributors to the project.

## License

This project is licensed under GNU General Public License (GPL) v3.0 to ensure all improvements are kept in the public domain. This also means the authors assume no warranty or liability except where explicitly stated by law for the use of this software. The GPL is found in LICENSE.txt. Handle with care, especially around robotic arms :)

## Acknowledgements

If you contributed to this project in any way, be it code revision, feature development, grammar corrections, or general suggestions, then shout out to you.
We would also like to thank our supervisors, Thomas Moeslund and Stefan Bengtson, of Aalborg University for all their help. Stefan's technical and concrete assistance with both the report and implementation have been invaluable. Thomas' help with structuring work, grander considerations and project scope have helped guide the project significantly.
