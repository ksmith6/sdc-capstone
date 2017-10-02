# Team America

### Team Members:
* Jean-Paul Haddad - jeanpaul.haddad@gmail.com
* Shaobo Luo - shaobo.luo@gmail.com
* Babi Seal - sourav.seal@gmail.com
* Kelly Smith - kellymichaelsmith@gmail.com
* Apik Zorian - apikzorian@gmail.com

## Introduction
@Apik

## Waypoint Updater
Upon initialization, the Waypoint Updater will receive a copy of the base waypoints, and it will cache it internally.  

At each 5Hz cycle of the waypoint updater code, it publishes the final waypoints (50) and their speeds to the `/final_waypoints` topic.

The speed at each waypoint is determined by whether or not a red light has been detected or not.  If a red light has been detected and the vehicle is within the braking distance (4 meters), then it will set all waypoint velocities to zero to stop the vehicle.

If no red traffic light has been detected, then the waypoint updater will command all final waypoints to drive at the maximum speed (10 mph). 

## Traffic Light Detection
@Apik 

## Drive-By-Wire (DBW)

### `dbw_node.py`

The `dbw_node.py` logic will call the `Controller` object with the current state information (speed, etc) to obtain throttle, brake, and steering commands.

If drive-by-wire (DBW) flag is enabled, then the `Controller` computed values for throttle, braking, and steering are published to `/vehicle/throttle_cmd`, `/vehicle/braking_cmd`, and `/vehicle/steering_cmd` respectively.

The `Controller` logic resides within the `twist_controller.py` file.  This file leverages the `PID.py` file to control the throttle and brake commands.  Additionally, the steering commands are generated based on commands from `yaw_controller.py` and are smoothed via a low-pass filter from `lowpass.py` to remove jitter from the commanded steering angle.

If the DBW flag becomes disabled (manual control), then all control values are reset.  This is critical for clearing out the running integral error term for a PID controller.

## Conclusions
@Apik


## Installation Details

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

### Installation 

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop). 
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space
  
  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

