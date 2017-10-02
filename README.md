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
Our approach to traffic light detection consisted of two parts: Detecting the traffic closest traffic light ahead, and then classifying this light as either a red light or not.  We subscribe to /base_waypoints, which contains all of the base waypoints, /camera/image_raw, which provides the current image taken by the camera on the vehicle, and /current_pose, which gives us the car’s current position.  

To detect the closest traffic light, we utilized the config file provided by Udacity (sdc-capstone/ros/src/tl_detector/sim_traffic_light_config.yaml). We take generate the waypoints based on the current position of the car and generate the waypoints for the 8 traffic lights. Then, we loop through the traffic lights and find the closest one to the car’s waypoints. We also make sure to check that this traffic light is ahead of us, as we do not want to be checking the state of a traffic light that is behind us.

The next step was to train a model to classify the image received from /camera/image_raw. We used a CNN-based model for classification. Below are the specs for this model:



Layer (type)                 Output Shape              Param #   
=================================================================
conv2d_1 (Conv2D)            (None, 398, 398, 32)      896       
_________________________________________________________________
activation_1 (Activation)    (None, 398, 398, 32)      0         
_________________________________________________________________
conv2d_2 (Conv2D)            (None, 396, 396, 32)      9248      
_________________________________________________________________
activation_2 (Activation)    (None, 396, 396, 32)      0         
_________________________________________________________________
max_pooling2d_1 (MaxPooling2 (None, 198, 198, 32)      0         
_________________________________________________________________
conv2d_3 (Conv2D)            (None, 196, 196, 32)      9248      
_________________________________________________________________
activation_3 (Activation)    (None, 196, 196, 32)      0         
_________________________________________________________________
max_pooling2d_2 (MaxPooling2 (None, 98, 98, 32)        0         
_________________________________________________________________
conv2d_4 (Conv2D)            (None, 96, 96, 32)        9248      
_________________________________________________________________
activation_4 (Activation)    (None, 96, 96, 32)        0         
_________________________________________________________________
max_pooling2d_3 (MaxPooling2 (None, 48, 48, 32)        0         
_________________________________________________________________
conv2d_5 (Conv2D)            (None, 46, 46, 32)        9248      
_________________________________________________________________
activation_5 (Activation)    (None, 46, 46, 32)        0         
_________________________________________________________________
max_pooling2d_4 (MaxPooling2 (None, 23, 23, 32)        0         
_________________________________________________________________
conv2d_6 (Conv2D)            (None, 21, 21, 32)        9248      
_________________________________________________________________
activation_6 (Activation)    (None, 21, 21, 32)        0         
_________________________________________________________________
max_pooling2d_5 (MaxPooling2 (None, 10, 10, 32)        0         
_________________________________________________________________
conv2d_7 (Conv2D)            (None, 8, 8, 32)          9248      
_________________________________________________________________
activation_7 (Activation)    (None, 8, 8, 32)          0         
_________________________________________________________________
max_pooling2d_6 (MaxPooling2 (None, 4, 4, 32)          0         
_________________________________________________________________
dropout_1 (Dropout)          (None, 4, 4, 32)          0         
_________________________________________________________________
flatten_1 (Flatten)          (None, 512)               0         
_________________________________________________________________
dense_1 (Dense)              (None, 128)               65664     
_________________________________________________________________
activation_8 (Activation)    (None, 128)               0         
_________________________________________________________________
dropout_2 (Dropout)          (None, 128)               0         
_________________________________________________________________
dense_2 (Dense)              (None, 2)                 258       
_________________________________________________________________
activation_9 (Activation)    (None, 2)                 0         
=================================================================
We trained our model on images taken from the simulator, which we manually labeled as “red” and “not red”. We also implemented code to zoom in on the image of the traffic light. This helped get rid of extraneous data in the images and allowed the classifier to focus on the traffic lights. We also applied image augmentation techniques to help our classifier adapt to a larger variety of images it may encounter. These techniques included flipping images and shifting the images horizontally/vertically to create more data. This process can be found in /ros/src/tl_detector/train.py. 

We eventually ended up with the model weights (/ros/src/tl_detector/model.h5) and were able to test our program to predict traffic lights. For testing on real world model data, we followed 2.5 Tensorflow Model: Object Detection API of John Chen's work. https://github.com/diyjac/SDC-System-Integration/tree/master/classifier We use the best checkpoint 1911 as the finally model.



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

