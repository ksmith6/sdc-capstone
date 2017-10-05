# Team America

### Team Members:
* Jean-Paul Haddad - jeanpaul.haddad@gmail.com
* Shaobo Luo - shaobo.luo@gmail.com
* Babi Seal - sourav.seal@gmail.com
* Kelly Smith - kellymichaelsmith@gmail.com
* Apik Zorian - apikzorian@gmail.com

## Introduction
For this project, our team designed a fully autonomous vehicle system, initially to be tested out on a simulator, and then on Udacity’s real self-driving car. The project can be broken up into three parts: (1) Traffic Light detection, (2)  Control, and (3) Waypoint Following. In Traffic Light Detection, we designed a detection node that would take the current waypoints of the car and an image taken from the car and determine if the closest traffic light was red or not. For Control, we designed a drive-by-wire (dbw) node that could take the target linear and angular velocities and publish commands for the throttle, brake, and steering of the car. Finally, the Waypoint Follower would take information from the traffic light detection and the current waypoints and update the target velocities for each waypoint based on this information. 

## Waypoint Updater
Upon initialization, the Waypoint Updater will receive a copy of the base waypoints, and it will cache it internally.  

At each 5Hz cycle of the waypoint updater code, it publishes the final waypoints (50) and their speeds to the `/final_waypoints` topic.

If no red light has been detected by the traffic light detection algorithm, then the waypoint updater will command a constant acceleration or deceleration to return to the prescribed maximum speed.

However, if a red light has been detected and if the distance to the upcoming red light is less than some slowdown threshold distance, then the waypoint updater will linearly decelerate all the the upcoming waypoints towards zero at some offset distance prior to the actual intersection.  If the vehicle is less than this offset distance, then it will completely stop and wait at the traffic light.  Extra logic is also provided to help the vehicle approach the traffic light slowly if it is currently stopped (speed=0) within the slowdown zone.  In this case, it will slowly approach the intersection before stopping.   

If a red light has been detected but the vehicle is beyond the slowdown threshold distance, then the waypoint updater will command the vehicle to linearly accelerate or decelerate to the maximum speed.

## Traffic Light Detection
Our approach to traffic light detection consisted of two parts: Detecting the traffic closest traffic light ahead, and then classifying this light as either a red light or not.  We subscribe to `/base_waypoints`, which contains all of the base waypoints, `/camera/image_raw`, which provides the current image taken by the camera on the vehicle, and `/current_pose`, which gives us the car’s current position.  

To detect the closest traffic light, we utilized the config file provided by Udacity (`sdc-capstone/ros/src/tl_detector/sim_traffic_light_config.yaml`). We take generate the waypoints based on the current position of the car and generate the waypoints for the 8 traffic lights. Then, we loop through the traffic lights and find the closest one to the car’s waypoints. We also make sure to check that this traffic light is ahead of us, as we do not want to be checking the state of a traffic light that is behind us.

The next step was to train a model to classify the image received from `/camera/image_raw`. We used a CNN-based model for classification. Below are the specs for this model:



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
We trained our model on images taken from the simulator, which we manually labeled as “red” and “not red”. We also implemented code to zoom in on the image of the traffic light. This helped get rid of extraneous data in the images and allowed the classifier to focus on the traffic lights. We also applied image augmentation techniques to help our classifier adapt to a larger variety of images it may encounter. These techniques included flipping, zoom and shifting the images horizontally/vertically to create more data. This process can be found in `/ros/src/tl_detector/train.py`. 

We eventually ended up with the model weights (`/ros/src/tl_detector/test_1.h5`) and were able to test our program to predict traffic lights.

For the real-world Carla implementation we used the [Tensorflow Object Detection API] (https://github.com/tensorflow/models/tree/master/research/object_detection). We followed the guidance of Team Vulture's/John Chen work on [GitHub](https://github.com/diyjac/SDC-System-Integration/tree/master/classifier) where they looked at different options. This was also recommended by Anthony Sarkis on Slack channel [Anthony Sarkis Medium blog](https://medium.com/@anthony_sarkis/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58).

We followed the instructions in John Chen's document Section 2.5 to download and extract the pre-trained model and weights from [Faster R-CNN with Resnet] (http://download.tensorflow.org/models/object_detection/faster_rcnn_resnet101_coco_11_06_2017.tar.gz), and leveraged the training data on `just_traffic_light.bag` rosbag data. 

After training was complete, we freeze the best checkpoint 1911 as the final model. We ran the scripts John Chen's team provided to verify the classification of `rosbag` images.

We toggle between the sim model and the real Carla model by setting ros parameter `model_type` to CNN for the launch files corresponding to sim (`styx.launch`) vs. the Carla model (`site.launch`).

## Drive-By-Wire (DBW)

### `dbw_node.py`

The `dbw_node.py` logic will call the `Controller` object with the current state information (speed, etc) to obtain throttle, brake, and steering commands.

If drive-by-wire (DBW) flag is enabled, then the `Controller` computed values for throttle, braking, and steering are published to `/vehicle/throttle_cmd`, `/vehicle/braking_cmd`, and `/vehicle/steering_cmd` respectively.

The `Controller` logic resides within the `twist_controller.py` file.  This file leverages the `PID.py` file to control the throttle and brake commands.  Additionally, the steering commands are generated based on commands from `yaw_controller.py` and are smoothed via a low-pass filter from `lowpass.py` to remove jitter from the commanded steering angle.

If the DBW flag becomes disabled (manual control), then all control values are reset.  

## Partial Video of sim run
[Three Intermediate Traffic light Video](https://youtu.be/tenwII6HU1k)

## Conclusions
This project was a great opportunity to collaborate with a team of students who had been through this nano-degree, while also putting together all of the knowledge we have compiled over the last 3 terms into one final project. By designating tasks among ourselves, we each tackled our area of expertise for the project and were always ready to help when we needed guidance or were stuck on a problem. Some areas of improvements would be increasing the amount of training data, improving the steering controls and motion planning, as well as better estimation of traffic lights for images. Overall, we are thrilled to have completed this course and are very thankful to Udacity for providing us with an opportunity to showcase our talents on a project that was both challenging and rewarding.

## Acknowledgments
We would like to thank all those individuals that have provided very useful tips and information on the project Slack Channel. In particular:
- John Chen for all the generous information that he has provided online; and 
- Alexey Makurin for the Github link he provided [lag fixes](https://github.com/amakurin/CarND-Capstone/commit/9809bc60d51c06174f8c8bfe6c40c88ec1c39d50) offering suggestions on how to fix lag issues in the simulator. Many of these suggestions were applied in the code.

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

