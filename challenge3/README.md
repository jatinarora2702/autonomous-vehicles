# Challenge 3

Does Direct SLAM work better in an indoor area or outdoor area?

## Approach

During the course of this challenge, we faced some system setup issues in installing packages for Direct SLAM, hence our experiments use LIDAR based [Hector SLAM](https://github.com/tu-darmstadt-ros-pkg/hector_slam/tree/catkin) (branch: ```catkin```).

However, from a Direct SLAM perspective, you may find these GitHub repos helpful. They make use of the popular LSD Slam package.
* https://github.com/IshitaTakeshi/lsd_slam_noros
* https://github.com/tum-vision/lsd_slam

## Observations

**Indoor**: 
* Hector SLAM works pretty well indoors with good detailing. 
* The middle pillar in High Bay is also well detected. 
* There are glass windows on one side which can be noticed as rays are not able to reflect back to the sensor.

**Outdoor**: 
* As vehicle comes out of lab, gate is detected well. Vehicle localization is also good.
* Building boundary is detected well
* Then, due to open space, LIDAR sensor cannot capture back the reflected laser rays and the mapping and localization messes up.


## Demo

**Indoor SLAM**

[![Indoor SLAM](https://img.youtube.com/vi/ZigVMt5cRA4/0.jpg)](https://www.youtube.com/watch?v=ZigVMt5cRA4)

**Outdoor SLAM**

[![Outdoor SLAM](https://img.youtube.com/vi/_qh8o3lFWdw/0.jpg)](https://www.youtube.com/watch?v=_qh8o3lFWdw)

## Setup

```commandline
$ sudo apt-get install ros-kinetic-hector-slam
```

Make sure ```catkin``` is setup and a Catkin workspace is there. In my case, I call it, ```catkin_ws```. For help, look at [this](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

Then, clone the above Hector Slam GitHub repo into ```catkin_ws/src``` and run ```catkin_make``` to build it.

I found [this tutorial](https://hackaday.io/project/7284-oscar-omni-service-cooperative-assistant-robot/log/26164-first-foray-into-ros) helpful for understanding how to use Catkin and work with Hector SLAM. 

All Set!

## Dataset

The ROS Bag files (indoor and outdoor) can be downloaded from [here](https://zenodo.org/record/4467486#.YBJeu-hKhPZ).

## Running

No separate code files are required for completing this challenge. All required changes are done in already present internal files as detailed.

We basically follow the instructions in [this tutorial](http://wiki.ros.org/hector_slam/Tutorials/MappingUsingLoggedData). With no changes, you should be able to see the map being formed on the default tutorial rosbag file. 

For us, we recorded the sensor data from the autonomous vehicle into a rosbag where the lidar data comes under the ```lidar1``` topic. Hence, we will make some changes in the default tuturial run file.

Open ```catkin_ws/src/hector_slam/hector_mapping/launch/mapping_default.launch``` file and change the default value of ```arg name="base_frame"``` and ```arg name="odom_frame"```, both to ```lidar1```.

Finally when running, map the ```lidar1/scan``` topic to ```/scan``` topic. 

### Indoor SLAM

```commandline
rosbag play insidehighbay.bag /lidar1/scan:=/scan --clock -r 2.5
```

**Note:** The ```clock``` argument just runs the clock through the bag faster.

### Outdoor SLAM

```commandline
rosbag play outsidehighbay.bag /lidar1/scan:=/scan --clock -r 2.5
```
