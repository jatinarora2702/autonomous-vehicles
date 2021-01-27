# Challenge 1

Cause the vehicle to brake after detecting a person.

## Approach

We create a subscriber for the camera ROS topic to get image frames (input) and a publisher to control the vehicle brakes (output). We run OpenCV's person detection algorithm on input image frames and if a person is found, we publish the brake command to the vehicle.

## Demo

**Outside Vehicle View** 

[![Brake on Detecting Pedestrian: Outside Vehicle View](https://img.youtube.com/vi/Gz7MKb-auIo/0.jpg)](https://www.youtube.com/watch?v=Gz7MKb-auIo)

**Inside Vehicle Command Line View**

[![Brake on Detecting Pedestrian: Inside Vehicle Command Line View](https://img.youtube.com/vi/ZxLAGkmdylo/0.jpg)](https://www.youtube.com/watch?v=ZxLAGkmdylo)

## Running

For installing some dependencies like, ```pacmod_msgs``` and ```cv_bridge```, use the ```apt-get``` command like:

```commandline
$ sudo apt-get install ros-kinetic-pacmod-msgs
```

For running, startup ROS Core (in one terminal) and run ```brake.py``` (in separate terminal).

```bash
# start up ROS Core
$ roscore

# run the code in separate terminal
$ python brake.py
```
