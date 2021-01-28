# Project: Pedestrian Follower

As final course project, we programmed the autonomous vehicle to follow a selected person. 


## Approach

* The vehicle first scans for pedestrians in its view. 
* The user then marks a specific pedestrian and as that pedestrian moves, the vehicle tracks that specific person.
* The vehicle can take turns (left, right), accelerate, brake, reverse.
* Implemented with 2 PID controllers (steering angle and speed of vehicle) + [Dlib](https://pypi.org/project/dlib/) (for object tracking)
* For steering angle calculation, we use horizontal deviation of the bounding box from center.
* To speed control, we estimate the pedestrian position from the height of the bounding box.

## Demo

[![Pedestrian Follower Demo 1](https://img.youtube.com/vi/QFCNB3Rrjog/0.jpg)](https://www.youtube.com/watch?v=QFCNB3Rrjog)

[![Pedestrian Follower Demo 2](https://img.youtube.com/vi/uu84T8Am1ms/0.jpg)](https://www.youtube.com/watch?v=uu84T8Am1ms)

## Setup

Install DLib for object tracking. Note that DLib has some CUDA toolkit dependencies as well. You might have to install them separately.

```commandline
$ pip install dlib
```

## Running

Start up ROS Core Node on Terminal 1

```commandline
$ roscore
```

Start up RViz on Terminal 2
```commandline
$ rviz
```

Run the code on Terminal 3
```commandline
python pedestrian_follower.py
```

Press '```s```' key on keyboard and click and drag a bounding box around the pedestrian to follow.

Press '```p```' key on keyboard to enable PID controller and DLib to start tracking.

You are good to go!
