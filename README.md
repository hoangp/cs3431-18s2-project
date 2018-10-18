# COMP3431/COMP9431 18s2 Project

Speech and Person Recognition

Platform: TurtleBot3, ROS Kinetic, Ubuntu 16.04

Team Members: Ella, Hoang, Robin, Faris

## Installation

The program requires some (very) small modification on TurtleBot3 Nagigation (ACML), and TurtleBot3 Application Follower packages, which you might already have in your catkin workspace.

If you want to keep the original TurtleBot3 pakages, you might want to check the modification. Otherwise, you could remove your TurtleBot3 packages and clone this repo `--recursive` (with the submodules):

```git
git clone --recursive https://github.com/hoangp/cs3431-18s2-project.git
```

It also requires [OpenPose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) installed on the Remote PC.

The TurtleBot3 need a Lidar and a Camera

For other installation instructions (TurtleBot3, ROS Kinetic, Ubuntu 16.04) Please refer to [Robotis TurtleBot3 e-Manual](http://emanual.robotis.com/docs/en/platform/turtlebot3/pc_setup/)

## Setup

Below is the steps and command lines in our PC. It might be different in your PC.

- Run roscore
- Bringup TurtleBot3 Lidar and Camera

```bash
roslaunch turtlebot3_bringup turtlebot3_robot.launch
roslaunch realsense_camera r200_nodelet_default.launch
```

- Run TurtleBot3 SLAM and Navigation

```bash
roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping
roslaunch turtlebot3_navigation person_recognition_navigation.launch
```

- Run OpenPose listener

```bash
rosrun person_recognition listener
```

- Run Follower scripts

```bash
roslaunch turtlebot3_follow_filter turtlebot3_follow_filter.launch
roslaunch turtlebot3_follower turtlebot3_follower.launch
```

- Run our main scripts

```bash
cd scripts
python speech_cmd.py # speech recognition and send command to 'pr/cmd'
python run.py # excute command from topic 'pr/cmd'
```