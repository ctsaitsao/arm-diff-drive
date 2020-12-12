# ME495 (Embedded Systems for Robotics) Homework 3 "diff_drive" Package

Author: Christopher Tsai

## Overview

This package contains nodes that accomplish two tasks:

1. Cause a simulated differential drive robot to follow a rectangular trajectory in Gazebo.

2. Cause a simulated differential drive robot to go in a straight line, flip over itself, and go back the same path over and over.

## Demos

Rectangular trajectory: https://youtu.be/BcLgBR_f7zI
Flipping: https://youtu.be/OnJZ-ByWNCY

## Usage Instructions

1. Create a new workspace and clone the demonstration code.
```Shell
# Create a new workspace
mkdir -p ws/src

# clone the demonstration code
cd ws/src
git clone https://github.com/ME495-EmbeddedSystems/homework-3-ctsaitsao.git homework3

# return to ws root
cd ../
```

2. Build the workspace and activate it.
```Shell
catkin_make install
. devel/setup.bash
```

3. To run the rectangular trajectory demo, run:
```Shell
roslaunch diff_drive ddrive_follow_rect.launch
```

4. To run the flipping demo, run:
```Shell
roslaunch diff_drive ddrive_flip.launch
```

## Configuration Options

- Both demos by default start in a paused state and the user must click the "play" button in Gazebo in order to start the simulation. In order to start immediately after running, add the `paused:=False` option to the launch command.
- The `config` folder contains a few configuration options for the appearance and weight of the differential drive robot (in `ddrive_params.yaml`) as well as options to edit the control and shape of the rectangular trajectory (in `follow_rect.yaml`).
- The Gazebo world used can be edited in the `<arg name="world_name" value="$(find diff_drive)/worlds/ddrive.world"/>` line in the `ddrive.launch` file.
