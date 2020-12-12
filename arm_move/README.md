# ME495 (Embedded Systems for Robotics) Homework 3 "arm_move" Package

Author: Christopher Tsai

## Overview

This package contains code that causes an Interbotix PincherX-100 ("px100") robot arm to do planning tasks using Moveit!. Its main node, `Mover`, has ROS services that allow a user to create and execute a waypoint path that the robot follows while avoiding the ground (or table) and one obstacle, a RealSense camera box.

## Dependencies

- moveit

## Demos

1. This is a demo of the robot looping through two waypoints and avoiding the RealSense box in between them. It also shows the RViz visualization: https://youtu.be/xHolN7LH-4I
2. This is a demo of the robot picking and placing an object through a similar waypoint path (waypoints also include gripper open/close data): https://youtu.be/1QW-ObcVXuI 

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

3. The current stored waypoints are for the pick-and-place demo. To run this demo, first launch:
    ```Shell
    roslaunch arm_move arm.launch use_actual:=True
    ```

4. Wait for the robot arm to move to its home configuration and for the planning scene objects to appear. Next, call the `follow` service:
    ```Shell
    rosservice call /px100/follow "repeat: true"
    ```

5. Put any small item where the grippers close and observe the pick-and-place operation. If gripper does not close properly during the operation, try opening the gripper using RViz's motion planning GUI (current -> open state) prior to starting `follow` service.

## Configuration Options

- The demo can be run in RVIz instead of in real life. To do this, replace the `use_actual:=True` flag with `use_fake:=True`.
- The RealSense box obstacle can be placed at a user-specified position using the `reset` service.
- The `waypoints.yaml` file contains the waypoints that the robot follows and can be configured.
- The `step` and `get_pose` services can be used to create a new waypoint trajectory. A user can disable joint torques, move the arm to a position of choice, and use `get_pose` to get the current position's specific pose numbers. The user can then feed these pose numbers to the `step` service, which plans and moves the robot to the specified pose and adds the pose to the node's list of waypoints. The `follow` service can be used see how the robot moves through all the waypoints. If the path looks good, these waypoints can be saved back to `waypoints.yaml`.

## Testing

This package contains a test node `mover_test` that runs the demo in RViz and calls the `step` service in such a way that the robot crashes into the floor, returning an error code. To run this, run `catkin_make run_tests`.
