# VESC Drivers

## Overview
1. `vesc_msgs`: ROS message definition for VESC feedback.
1. `vesc_driver`: Core VESC driver. Communicates with the VESC over USB-serial.
1. `vesc_ackermann`: Translates Ackermann steering commands to VESC commands, and VESC feedback to odometry.

## Dependencies
1. [ROS](http://wiki.ros.org/ROS/Installation)
2. Install ROS packages `serial`, `ackermann-msgs`, and `tf` :
    ```
    sudo apt-get install ros-melodic-serial ros-melodic-ackermann-msgs ros-melodic-tf
    ```
    
## Build
In each subdir, run `make`, in the following order:
1. vesc_msgs
1. vesc_driver
1. vesc_ackermann
