# VESC ROS Driver

## Dependencies
1. [ROS](http://wiki.ros.org/ROS/Installation)
2. Install ROS packages `serial`, `ackermann-msgs`, and `tf` :
    ```
    sudo apt-get install ros-melodic-serial ros-melodic-ackermann-msgs ros-melodic-tf
    ```

## Build
1. Add the project path to `ROS_PACKAGE_PATH`
1. Run `make`

## Run
1. To run the driver:
    ```
    roslaunch vesc vesc_driver_node.launch
    ```
1. A keyboard teleop tool is also available:
    ```
    rosrun vesc keyboard_teleop.py
    ```

## Configuration
All config parameters are included in `config/vesc.yaml`.
