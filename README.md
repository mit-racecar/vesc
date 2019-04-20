# VESC Drivers

## Dependencies
1. ROS
2. Manually installed ROS packages `serial` and `ackermann-msgs`
    ```
    sudo apt-get install ros-melodic-serial ros-melodic-ackermann-msgs
    ```
    
## Build
In each subdir, run `build`, in the following order:
1. vesc_msgs
1. vesc_driver
1. vesc_ackermann
