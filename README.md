# RWA67 Assignment 
Repo for RWAs of ENPM663 - Building Manufacturing Robots Software Systems

## Instructions to run RWA67
- This project has one launch file which launches the gazebo launch file and all other c++ and python nodes

- CLI commands:
    ```
    cd ~/ariac_ws/
    ```
    ```
    colcon build --packages-select robot_msgs
    colcon build --packages-select rwa67
    ```
    ```
    ros2 launch rwa67 rwa67_app.launch.py
    ```
