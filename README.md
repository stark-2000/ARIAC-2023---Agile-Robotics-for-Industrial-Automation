# ENPM663_RWA
Repo for RWAs of ENPM663

Instructions to run RWA4 
- This project does not have a consolidated launch file.  You will need 6 terminals open to run the project
- In each terminal launch the nodes with the following commands
    1) ros2 launch ariac_gazebo ariac.launch.py trial_name:=rwa4_summer2023
    2) ros2 run rwa4 ship_order_exe
    3) ros2 run rwa4 order_manager.py
    4) ros2 run rwa4 submit_order.py
    5) ros2 run rwa4 end_comp_client_exe.py
    6) ros2 run rwa4 service_client_exe

- The order does not matter, except for the service_client_exe, which must be run last because it starts the competition.