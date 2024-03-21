## Overview: 
Developed an 𝗜𝗻𝗱𝘂𝘀𝘁𝗿𝗶𝗮𝗹 𝗥𝗼𝗯𝗼𝘁𝗶𝗰 𝗠𝗮𝗻𝘂𝗳𝗮𝗰𝘁𝘂𝗿𝗶𝗻𝗴 𝗦𝘆𝘀𝘁𝗲𝗺 featuring 𝗔𝗚𝗩𝘀, 𝗺𝗮𝗻𝗶𝗽𝘂𝗹𝗮𝘁𝗼𝗿𝘀, and sensors for automated component manufacturing in a 𝗥𝗢𝗦𝟮 Gazebo environment. Modeled after the 𝗔𝗥𝗜𝗔𝗖 𝟮𝟬𝟮𝟯 challenge, focusing on agility and autonomy in 𝗸𝗶𝘁𝘁𝗶𝗻𝗴 𝘁𝗮𝘀𝗸𝘀.


## Key Achievements:
✦ 𝗖𝗼𝘂𝗿𝘀𝗲𝘄𝗼𝗿𝗸 𝗦𝗶𝗺𝘂𝗹𝗮𝘁𝗶𝗼𝗻:
- Developed as part of ENPM663 coursework, mirroring the ARAIC 2023 challenge to assess industrial robot 𝗮𝗴𝗶𝗹𝗶𝘁𝘆.

✦ 𝗥𝗢𝗦𝟮 𝗮𝗻𝗱 𝗚𝗮𝘇𝗲𝗯𝗼 𝗙𝗿𝗮𝗺𝗲𝘄𝗼𝗿𝗸𝘀:
- Utilized ROS2, Gazebo, and 𝗥𝗩𝗶𝘇 for system development, integrating 𝗨𝗥𝟱 manipulators, AGVs, IR cameras, 𝗽𝗿𝗼𝘅𝗶𝗺𝗶𝘁𝘆 sensors, and 𝟯𝗗 𝗱𝗲𝗽𝘁𝗵 cameras.

✦ 𝗞𝗶𝘁𝘁𝗶𝗻𝗴 𝗧𝗮𝘀𝗸 𝗔𝘂𝘁𝗼𝗺𝗮𝘁𝗶𝗼𝗻:
- Addressed challenges such as 𝗳𝗮𝘂𝗹𝘁𝘆 𝗮𝗻𝗱 𝗶𝗻𝘀𝘂𝗳𝗳𝗶𝗰𝗶𝗲𝗻𝘁 𝗽𝗮𝗿𝘁𝘀, emphasizing autonomous completion of kitting tasks within industrial manufacturing.

✦ 𝗔𝘂𝘁𝗼𝗻𝗼𝗺𝘆 𝗮𝗻𝗱 𝗣𝗿𝗼𝗱𝘂𝗰𝘁𝗶𝘃𝗶𝘁𝘆 𝗙𝗼𝗰𝘂𝘀:
- Aligned with ARIAC's objective of enhancing industrial robot 𝗮𝘂𝘁𝗼𝗻𝗼𝗺𝘆 and productivity, minimizing 𝗵𝘂𝗺𝗮𝗻 𝗶𝗻𝘁𝗲𝗿𝘃𝗲𝗻𝘁𝗶𝗼𝗻 in complex manufacturing processes.


## Instructions to run the simuation:
- This project has one launch file which launches the gazebo launch file and all other c++ and python nodes
- robot_msgs package has to be built first before building the package rwa67.

- CLI commands:
    ```
    cd ~/ariac_ws/
    ```
    ```
    colcon build --packages-select robot_msgs
    ```
    ```
    colcon build --packages-select rwa67
    ```
    ```
    ros2 launch rwa67 rwa67_app.launch.py
    ```

## Documentation:
- Project Report: [click here](./Documentation/Report/ENPM663_RWA67_Report.pdf)
- Project Presentation: [click here](./Documentation/Presentation/ENPM663_RWA67_PPT.pdf)
