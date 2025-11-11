#!/bin/bash

cmds=(
    # 启动雷达（同时 source 两个工作空间）
    "source install/setup.bash; source ~/livox_ros_driver2_ws/install/setup.bash; ros2 launch turn_on_yhs_robot turn_on_yhs_robot.launch.py"

    "source install/setup.bash; source ~/livox_ros_driver2_ws/install/setup.bash; ros2 launch turn_on_yhs_robot yhs_lidar.launch.py"

    "source install/setup.bash; source ~/livox_ros_driver2_ws/install/setup.bash; ros2 launch yhs_nav2_launcher localization.launch.py "

    "source install/setup.bash; source ~/livox_ros_driver2_ws/install/setup.bash; ros2 launch yhs_nav2_launcher yhs_nav2.launch.py"
)

for cmd in "${cmds[@]}"
do
    echo "Current CMD : $cmd"
    gnome-terminal -- bash -c "cd $(pwd); $cmd; exec bash;"
    sleep 0.2
done
