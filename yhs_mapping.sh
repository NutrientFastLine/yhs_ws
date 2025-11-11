#!/bin/bash

cmds=(
    # 启动雷达（同时 source 两个工作空间）
    "source install/setup.bash; source ~/livox_ros_driver2_ws/install/setup.bash; ros2 launch turn_on_yhs_robot yhs_lidar.launch.py"

    "source install/setup.bash; source ~/livox_ros_driver2_ws/install/setup.bash; ros2 launch fast_lio mapping.launch.py "
)

for cmd in "${cmds[@]}"
do
    echo "Current CMD : $cmd"
    gnome-terminal -- bash -c "cd $(pwd); $cmd; exec bash;"
    sleep 0.2
done
