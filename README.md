# 启动can卡
sudo  ip  link  set  can0  up  type  can  bitrate  500000
注意遥控器切换为底盘模式

# 保存三维点云地图：

```sh
ros2 service call /map_save std_srvs/srv/Trigger {}\
```

# 保存栅格地图：

```sh
ros2 run nav2_map_server map_saver_cli -f map
```

# 开始记录路径
```sh
ros2 service call /start_recording_path std_srvs/srv/Trigger
```
# 保存路径（会自动停止记录）
```sh
ros2 service call /save_recorded_path std_srvs/srv/Trigger
```
# 开始跟踪路径
```sh
ros2 service call /start_following_path std_srvs/srv/Trigger
```

# 停止跟踪
```sh
ros2 service call /stop_following_path std_srvs/srv/Trigger
```

[component_container_isolated-1] [WARN 1762500147.935269255] [local_costmap.local_costmap]: Sensor origin at (1.68, -0.07 -0.04) is out of map bounds (-1.45, -2.55, 0.00) to (3.53, 2.43, 0.78). The costmap cannot raytrace for it. (raytraceFreespace() at ./plugins/voxel_layer.cpp:276)

