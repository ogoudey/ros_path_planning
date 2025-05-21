# Path Planning with ROS

```
$ ros2 run basics grids # A stand in for real maps.

$ ros2 launch nav2_bringup navigation_launch.py   map_subscribe_transient_local:=true   map_topic:="/map"   autostart:=true   use_sim_time:=false
$ ros2 run nav2_map_server map_server   --ros-args     -p subscribe_to_map_topic:=true     -p topic_name:='/map'

$ ros2 run rviz2 rviz2

$ ros2 run basics paths
```

