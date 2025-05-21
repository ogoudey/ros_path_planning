# Path Planning with ROS

```
$ ros2 run basics grids # A stand in for real maps.

$ ros2 launch nav2_bringup navigation_launch.py   map_subscribe_transient_local:=true   map_topic:="/map"   autostart:=true   use_sim_time:=false
$ ros2 run nav2_map_server map_server   --ros-args     -p subscribe_to_map_topic:=true     -p topic_name:='/map'

$ ros2 run rviz2 rviz2

$ ros2 run basics paths

(may also need:

$ ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map odom
$ ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom base_link

)
```

## This is a ROS Package
1. Clone to a ROS workspace in `root/src/`
2. Use `colcon build` and source the overlay with `. install/setup.bash`

