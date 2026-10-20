# Bag Readme

This document provides info on how to get bags running and what each bag does

## Working bag

After colcon building and sourcing:

Launch map server (bag has offset, map server not automatically launched)

In same terminal as building and sourcing:
```
ros2 launch robot_localization launch_map_server.py map_yaml:=$HOME/~/ros2_ws/src/akil_sid_robot_localization_p2/maps/mac_1st_floor_final.yaml

```
In a second terminal (initialize rviz)
```
rviz2 -d ~/ros2_ws/src/akil_sid_robot_localization_p2/rviz/turtlebot_bag_files.rviz --ros-args -p use_sim_time:=true
```

In third terminal (play rosbag)
```
ros2 bag play ~/Documents/Sem3/CompRobo/ros2_ws/src/akil_sid_robot_localization_p2/bags/working_run_this --clock --start-offset 235
```