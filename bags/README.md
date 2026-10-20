# Bag Readme

This document provides info on how to get bags running and what each bag does

## Working filter rosbag (`pf_live_run`)

Recording of our particle filter (`pf.py`) running on the robot's laser and odometry data. The data starts right away, so no start offset is needed.

After colcon building and sourcing:

Launch map server

In same terminal as building and sourcing:
```
ros2 launch robot_localization launch_map_server.py map_yaml:=$HOME/ros2_ws/src/akil_sid_robot_localization_p2/maps/mac_1st_floor_final.yaml
```

In a second terminal (initialize rviz)
```
rviz2 -d ~/ros2_ws/src/akil_sid_robot_localization_p2/rviz/turtlebot_bag_files.rviz --ros-args -p use_sim_time:=true
```

In third terminal (play rosbag)
```
ros2 bag play ~/ros2_ws/src/akil_sid_robot_localization_p2/bags/pf_live_run --clock
```

## Alternative rosbag (`alternative_high_initial_wait`)

Earlier run of our filter. The bag has a long wait at the start, so it is played with an offset. The offset skips the map in the bag, so the map server is launched separately.

After colcon building and sourcing:

Launch map server

In same terminal as building and sourcing:
```
ros2 launch robot_localization launch_map_server.py map_yaml:=$HOME/ros2_ws/src/akil_sid_robot_localization_p2/maps/mac_1st_floor_final.yaml
```

In a second terminal (initialize rviz)
```
rviz2 -d ~/ros2_ws/src/akil_sid_robot_localization_p2/rviz/turtlebot_bag_files.rviz --ros-args -p use_sim_time:=true
```

In third terminal (play rosbag)
```
ros2 bag play ~/ros2_ws/src/akil_sid_robot_localization_p2/bags/alternative_high_initial_wait --clock --start-offset 235
```
