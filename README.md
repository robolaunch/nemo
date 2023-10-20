# Launch Cloudy Nemo Simulator

Launch robot on gazebo

```
ros2 launch articubot_one launch_sim.launch.py 
```
Launch ekf 
```
ros2 launch articubot_one ekf.launch.py 
```
Launch Slam

```
ros2 launch articubot_one online_async_launch.py
```

Launch Navigation

```
ros2 launch articubot_one navigation_launch.py
```

Launch costmap filter

```
ros2 launch nav2_costmap_filters_demo costmap_filter_info.launch.py
```
