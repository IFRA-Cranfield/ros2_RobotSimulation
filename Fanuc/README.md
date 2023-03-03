### Launch Gazebo Simulation Environment

Fanuc CR35-iA:
```sh
ros2 launch cr35ia_ros2_gazebo cr35ia_simulation.launch.py
```

### Launch Gazebo + MoveIt!2 Environment

Fanuc CR35-iA:
```sh
ros2 launch cr35ia_ros2_moveit2 cr35ia.launch.py
```

### Launch Gazebo + MoveIt!2 Environment + ROS2 Robot Triggers/Actions

Fanuc CR35-iA:
```sh
ros2 launch cr35ia_ros2_moveit2 cr35ia_interface.launch.py
```

[NOTE]: ROS2 Humble Packages for Fanuc CR35-iA are not ready yet.
Simulation issues are related to joint dumping and friction parametres.