### Launch Gazebo Simulation Environment

ABB IRB-120:
```sh
ros2 launch irb120_ros2_gazebo irb120_simulation.launch.py
```

ABB IRB-1200:
```sh
ros2 launch irb1200_ros2_gazebo irb120_simulation.launch.py
```

ABB IRB-6640:
```sh
ros2 launch irb6640_ros2_gazebo irb6640_simulation.launch.py
```

### Launch Gazebo + MoveIt!2 Environment

ABB IRB-120:
```sh
ros2 launch irb120_ros2_moveit2 irb120.launch.py
```

ABB IRB-1200:
```sh
ros2 launch irb1200_ros2_moveit2 irb1200.launch.py
```

ABB IRB-6640:
```sh
ros2 launch irb6640_ros2_moveit2 irb6640.launch.py
```

### Launch Gazebo + MoveIt!2 Environment + ROS2 Robot Triggers/Actions

ABB IRB-120:
```sh
ros2 launch irb120_ros2_moveit2 irb120_interface.launch.py
```

ABB IRB-1200:
```sh
ros2 launch irb1200_ros2_moveit2 irb1200_interface.launch.py
```

ABB IRB-6640:
```sh
ros2 launch irb6640_ros2_moveit2 irb6640_interface.launch.py
```