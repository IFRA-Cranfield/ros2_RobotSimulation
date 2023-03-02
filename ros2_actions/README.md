TBD.

Explain how ros2 actions can be triggered from the terminal.


<h4><u>Execute Robot Simulation</u></h4>

* Panda Robot:
  ```sh
  ros2 launch panda_ros2_gazebo panda_simulation.launch.py
  ```
* ABB IRB-120:
  ```sh
  ros2 launch irb120_ros2_gazebo irb120_simulation.launch.py
  ```
* ABB IRB-120 + Schunk EGP-64:
  ```sh
  ros2 launch irb120egp64_ros2_gazebo irb120egp64_simulation.launch.py
  ```
* ABB IRB-1200:
  ```sh
  ros2 launch irb1200_ros2_gazebo irb1200_simulation.launch.py
  ```
* ABB IRB-6640:
  ```sh
  ros2 launch irb6640_ros2_gazebo irb6640_simulation.launch.py
  ```
* UR3:
  ```sh
  ros2 launch ur3_ros2_gazebo ur3_simulation.launch.py
  ```
* UR5:
  ```sh
  ros2 launch ur5_ros2_gazebo ur5_simulation.launch.py
  ```
* UR10:
  ```sh
  ros2 launch ur10_ros2_gazebo ur10_simulation.launch.py
  ```
* Fanuc CR35-iA:
  ```sh
  ros2 launch cr35ia_ros2_gazebo cr35ia_simulation.launch.py
  ```
* Kuka LBR-IIWA:
  ```sh
  ros2 launch iiwa_ros2_gazebo iiwa_simulation.launch.py
  ```

<h4><u>Execute Robot Simulation w/ MoveIt!2</u></h4>

* Panda Robot:
  ```sh
  ros2 launch panda_ros2_moveit2 panda.launch.py
  ```
* ABB IRB-120:
  ```sh
  ros2 launch irb120_ros2_moveit2 irb120.launch.py
  ```
* ABB IRB-120 + Schunk EGP-64:
  ```sh
  ros2 launch irb120egp64_ros2_moveit2 irb120egp64.launch.py
  ```
* ABB IRB-1200:
  ```sh
  ros2 launch irb1200_ros2_moveit2 irb1200.launch.py
  ```
* ABB IRB-6640:
  ```sh
  ros2 launch irb6640_ros2_moveit2 irb6640.launch.py
  ```
* UR3:
  ```sh
  ros2 launch ur3_ros2_moveit2 ur3.launch.py
  ```
* UR5:
  ```sh
  ros2 launch ur5_ros2_moveit2 ur5.launch.py
  ```
* UR10:
  ```sh
  ros2 launch ur10_ros2_moveit2 ur10.launch.py
  ```
* Fanuc CR35-iA:
  ```sh
  ros2 launch cr35ia_ros2_moveit2 cr35ia.launch.py
  ```
* Kuka LBR-IIWA:
  ```sh
  ros2 launch iiwa_ros2_moveit2 iiwa.launch.py
  ```

<h4><u>Execute Robot Simulation w/ MoveIt!2 and Robot/Gripper Triggers (Action Servers)</u></h4>

* Panda Robot:
  ```sh
  ros2 launch panda_ros2_moveit2 panda_interface.launch.py
  ```
* ABB IRB-120:
  ```sh
  ros2 launch irb120_ros2_moveit2 irb120_interface.launch.py
  ```
* ABB IRB-120 + Schunk EGP-64:
  ```sh
  ros2 launch irb120egp64_ros2_moveit2 irb120egp64_interface.launch.py
  ```
* ABB IRB-1200:
  ```sh
  ros2 launch irb1200_ros2_moveit2 irb1200_interface.launch.py
  ```
* ABB IRB-6640:
  ```sh
  ros2 launch irb6640_ros2_moveit2 irb6640_interface.launch.py
  ```
* UR3:
  ```sh
  ros2 launch ur3_ros2_moveit2 ur3_interface.launch.py
  ```
* UR5:
  ```sh
  ros2 launch ur5_ros2_moveit2 ur5_interface.launch.py
  ```
* UR10:
  ```sh
  ros2 launch ur10_ros2_moveit2 ur10_interface.launch.py
  ```
* Fanuc CR35-iA:
  ```sh
  ros2 launch cr35ia_ros2_moveit2 cr35ia_interface.launch.py
  ```
* Kuka LBR-IIWA:
  ```sh
  ros2 launch iiwa_ros2_moveit2 iiwa_interface.launch.py
  ```

<h4><u>Robot/Gripper Triggers: ROS2.0 Action Calls</u></h4>

The list below contains all different Robot/Gripper Triggers that have been implemented in this repository, and how these are executed by making different ROS2.0 Action Calls from a terminal shell:

* MoveJ: The Robot moves to the specific waypoint, which is specified by Joint Pose values.
  ```sh
  ros2 action send_goal -f /MoveJ ros2_data/action/MoveJ "{goal: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00}}" # (6-DOF)
  ros2 action send_goal -f /MoveJs ros2_data/action/MoveJs "{goal: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00, joint7: 0.00}}" # For Panda and Kuka LBR-IIWA Robots. (7-DOF)
  ```

* MoveG: The Gripper fingers move to the specific pose.
  ```sh
  ros2 action send_goal -f /MoveG ros2_data/action/MoveG "{goal: 0.00}"
  ```

* MoveL: The Robot executes a CARTESIAN/LINEAR path. The End-Effector orientation is kept constant, and the position changes by +-(x,y,z).
  ```sh
  ros2 action send_goal -f /MoveL ros2_data/action/MoveL "{movex: 0.00, movey: 0.00, movez: 0.00}"
  ```

* MoveR: The Robot rotates the selected joint a specific amount of degrees.
  ```sh
  ros2 action send_goal -f /MoveR ros2_data/action/MoveR "{joint: '---', value: 0.00}"
  ```

* MoveXYZW: The Robot moves to the specific waypoint, which is represented by the Position(x,y,z) + EulerAngles(yaw,pitch,roll) End-Effector coordinates.
  ```sh
  ros2 action send_goal -f /MoveXYZW ros2_data/action/MoveXYZW "{positionx: 0.00, positiony: 0.00, positionz: 0.00, yaw: 0.00, pitch: 0.00, roll: 0.00}"
  ```

* MoveXYZ: The Robot moves to the specific waypoint -> Position(x,y,z) maintaining the End-Effector orientation.
  ```sh
  ros2 action send_goal -f /MoveXYZ ros2_data/action/MoveXYZ "{positionx: 0.00, positiony: 0.00, positionz: 0.00}"
  ```

* MoveYPR: The Robot rotates/orientates the End-Effector frame according to the input: EulerAngles(yaw,pitch,roll). The YPR(yaw,pitch,roll)determines the FINAL ROTATION of the End-Effector, which is related to the GLOBAL COORDINATE FRAME.
  ```sh
  ros2 action send_goal -f /MoveYPR ros2_data/action/MoveYPR "{yaw: 0.00, pitch: 0.00, roll: 0.00}"
  ```

* MoveROT: The Robot rotates/orientates the End-Effector frame according to the input: EulerAngles(yaw,pitch,roll). THE ROT(yaw,pitch,roll) determines the ADDED ROTATION of the End-Effector, which is applied to the END-EFFECTOR COORDINATE FRAME.
  ```sh
  ros2 action send_goal -f /MoveROT ros2_data/action/MoveROT "{yaw: 0.00, pitch: 0.00, roll: 0.00}"
  ```

* MoveRP: End-Effector rotation AROUND A POINT -> The Robot rotates/orientates + moves the End-Effector frame according to the input: EulerAngles(yaw,pitch,roll) + Point(x,y,z). THE ROT(yaw,pitch,roll) determines the ADDED ROTATION of the End-Effector, which is applied to the END-EFFECTOR COORDINATE FRAME, AROUND THE (x,y,z) POINT.
  ```sh
  ros2 action send_goal -f /MoveRP ros2_data/action/MoveRP "{yaw: 0.00, pitch: 0.00, roll: 0.00, x: 0.0, y: 0.0, z: 0.0}"
  ``

* NOTE: For the (Yaw - Pitch - Roll) Euler Angles rotation, the following [coordinate system](TBD) has been used as the reference frame for the rotations. In fact, all YPR action calls rotate the robot end-effector to the orientation specified by the (input) Euler Angles, relative to the reference frame.

<h4><u>Example</u>: ABB IRB120 + Schunk EGP64</h4>
https://user-images.githubusercontent.com/98389310/176437077-6e583a0f-50e6-4e00-ae09-56e233f976f5.mp4