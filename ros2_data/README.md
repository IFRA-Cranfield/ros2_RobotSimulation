## ros2_RobotSimulation: ROS2 Data
The ros2_data package contains all different ROS2 data structures that are required to execute the ROS2 Actions defined in ros2_actions package:

### MoveJ

MoveJ.action:
* Input: goal(JointPose)
* Output: result(string), feedback(string)

MoveJs.action:
* Input: goal(JointPoseS)
* Output: result(string), feedback(string)

JointPose.msg:
* Data: joint1(float64), joint2(float64), joint3(float64), joint4(float64), joint5(float64), joint6(float64)

JointPoseS.msg:
* Data: joint1(float64), joint2(float64), joint3(float64), joint4(float64), joint5(float64), joint6(float64), joint7(float64)

### MoveG

MoveG.action:
* Input: goal(float64)
* Output: result(string), feedback(string)

### MoveL

MoveL.action:
* Input: movex(float64), movey(float64), movez(float64)
* Output: result(string), feedback(string)

### MoveR

MoveR.action:
* Input: joint(string), value(float64)
* Output: result(string), feedback(string)

### MoveXYZW

MoveXYZW.action:
* Input: positionx(float64), positiony(float64), positionz(float64), yaw(float64), pitch(float64), roll(float64)
* Output: result(string), feedback(string)

### MoveXYZ

MoveXYZ.action:
* Input: positionx(float64), positiony(float64), positionz(float64)
* Output: result(string), feedback(string)

### MoveYPR

MoveYPR.action:
* Input: yaw(float64), pitch(float64), roll(float64)
* Output: result(string), feedback(string)

### MoveROT

MoveROT.action:
* Input: yaw(float64), pitch(float64), roll(float64)
* Output: result(string), feedback(string)

### MoveRP

MoveRP.action:
* Input: yaw(float64), pitch(float64), roll(float64), x(float64), y(float64), z(float64)
* Output: result(string), feedback(string)