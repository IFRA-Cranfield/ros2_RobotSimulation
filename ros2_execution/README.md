## ros2_RobotSimulation: ROS2 Execution

The ros2_execution package allows us to run programs/sequences by executing ROS2 Actions one after the other. This can be done thanks to the ros2_execution.py script, which contains the Action Clients for the ROS2 Actions (defined in ros2_actions package), and executes the robot movements one after the other according to a pre-defined sequence of actions. 

Nowadays, every single robot manufacturer has its own programming language, a customised way of generating/executing programs and a tailored user-interface, which makes it difficult to combine different robots and recycle programs/sequences. However, the combination of ROS2 and MoveIt!2, the Robot Simulation packages in ros2_RobotSimulation and the ROS2 Robot Triggers in ros2_actions presents a huge opportunity to create a tool that executes programs and sequences for different robots/end-effectors in the exact same way. 

And that is what ros2_execution basically does! It replicates the essential feature of executing pre-defined or pre-generated robot programs/scripts, but with an extra feature: It could perfectly execute a program for a UR, ABB, Fanuc, Kuka, Panda... with a simple requirement: The Hardware must support ROS2. 

Please find below a brief explanation about how the programs are executed and how the sequences are defined and saved in the system.

### Program execution
Programs can be executed by running the following command in the Ubuntu Terminal:
```sh
ros2 run ros2_execution ros2_execution.py --ros-args -p PROGRAM_FILENAME:="---" -p ROBOT_MODEL:="---" -p EE_MODEL:="---"'
```
* The PROGRAM_FILENAME parameter is the name of the file which contains the program. The program is saved in a .txt file, and the name must be inputted excluding the ".txt" extension.
* The ROBOT_MODEL parameter represents the model of the robot. Options: irb120 - irb1200 - irb6640 - cr35ia - ur3 - ur5 - ur10 - iiwa - panda.
* The EE_MODEL parameter represents the model of the end-effector. Options: schunk - panda_hand - none.

### Pre-defined sequence: Format
The pre-defined programs are saved inside the /programs folder as .txt files. Every single line of the .txt file represents an execution step (being the 1st line: 1st step, 2nd line: 2nd step, ...), and it is represented as a python dictionary. The following list showcases how every single ROS2 Action has to be inputted in the program.txt:
* For MoveJ ---> {'action': 'MoveJ', 'value': {'joint1': 0.0, 'joint2': 0.0, 'joint3': 0.0, 'joint4': 0.0, 'joint5': 0.0, 'joint6': 0.0}, 'speed': 1.0}
* For MoveL ---> {'action': 'MoveL', 'value': {'movex': 0.0, 'movey': 0.0, 'movez': 0.0}, 'speed': 1.0}
* For MoveR ---> {'action': 'MoveR', 'value': {'joint': '---', 'value': 0.0}, 'speed': 1.0}
* For MoveXYZW ---> {'action': 'MoveXYZW', 'value': {'positionx': 0.0, 'positiony': 0.0, 'positionz': 0.0, 'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 'speed': 1.0}
* For MoveXYZ ---> {'action': 'MoveXYZ', 'value': {'positionx': 0.0, 'positiony': 0.0, 'positionz': 0.0}, 'speed': 1.0}
* For MoveYPR ---> {'action': 'MoveYPR', 'value': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 'speed': 1.0}
* For MoveROT ---> {'action': 'MoveROT', 'value': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0}, 'speed': 1.0}
* For MoveRP ---> {'action': 'MoveRP', 'value': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0}, 'speed': 1.0}
* For MoveG: 
    * To Open Gripper ---> {'action': 'GripperOpen'}
    * To Close Gripper ---> {'action': 'GripperClose'}
* For the ros2_grasping feature:
    * To attach object to end-effector ---> {'action': 'Attach', 'value': {'object': '---', 'endeffector': '---'}}
    * To detach object ---> {'action': 'Detach', 'value': {'object': '---'}}