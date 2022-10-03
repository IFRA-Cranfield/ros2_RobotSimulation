<div id="top"></div>

<!-- 

# ===================================== COPYRIGHT ===================================== #
#                                                                                       #
#  IFRA (Intelligent Flexible Robotics and Assembly) Group, CRANFIELD UNIVERSITY        #
#  Created on behalf of the IFRA Group at Cranfield University, United Kingdom          #
#  E-mail: IFRA@cranfield.ac.uk                                                         #
#                                                                                       #
#  Licensed under the Apache-2.0 License.                                               #
#  You may not use this file except in compliance with the License.                     #
#  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0  #
#                                                                                       #
#  Unless required by applicable law or agreed to in writing, software distributed      #
#  under the License is distributed on an "as-is" basis, without warranties or          #
#  conditions of any kind, either express or implied. See the License for the specific  #
#  language governing permissions and limitations under the License.                    #
#                                                                                       #
#  IFRA Group - Cranfield University                                                    #
#  AUTHORS: Mikel Bueno Viso - Mikel.Bueno-Viso@cranfield.ac.uk                         #
#           Seemal Asif      - s.asif@cranfield.ac.uk                                   #
#           Phil Webb        - p.f.webb@cranfield.ac.uk                                 #
#                                                                                       #
#  Date: October, 2022.                                                                 #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

-->

<!--

  README.md TEMPLATE obtined from:
      https://github.com/othneildrew/Best-README-Template
      AUTHOR: OTHNEIL DREW 

-->

<!-- HEADER -->
<br />
<div align="center">
  <a>
    <img src="media/header.jpg" alt="header" width="651" height="190.5">
  </a>

  <br />

  <h2 align="center">ROS2.0 ROBOT SIMULATION</h2>

  <p align="center">
    IFRA (Intelligent Flexible Robotics and Assembly) Group
    <br />
    Centre for Structures, Assembly and Intelligent Automation
    <br />
    Cranfield University
  </p>
</div>

<!-- TABLE OF CONTENTS -->
<details>
  <summary>Table of Contents</summary>
  <ol>
    <li>
      <a href="#about">About</a>
      <ul>
        <li><a href="#intelligent-flexible-robotics-and-assembly-group">IFRA Group</a></li>
        <li><a href="#ros2robotsimulation-repository">ros2_RobotSimulation Repository</a></li>
      </ul>
    </li>
    <li>
      <a href="#getting-started">Getting Started</a>
      <ul>
        <li><a href="#ros20-foxy-environment-set-up">ROS2.0 Foxy Environment Set-Up</a></li>
        <li><a href="#installation">Installation</a></li>
      </ul>
    </li>
    <li><a href="#supported-robots">Supported Robots</a></li>
    <li><a href="#usage">Usage</a></li>
    <li><a href="#contributing">Contributing</a></li>
    <li><a href="#license">License</a></li>
    <li><a href="#cite-our-work">Cite our work</a></li>
    <li><a href="#contact">Contact</a></li>
    <li><a href="#acknowledgments">Acknowledgments</a></li>
  </ol>
</details>

<!-- ABOUT THE PROJECT -->
## About

### Intelligent Flexible Robotics and Assembly Group

The IFRA (Intelligent Flexible Robotics and Assembly) Group is part of the Centre for Structures, Assembly and Intelligent Automation at Cranfield University.

IFRA Group pushes technical boundaries. At IFRA we provide high tech automation & assembly solutions, and we support smart manufacturing with Smart Industry technologies and solutions. Flexible Manufacturing Systems (FMS) are a clear example. They can improve overall operations and throughput quality by adapting to real-time changes and situations, supporting and pushing the transition towards flexible, intelligent and responsive automation, which we continuously seek and support.

The implementation of IIoT and the use of computer numerical control equipment enable interconnectivity and the exchange of data across the shop floor, and the interconnected shop floor enables the automation of work machines to support FMS. In a nutshell, Automated Industrial Processes bring agility to production cycles and enable shop floor equipment to pivot facility operations when dealing with changing demand cycles.

The IFRA Group undertakes innovative research to design, create and improve Intelligent, Responsive and Flexible automation & assembly solutions, and this series of GitHub repositories provide background information and resources of how these developments are supported.

### ros2_RobotSimulation Repository

ROS (Robotics Operating System) is a great tool that, combined with Gazebo and MoveIt! frameworks, provides a great amount of resources and capabilities to develop different Robotics Applications with a huge range of Industrial and Collaborative Robots.

Nonetheless, developing new applications in ROS requires a huge previous work, which consists in developing ROS Packages that contain all the Robot data required for the Simulation and Control, including:
  - Kinematics.
  - Control parametres.
  - CAD files.
  - Physical parametres.
  - Joint limits.
  - Extra features.

As a common rule, the software stack needed to execute and test an Industrial Robot Simulation in ROS is:
  - A "robot_gazebo" package, which simulates the behaviour of the Robot.
  - A "robot_moveit" package, which controls the performance of the Robot.
With both combined, different applications and implementations can be developed using the Robotics Operating System (ROS).

ROS is now undertaking a transformation process to ROS2, a newer and improved version of ROS1. Thus, this involves that all the developments, implementations and packages released for ROS1 have to be forked/translated to ROS2 or are directly unavailable in ROS2.

The IFRA Group in Cranfield University (Bedfordshire, UK) has identified a huge gap in the availability of "ready-to-use" ROS2 Industrial Robot Simulation packages, and that is why the ros2_RobotSimulation ROS2 repository has been developed and released. The repository consists of Gazebo (simulation) + MoveIt!2 (control) package combinations for each supported Industrial Robot (or Robot + Gripper combinations), and follows a common standard for a better understanding and further development.

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- GETTING STARTED -->
## Getting Started

All packages in this repository have been developed, executed and tested in an Ubuntu 20.04 machine with ROS2.0 Foxy. Please find below all the required steps to set-up a ROS2.0 Foxy environment in Ubuntu and install the Robot Simulation packages.

### ROS2.0 Foxy Environment Set-Up

1. Install Ubuntu 20.04: [https://ubuntu.com/desktop](https://ubuntu.com/desktop)
2. Install Git:
    * In terminal shell:
        ```sh
        sudo apt install git
        ```
    * Git account configuration:
        ```sh
        git config --global user.name YourUsername
        git config --global user.email YourEmail
        git config --global color.ui true
        git config --global core.editor code --wait # Visual Studio Code is recommended.
        git config --global credential.helper store
        ```
3. Install ROS2.0 Foxy: 
    * Follow instructions in: [ROS2 Foxy Tutorials - Installation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
    * Source the ROS2.0 Foxy installation in the .bashrc file (hidden file in /home):
        ```sh
        source opt/ros/foxy/setup.bash
        ```
4. Install some additional (required) packages:
    * Colcon: [ROS2 Foxy Tutorials - Colcon](https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
    * ROSdep: [ROS Wiki - Installing ROSdep](https://wiki.ros.org/rosdep#Installing_rosdep)
    * Turtlesim + rqt: [ROS2 Foxy Tutorials - Turtlesim](https://docs.ros.org/en/foxy/Tutorials/Turtlesim/Introducing-Turtlesim.html)
5. Configure the ROS2.0 Foxy ~/dev_ws environment/workspace:
    * Follow instructions in: [ROS2 Foxy Tutorials - Create a Workspace](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html)
    * Source ~/dev_ws workspace in .bashrc file:
        ```sh
        source ~/dev_ws/install/local_setup.bash
        ```
6. Install ROS2.0 MoveIt!2 framework:
    * Follow instructions in: [Picnik Robotics - MoveIt!2](https://moveit.picknik.ai/foxy/doc/getting_started/getting_started.html)
    * Source MoveIt!2 workspace in .bashrc file:
        ```sh
        source ~/ws_moveit2/install/setup.bash
        ```
7. Install required packages for Gazebo + MoveIt!2 Robot Simulation:
    * Catkin Tools:
        ```sh
        sudo apt-get install python3-catkin-tools
        ```
    * gazebo-ros-pkgs:
        ```sh
        sudo apt install ros-foxy-gazebo-ros-pkgs
        ```
    * ROSBridge:
        ```sh
        cd ~/dev_ws/src
        git clone https://github.com/RobotWebTools/rosbridge_suite.git -b ros2
        cd ..
        colcon build --symlink-install
        ```
    * gazebo_ros2_control:
        ```sh
        cd ~/dev_ws/src
        git clone https://github.com/ros-simulation/gazebo_ros2_control.git -b foxy
        cd ..
        colcon build --symlink-install
        ```
    * gazebo_ros_demos:
        ```sh
        cd ~/dev_ws/src
        git clone https://github.com/ros-simulation/gazebo_ros_demos.git -b ahcorde/port/ros2
        cd ..
        colcon build --symlink-install
        ```
    * ROS2.0 Control packages (download the packages, paste them into the [~/dev_ws/src] folder and colcon build):
      * ros2_control: [https://github.com/ros-controls/ros2_control/tree/foxy](https://github.com/ros-controls/ros2_control/tree/foxy) 
      * ros2_controllers: [https://github.com/ros-controls/ros2_controllers/tree/foxy](https://github.com/ros-controls/ros2_controllers/tree/foxy)
      * ros2_control_demos: [https://github.com/ros-controls/ros2_control_demos/tree/foxy](https://github.com/ros-controls/ros2_control_demos/tree/foxy)


### Move Group Interface

A small improvement of the move_group_interface.h file has been developed in order to execute the Robot/Gripper triggers in this repository. Both the upgraded file and the instructions of how to implement it can be found here:
* [move_group_interface_improved.h](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/include)


### Installation

```sh
cd ~/dev_ws/src
git clone https://github.com/IFRA-Cranfield/ros2_RobotSimulation.git 
cd ..
colcon build --symlink-install
```

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Supported Robots

The Simulation & Control packages of the following Robots are currently available:
- [Panda Robot](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/PandaRobot)
- [ABB IRB-120 Robot](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/ABBRobots/IRB120)
- [ABB IRB-120 Robot with Schunk EGP-64 Gripper](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/ABBRobots/IRB120)
- [ABB IRB-1200 Robot](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/ABBRobots/IRB1200)
- [ABB IRB-6640 Robot](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/ABBRobots/IRB6640)
- [UR3 Robot](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/UniversalRobots/UR3)
- [UR5 Robot](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/UniversalRobots/UR5)
- [UR10 Robot](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/UniversalRobots/UR10)
- [Fanuc CR35-iA Robot](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/Fanuc/CR35iA)
- [Kuka LBR-IIWA](https://github.com/IFRA-Cranfield/ros2_RobotSimulation/tree/master/Kuka/LBRiiwa)

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- USAGE EXAMPLES -->
## Usage

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
  ros2 action send_goal -f /MoveJ ros2_data/action/MoveJ "{goal: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00}}"
  ros2 action send_goal -f /MoveJpanda ros2_data/action/MoveJpanda "{goal: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00, joint7: 0.00}}" # For Panda Robot.
  ros2 action send_goal -f /MoveJiiwa ros2_data/action/MoveJiiwa "{goal: {joint1: 0.00, joint2: 0.00, joint3: 0.00, joint4: 0.00, joint5: 0.00, joint6: 0.00, joint7: 0.00}}" # For Kuka LBR-IIWA Robot.
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

* MoveYPR: The Robot rotates/orientates the End-Effector frame according to the input: EulerAngles(yaw,pitch,roll).
  ```sh
  ros2 action send_goal -f /MoveYPR ros2_data/action/MoveYPR "{yaw: 0.00, pitch: 0.00, roll: 0.00}"
  ```

* NOTE: For the (Yaw - Pitch - Roll) Euler Angles rotation, the following [coordinate system](TBD) has been used as the reference frame for the rotations. In fact, all YPR action calls rotate the robot end-effector to the orientation specified by the (input) Euler Angles, relative to the reference frame.

<h4><u>Example</u>: ABB IRB120 + Schunk EGP64</h4>
https://user-images.githubusercontent.com/98389310/176437077-6e583a0f-50e6-4e00-ae09-56e233f976f5.mp4

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to learn, inspire, and create. Any contributions you make are **greatly appreciated**.

If you have a suggestion that would make this better, please fork the repo and create a pull request. You can also simply open an issue with the tag "enhancement".
Don't forget to give the project a star! Thanks again!

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- LICENSE -->
## License

<p>
  Intelligent Flexible Robotics and Assembly Group
  <br />
  Created on behalf of the IFRA Group at Cranfield University, United Kingdom
  <br />
  E-mail: IFRA@cranfield.ac.uk 
  <br />
  <br />
  Licensed under the Apache-2.0 License.
  <br />
  You may obtain a copy of the License at: http://www.apache.org/licenses/LICENSE-2.0
  <br />
  <br />
  <a href="https://www.cranfield.ac.uk/">Cranfield University</a>
  <br />
  School of Aerospace, Transport and Manufacturing
  <br />
    <a href="https://www.cranfield.ac.uk/centres/centre-for-structures-assembly-and-intelligent-automation">Centre for Structures, Assembly and Intelligent Automation</a>
  <br />
  College Road, Cranfield
  <br />
  MK43 0AL, UK
  <br />
</p>

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CITE OUR WORK -->
## Cite our work

<p>
  You can cite our work with the following statement:
  <br />
  IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.
</p>

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

<p>
  Mikel Bueno Viso - Research Assistant in Intelligent Automation at Cranfield University
  <br />
  E-mail: Mikel.Bueno-Viso@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/mikel-bueno-viso/
  <br />
  <br />
  Dr. Seemal Asif - Lecturer in Artificial Intelligence and Robotics at Cranfield University
  <br />
  E-mail: s.asif@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/dr-seemal-asif-ceng-fhea-miet-9370515a/
  <br />
  <br />
  Professor Phil Webb - Professor of Aero-Structure Design and Assembly at Cranfield University
  <br />
  E-mail: p.f.webb@cranfield.ac.uk
  <br />
  LinkedIn: https://www.linkedin.com/in/phil-webb-64283223/
  <br />
</p>

<p align="right">(<a href="#top">back to top</a>)</p>

<!-- ACKNOWLEDGMENTS -->
## Acknowledgments

* [README.md template - Othneil Drew](https://github.com/othneildrew/Best-README-Template).
* [ROS2.0 Documentation - Foxy](https://docs.ros.org/en/foxy/index.html).
* [PicNik Robotics - MoveIt!2 Documentation](https://moveit.picknik.ai/foxy/index.html).
* [Panda Robot - ROS Repository](https://github.com/ros-planning/panda_moveit_config).
* [ABB - ROS Repositories](http://wiki.ros.org/abb).
* [UR Robots - ROS2 Description](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description).
* [UR Robots - ROS2 Gazebo](https://github.com/UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation).
* [UR Robots - ROS2 MoveIt!2](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/tree/main/ur_moveit_config).
* [The Construct - ROS2 Control Tutorial](https://www.youtube.com/watch?v=lo1bXm8Aoqc).

<p align="right">(<a href="#top">back to top</a>)</p>


