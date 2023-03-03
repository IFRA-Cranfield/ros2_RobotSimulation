#!/usr/bin/python3

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
#  Date: October, 2022.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

# cr35ia_simulation.launch.py:
# Launch file for the FANUC CR35-iA Robot GAZEBO SIMULATION in ROS2 Humble:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
import xacro
import yaml

# LOAD FILE:
def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None
# LOAD YAML:
def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        # parent of IOError, OSError *and* WindowsError where available.
        return None

# ========== **GENERATE LAUNCH DESCRIPTION** ========== #
def generate_launch_description():
    
    # ***** GAZEBO ***** #   
    # DECLARE Gazebo WORLD file:
    cr35ia_ros2_gazebo = os.path.join(
        get_package_share_directory('cr35ia_ros2_gazebo'),
        'worlds',
        'cr35ia.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': cr35ia_ros2_gazebo}.items(),
             )

    # ========== COMMAND LINE ARGUMENTS ========== #
    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ros2_RobotSimulation --> Fanuc CR35-iA")
    print("Launch file -> cr35ia_simulation.launch.py")

    print("")
    print("Robot configuration:")
    print("")

    # Cell Layout:
    print("- Cell layout:")
    print("     + No cell layout variants for this robot.")
    cell_layout_1 = "true"
    
    # error = True
    # while (error == True):
    #     print("     + Option N1: Fanuc CR35-iA alone.")
    #     print("     + Option N2: ***.")
    #     cell_layout = input ("  Please select: ")
    #     if (cell_layout == "1"):
    #         error = False
    #         cell_layout_1 = "true"
    #         cell_layout_2 = "false"
    #     elif (cell_layout == "2"):
    #         error = False
    #         cell_layout_1 = "false"
    #         cell_layout_2 = "true"
    #     else:
    #         print ("  Please select a valid option!")
    print("")

    # End-Effector:
    print("- End-effector:")
    print("     + No EE variants for this robot.")
    EE_no = "true"
    
    # error = True
    # while (error == True):
    #     print("     + Option N1: No end-effector.")
    #     print("     + Option N2: ***.")
    #     end_effector = input ("  Please select: ")
    #     if (end_effector == "1"):
    #         error = False
    #         EE_no = "true"
    #         EE_*** = "false"
    #     elif (end_effector == "2"):
    #         error = False
    #         EE_no = "false"
    #         EE_*** = "true"
    #     else:
    #         print ("  Please select a valid option!")
    print("")

    # ***** ROBOT DESCRIPTION ***** #
    # Fanuc CR35-iA Description file package:
    cr35ia_description_path = os.path.join(
        get_package_share_directory('cr35ia_ros2_gazebo'))
    # Fanuc CR35-iA ROBOT urdf file path:
    xacro_file = os.path.join(cr35ia_description_path,
                              'urdf',
                              'cr35ia.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for Fanuc CR35-iA:
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={
        "cell_layout_1": cell_layout_1,
        # "cell_layout_2": cell_layout_2,
        "EE_no": EE_no,
        # "EE_**": EE_**,
        })
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # ROBOT STATE PUBLISHER NODE:
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'cr35ia'],
                        output='screen')

    # ***** CONTROLLERS ***** #
    # Joint state broadcaster:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Joint trajectory controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["cr35ia_controller", "-c", "/controller_manager"],
    )

    # ***** RETURN LAUNCH DESCRIPTION ***** #
    return LaunchDescription([
        
        gazebo, 
        node_robot_state_publisher,
        spawn_entity,

        RegisterEventHandler(
            OnProcessExit(
                target_action = spawn_entity,
                on_exit = [
                    joint_state_broadcaster_spawner,
                ]
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action = joint_state_broadcaster_spawner,
                on_exit = [
                    joint_trajectory_controller_spawner,
                ]
            )
        ),

    ])