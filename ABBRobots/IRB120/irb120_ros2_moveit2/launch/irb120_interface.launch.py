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
#  Date: July, 2022.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

# irb120.launch.py:
# Launch file for the ABB-IRB120 Robot GAZEBO + MoveIt!2 SIMULATION in ROS2 Humble:

# Import libraries:
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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


    # *********************** Gazebo *********************** # 
    
    # DECLARE Gazebo WORLD file:
    irb120_ros2_gazebo = os.path.join(
        get_package_share_directory('irb120_ros2_gazebo'),
        'worlds',
        'irb120.world')
    # DECLARE Gazebo LAUNCH file:
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
                launch_arguments={'world': irb120_ros2_gazebo}.items(),
             )

    # ***** COMMAND LINE ARGUMENTS ***** #
    print("")
    print(" --- Cranfield University --- ")
    print("        (c) IFRA Group        ")
    print("")

    print("ros2_RobotSimulation --> ABB IRB-120")
    print("Launch file -> irb120_interface.launch.py")

    print("")
    print("Robot configuration:")
    print("")

    # Cell Layout:
    print("- Cell layout:")
    error = True
    while (error == True):
        print("     + Option N1: ABB IRB-120 alone.")
        print("     + Option N2: ABB IRB-120 in Cranfield University cell.")
        print("     + Option N3: ABB IRB-120 Pick&Place Use-Case.")
        cell_layout = input ("  Please select: ")
        if (cell_layout == "1"):
            error = False
            cell_layout_1 = "true"
            cell_layout_2 = "false"
            cell_layout_3 = "false"
        elif (cell_layout == "2"):
            error = False
            cell_layout_1 = "false"
            cell_layout_2 = "true"
            cell_layout_3 = "false"
        elif (cell_layout == "3"):
            error = False
            cell_layout_1 = "false"
            cell_layout_2 = "false"
            cell_layout_3 = "true"
        else:
            print ("  Please select a valid option!")
    print("")

    # End-Effector:
    print("- End-effector:")
    error = True
    while (error == True):
        print("     + Option N1: No end-effector.")
        print("     + Option N2: Schunk EGP-64 parallel gripper.")
        end_effector = input ("  Please select: ")
        if (end_effector == "1"):
            error = False
            EE_no = "true"
            EE_schunk = "false"
        elif (end_effector == "2"):
            error = False
            EE_no = "false"
            EE_schunk = "true"
        else:
            print ("  Please select a valid option!")
    print("")

    # ***** ROBOT DESCRIPTION ***** #
    # ABB-IRB120 Description file package:
    irb120_description_path = os.path.join(
        get_package_share_directory('irb120_ros2_gazebo'))
    # ABB-IRB120 ROBOT urdf file path:
    xacro_file = os.path.join(irb120_description_path,
                              'urdf',
                              'irb120.urdf.xacro')
    # Generate ROBOT_DESCRIPTION for ABB-IRB120:
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc, mappings={
        "cell_layout_1": cell_layout_1,
        "cell_layout_2": cell_layout_2,
        "cell_layout_3": cell_layout_3,
        "EE_no": EE_no,
        "EE_schunk": EE_schunk,
        })
    robot_description_config = doc.toxml()
    robot_description = {'robot_description': robot_description_config}

    # SPAWN ROBOT TO GAZEBO:
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'irb120'],
                        output='screen')

    # ***** STATIC TRANSFORM ***** #
    # NODE -> Static TF:
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )
    # Publish TF:
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[
            robot_description,
            {"use_sim_time": True}
        ]
    )

    # ***** ROS2_CONTROL -> LOAD CONTROLLERS ***** #

    # Joint STATE BROADCASTER:
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    # Joint TRAJECTORY Controller:
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["irb120_controller", "-c", "/controller_manager"],
    )
    
    # === SCHUNK EGP-64 === #
    egp64left_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["egp64_finger_left_controller", "-c", "/controller_manager"],
    )
    egp64right_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["egp64_finger_right_controller", "-c", "/controller_manager"],
    )
    # === SCHUNK EGP-64 === #


    # *********************** MoveIt!2 *********************** #   
    
    # Command-line argument: RVIZ file?
    rviz_arg = DeclareLaunchArgument("rviz_file", default_value="False", description="Load RVIZ file.")

    # *** PLANNING CONTEXT *** #
    # Robot description, SRDF:
    if (EE_no == "true"):
        robot_description_semantic_config = load_file(
            "irb120_ros2_moveit2", "config/irb120.srdf"
        )
    # === SCHUNK EGP-64 === #
    elif (EE_schunk == "true"):
        robot_description_semantic_config = load_file("irb120_ros2_moveit2", "config/irb120egp64.srdf")
    # === SCHUNK EGP-64 === #

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}
    
    # Kinematics.yaml file:
    kinematics_yaml = load_yaml("irb120_ros2_moveit2", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    # Move group: OMPL Planning.
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    # Load ompl_planning.yaml file:
    if (EE_no == "true"):
        ompl_planning_yaml = load_yaml("irb120_ros2_moveit2", "config/ompl_planning.yaml")
    # === SCHUNK EGP-64 === #
    elif (EE_schunk == "true"):ompl_planning_yaml = load_yaml("irb120_ros2_moveit2", "config/ompl_planning_egp64.yaml")
    # === SCHUNK EGP-64 === #
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)

    # MoveIt!2 Controllers:
    if (EE_no == "true"):
        moveit_simple_controllers_yaml = load_yaml("irb120_ros2_moveit2", "config/irb120_controllers.yaml")
    # === SCHUNK EGP-64 === #
    elif (EE_schunk == "true"):
        moveit_simple_controllers_yaml = load_yaml("irb120_ros2_moveit2", "config/irb120egp64_controllers.yaml")
    # === SCHUNK EGP-64 === #
    
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_simple_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }
    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }
    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    # START NODE -> MOVE GROUP:
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True}, 
        ],
    )

    # RVIZ:
    load_RVIZfile = LaunchConfiguration("rviz_file")
    rviz_base = os.path.join(get_package_share_directory("irb120_ros2_moveit2"), "config")
    
    if (EE_no == "true"):
        rviz_full_config = os.path.join(rviz_base, "irb120_moveit2.rviz")
    # === SCHUNK EGP-64 === #
    elif (EE_schunk == "true"):
        rviz_full_config = os.path.join(rviz_base, "irb120egp64_moveit2.rviz")
    # === SCHUNK EGP-64 === #

    rviz_node_full = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_full_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            kinematics_yaml,
            {"use_sim_time": True},
        ],
        condition=UnlessCondition(load_RVIZfile),
    )

    # *********************** ROS2.0 Robot/End-Effector Actions/Triggers *********************** #
    # MoveJ ACTION:
    moveJ_interface = Node(
        name="moveJ_action",
        package="ros2_actions",
        executable="moveJ_action",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": 'irb120_arm'}],
    )
    # MoveG ACTION:
    moveG_interface = Node(
        name="moveG_action",
        package="ros2_actions",
        executable="moveG_action",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": 'egp64'}],
    )
    # MoveXYZW ACTION:
    moveXYZW_interface = Node(
        name="moveXYZW_action",
        package="ros2_actions",
        executable="moveXYZW_action",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": 'irb120_arm'}],
    )
    # MoveL ACTION:
    moveL_interface = Node(
        name="moveL_action",
        package="ros2_actions",
        executable="moveL_action",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": 'irb120_arm'}],
    )
    # MoveR ACTION:
    moveR_interface = Node(
        name="moveR_action",
        package="ros2_actions",
        executable="moveR_action",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": 'irb120_arm'}],
    )
    # MoveXYZ ACTION:
    moveXYZ_interface = Node(
        name="moveXYZ_action",
        package="ros2_actions",
        executable="moveXYZ_action",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": 'irb120_arm'}],
    )
    # MoveYPR ACTION:
    moveYPR_interface = Node(
        name="moveYPR_action",
        package="ros2_actions",
        executable="moveYPR_action",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": 'irb120_arm'}],
    )
    # MoveROT ACTION:
    moveROT_interface = Node(
        name="moveROT_action",
        package="ros2_actions",
        executable="moveROT_action",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": 'irb120_arm'}],
    )
    # MoveRP ACTION:
    moveRP_interface = Node(
        name="moveRP_action",
        package="ros2_actions",
        executable="moveRP_action",
        output="screen",
        parameters=[robot_description, robot_description_semantic, kinematics_yaml, {"use_sim_time": True}, {"ROB_PARAM": 'irb120_arm'}],
    )

    # ATTACHER action for ros2_grasping plugin:
    Attacher = Node(
        name="ATTACHER_action",
        package="ros2_grasping",
        executable="attacher_action.py",
        output="screen",
    )

    return LaunchDescription(
        [
            # Gazebo nodes:
            gazebo, 
            spawn_entity,
            # ROS2_CONTROL:
            static_tf,
            robot_state_publisher,
            
            # ROS2 Controllers:
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
            RegisterEventHandler(
                OnProcessExit(
                    target_action = joint_trajectory_controller_spawner,
                    on_exit = [
                        egp64left_controller_spawner,
                    ]
                )
            ),
            RegisterEventHandler(
                OnProcessExit(
                    target_action = egp64left_controller_spawner,
                    on_exit = [
                        egp64right_controller_spawner,
                    ]
                )
            ),

            RegisterEventHandler(
                OnProcessExit(
                    target_action = egp64right_controller_spawner,
                    on_exit = [

                        # MoveIt!2:
                        TimerAction(
                            period=5.0,
                            actions=[
                                rviz_arg,
                                rviz_node_full,
                                run_move_group_node
                            ]
                        ),

                        # ROS2.0 Actions:
                        TimerAction(
                            period=2.0,
                            actions=[
                                moveJ_interface,
                                moveG_interface,
                                moveL_interface,
                                moveR_interface,
                                moveXYZ_interface,
                                moveXYZW_interface,
                                moveYPR_interface,
                                moveROT_interface,
                                moveRP_interface,
                                Attacher,
                            ]
                        ),

                    ]
                )
            )
        ]
    )