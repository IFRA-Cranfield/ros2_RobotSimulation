/* 

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

*/

// MoveJ SERVICE SERVER for ABB IRB-120 Robot.

#include "rclcpp/rclcpp.hpp"
#include "ros2_data/srv/joint_pose.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <memory>
#include <iomanip>

#include <sstream>

using namespace std;

// Declaration of global constants:
const double pi = 3.14159265358979;
const double k = pi/180.0;

// Declaration of GLOBAL VARIABLE: MoveIt!2 Interface -> move_group_interface:
moveit::planning_interface::MoveGroupInterface move_group_interface;

// Declaration of GLOBAL VARIABLE -> ROS LOGGER:
auto const LOGGER = rclcpp::get_logger("irb120_LOG");

// ROS2.0 SERVICE CALLBACK -> MoveJ robot motion:
void MoveJ(const std::shared_ptr<ros2_data::srv::JointPose::Request> request,
                 std::shared_ptr<ros2_data::srv::JointPose::Response> response)
{

  // Declare joint value variables:
  double j1 = request -> joint1;
  double j2 = request -> joint2;
  double j3 = request -> joint3;
  double j4 = request -> joint4;
  double j5 = request -> joint5;
  double j6 = request -> joint6;
  
  // Log REQUEST:
  ostringstream oss1;
  oss1 << "Incoming request: MOVE ROBOT to [" << j1 << "°," << j2 << "°," << j3 << "°," << j4 << "°," << j5 << "°," << j6 << "°]";
  RCLCPP_INFO(LOGGER, oss1.str());

  // Joint model group:
  const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("irb120_arm");

  // JOINT-goal planning:
  moveit::core::RobotStatePtr current_state = move_group_interface.getCurrentState(10);
  std::vector<double> joint_group_positions;
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  joint_group_positions[0] = j1 * k;
  joint_group_positions[1] = j2 * k;
  joint_group_positions[2] = j3 * k;
  joint_group_positions[3] = j4 * k;
  joint_group_positions[4] = j5 * k;
  joint_group_positions[5] = j6 * k; 
  move_group_interface.setJointValueTarget(joint_group_positions);

  /* Without feedback (working):
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  move_group_interface.move();
  response -> result = "JointPose successfully sent to IRB120 Robot!";
  */

  // Plan, execute and inform (with feedback):
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  
  bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  //success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if(success) {
    RCLCPP_INFO(LOGGER, "ABB IRB-120 - MoveJ: Planning successful!");
    move_group_interface.move();
    RCLCPP_INFO(LOGGER, "ABB IRB-120 - MoveJ: Movement executed!");
    ostringstream oss2;
    oss2 << "ABB IRB-120 - MoveJ: [" << j1 << "°," << j2 << "°," << j3 << "°," << j4 << "°," << j5 << "°," << j6 << "°] --> SERVICE CALL SUCCESSFUL.";
    response -> result = oss2.str();
  } else {
    RCLCPP_INFO(LOGGER, "ABB IRB-120 - MoveJ: Planning failed!");
    response -> result = "ABB IRB-120 - MoveJ: PLANNING FAILED! Check JointPose for potential collisions or singularities.";
  }

}

int main(int argc, char **argv)
{
  // Some initial logging:
  RCLCPP_INFO(LOGGER, "IFRA Group - CRANFIELD UNIVERSITY");
  RCLCPP_INFO(LOGGER, "ROS2.0 Communications & Data Flow: ROS SERVICE SERVER that...");
  RCLCPP_INFO(LOGGER, "  --- Sends GOAL JOINT POSE to ABB IRB-120 Robot ---  ");

  //*** Launch NODE and SERVICE: ***//
  // Initialise ROS2.0 C++ client library:
  rclcpp::init(argc, argv);
  // Create node:
  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("irb120_MoveJ_server");

  // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
  auto const node2= std::make_shared<rclcpp::Node>(
      "irb120_moveit2_interface", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor; 
  executor.add_node(node2);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create the Move Group Interface:
  using moveit::planning_interface::MoveGroupInterface;
  move_group_interface = MoveGroupInterface(node2, "irb120_arm");
  // Create the MoveIt PlanningScene Interface:
  using moveit::planning_interface::PlanningSceneInterface;
  auto planning_scene_interface = PlanningSceneInterface();
  
  // Create and spin SERVICE:
  rclcpp::Service<ros2_data::srv::JointPose>::SharedPtr service = node->create_service<ros2_data::srv::JointPose>("irb120_MoveJ", &MoveJ);
  rclcpp::spin(node);

  // Shutdown:
  rclcpp::shutdown();
}