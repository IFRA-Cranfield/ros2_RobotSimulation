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
#  Date: November, 2022.                                                                #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

*/

// ***** MoveR (6-DOF) ACTION SERVER ***** //

#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ros2_data/action/move_r.hpp"
#include "ros2_data/msg/joint_pose.hpp"

// Declaration of global constants:
const double pi = 3.14159265358979;
const double k = pi/180.0;

// Declaration of GLOBAL VARIABLE: MoveIt!2 Interface -> move_group_interface:
moveit::planning_interface::MoveGroupInterface move_group_interface;

// Declaration of GLOBAL VARIABLE --> ROBOT / END-EFFECTOR PARAMETER:
std::string my_param = "none";

class ros2_RobotTrigger : public rclcpp::Node
{
public:
    ros2_RobotTrigger() : Node("ros2_RobotTrigger_PARAM") 
    {
        this->declare_parameter("ROB_PARAM", "null");
        my_param = this->get_parameter("ROB_PARAM").get_parameter_value().get<std::string>();
        RCLCPP_INFO(this->get_logger(), "ROB_PARAM received -> %s", my_param.c_str());
    }
private:
};

class ActionServer : public rclcpp::Node
{
public:
    using MoveR = ros2_data::action::MoveR;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveR>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("MoveR_ActionServer", options)
    {

        action_server_ = rclcpp_action::create_server<MoveR>(
            this,
            "/MoveR",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

    }

private:
    rclcpp_action::Server<MoveR>::SharedPtr action_server_;
    
    // Function that checks the goal received, and accepts it accordingly:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveR::Goal> goal)
    {
        auto joint = goal->joint;
        double value = goal->value;
        RCLCPP_INFO(get_logger(), "Received a moveR request, with Joint+Value -> (%s -> %.2f)", joint.c_str(), value);
        //(void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE; // Accept and execute the goal received.
    }

    // No idea about what this function does:
    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        // This needs to return quickly to avoid blocking the executor, so spin up a new thread:
        std::thread(
            [this, goal_handle]() {
                execute(goal_handle);
            }).detach();
        
    }

    // Function that cancels the goal request:
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received a cancel request.");

        // We call the -> void moveit::planning_interface::MoveGroupInterface::stop(void) method,
        // which stops any trajectory execution, if one is active.
        move_group_interface.stop();

        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // MAIN LOOP OF THE ACTION SERVER -> EXECUTION:
    void execute(const std::shared_ptr<GoalHandle> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Starting MoveR motion...");
        
        rclcpp::Rate loop_rate(0.5);
        
        // Obtain input value (goal -- Joint+Value):
        const auto goal = goal_handle->get_goal();
        auto joint = goal->joint;
        double value = goal->value;
        
        // Initialise empty jx variables:
        double j1, j2, j3, j4, j5, j6;
        j1 = j2 = j3 = j4 = j5 = j6 = 0.0;
        
        // Obtain JOINT SPEED and apply it into MoveIt!2:
        auto SPEED = goal->speed;
        move_group_interface.setMaxVelocityScalingFactor(SPEED);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);
        
        // FEEDBACK?
        // No feedback needed for MoveR Action Calls.
        // No loop needed for MoveR Action Calls.

        // Declare RESULT:
        auto result = std::make_shared<MoveR::Result>();

        // Joint model group:
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(my_param);

        // Obtain current JOINT VALUES:
        auto current_JointValues = move_group_interface.getCurrentJointValues();
        j1 = current_JointValues[0] * (1/k);
        j2 = current_JointValues[1] * (1/k);
        j3 = current_JointValues[2] * (1/k); 
        j4 = current_JointValues[3] * (1/k);
        j5 = current_JointValues[4] * (1/k);
        j6 = current_JointValues[5] * (1/k);

        RCLCPP_INFO(this->get_logger(), "Current JOINT VALUES before MoveR are -> (j1 = %.2f, j2 = %.2f, j3 = %.2f, j4 = %.2f, j5 = %.2f, j6 = %.2f)", j1, j2, j3, j4, j5, j6);
        
        double j1UL, j1LL, j2UL, j2LL, j3UL, j3LL, j4UL, j4LL, j5UL, j5LL, j6UL, j6LL = 0.0;

        // ***** JOINT VALUES (MAX/MIN) ***** //
        if (my_param == "irb120_arm"){
            j1UL = 165;
            j1LL = -165;
            j2UL = 110;
            j2LL = -110;
            j3UL = 70;
            j3LL = -110;
            j4UL = 160;
            j4LL = -160;
            j5UL = 120;
            j5LL = -120;
            j6UL = 400;
            j6LL = -400;
        } else if (my_param == "irb1200_arm"){
            j1UL = 170;
            j1LL = -170;
            j2UL = 130;
            j2LL = -100;
            j3UL = 70;
            j3LL = -200;
            j4UL = 270;
            j4LL = -270;
            j5UL = 130;
            j5LL = -130;
            j6UL = 360;
            j6LL = -360;
        } else if (my_param == "irb6640_arm"){
            j1UL = 170;
            j1LL = -170;
            j2UL = 85;
            j2LL = -65;
            j3UL = 70;
            j3LL = -180;
            j4UL = 300;
            j4LL = -300;
            j5UL = 120;
            j5LL = -120;
            j6UL = 360;
            j6LL = -360;
        } else if (my_param == "cr35ia_arm"){
            j1UL = 170;
            j1LL = -170;
            j2UL = 120;
            j2LL = 45;
            j3UL = 135;
            j3LL = -122;
            j4UL = 200;
            j4LL = -200;
            j5UL = 110;
            j5LL = -110;
            j6UL = 450;
            j6LL = -450;
        } else if (my_param == "ur3_arm"){
            j1UL = 360;
            j1LL = -360;
            j2UL = 360;
            j2LL = -360;
            j3UL = 180;
            j3LL = -180;
            j4UL = 360;
            j4LL = -360;
            j5UL = 360;
            j5LL = -360;
            j6UL = 360;
            j6LL = -360;
        } else if (my_param == "ur5_arm"){
            j1UL = 360;
            j1LL = -360;
            j2UL = 360;
            j2LL = -360;
            j3UL = 180;
            j3LL = -180;
            j4UL = 360;
            j4LL = -360;
            j5UL = 360;
            j5LL = -360;
            j6UL = 360;
            j6LL = -360;
        } else if (my_param == "ur10_arm"){
            j1UL = 360;
            j1LL = -360;
            j2UL = 360;
            j2LL = -360;
            j3UL = 180;
            j3LL = -180;
            j4UL = 360;
            j4LL = -360;
            j5UL = 360;
            j5LL = -360;
            j6UL = 360;
            j6LL = -360;
        };
        
        // Check if INPUT JOINT VALUES are within the JOINT LIMIT VALUES:
        bool LimitCheck = false;
        auto InputJoint = "Valid";

        if (joint == "joint1"){
            j1 = j1 + value;
            if (j1 <= j1UL && j1 >= j1LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint2"){
            j2 = j2 + value;
            if (j2 <= j2UL && j2 >= j2LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint3"){
            j3 = j3 + value;
            if (j3 <= j3UL && j3 >= j3LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint4"){
            j4 = j4 + value;
            if (j4 <= j4UL && j4 >= j4LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint5"){
            j5 = j5 + value;
            if (j5 <= j5UL && j5 >= j5LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint6"){
            j6 = j6 + value;
            if (j6 <= j6UL && j6 >= j6LL && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else {
            LimitCheck = true;
            InputJoint = "NotValid";
        }
            
        // EXECUTE IF -> JointValues are within the limits:
        if (LimitCheck == false){
            
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

            // Plan, execute and inform (with feedback):
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            
            bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if(success) {

                RCLCPP_INFO(this->get_logger(), "%s - MoveR: Planning successful!", my_param.c_str());
                move_group_interface.move();
                
                // Do if GOAL CANCELLED:
                if (goal_handle->is_canceling()) {
                    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                    result->result = "MoveR:CANCELED";
                    goal_handle->canceled(result);
                    return;
                } else {
                    RCLCPP_INFO(this->get_logger(), "%s - MoveR: Movement executed!", my_param.c_str());
                    result->result = "MoveR:SUCCESS";
                    goal_handle->succeed(result);
                }

            } else {
                RCLCPP_INFO(this->get_logger(), "%s - MoveR: Planning failed!", my_param.c_str());
                result->result = "MoveR:FAILED";
                goal_handle->succeed(result);
            }    

        } else if (LimitCheck == true && InputJoint == "Valid") {

            RCLCPP_INFO(this->get_logger(), "%s - MoveR: Planning failed, JOINT LIMITS exceeded!", my_param.c_str());
            result->result = "MoveR:FAILED";
            goal_handle->succeed(result);

        } else {

            RCLCPP_INFO(this->get_logger(), "%s - MoveR: Planning failed, JOINT NAME NOT VALID!", my_param.c_str());
            result->result = "MoveR:FAILED";
            goal_handle->succeed(result);

        }

    }

};

int main(int argc, char ** argv)
{
  // Initialise MAIN NODE:
  rclcpp::init(argc, argv);

  // Obtain ros2_RobotTrigger parameter:
  auto node_PARAM = std::make_shared<ros2_RobotTrigger>();
  rclcpp::spin_some(node_PARAM);

  // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
  auto name = "_MoveR_interface";
  auto node2name = my_param + name;
  auto const node2 = std::make_shared<rclcpp::Node>(
      node2name, rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor; 
  executor.add_node(node2);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create the Move Group Interface:
  using moveit::planning_interface::MoveGroupInterface;
  move_group_interface = MoveGroupInterface(node2, my_param);
  // Create the MoveIt PlanningScene Interface:
  using moveit::planning_interface::PlanningSceneInterface;
  auto planning_scene_interface = PlanningSceneInterface();

  // Declare and spin ACTION SERVER:
  auto action_server = std::make_shared<ActionServer>();
  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}