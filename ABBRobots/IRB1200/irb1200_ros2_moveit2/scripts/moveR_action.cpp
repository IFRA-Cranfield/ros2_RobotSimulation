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
#  Date: September, 2022.                                                                    #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2022) ROS2.0 ROBOT SIMULATION. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

*/

// MoveR ACTION SERVER for ABB IRB-1200 Robot.

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
        
        // FEEDBACK?
        // No feedback needed for MoveR Action Calls.
        // No loop needed for MoveR Action Calls.

        // Declare RESULT:
        auto result = std::make_shared<MoveR::Result>();

        // Joint model group:
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup("irb1200_arm");

        // Obtain current JOINT VALUES:
        auto current_JointValues = move_group_interface.getCurrentJointValues();
        j1 = current_JointValues[0] * (1/k);
        j2 = current_JointValues[1] * (1/k);
        j3 = current_JointValues[2] * (1/k); 
        j4 = current_JointValues[3] * (1/k);
        j5 = current_JointValues[4] * (1/k);
        j6 = current_JointValues[5] * (1/k);

        RCLCPP_INFO(this->get_logger(), "Current JOINT VALUES before MoveR are -> (j1 = %.2f, j2 = %.2f, j3 = %.2f, j4 = %.2f, j5 = %.2f, j6 = %.2f)", j1, j2, j3, j4, j5, j6);
        
        // Check if INPUT JOINT VALUES are within the JOINT LIMIT VALUES:
        bool LimitCheck = false;
        auto InputJoint = "Valid";

        if (joint == "joint1"){
            j1 = j1 + value;
            double j1upper = 170;
            double j1lower = -170;
            if (j1 <= j1upper && j1 >= j1lower && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint2"){
            j2 = j2 + value;
            double j2upper = 130;
            double j2lower = -100;
            if (j2 <= j2upper && j2 >= j2lower && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint3"){
            j3 = j3 + value;
            double j3upper = 70;
            double j3lower = -200;
            if (j3 <= j3upper && j3 >= j3lower && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint4"){
            j4 = j4 + value;
            double j4upper = 270;
            double j4lower = -270;
            if (j4 <= j4upper && j4 >= j4lower && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint5"){
            j5 = j5 + value;
            double j5upper = 130;
            double j5lower = -130;
            if (j5 <= j5upper && j5 >= j5lower && LimitCheck == false) {
                // Do nothing, check complete.
            } else {
                LimitCheck = true;
            }
        } else if (joint == "joint6"){
            j6 = j6 + value;
            double j6upper = 360;
            double j6lower = -360;
            if (j6 <= j6upper && j6 >= j6lower && LimitCheck == false) {
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

                RCLCPP_INFO(this->get_logger(), "ABB IRB1200 Robot - MoveR: Planning successful!");
                move_group_interface.move();
                
                // Do if GOAL CANCELLED:
                if (goal_handle->is_canceling()) {
                    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                    result->result = "MoveR:CANCELED";
                    goal_handle->canceled(result);
                    return;
                } else {
                    RCLCPP_INFO(this->get_logger(), "ABB IRB1200 Robot - MoveR: Movement executed!");
                    result->result = "MoveR:SUCCESS";
                    goal_handle->succeed(result);
                }

            } else {
                RCLCPP_INFO(this->get_logger(), "ABB IRB1200 Robot - MoveR: Planning failed!");
                result->result = "MoveR:FAILED";
                goal_handle->succeed(result);
            }    

        } else if (LimitCheck == true && InputJoint == "Valid") {

            RCLCPP_INFO(this->get_logger(), "ABB IRB1200 Robot - MoveR: Planning failed, JOINT LIMITS exceeded!");
            result->result = "MoveR:FAILED";
            goal_handle->succeed(result);

        } else {

            RCLCPP_INFO(this->get_logger(), "ABB IRB1200 Robot - MoveR: Planning failed, JOINT NAME NOT VALID!");
            result->result = "MoveR:FAILED";
            goal_handle->succeed(result);

        }

    }

};

int main(int argc, char ** argv)
{
  // Initialise MAIN NODE:
  rclcpp::init(argc, argv);

  // Launch and spin (EXECUTOR) MoveIt!2 Interface node:
  auto const node2= std::make_shared<rclcpp::Node>(
      "irb1200_moveit2_interface", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  rclcpp::executors::SingleThreadedExecutor executor; 
  executor.add_node(node2);
  std::thread([&executor]() { executor.spin(); }).detach();

  // Create the Move Group Interface:
  using moveit::planning_interface::MoveGroupInterface;
  move_group_interface = MoveGroupInterface(node2, "irb1200_arm");
  // Create the MoveIt PlanningScene Interface:
  using moveit::planning_interface::PlanningSceneInterface;
  auto planning_scene_interface = PlanningSceneInterface();

  // Set max. VELOCITY and ACELLERATION scaling values to unit:
  move_group_interface.setMaxVelocityScalingFactor(0.7);
  move_group_interface.setMaxAccelerationScalingFactor(0.7);
  
  // Declare and spin ACTION SERVER:
  auto action_server = std::make_shared<ActionServer>();
  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}