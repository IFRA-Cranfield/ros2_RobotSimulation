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

// MoveJ ACTION SERVER for ABB IRB-120 Robot.

#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ros2_data/action/move_j.hpp"
#include "ros2_data/msg/joint_pose.hpp"

// Declaration of global constants:
const double pi = 3.14159265358979;
const double k = pi/180.0;

// Declaration of GLOBAL VARIABLE: MoveIt!2 Interface -> move_group_interface:
moveit::planning_interface::MoveGroupInterface move_group_interface;

class ActionServer : public rclcpp::Node
{
public:
    using MoveJ = ros2_data::action::MoveJ;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveJ>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("MoveJ_ActionServer", options)
    {

        action_server_ = rclcpp_action::create_server<MoveJ>(
            this,
            "/MoveJ",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

    }

private:
    rclcpp_action::Server<MoveJ>::SharedPtr action_server_;
    
    // Function that checks the goal received, and accepts it accordingly:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveJ::Goal> goal)
    {
        auto JointGoal = ros2_data::msg::JointPose();
        JointGoal =  goal->goal;
        RCLCPP_INFO(get_logger(), "Received a goal request, with JointPose -> (%.2f,%.2f,%.2f,%.2f,%.2f,%.2f)",JointGoal.joint1,JointGoal.joint2,JointGoal.joint3,JointGoal.joint4,JointGoal.joint5,JointGoal.joint6);
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
        RCLCPP_INFO(this->get_logger(), "Starting MoveJ motion to desired waypoint...");
        
        rclcpp::Rate loop_rate(0.5);
        
        // Obtain input value (goal -- JointPose):
        const auto goal = goal_handle->get_goal();
        auto JointGoal = goal->goal;
        
        // FEEDBACK?
        // No feedback needed for MoveJ Action Calls.
        // No loop needed for MoveJ Action Calls.
        
        // Declare RESULT:
        auto result = std::make_shared<MoveJ::Result>();
    
        // Declare joint value variables:
        double j1 = JointGoal.joint1;
        double j2 = JointGoal.joint2;
        double j3 = JointGoal.joint3;
        double j4 = JointGoal.joint4;
        double j5 = JointGoal.joint5;
        double j6 = JointGoal.joint6;
        
        // Check if INPUT JOINT VALUES are within the JOINT LIMIT VALUES:
        bool LimitCheck = false;
        
        double j1upper = 165;
        double j1lower = -165;
        if (j1 <= j1upper && j1 >= j1lower && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
        double j2upper = 110;
        double j2lower = -110;
        if (j2 <= j2upper && j2 >= j2lower && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
        double j3upper = 70;
        double j3lower = -110;
        if (j3 <= j3upper && j3 >= j3lower && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
        double j4upper = 160;
        double j4lower = -160;
        if (j4 <= j4upper && j4 >= j4lower && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
        double j5upper = 120;
        double j5lower = -120;
        if (j5 <= j5upper && j5 >= j5lower && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }
        double j6upper = 400;
        double j6lower = -400;
        if (j6 <= j6upper && j6 >= j6lower && LimitCheck == false) {
            // Do nothing, check complete.
        } else {
            LimitCheck = true;
        }

        // EXECUTE IF -> JointValues are within the limits:
        if (LimitCheck == false){
            
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

            // Plan, execute and inform (with feedback):
            moveit::planning_interface::MoveGroupInterface::Plan my_plan;
            
            bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            if(success) {

                RCLCPP_INFO(this->get_logger(), "ABB IRB120 Robot - MoveJ: Planning successful!");
                move_group_interface.move();
                
                // Do if GOAL CANCELLED:
                if (goal_handle->is_canceling()) {
                    RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                    result->result = "MoveJ:CANCELED";
                    goal_handle->canceled(result);
                    return;
                } else {
                    RCLCPP_INFO(this->get_logger(), "ABB IRB120 Robot - MoveJ: Movement executed!");
                    result->result = "MoveJ:SUCCESS";
                    goal_handle->succeed(result);
                }

            } else {
                RCLCPP_INFO(this->get_logger(), "ABB IRB120 Robot - MoveJ: Planning failed!");
                result->result = "MoveJ:FAILED";
                goal_handle->succeed(result);
            }    

        } else {

            RCLCPP_INFO(this->get_logger(), "ABB IRB120 Robot - MoveJ: Planning failed, JOINT LIMITS exceeded!");
            result->result = "MoveJ:FAILED";
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

  // Set max. VELOCITY and ACELLERATION scaling values to unit:
  move_group_interface.setMaxVelocityScalingFactor(0.7);
  move_group_interface.setMaxAccelerationScalingFactor(0.7);
  
  // Declare and spin ACTION SERVER:
  auto action_server = std::make_shared<ActionServer>();
  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}