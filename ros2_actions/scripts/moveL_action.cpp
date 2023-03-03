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

// ***** MoveL ACTION SERVER ***** //

#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ros2_data/action/move_l.hpp"

// Declaration of global constants:
const double pi = 3.14159265358979;
const double k = pi/180.0;

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

// Declaration of GLOBAL VARIABLE: MoveIt!2 Interface -> move_group_interface:
moveit::planning_interface::MoveGroupInterface move_group_interface;

class ActionServer : public rclcpp::Node
{
public:
    using MoveL = ros2_data::action::MoveL;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveL>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("MoveL_ActionServer", options)
    {

        action_server_ = rclcpp_action::create_server<MoveL>(
            this,
            "/MoveL",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

    }

private:
    rclcpp_action::Server<MoveL>::SharedPtr action_server_;
    
    // Function that checks the goal received, and accepts it accordingly:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveL::Goal> goal)
    {
        double moveX =  goal->movex;
        double moveY =  goal->movey;
        double moveZ =  goal->movez;
        RCLCPP_INFO(get_logger(), "Received a LINEAR GOAL request, with XYZ VECTOR -> (x = %.2f, y = %.2f, z = %.2f)", moveX, moveY, moveZ);
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
        RCLCPP_INFO(this->get_logger(), "Starting MoveL motion...");
        
        // Obtain input value (goal -- GoalPOSE):
        const auto goal = goal_handle->get_goal();
        double moveX =  goal->movex;
        double moveY =  goal->movey;
        double moveZ =  goal->movez;

        // Obtain JOINT SPEED and apply it into MoveIt!2:
        auto SPEED = goal->speed;
        move_group_interface.setMaxVelocityScalingFactor(SPEED);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);
        
        // FEEDBACK?
        // No feedback needed for MoveL Action Calls.
        // No loop needed for MoveL Action Calls.
        
        // Declare RESULT:
        auto result = std::make_shared<MoveL::Result>();
        
        // Joint model group:
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(my_param);

        // Get CURRENT POSE:
        auto current_pose = move_group_interface.getCurrentPose();
        
        // Initialise LINEAR TRAJECTORY -> Waypoints VECTOR:
        std::vector<geometry_msgs::msg::Pose> waypoints;
        
        // Add INITIAL POINT to waypoints vector:
        // waypoints.push_back(current_pose.pose); -> No needed, it breaks the execution!

        // Define GOAL POSE by adding x, y and z:
        geometry_msgs::msg::Pose target_pose = current_pose.pose;
        target_pose.position.x = target_pose.position.x + moveX;
        target_pose.position.y = target_pose.position.y + moveY;
        target_pose.position.z = target_pose.position.z + moveZ;

        // *************************************************** //
        // === Check if -> GOAL POSE is within ROBOT REACH === //

        // When generating and executing cartesian paths, MoveIt!2 does not check whether the end of the generated LINEAR TRAJECTORY
        // is within the ROBOT REACH or not. It starts executing the trajectory, and if the robot cannot continue, it finishes and it 
        // returns the percentage of the trajectory which was successfully followed. 

        // Thus, this must be checked beforehand for safety/quality reasons. There are 2 different ways of solving this:
        //  - The optimal way is solving IK for the specific GOAL POSE, but this is time consuming and hard to code.
        //  - A simple way of checking is by simply planning a POSE GOAL with the standard MoveIt!2 planner, as if a MoveXYZW motion was 
        //    going to be done to the GOAL POSE. If a robot is able to perform a MoveL to a specific pose, it should be able to perform
        //    a MoveXYZW as well (MoveL is more complex than MoveXYZW). This solution not only checks if the GOAL POSE is within the reach 
        //    of the robot or not, but it should check potential singularities as well.

        // POSE-goal planning:
        move_group_interface.setPoseTarget(target_pose);

        // Plan, execute and inform (with feedback):
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        bool success1 = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success1) {

            // ************************************************** //
            // ============= EXECUTE CARTESIAN PATH ============= //
            
            // Add FINAL POINT to waypoints vector:
            waypoints.push_back(target_pose);

            // Generate TRAJECTORY (LINEAR):
            moveit_msgs::msg::RobotTrajectory trajectory;
            const double jump_threshold = 0.0;
            const double eef_step = 0.001;
            double fraction = move_group_interface.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
            
            bool success2 = (move_group_interface.execute(trajectory) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

            // Do if GOAL CANCELLED:
            if (goal_handle->is_canceling()) {

                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                result->result = "MoveL:CANCELED";
                goal_handle->canceled(result);
                return;

            } else {
                
                if(success2) {
                    RCLCPP_INFO(this->get_logger(), "%s - MoveL: Execution successful!", my_param.c_str());
                    result->result = "MoveL:SUCCESS";
                    goal_handle->succeed(result);
                } else {
                    RCLCPP_INFO(this->get_logger(), "%s - MoveL: Execution failed!", my_param.c_str());
                    result->result = "MoveL:EXECUTION-FAILED";
                    goal_handle->succeed(result);
                } 
                
            }

        } else {

            RCLCPP_INFO(this->get_logger(), "%s - MoveL: Planning failed!", my_param.c_str());
            result->result = "MoveL:PLANNING-FAILED";
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
  auto name = "_MoveL_interface";
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