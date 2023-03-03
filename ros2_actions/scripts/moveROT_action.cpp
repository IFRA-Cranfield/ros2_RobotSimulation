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

// ***** MoveROT ACTION SERVER ***** //

#include <functional>
#include <memory>
#include <thread>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ros2_data/action/move_rot.hpp"

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
    using MoveROT = ros2_data::action::MoveROT;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveROT>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("MoveROT_ActionServer", options)
    {

        action_server_ = rclcpp_action::create_server<MoveROT>(
            this,
            "/MoveROT",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

    }

private:
    rclcpp_action::Server<MoveROT>::SharedPtr action_server_;
    
    // Function that checks the goal received, and accepts it accordingly:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveROT::Goal> goal)
    {
        double roll =  goal->roll;
        double pitch =  goal->pitch;
        double yaw =  goal->yaw;
        RCLCPP_INFO(get_logger(), "Received a PoseGoal/Orientation (ROT) request:");
        RCLCPP_INFO(this->get_logger(), "Relative ORIENTATION (Euler) -> (yaw = %.2f, pitch = %.2f, roll = %.2f)", yaw, pitch, roll);
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
        RCLCPP_INFO(this->get_logger(), "Starting MoveROT motion to desired orientation...");
        
        // Obtain input value (goal -- GoalPOSE):
        const auto goal = goal_handle->get_goal();
        double roll =  goal->roll;
        double pitch =  goal->pitch;
        double yaw =  goal->yaw;
        
        // Obtain JOINT SPEED and apply it into MoveIt!2:
        auto SPEED = goal->speed;
        move_group_interface.setMaxVelocityScalingFactor(SPEED);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);
        
        // FEEDBACK?
        // No feedback needed for MoveROT Action Calls.
        // No loop needed for MoveROT Action Calls.
        
        // Declare RESULT:
        auto result = std::make_shared<MoveROT::Result>();
        
        // Joint model group:
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(my_param);

        // Get CURRENT POSE:
        auto current_pose = move_group_interface.getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "Current POSE before the new MoveROT was:");
        RCLCPP_INFO(this->get_logger(), "POSITION -> (x = %.2f, y = %.2f, z = %.2f)", current_pose.pose.position.x, current_pose.pose.position.y,current_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "ORIENTATION (quaternion) -> (x = %.2f, y = %.2f, z = %.2f, w = %.2f)", current_pose.pose.orientation.x, current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w);

        // Get INITIAL ROTATION -> Quaternion:
        double Ax = current_pose.pose.orientation.x;
        double Ay = current_pose.pose.orientation.y;
        double Az = current_pose.pose.orientation.z;
        double Aw = current_pose.pose.orientation.w;

        // Get desired RELATIVE ROTATION -> Given into MoveROT by Euler Angles:
        double cy = cos(k*yaw * 0.5);
        double sy = sin(k*yaw * 0.5);
        double cp = cos(k*pitch * 0.5);
        double sp = sin(k*pitch * 0.5);
        double cr = cos(k*roll * 0.5);
        double sr = sin(k*roll * 0.5);
        double Bx = sr * cp * cy - cr * sp * sy;
        double By = cr * sp * cy + sr * cp * sy;
        double Bz = cr * cp * sy - sr * sp * cy;
        double Bw = cr * cp * cy + sr * sp * sy;

        // QUATERNION MULTIPLICATION:
        double w = Aw*Bw - Ax*Bx - Ay*By - Az*Bz;
        double x = Aw*Bx + Ax*Bw + Ay*Bz - Az*By;
        double y = Aw*By - Ax*Bz + Ay*Bw + Az*Bx;
        double z = Aw*Bz + Ax*By - Ay*Bx + Az*Bw; 

        // POSE-goal planning:
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = current_pose.pose.position.x;
        target_pose.position.y = current_pose.pose.position.y;
        target_pose.position.z = current_pose.pose.position.z;
        target_pose.orientation.x = x;
        target_pose.orientation.y = y;
        target_pose.orientation.z = z;
        target_pose.orientation.w = w;
        move_group_interface.setPoseTarget(target_pose);

        // Plan, execute and inform (with feedback):
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success) {

            RCLCPP_INFO(this->get_logger(), "%s - MoveROT: Planning successful!", my_param.c_str());
            move_group_interface.move();
            
            // Do if GOAL CANCELLED:
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                result->result = "MoveROT:CANCELED";
                goal_handle->canceled(result);
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "%s - MoveROT: Movement executed!", my_param.c_str());
                result->result = "MoveROT:SUCCESS";
                goal_handle->succeed(result);
            }

        } else {
            RCLCPP_INFO(this->get_logger(), "%s - MoveROT: Planning failed!", my_param.c_str());
            result->result = "MoveROT:FAILED";
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
  auto name = "_MoveROT_interface";
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