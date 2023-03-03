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

// ***** MoveRP ACTION SERVER ***** //

#include <functional>
#include <memory>
#include <thread>
#include <string>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/move_group_interface/move_group_interface_improved.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include "ros2_data/action/move_rp.hpp"

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
    using MoveRP = ros2_data::action::MoveRP;
    using GoalHandle = rclcpp_action::ServerGoalHandle<MoveRP>;

    explicit ActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("MoveRP_ActionServer", options)
    {

        action_server_ = rclcpp_action::create_server<MoveRP>(
            this,
            "/MoveRP",
            std::bind(&ActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ActionServer::handle_accepted, this, std::placeholders::_1));

    }

private:
    rclcpp_action::Server<MoveRP>::SharedPtr action_server_;
    
    // Function that checks the goal received, and accepts it accordingly:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveRP::Goal> goal)
    {
        double roll =  goal->roll;
        double pitch =  goal->pitch;
        double yaw =  goal->yaw;
        double x =  goal->x;
        double y =  goal->y;
        double z =  goal->z;
        RCLCPP_INFO(get_logger(), "Received a {ROTATION AROUND A POINT - in the end-effector frame-} request:");
        RCLCPP_INFO(this->get_logger(), "Relative ORIENTATION (Euler) -> (yaw = %.2f, pitch = %.2f, roll = %.2f)", yaw, pitch, roll);
        RCLCPP_INFO(this->get_logger(), "Rotation origin (x,y,z): Point relative to end-effector origin -> (x = %.2f, y = %.2f, z = %.2f)", x, y, z);
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
        RCLCPP_INFO(this->get_logger(), "Starting MoveRP motion to desired orientation...");
        
        // Obtain input value (goal -- GoalPOSE):
        const auto goal = goal_handle->get_goal();
        double roll =  goal->roll;
        double pitch =  goal->pitch;
        double yaw =  goal->yaw;
        double x =  goal->x;
        double y =  goal->y;
        double z =  goal->z;
        
        // Obtain JOINT SPEED and apply it into MoveIt!2:
        auto SPEED = goal->speed;
        move_group_interface.setMaxVelocityScalingFactor(SPEED);
        move_group_interface.setMaxAccelerationScalingFactor(1.0);
        
        // FEEDBACK?
        // No feedback needed for MoveRP Action Calls.
        // No loop needed for MoveRP Action Calls.
        
        // Declare RESULT:
        auto result = std::make_shared<MoveRP::Result>();
        
        // Joint model group:
        const moveit::core::JointModelGroup* joint_model_group = move_group_interface.getCurrentState()->getJointModelGroup(my_param);

        // Get CURRENT POSE:
        auto current_pose = move_group_interface.getCurrentPose();
        RCLCPP_INFO(this->get_logger(), "Current POSE before the new MoveRP was:");
        RCLCPP_INFO(this->get_logger(), "POSITION -> (x = %.2f, y = %.2f, z = %.2f)", current_pose.pose.position.x, current_pose.pose.position.y,current_pose.pose.position.z);
        RCLCPP_INFO(this->get_logger(), "ORIENTATION (quaternion) -> (x = %.2f, y = %.2f, z = %.2f, w = %.2f)", current_pose.pose.orientation.x, current_pose.pose.orientation.y,current_pose.pose.orientation.z,current_pose.pose.orientation.w);

        
        // ********** ROTATION ********** //

        // Get INITIAL ROTATION -> Quaternion:
        double Ax = current_pose.pose.orientation.x;
        double Ay = current_pose.pose.orientation.y;
        double Az = current_pose.pose.orientation.z;
        double Aw = current_pose.pose.orientation.w;
        // Get desired RELATIVE ROTATION -> Given into MoveRP by Euler Angles:
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
        double ROTw = Aw*Bw - Ax*Bx - Ay*By - Az*Bz;
        double ROTx = Aw*Bx + Ax*Bw + Ay*Bz - Az*By;
        double ROTy = Aw*By - Ax*Bz + Ay*Bw + Az*Bx;
        double ROTz = Aw*Bz + Ax*By - Ay*Bx + Az*Bw; 


        // ********** TRANSLATION ********** //
        
        // 1. END-EFFECTOR POSE:
        // Rotation quaternion from MoveIt!2:
        // Ax, Ay, Az and Aw.
        // Position from MoveIt!2:
        double Ex = current_pose.pose.position.x;
        double Ey = current_pose.pose.position.y;
        double Ez = current_pose.pose.position.z;
        // 2. Normalise quaternion:
        double norm = sqrt((Ax*Ax)+(Ay*Ay)+(Az*Az)+(Aw*Aw));
        double Qx = Ax/norm;
        double Qy = Ay/norm;
        double Qz = Az/norm;
        double Qw = Aw/norm;
        // 3. Obtain ROTATION MATRIX:
        double R_00 = 1 - 2*(Qy*Qy) - 2*(Qz*Qz);
        double R_01 = 2*(Qx*Qy) - 2*(Qw*Qz);
        double R_02 = 2*(Qx*Qz) + 2*(Qw*Qy);
        double R_10 = 2*(Qx*Qy) + 2*(Qw*Qz);
        double R_11 = 1 - 2*(Qx*Qx) - 2*(Qz*Qz);
        double R_12 = 2*(Qy*Qz) - 2*(Qw*Qx);
        double R_20 = 2*(Qx*Qz) - 2*(Qw*Qy);
        double R_21 = 2*(Qy*Qz) + 2*(Qw*Qx);
        double R_22 = 1 - 2*(Qx*Qx) - 2*(Qy*Qy);
        // 4. From (x,y,z) to (xR,yR,zR) -- ROTATION AROUND A POINT:
        // ROTATION for EULER -> 1.ROLL + 2.PITCH + 3.YAW:
        double rEUL00 = cos(k*yaw)*cos(k*pitch);
        double rEUL01 = cos(k*yaw)*sin(k*pitch)*sin(k*roll) - sin(k*yaw)*cos(k*roll);
        double rEUL02 = cos(k*yaw)*sin(k*pitch)*cos(k*roll) + sin(k*yaw)*sin(k*roll);
        double rEUL10 = sin(k*yaw)*cos(k*pitch);
        double rEUL11 = sin(k*yaw)*sin(k*pitch)*sin(k*roll) + cos(k*yaw)*cos(k*roll);
        double rEUL12 = sin(k*yaw)*sin(k*pitch)*cos(k*roll) - cos(k*yaw)*sin(k*roll);
        double rEUL20 = -sin(k*pitch);
        double rEUL21 = cos(k*pitch)*sin(k*roll);
        double rEUL22 = cos(k*pitch)*cos(k*roll);
        // Displaced End-Effector point -> (xR,yR,zR):
        double xR = x - rEUL00*x - rEUL01*y - rEUL02*z;
        double yR = y - rEUL10*x - rEUL11*y - rEUL12*z;
        double zR = z - rEUL20*x - rEUL21*y - rEUL22*z;
        // 5. From (xR,yR,zR) to (Px,Py,Pz) -- TRANSFORMATION FROM LOCAL (end-effector) to GLOBAL:
        double Px = Ex + R_00*xR + R_01*yR + R_02*zR;
        double Py = Ey + R_10*xR + R_11*yR + R_12*zR;
        double Pz = Ez + R_20*xR + R_21*yR + R_22*zR;

        // ********** MoveIt!2 PLANNING ********** //
        // POSE-goal planning:
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = Px;
        target_pose.position.y = Py;
        target_pose.position.z = Pz;
        target_pose.orientation.x = ROTx;
        target_pose.orientation.y = ROTy;
        target_pose.orientation.z = ROTz;
        target_pose.orientation.w = ROTw;
        move_group_interface.setPoseTarget(target_pose);

        // Plan, execute and inform (with feedback):
        moveit::planning_interface::MoveGroupInterface::Plan my_plan;
        
        bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

        if(success) {

            RCLCPP_INFO(this->get_logger(), "%s - MoveRP: Planning successful!", my_param.c_str());
            move_group_interface.move();
            
            // Do if GOAL CANCELLED:
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                result->result = "MoveRP:CANCELED";
                goal_handle->canceled(result);
                return;
            } else {
                RCLCPP_INFO(this->get_logger(), "%s - MoveRP: Movement executed!", my_param.c_str());
                result->result = "MoveRP:SUCCESS";
                goal_handle->succeed(result);
            }

        } else {
            RCLCPP_INFO(this->get_logger(), "%s - MoveRP: Planning failed!", my_param.c_str());
            result->result = "MoveRP:FAILED";
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
  auto name = "_MoveRP_interface";
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