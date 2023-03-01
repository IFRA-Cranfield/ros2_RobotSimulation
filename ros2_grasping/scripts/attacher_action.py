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
#  Date: NFebruary, 2023.                                                               #
#                                                                                       #
# ===================================== COPYRIGHT ===================================== #

# ======= CITE OUR WORK ======= #
# You can cite our work with the following statement:
# IFRA (2023) ROS2.0 Grasp Plugin. URL: https://github.com/IFRA-Cranfield/ros2_RobotSimulation.

# ***** ATTACHER - ACTION SERVER ***** #

# Import libraries:
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from ros2_grasping.action import Attacher                                                              

from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.srv import GetEntityState
from gazebo_msgs.srv import SetLinkProperties

from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from std_msgs.msg import String
DETACH_MSG = ""

##################################################################

# ===== GAZEBO SERVICES ===== #

# Create NODE + SERVICE CLIENT for Gz Plugin (GET STATE) service:
class serviceClientGET(Node):
    def __init__(self):
        super().__init__('GraspGET_node')                                                           # Node NAME.
        self.cli = self.create_client(GetEntityState, "ros2_grasp/get_entity_state")                # Declare SERVICE CLIENT.
        while not self.cli.wait_for_service(timeout_sec=1.0):                                       # Wait until the ROS2 SERVICE SERVER IS RUNNING.
            self.get_logger().info("[wait] get_entity_state not still available, waiting...")
        self.req = GetEntityState.Request()                                                         # Declare the .srv.request variables.     

    def GET(self, name, ref):
        self.req.name = name
        self.req.reference_frame = ref
        self.future = self.cli.call_async(self.req)                                                 # Future: RESPONSE.

# Create NODE + SERVICE CLIENT for Gz Plugin (SET STATE) service:
class serviceClientSET(Node):
    def __init__(self):
        super().__init__('GraspSET_node')                                                           # Node NAME.
        self.cli = self.create_client(SetEntityState, "ros2_grasp/set_entity_state")                # Declare SERVICE CLIENT.
        while not self.cli.wait_for_service(timeout_sec=1.0):                                       # Wait until the ROS2 SERVICE SERVER IS RUNNING.
            self.get_logger().info("[wait] set_entity_state not still available, waiting...")
        self.req = SetEntityState.Request()                                                         # Declare the .srv.request variables.                                                        

    def SET(self, state):
        self.req.state = state
        self.future = self.cli.call_async(self.req)                                                 # Future: RESPONSE.

# Disable/Enable GRAVITY for the object to be manipulated:
class LinkProperties(Node):
    def __init__(self):
        super().__init__('GzLinkProperties_node')                                                   # Node NAME.
        self.cli = self.create_client(SetLinkProperties, "/ros2_grasp/set_link_properties")         # Declare SERVICE CLIENT.
        while not self.cli.wait_for_service(timeout_sec=1.0):                                       # Wait until the ROS2 SERVICE SERVER IS RUNNING.
            self.get_logger().info("[wait] set_link_properties not still available, waiting...")
        self.req = SetLinkProperties.Request()                                                      # Declare the .srv.request variables.                                                        

    def DisableGravity(self, name):
        self.req.link_name = name
        self.req.gravity_mode = False
        self.future = self.cli.call_async(self.req)                                                 # Future: RESPONSE.

    def EnableGravity(self, name):
        self.req.link_name = name
        self.req.gravity_mode = True
        self.future = self.cli.call_async(self.req)                                                 # Future: RESPONSE.

##################################################################

# ===== DETACH Topic ===== #

# Create NODE:
class Detach(Node):
    def __init__(self):
        # Declare NODE:
        super().__init__("DetachSUBSCRIBER_node")
        # Declare SUBSCRIBER:
        self.subscription = self.create_subscription(
            String, # ROS2 Topic/msg type.                                            
            "ros2_Detach", # ROS2 Topic name.
            self.listener_callback, # Callback function.
            5) # Queue size.
        self.subscription # Prevent unused variable warning.
    def listener_callback(self, MSG):
        global DETACH_MSG
        DETACH_MSG = str(MSG.data) 
        self.get_logger().info('Detach_MSG:' + DETACH_MSG) 

##################################################################

# ===== ACTION SERVER ===== #

# CREATE NODE + ACTION SERVER for ATTACHER:
class GraspingActionServer(Node):

    def __init__(self):
        super().__init__('Attacher_node')
        self._action_server = ActionServer(
            self,
            Attacher,
            'Attacher',
            execute_callback = self.execute_callback,                               # Declare EXECUTE CALBACK function.
            callback_group = ReentrantCallbackGroup(),                              # Declare CALLBACK GROUP.
            goal_callback = self.goal_callback,                                     # Declare GOAL CALLBACK function.
            cancel_callback = self.cancel_callback                                  # Declare CANCEL CALLBACK function.
        )

    def destroy(self):                                                              # destroy() will close the ROS2 Action Server.
        self._action_server.destroy()
        super().destroy_node()
    
    def goal_callback(self, goal_request):

        object = goal_request.object
        endeffector = goal_request.endeffector

        self.get_logger().info('Received a request for the "Attacher" ROS2-ACTION:')
        self.get_logger().info('Object to be attached/detached: ' + object)
        self.get_logger().info('End-effector: ' + endeffector)
        return GoalResponse.ACCEPT

    # The cancel_callback() function is used to decide whether the CANCELLATION of an Action Call is acceptable or not.
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received a cancel request.')
        return CancelResponse.ACCEPT

    # CALLBACK FUNCTION (this is what the ACTION CALL DOES):
    async def execute_callback(self, goal_handle):
        global DETACH_MSG

        self.get_logger().info('Executing "Attacher" ROS2-ACTION...')

        # Assign REQUEST variables:
        object = goal_handle.request.object
        endeffector = goal_handle.request.endeffector

        # Assign object name variable for Gravity manipulation (example -> instead of box, it has to be box::box since it refers to the link instead of the object):
        objectname = object + "::" + object
 
        # 0. Get INITIAL POSE:
        Pose_EE_BEFORE = self.GetPose(endeffector, "")
        Pose_OBJ_BEFORE = self.GetPose(object, "")
                                                            
        while (DETACH_MSG != "True"):                                                   

            # Check if a CANCEL REQUEST has been received:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled.')

                # (+) ENABLE GRAVITY for object:
                self.Gravity(objectname, True)

                return Attacher.Result()                                           

            # ===== OPERATE ===== #

            # 1. Get position and orientation of END EFFECTOR:
            Pose_EE_AFTER = self.GetPose(endeffector, "")

            # 2. Obtain POSE DIFFERENCE:
            DIF = EntityState()
            # Position:
            DIF.pose.position.x = Pose_EE_AFTER.pose.position.x - Pose_EE_BEFORE.pose.position.x
            DIF.pose.position.y = Pose_EE_AFTER.pose.position.y - Pose_EE_BEFORE.pose.position.y
            DIF.pose.position.z = Pose_EE_AFTER.pose.position.z - Pose_EE_BEFORE.pose.position.z
            # Orientation:
            DIF.pose.orientation = self.MULT(Pose_EE_AFTER.pose.orientation, self.INV(Pose_EE_BEFORE.pose.orientation))

            # 3. APPLY the difference to the OBJECT:
            Pose_OBJ_AFTER = EntityState()
            # Position:
            Pose_OBJ_AFTER.pose.position.x = Pose_OBJ_BEFORE.pose.position.x + DIF.pose.position.x
            Pose_OBJ_AFTER.pose.position.y = Pose_OBJ_BEFORE.pose.position.y + DIF.pose.position.y
            Pose_OBJ_AFTER.pose.position.z = Pose_OBJ_BEFORE.pose.position.z + DIF.pose.position.z
            # Orientation:
            Pose_OBJ_AFTER.pose.orientation = self.MULT(Pose_OBJ_BEFORE.pose.orientation, DIF.pose.orientation)

            # 4. Disable GRAVITY for object:
            self.Gravity(objectname, False)

            # 5. CALL SERVICE -> SET POSE of OBJECT:
            Pose_OBJ_AFTER.name = object
            self.SetPose(Pose_OBJ_AFTER)

            # 6. Re-set BEFORE values:
            Pose_EE_BEFORE = Pose_EE_AFTER
            Pose_OBJ_BEFORE = Pose_OBJ_AFTER

            Detach_Check = Detach()
            rclpy.spin_once(Detach_Check)

        # Detach object:
        self.Gravity(objectname, True)
        DETACH_MSG = ""
        
        # We are out from the loop, therefore -> Declare goal as SUCEEDED:
        goal_handle.succeed()

        # ASSIGN RESULT:
        result = Attacher.Result()
        return result

    # GET pose of object:
    def GetPose(self, object, ref):
        NodeGET = serviceClientGET()
        NodeGET.GET(object,ref)
        while rclpy.ok():
            rclpy.spin_once(NodeGET)
            if NodeGET.future.done():
                try:
                    response = NodeGET.future.result()
                except Exception as exc:
                    NodeGET.get_logger().info("GET STATE: Service call failed: " + str(exc))
                break
        return(response.state)

    # GET pose of object:
    def SetPose(self, state):
        NodeSET = serviceClientSET()
        NodeSET.SET(state)
        while rclpy.ok():
            rclpy.spin_once(NodeSET)
            if NodeSET.future.done():
                try:
                    response = NodeSET.future.result()
                except Exception as exc:
                    NodeSET.get_logger().info("SET STATE: Service call failed: " + str(exc))
                break
        return(response.success)

    # Enable/Disable Gravity:
    def Gravity(self, name, action):
        NodeGravity = LinkProperties()
        if (action == True):
            NodeGravity.EnableGravity(name)
        elif (action == False):
            NodeGravity.DisableGravity(name)
        while rclpy.ok():
            rclpy.spin_once(NodeGravity)
            if NodeGravity.future.done():
                try:
                    response = NodeGravity.future.result()
                except Exception as exc:
                    NodeGravity.get_logger().info("Enable/Disable gravity: Service call failed: " + str(exc))
                break
        return(response.success)


    # MULTIPLY quaternions:
    def MULT(self, A, B):
        C = Quaternion()
        C.x = A.w * B.x + A.x * B.w + A.y * B.z - A.z * B.y
        C.y = A.w * B.y - A.x * B.z + A.y * B.w + A.z * B.x
        C.z = A.w * B.z + A.x * B.y - A.y * B.x + A.z * B.w
        C.w = A.w * B.w - A.x * B.x - A.y * B.y - A.z * B.z
        return (C)
    # INVERSE of a quaternion, which is q(-1) = q* for rotation quaternions, since norm is equal to 1:
    def INV(self, A):
        A.x = -A.x
        A.y = -A.y
        A.z = -A.z
        A.w = A.w
        return(A)


# =================== MAIN =================== #
def main(args=None):

    # Initialise NODES:
    rclpy.init(args=args)
    NodeAS = GraspingActionServer()

    # Spin ACTION:
    rclpy.spin(NodeAS)

    rclpy.shutdown()

if __name__ == '__main__':
    main()