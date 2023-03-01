#!/usr/bin/python3
# -*- coding: utf-8 -*-

# IMPORT LIBRARIES:
import argparse
import os
import xacro
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity
import rclpy

# Reference to SPAWN OBJECT (.urdf or .xacro file) from the terminal shell:
# EXAMPLE: BOX -> ros2 run ros2_grasping spawn_object.py --package "ros2_grasping" --urdf "box.urdf" --name "box" --x 0.5 --y -0.3 --z 0.75

def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn object into our Gazebo world.')
    parser.add_argument('--package', type=str, default='', help='Package where URDF/XACRO file is located.')
    parser.add_argument('--urdf', type=str, default='', help='URDF of the object to spawn.')
    parser.add_argument('--name', type=str, default='OBJECT', help='Name of the object to spawn.')
    parser.add_argument('--namespace', type=str, default='ros2Grasp', help='ROS namespace to apply to the tf and plugins.')
    parser.add_argument('--ns', type=bool, default=True, help='Whether to enable namespacing')
    parser.add_argument('--x', type=float, default=0.0, help='the x component of the initial position [meters].')
    parser.add_argument('--y', type=float, default=0.0, help='the y component of the initial position [meters].')
    parser.add_argument('--z', type=float, default=0.0, help='the z component of the initial position [meters].')
    
    args, unknown = parser.parse_known_args()

    # Start node:
    rclpy.init()
    node = rclpy.create_node('entity_spawner')

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    # Set data for request:
    request = SpawnEntity.Request()
    request.name = args.name

    urdf_file_path = os.path.join(get_package_share_directory(args.package), 'urdf', args.urdf) # It is assumed that the .urdf/.xacro file is located in /urdf folder!
    xacro_file = xacro.process_file(urdf_file_path)
    request.xml = xacro_file.toxml() 

    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)

    if args.namespace is True:
        node.get_logger().info('spawning `{}` on namespace `{}` at {}, {}, {}'.format(
            args.name, args.namespace, args.x, args.y, args.z))

        request.namespace = args.namespace
        print(args.namespace)

    else:
        node.get_logger().info('spawning `{}` at {}, {}, {}'.format(
            args.name, args.x, args.y, args.z))

    node.get_logger().info('Spawning OBJECT using service: `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
