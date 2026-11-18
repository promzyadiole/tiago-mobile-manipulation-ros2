#!/usr/bin/env python3
"""Drive Tiago to one pose on the map with Nav2 (Simple Commander API).

Usage:
    ros2 run my_tiago_tasks go_to_pose X Y [YAW_DEG]

    ros2 run my_tiago_tasks go_to_pose 2.29 1.0 90    # in front of the middle table
    ros2 run my_tiago_tasks go_to_pose 0 0            # back to the start

X and Y are in metres in the map frame, YAW_DEG is the final heading in
degrees (0 = map +x, 90 = map +y). Needs my_tiago_gazebo.launch.xml and
my_tiago_navigation.launch.xml to be running.
"""
import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator, x, y, yaw):
    """PoseStamped in the map frame; yaw (rad) about z as a quaternion."""
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    # Rotation about z only: q = (0, 0, sin(yaw/2), cos(yaw/2))
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    parser = argparse.ArgumentParser(description='Send Tiago to a pose with Nav2.')
    parser.add_argument('x', type=float, help='goal x in the map frame [m]')
    parser.add_argument('y', type=float, help='goal y in the map frame [m]')
    parser.add_argument('yaw', type=float, nargs='?', default=0.0,
                        help='goal heading [deg], default 0')
    # rclpy.init() takes the --ros-args part, argparse the rest
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    navigator = BasicNavigator(node_name='go_to_pose')

    # 1. Wait for Nav2.
    # waitUntilNav2Active(localizer='amcl') would first publish an initial pose
    # of (0, 0, 0), which resets AMCL to the spawn point every time this
    # script runs. AMCL already gets its start pose from the params file, so
    # only wait for the two nodes to be active.
    navigator._waitForNodeToActivate('amcl')
    navigator._waitForNodeToActivate('bt_navigator')

    # 2. Send the goal (NavigateToPose action, like "Nav2 Goal" in RViz).
    goal = make_pose(navigator, args.x, args.y, math.radians(args.yaw))
    navigator.info(f'Going to x={args.x:.2f} y={args.y:.2f} yaw={args.yaw:.0f} deg')
    if not navigator.goToPose(goal):
        navigator.error('Goal was rejected')
        rclpy.shutdown()
        sys.exit(1)

    # 3. Wait for the result, printing progress about once a second.
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 10 == 0:
            navigator.info(f'Distance remaining: {feedback.distance_remaining:.2f} m')

    # 4. Report.
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.info('Goal reached')
    elif result == TaskResult.CANCELED:
        navigator.warn('Goal was canceled')
    else:
        navigator.error('Goal failed')

    navigator.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if result == TaskResult.SUCCEEDED else 1)


if __name__ == '__main__':
    main()
