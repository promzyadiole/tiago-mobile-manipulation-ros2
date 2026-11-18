#!/usr/bin/env python3
"""Exercise every part of robot.py once, without driving the base.

Usage (Gazebo + MoveIt running):
    ros2 run my_tiago_tasks robot_demo

Steps: head, gripper, home pose, a table in the planning scene, the gripper
above the table, a straight 10 cm move down and back up, home again.
"""
import math

import rclpy

from my_tiago_tasks.robot import TiagoRobot, make_pose

# Pretend table in front of the robot (base_footprint frame), only for MoveIt
TABLE_X, TABLE_Z = 0.85, 0.73          # centre distance, top height [m]
TABLE_SIZE = (0.6, 1.0, TABLE_Z)       # depth, width, height [m]


def step(robot, text):
    robot.get_logger().info(f'--- {text}')


def main():
    rclpy.init()
    robot = TiagoRobot('robot_demo')
    ok = True
    try:
        step(robot, 'head: look down at the area in front of the robot')
        robot.look_at(0.8, 0.0, 0.7)

        step(robot, 'gripper: open, then close')
        robot.open_gripper()
        opened = robot.gripper_opening()
        robot.set_gripper(0.0)
        robot.get_logger().info(f'opening: {opened:.3f} m -> {robot.gripper_opening():.3f} m')

        step(robot, 'arm: home pose')
        ok &= robot.home_arm()

        step(robot, 'planning scene: add a table')
        robot.add_box('demo_table',
                      make_pose('base_footprint', TABLE_X, 0.0, TABLE_Z / 2), TABLE_SIZE)

        step(robot, 'arm: gripper 20 cm above the table, pointing forward')
        above = make_pose('base_footprint', 0.6, -0.15, TABLE_Z + 0.20)
        ok &= robot.move_to_pose(above)
        p = robot.current_ee_pose()
        robot.get_logger().info(f'arm_tool_link at x={p.position.x:.3f} y={p.position.y:.3f} '
                                f'z={p.position.z:.3f}')

        step(robot, 'arm: straight 10 cm down and back up (Cartesian)')
        down = robot.current_ee_pose()
        down.position.z -= 0.10
        ok &= robot.move_cartesian([down])
        up = robot.current_ee_pose()
        up.position.z += 0.10
        ok &= robot.move_cartesian([up])

        step(robot, 'arm: home pose, remove the table, head straight')
        ok &= robot.home_arm()
        robot.remove_object('demo_table')
        robot.move_head(0.0, math.radians(-20))
    finally:
        robot.get_logger().info('Demo ' + ('succeeded' if ok else 'had FAILURES'))
        robot.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
