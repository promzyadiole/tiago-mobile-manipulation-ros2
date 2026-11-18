#!/usr/bin/env python3
"""Detect the table, the bottle and the cup with the head camera, and compare
the result with the true positions from the world file.

Usage (Gazebo, navigation and MoveIt running):
    ros2 run my_tiago_tasks perception_demo            # drive to the middle table first
    ros2 run my_tiago_tasks perception_demo --here     # robot is already at a table

The detections are drawn in RViz on /detected_objects (MarkerArray).
"""
import math
import sys

import rclpy

from my_tiago_tasks.exercise1 import APPROACH, OBJECTS, TABLE_TOP
from my_tiago_tasks.perception import TabletopPerception
from my_tiago_tasks.robot import TiagoRobot


def main():
    rclpy.init()
    robot = TiagoRobot('perception_demo')
    perception = TabletopPerception(robot)
    found = set()
    try:
        robot.wait_for_nav2()
        if '--here' not in sys.argv:
            robot.stow_arm()
            if not robot.go_to(*APPROACH):
                robot.get_logger().error('Could not reach the table')
                return
        robot.stow_arm()                      # keep the arm out of the camera view
        robot.move_head(0.0, -0.6)            # look down at the table
        table, objects = perception.detect()
        if table is None:
            robot.get_logger().error('No table found')
            return
        if objects:
            # Objects at the edge of the image are cut off (wrong height and
            # radius). Centre the camera on them and measure again.
            mx = sum(o.pose.pose.position.x for o in objects) / len(objects)
            my = sum(o.pose.pose.position.y for o in objects) / len(objects)
            robot.look_at(mx, my, table.center[2] + 0.10)
            table, objects = perception.detect()
            if table is None:
                robot.get_logger().error('Table lost after re-centring the camera')
                return

        # Compare with the world file (true positions, map frame -> robot frame)
        robot.get_logger().info(f'table top: measured z={table.center[2]:.3f} m, '
                                f'true {TABLE_TOP:.3f} m, near edge {table.near_x:.2f} m ahead')
        for o in objects:
            p = o.pose.pose.position
            line = (f'{o.label:7s} at base ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}), '
                    f'r={o.radius * 100:.1f} cm, h={o.height * 100:.1f} cm')
            if o.label in OBJECTS:
                tx, ty, th, _ = OBJECTS[o.label]
                bx, by = robot.map_to_base(tx, ty)
                err = math.hypot(p.x - bx, p.y - by)
                line += f'; true ({bx:.3f}, {by:.3f}), error {err * 100:.1f} cm, true h {th * 100:.0f} cm'
                found.add(o.label)
            robot.get_logger().info(line)
    finally:
        ok = found == set(OBJECTS)
        robot.get_logger().info('Perception demo: ' + ('bottle and cup found' if ok else
                                                        f'MISSING {set(OBJECTS) - found}'))
        robot.shutdown()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
