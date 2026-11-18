#!/usr/bin/env python3
"""Pick one object from the middle table, hold it up, and put it back.

Tests the grasp chain (perception, MoveIt, gripper, link attacher) without
the long drive to the corner table.

Usage (Gazebo, navigation and MoveIt running):
    ros2 run my_tiago_tasks pick_demo            # the bottle
    ros2 run my_tiago_tasks pick_demo cup
"""
import sys

import rclpy

from my_tiago_tasks.pick_place import MIDDLE_TABLE, OBJECT_ORDER, PickAndPlace
from my_tiago_tasks.robot import TiagoRobot


def main():
    label = next((a for a in sys.argv[1:] if a in OBJECT_ORDER), 'bottle')
    rclpy.init()
    robot = TiagoRobot('pick_demo')
    pp = PickAndPlace(robot)
    log = robot.get_logger()
    ok = False
    try:
        robot.wait_for_nav2()
        robot.clear_scene(OBJECT_ORDER + ['table'])
        robot.open_gripper()
        pp.stow_arm()
        if not pp.drive_to(MIDDLE_TABLE['approach']):
            log.error('Could not reach the middle table')
            return
        picked = pp.pick(label)
        if not picked:
            log.error(f'Could not pick the {label}')
            return
        target, grasp_h = picked
        pp.carry_pose()
        robot.sleep(2.0)                       # hold it up for a moment
        ok = pp.place(label, target, grasp_h, MIDDLE_TABLE)
    finally:
        log.info(f'Pick demo ({label}): ' + ('picked and put back' if ok else 'FAILED'))
        robot.shutdown()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
