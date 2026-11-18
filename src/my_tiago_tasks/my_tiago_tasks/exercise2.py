#!/usr/bin/env python3
"""Exercise 2: move the bottle and the cup from the middle table to the corner table.

For each object the robot:
  1. drives to the middle table (Nav2),
  2. looks at the table and detects the objects with the head RGB-D camera
     (table plane + 6-DOF pose, size and label of each object),
  3. builds a MoveIt collision scene from the detections,
  4. picks the object: pre-grasp (MoveIt plan) -> straight approach
     (Cartesian) -> close the gripper -> attach -> lift -> camera check,
  5. holds it close to the body and drives to the corner table (Nav2),
  6. detects the corner table top and chooses a free spot on it,
  7. places the object: pre-place -> straight down -> release -> retreat,
  8. checks with the camera that the object is on the corner table.

Only where the two tables are is known in advance (map); the objects are
found by perception. The pick and place steps are in pick_place.py.

Usage (Gazebo, navigation and MoveIt running, robot freshly spawned):
    ros2 run my_tiago_tasks exercise2
"""
import sys

import rclpy

from my_tiago_tasks.pick_place import CORNER_TABLE, MIDDLE_TABLE, OBJECT_ORDER, PickAndPlace
from my_tiago_tasks.robot import TiagoRobot

PICK_ATTEMPTS = 3


def run(robot):
    pp = PickAndPlace(robot)
    log = robot.get_logger()
    robot.wait_for_nav2()
    robot.clear_scene(OBJECT_ORDER + ['table'])
    robot.open_gripper()
    pp.stow_arm()

    results = {}
    for i, label in enumerate(OBJECT_ORDER):
        log.info(f'===== {label.upper()} =====')
        if i > 0:
            robot.back_up()          # get clear of the corner table before turning
        if not pp.drive_to(MIDDLE_TABLE['approach']):
            log.error('Could not reach the middle table; stopping.')
            results[label] = False
            break

        # Pick, with retries
        picked = None
        for attempt in range(PICK_ATTEMPTS):
            picked = pp.pick(label)
            if picked:
                break
            log.warn(f'Pick attempt {attempt + 1} of {PICK_ATTEMPTS} failed')
            # Stow with the collision scene still in place, then clear it
            # (stowing in an empty scene can sweep the arm through the table).
            pp.stow_arm()
            robot.clear_scene(OBJECT_ORDER + ['table'])
            if pp.last_table is None:
                # No table where the map says there is one: the robot is
                # probably mislocalised. Give AMCL time to recover, then drive
                # to the table again before retrying.
                log.warn('Table not seen: re-localising and driving there again')
                robot.sleep(5.0)
                robot.back_up(0.2)
                pp.drive_to(MIDDLE_TABLE['approach'])
        if not picked:
            results[label] = False
            continue
        target, grasp_h = picked

        # Carry it to the corner table
        pp.carry_pose()
        robot.clear_scene(['table'] + [o for o in OBJECT_ORDER if o != label])
        robot.back_up()              # get clear of the middle table before turning
        if not pp.drive_to(CORNER_TABLE['approach']):
            log.error('Could not reach the corner table; stopping.')
            results[label] = False
            break

        # Place it
        results[label] = pp.place(label, target, grasp_h, CORNER_TABLE)
        if not results[label] and pp.holding:
            # Never continue with an object still in the hand
            log.error(f'Could not place the {label}; stopping.')
            break

    log.info('Exercise 2 result: ' + ', '.join(
        f'{k}={"placed" if v else "FAILED"}' for k, v in results.items()))
    return len(results) == len(OBJECT_ORDER) and all(results.values())


def main():
    rclpy.init()
    robot = TiagoRobot('exercise2')
    ok = False
    try:
        ok = run(robot)
    finally:
        robot.shutdown()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
