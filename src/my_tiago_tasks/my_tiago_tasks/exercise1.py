#!/usr/bin/env python3
"""Exercise 1: drive to the middle table and move the arm above it.

The robot starts in the corner of the room (map origin). This script:
  1. waits for Nav2 and MoveIt,
  2. folds the arm at the side (small footprint for driving),
  3. drives in front of the middle table with Nav2,
  4. adds the table, the bottle and the cup to MoveIt's planning scene,
  5. moves the arm so the gripper points down, right above the table.

The table and object positions come from the world file (known map).
Exercise 2 detects the objects with the head camera instead (perception.py).

Usage (Gazebo, navigation and MoveIt running):
    ros2 run my_tiago_tasks exercise1
"""
import math
import sys

import rclpy

from my_tiago_tasks.robot import TOOL_TO_GRASP, TiagoRobot, make_pose

# Middle table (my_tiago_room.world): centre (2.25, 2.25) in the map, top
# 0.8 m (x) by 1.2 m (y), surface at z = 0.75 m.
TABLE_CENTER = (2.25, 2.25)
TABLE_SIZE = (0.8, 1.2)
TABLE_TOP = 0.75
TABLE_NEAR_X = TABLE_CENTER[0] - TABLE_SIZE[0] / 2      # edge facing the robot: 1.85
# Objects on it: (x, y, height, radius) in the map; the cup radius includes its handle
OBJECTS = {'bottle': (2.0, 2.05, 0.26, 0.03),
           'cup': (2.0, 2.45, 0.10, 0.05)}

# Stop 0.5 m before the table edge, facing it (+x)
APPROACH = (TABLE_NEAR_X - 0.5, TABLE_CENTER[1], 0.0)
# Grasping frame (between the fingertips) above the table top [m]
HOVER_HEIGHT = 0.30
# Gripper target, measured from the robot: 0.70 m ahead of base_footprint.
# With the gripper pointing down at this height the arm reaches about 0.72 m
# (0.81 m failed: "Unable to sample any valid states for goal tree"), so the
# target is set relative to where the robot actually stopped, not to the
# table: Nav2 may stop up to 10 cm short and 20 deg off the goal.
REACH = 0.70


def build_scene(robot):
    """Table and objects as collision objects, so MoveIt plans around them.
    Without this the arm can sweep through the table top, and in Gazebo that
    collision throws the whole robot around."""
    robot.clear_scene(['table', 'bottle', 'cup'])
    robot.add_box('table', make_pose('map', TABLE_CENTER[0], TABLE_CENTER[1], TABLE_TOP / 2),
                  (TABLE_SIZE[0], TABLE_SIZE[1], TABLE_TOP))
    for name, (x, y, h, r) in OBJECTS.items():
        robot.add_cylinder(name, make_pose('map', x, y, TABLE_TOP + h / 2), h, r + 0.01)


def arm_above_table(robot):
    """Gripper pointing down, HOVER_HEIGHT above the table."""
    _, _, byaw = robot.base_pose()
    x, y = robot.base_to_map(REACH, 0.0)               # REACH straight ahead of the robot
    z = TABLE_TOP + HOVER_HEIGHT + TOOL_TO_GRASP      # arm_tool_link is above the fingertips
    # pitch +90 deg turns the tool x axis down; yaw follows the robot heading
    target = make_pose('map', x, y, z, roll=0.0, pitch=math.pi / 2, yaw=byaw)
    robot.get_logger().info(f'Gripper target (map): x={x:.2f} y={y:.2f} z={z:.2f}, pointing down')
    if robot.move_to_pose(target, planning_time=10.0):
        return True
    # Fallback: horizontal gripper (easier to reach), still above the table
    robot.get_logger().warn('Top-down pose not reachable, trying a horizontal gripper')
    x, y = robot.base_to_map(REACH - 0.1, 0.0)
    target = make_pose('map', x, y, TABLE_TOP + HOVER_HEIGHT, roll=math.pi / 2, yaw=byaw)
    return robot.move_to_pose(target, planning_time=10.0)


def report(robot):
    """Where the fingertips ended up, relative to the table."""
    g = robot.lookup('map', 'gripper_grasping_frame').transform.translation
    over = (abs(g.x - TABLE_CENTER[0]) <= TABLE_SIZE[0] / 2
            and abs(g.y - TABLE_CENTER[1]) <= TABLE_SIZE[1] / 2)
    robot.get_logger().info(
        f'Fingertips at map x={g.x:.2f} y={g.y:.2f}, {g.z - TABLE_TOP:.2f} m above the table top, '
        + ('over the table' if over else 'NOT over the table'))
    return over


def main():
    rclpy.init()
    robot = TiagoRobot('exercise1')
    ok = False
    try:
        robot.wait_for_nav2()
        sx, sy, _ = robot.base_pose()
        robot.get_logger().info(f'Start pose: x={sx:.2f} y={sy:.2f}')

        # 1) Fold the arm for driving
        robot.stow_arm()

        # 2) Drive in front of the middle table
        if not robot.go_to(*APPROACH):
            robot.get_logger().error('Could not reach the table')
            return

        # 3) Planning scene, look at the table, arm above it
        build_scene(robot)
        robot.look_at(0.8, 0.0, TABLE_TOP)
        ok = arm_above_table(robot) and report(robot)
        if ok:
            robot.get_logger().info('Exercise 1 complete: the gripper is above the table.')
        else:
            robot.get_logger().error('Exercise 1: arm motion failed')
    finally:
        robot.shutdown()
        rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
