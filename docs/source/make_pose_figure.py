#!/usr/bin/env python3
"""Labelled side view of the 'gripper pointing down' pose goal (study note).

The pose goal (C++ practice, Exercise 1):
    make_pose("base_footprint", 0.70, 0.00, 1.19, roll 0, pitch +90 deg, yaw 0)
drawn with the real robot meshes in the joint configuration MoveIt's IK chose
for it in Gazebo (recorded from /joint_states after the move), the table of
Exercise 1, and every number of the goal marked on the drawing.

Run:  python3 make_pose_figure.py
Inputs: make_joint_figures.py (URDF, meshes, forward kinematics)
Output: images/fig_pose_down.png
"""
import importlib.util
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location('mjf', os.path.join(HERE, 'make_joint_figures.py'))
mjf = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mjf)          # also fixes the mpl_toolkits import
plt = mjf.plt

# Joint values after the pose goal (Gazebo, /joint_states)
Q = {'torso_lift_joint': 0.35, 'arm_1_joint': 1.174, 'arm_2_joint': 0.929,
     'arm_3_joint': -0.700, 'arm_4_joint': 1.308, 'arm_5_joint': 1.164,
     'arm_6_joint': -1.336, 'arm_7_joint': -1.274,
     'gripper_left_finger_joint': 0.03, 'gripper_right_finger_joint': 0.03,   # drawn open
     'head_1_joint': 0.0, 'head_2_joint': 0.0}

TABLE_NEAR, TABLE_DEPTH, TABLE_TOP = 0.5, 0.8, 0.75     # exercise1.py (robot stops 0.5 m before the edge)
HOVER, TOOL_TO_GRASP = 0.30, 0.141                    # exercise1.py, robot.py
X, Z = 0.70, TABLE_TOP + HOVER + TOOL_TO_GRASP        # 1.191


def main():
    robot = mjf.Robot(mjf.URDF)
    links, _ = robot.fk(Q)
    fig = plt.figure(figsize=(9.2, 7.6))
    ax = fig.add_subplot(projection='3d', computed_zorder=False)
    ax.set_proj_type('ortho')

    # robot: whole body light grey, the gripper (arm_7 and what it carries) orange
    hand = robot.subtree('arm_7_link')
    colours = {lk: (np.array([0.70, 0.73, 0.78]), 0.55) for lk in robot.visuals}
    for lk in hand:
        colours[lk] = (mjf.CHILD_C, 0.95)
    mjf.draw_robot(ax, robot, Q, colours)

    # table of Exercise 1 (front edge 0.5 m ahead), drawn as its side outline
    y0 = -0.75
    tx = [TABLE_NEAR, TABLE_NEAR + TABLE_DEPTH]
    ax.plot([tx[0], tx[1], tx[1], tx[0], tx[0]], [y0] * 5, [0, 0, TABLE_TOP, TABLE_TOP, 0],
            color='#8d6e63', lw=1.6, zorder=5)
    ax.plot(tx, [y0, y0], [TABLE_TOP, TABLE_TOP], color='#5d4037', lw=4, zorder=6)
    ax.text(1.02, y0, TABLE_TOP - 0.06, 'middle table\n(top 0.75 m)', color='#5d4037', fontsize=8,
            ha='center', va='top', zorder=30)

    # floor and base_footprint frame
    ax.plot([-0.35, 1.5], [y0, y0], [0, 0], color='#555555', lw=1.2, zorder=5)
    mjf.arrow(ax, (0, y0, 0), (0.28, y0, 0), '#d62728', lw=2, scale=12, z=25)
    mjf.arrow(ax, (0, y0, 0), (0, y0, 0.28), '#1f77b4', lw=2, scale=12, z=25)
    ax.text(0.30, y0, -0.03, 'x (forward)', color='#d62728', fontsize=8, va='top', zorder=30)
    ax.text(0.02, y0, 0.30, 'z (up)', color='#1f77b4', fontsize=8, zorder=30)
    ax.text(-0.02, y0, -0.05, 'base_footprint\n(0, 0, 0): floor under\nthe robot centre',
            fontsize=7.5, ha='right', va='top', zorder=30)

    # the two points: arm_tool_link (pose goal) and the fingertips
    tool = links['arm_tool_link'][:3, 3]
    grasp = links['gripper_grasping_frame'][:3, 3]
    ax.scatter([X], [y0], [Z], s=45, color='black', zorder=40)
    ax.scatter([X], [y0], [TABLE_TOP + HOVER], s=45, color='#c0392b', zorder=40)
    ax.text(X - 0.03, y0, Z + 0.05, 'arm_tool_link = pose goal\n(0.70, 0.00, 1.19)',
            fontsize=8, ha='right', zorder=40, fontweight='bold')
    ax.text(X + 0.04, y0, TABLE_TOP + HOVER - 0.03, 'fingertips (gripper_grasping_frame)\nz = 1.05 m',
            fontsize=7.5, color='#c0392b', ha='left', va='top', zorder=40)

    # arm_tool_link axes: pitch +90 deg turns the hand's x axis (red) DOWN, its z axis forward
    T = links['arm_tool_link']
    for i, c, lab in [(0, '#d62728', 'x of arm_tool_link\npoints DOWN'), (2, '#1f77b4', 'z')]:
        tip = tool + (0.10 if i == 0 else 0.16) * T[:3, i]
        mjf.arrow(ax, (X, y0, Z), (X + tip[0] - tool[0], y0, Z + tip[2] - tool[2]), c, lw=2.2,
                  scale=13, z=45)
        lx, lz = (X + 0.04, Z - 0.035) if i == 0 else (X + 0.19, Z - 0.012)
        ax.text(lx, y0, lz, lab, color=c, fontsize=7.5, zorder=45)

    # dimension lines: x = 0.70 on the floor, z = 0.75 + 0.30 + 0.141 on the right
    def dim(p0, p1, text, off, colour='#222222', ha='left'):
        ax.plot([p0[0], p1[0]], [y0, y0], [p0[1], p1[1]], color=colour, lw=1.1, zorder=35)
        for p in (p0, p1):
            ax.scatter([p[0]], [y0], [p[1]], s=10, color=colour, zorder=35)
        m = ((p0[0] + p1[0]) / 2 + off[0], (p0[1] + p1[1]) / 2 + off[1])
        ax.text(m[0], y0, m[1], text, fontsize=8, color=colour, ha=ha, va='center', zorder=40)

    dim((0, -0.10), (X, -0.10), 'x = 0.70 m  (REACH: straight ahead of the robot)', (-0.30, -0.06))
    xd = 1.68
    dim((xd, 0.0), (xd, TABLE_TOP), '0.75 m\ntable top', (0.03, 0), '#5d4037')
    dim((xd, TABLE_TOP), (xd, TABLE_TOP + HOVER), '+ 0.30 m\nHOVER_HEIGHT\n(fingertips above\nthe table)', (0.03, 0),
        '#c0392b')
    dim((xd, TABLE_TOP + HOVER), (xd, Z), '+ 0.141 m\nTOOL_TO_GRASP\n(arm_tool_link above\nthe fingertips)',
        (0.03, 0.02), '#333333')
    ax.text(xd + 0.03, y0, Z + 0.10, '= z 1.19 m', fontsize=9, fontweight='bold', zorder=40)
    for zz in (TABLE_TOP, TABLE_TOP + HOVER, Z):
        ax.plot([X, xd], [y0, y0], [zz, zz], color='#999999', lw=0.7, ls=':', zorder=30)

    ax.text(-0.33, y0, 1.62, 'Pose goal: make_pose("base_footprint", 0.70, 0.00, 1.19, 0, +90°, 0)\n'
            'y = 0.00 m: straight ahead (this side view looks from the robot\'s right)\n'
            'roll 0, pitch +90°, yaw 0: the gripper points straight DOWN',
            fontsize=8.5, zorder=40)
    ax.text(-0.33, y0, -0.32, f'Joints chosen by IK (Gazebo): torso {Q["torso_lift_joint"]:.2f} m,  arm '
            + ', '.join(f'{Q[f"arm_{i}_joint"]:.2f}' for i in range(1, 8)) + ' rad (arm_1 ... arm_7)\n'
            f'Measured in Gazebo and by FK: arm_tool_link z = {tool[2]:.3f} m, fingertips z = {grasp[2]:.3f} m',
            fontsize=7, color='#555555', zorder=40)

    # side view: x to the right, z up (looking from the robot's right side, -y)
    c, half = np.array([0.72, 0.0, 0.70]), 1.02
    ax.set_xlim(c[0] - half, c[0] + half)
    ax.set_ylim(c[1] - half, c[1] + half)
    ax.set_zlim(c[2] - half, c[2] + half)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=0, azim=-90)
    ax.set_axis_off()
    fig.subplots_adjust(left=0, right=1, top=1, bottom=0)
    out = os.path.join(mjf.IMG, 'fig_pose_down.png')
    fig.savefig(out, dpi=160, bbox_inches='tight', pad_inches=0.05)
    # trim the empty margins that the 3D axes leave around the drawing
    from PIL import Image, ImageChops
    im = Image.open(out).convert('RGB')
    box = ImageChops.difference(im, Image.new('RGB', im.size, 'white')).getbbox()
    im.crop((max(box[0] - 20, 0), max(box[1] - 20, 0), box[2] + 20, box[3] + 20)).save(out)
    print('arm_tool_link (FK):', np.round(tool, 3), ' fingertips:', np.round(grasp, 3))
    print('written', out)


if __name__ == '__main__':
    main()
