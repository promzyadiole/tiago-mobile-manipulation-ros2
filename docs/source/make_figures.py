#!/usr/bin/env python3
"""Data figures for my_tiago_documentation.html (matplotlib).

Run:  python3 make_figures.py
Inputs: ../../src/my_tiago_bringup/maps/my_tiago_room.{pgm,yaml}, images/test_map_smeared.pgm
Output: images/fig_*.png
"""
import math
import os

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, 'images')
MAPS = os.path.join(HERE, '..', '..', 'src', 'my_tiago_bringup', 'maps')

plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9,
                     'axes.edgecolor': '#555555', 'axes.titleweight': 'bold'})

RES, OX, OY = 0.05, -1.31, -1.32       # from my_tiago_room.yaml


def load_map(path):
    return np.array(Image.open(path).convert('L'))


def extent(img, ox, oy, res):
    h, w = img.shape
    return [ox, ox + w * res, oy, oy + h * res]


def fig_map_annotated():
    m = load_map(os.path.join(MAPS, 'my_tiago_room.pgm'))
    fig, ax = plt.subplots(figsize=(6.4, 6.0))
    ax.imshow(m, cmap='gray', extent=extent(m, OX, OY, RES), interpolation='nearest')
    # image corner and map origin
    ax.plot(OX, OY, 'o', color='#d35400', ms=8)
    ax.annotate('image bottom-left corner\norigin: [-1.31, -1.32, 0]', (OX, OY), (OX + 0.25, OY + 0.35),
                color='#d35400', fontsize=8, fontweight='bold')
    ax.annotate('', (0.8, 0), (0, 0), arrowprops=dict(arrowstyle='->', color='#c0392b', lw=2))
    ax.annotate('', (0, 0.8), (0, 0), arrowprops=dict(arrowstyle='->', color='#1e8449', lw=2))
    ax.text(0.85, -0.05, 'x', color='#c0392b', fontweight='bold')
    ax.text(-0.08, 0.85, 'y', color='#1e8449', fontweight='bold')
    ax.text(0.08, -0.32, 'map (0,0) = spawn pose', fontsize=8, fontweight='bold')
    # objects (positions from my_tiago_room.world)
    ax.text(2.25, 2.25, 'middle table\n(2.25, 2.25)', ha='center', va='center', fontsize=7.5, color='#1f3a5f')
    ax.text(4.55, 4.55 - 0.55, 'corner table\n(4.55, 4.55)', ha='center', fontsize=7.5, color='#1f3a5f')
    # navigation goals used in the tests
    goals = [(2.29, 1.0, 90, 'goal A'), (4.0, 3.4, 90, 'goal B'), (0.0, 0.0, 0, '')]
    for x, y, yaw, name in goals:
        if name:
            ax.plot(x, y, '*', color='#8e44ad', ms=13)
            ax.text(x + 0.12, y - 0.25, f'{name} ({x}, {y}, {yaw}°)', color='#8e44ad', fontsize=7.5)
    ax.set_xlabel('map x [m]')
    ax.set_ylabel('map y [m]')
    ax.set_title('Saved SLAM map my_tiago_room (133 x 133 cells, 0.05 m)')
    ax.grid(color='#9fb3c8', lw=0.4, alpha=0.6)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'fig_map_annotated.png'), dpi=170)
    plt.close(fig)


def fig_map_compare():
    bad = load_map(os.path.join(IMG, 'test_map_smeared.pgm'))
    good = load_map(os.path.join(MAPS, 'my_tiago_room.pgm'))
    fig, axs = plt.subplots(1, 2, figsize=(8.6, 4.2))
    axs[0].imshow(bad, cmap='gray', extent=extent(bad, -5.09, -6.7, RES), interpolation='nearest')
    axs[0].set_title('Fast spins (0.6 rad/s): smeared, 13 x 12 m', color='#c0392b')
    axs[1].imshow(good, cmap='gray', extent=extent(good, OX, OY, RES), interpolation='nearest')
    axs[1].set_title('Slow turns (0.3 rad/s): clean, 6.65 x 6.65 m', color='#1e8449')
    for a in axs:
        a.set_xlabel('x [m]')
        a.set_ylabel('y [m]')
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'fig_map_compare.png'), dpi=160)
    plt.close(fig)


def fig_ekf():
    # Measured in the go_to_pose tests (Gazebo ground truth vs estimate)
    fig, axs = plt.subplots(1, 2, figsize=(8.6, 3.3))
    labels = ['wheel odom\n(no EKF)', 'AMCL\n(no EKF)']
    pos_err = [np.hypot(2.24 + 0.16, 1.50 + 0.83), np.hypot(2.24 - 0.96, 1.50 + 1.37)]
    yaw_err = [149 - 72, 177 - 72]
    after_labels = ['goal A', 'home', 'goal B', 'home']
    after_pos = [np.hypot(2.25 - 2.23, 0.90 - 0.92), np.hypot(0.07 - 0.08, 0.02 - 0.03),
                 np.hypot(4.03 - 4.00, 3.33 - 3.22), np.hypot(0.07 - 0.07, 0.02 - 0.04)]
    after_yaw = [0.2, 2.0, 0.6, 4.2]
    x1 = np.arange(len(labels))
    axs[0].bar(x1 - 0.18, pos_err, 0.36, color='#c0392b', label='position error [m]')
    ax0b = axs[0].twinx()
    ax0b.bar(x1 + 0.18, yaw_err, 0.36, color='#e8a598', label='heading error [deg]')
    axs[0].set_xticks(x1, labels)
    axs[0].set_ylabel('position error [m]', color='#c0392b')
    ax0b.set_ylabel('heading error [deg]', color='#b05a4a')
    axs[0].set_title('Before EKF: 2nd goal FAILED (robot lost)')
    x2 = np.arange(len(after_labels))
    axs[1].bar(x2 - 0.18, after_pos, 0.36, color='#1e8449')
    ax1b = axs[1].twinx()
    ax1b.bar(x2 + 0.18, after_yaw, 0.36, color='#9fd5b3')
    axs[1].set_xticks(x2, after_labels)
    axs[1].set_ylabel('AMCL position error [m]', color='#1e8449')
    ax1b.set_ylabel('AMCL heading error [deg]', color='#3d8b5f')
    axs[1].set_ylim(0, 0.2)
    ax1b.set_ylim(0, 10)
    axs[1].set_title('With EKF: 4 / 4 goals reached')
    from matplotlib.patches import Patch
    fig.legend(handles=[Patch(color='#555555', label='dark bar: position error [m] (left axis)'),
                        Patch(color='#bbbbbb', label='light bar: heading error [deg] (right axis)')],
               loc='lower center', ncol=2, frameon=False, fontsize=8)
    fig.tight_layout(rect=(0, 0.07, 1, 1))
    fig.savefig(os.path.join(IMG, 'fig_ekf_errors.png'), dpi=170)
    plt.close(fig)


def fig_place_spots():
    """Run the real choose_place_spot() on the corner table (numbers from a
    test run: near edge 0.56 m ahead, bottle already placed) and draw why each
    candidate spot was accepted or rejected."""
    import sys
    from types import SimpleNamespace as NS
    sys.path.insert(0, os.path.join(HERE, '..', '..', 'src', 'my_tiago_tasks'))
    import my_tiago_tasks.pick_place as ppm
    from my_tiago_tasks.perception import Table, TabletopPerception

    class Log:
        def info(self, m):
            pass
    class FakeRobot:                # robot standing straight in front of the corner table
        def __init__(self, x, y): self.x, self.y = x, y
        def base_to_map(self, x, y): return self.x + x, self.y + y
        def map_to_base(self, x, y): return x - self.x, y - self.y
    pp = ppm.PickAndPlace.__new__(ppm.PickAndPlace)
    pp.log = Log()
    edge = 0.56
    pp.robot = FakeRobot(4.20 - edge, 4.55)
    table = Table(center=np.array([edge + 0.37, 0.0, 0.752]), yaw=0.0, size=(0.74, 0.73),
                  normal=np.array([0, 0, 1.0]), near_x=edge)
    obj = lambda x, y: NS(pose=NS(pose=NS(position=NS(x=x, y=y))))  # noqa: E731
    bottle = (0.67, -0.03)            # where the bottle stood when the old choice got stuck
    cup = pp.choose_place_spot(table, [obj(*bottle)], ppm.CORNER_TABLE)

    # classify every candidate the same way choose_place_spot does
    groups = {'free': [], 'edge': [], 'near': [], 'path': [], 'left': []}
    x = edge + ppm.PLACE_INSET
    while x <= min(edge + ppm.PLACE_INSET_MAX, ppm.PLACE_REACH) + 1e-6:
        y = table.center[1] - 0.40
        while y <= table.center[1] + 0.40 + 1e-6:
            pt = np.array([[x, y, 0.752]])
            d = math.hypot(x, y)
            bx, by = bottle
            along = (bx * x + by * y) / d
            across = abs(-bx * y / d + by * x / d)
            if not TabletopPerception._inside_table(pt, table, margin=ppm.PLACE_EDGE_MARGIN)[0]:
                groups['edge'].append((x, y))
            elif math.hypot(bx - x, by - y) < ppm.PLACE_CLEARANCE:
                groups['near'].append((x, y))
            elif 0 < along < d and across < ppm.PLACE_CORRIDOR:
                groups['path'].append((x, y))
            elif y > by - ppm.PLACE_RIGHT_OF:
                groups['left'].append((x, y))
            else:
                groups['free'].append((x, y))
            y += 0.025
        x += 0.03

    fig, ax = plt.subplots(figsize=(6.6, 4.6))
    # robot frame drawn with x up: plot (y, x)
    ax.add_patch(plt.Rectangle((-0.365, edge), 0.73, 0.74, fc='#e8cfa5', ec='#8a6a3a', lw=1.2))
    ax.add_patch(plt.Rectangle((-0.365 + ppm.PLACE_EDGE_MARGIN, edge + ppm.PLACE_EDGE_MARGIN),
                               0.73 - 2 * ppm.PLACE_EDGE_MARGIN, 0.74 - 2 * ppm.PLACE_EDGE_MARGIN,
                               fc='none', ec='#8a6a3a', ls='--', lw=0.8))
    style = {'free': ('o', '#2e7d4f', 'free'), 'edge': ('x', '#999999', 'too close to the table edge'),
             'near': ('s', '#d35400', 'within 14 cm of the bottle'),
             'path': ('^', '#c0392b', 'bottle in the gripper\'s path'),
             'left': ('v', '#7f8c8d', 'not right of the bottle (arm side)')}
    for k, (m, c, lab) in style.items():
        if groups[k]:
            pts = np.array(groups[k])
            ax.scatter(pts[:, 1], pts[:, 0], marker=m, c=c, s=12, label=lab, zorder=3)
    ax.add_patch(plt.Circle((bottle[1], bottle[0]), 0.03, fc='#1e8449', ec='k', zorder=4))
    ax.annotate('bottle', (bottle[1], bottle[0]), (0.30, 0.62), fontsize=7.5,
                arrowprops=dict(arrowstyle='-', color='#1e8449', lw=0.8))
    ax.plot(cup[1], cup[0], '*', ms=16, mfc='#c0392b', mec='k', zorder=5)
    ax.annotate('chosen cup spot', (cup[1], cup[0]), (-0.08, 1.05), fontsize=7.5, color='#c0392b',
                arrowprops=dict(arrowstyle='-', color='#c0392b', lw=0.8))
    ax.plot(0.81 * 0 - 0.05, 0.81, 'D', ms=7, mfc='none', mec='#7d3c98', mew=1.5, zorder=5)
    ax.annotate('old choice (stuck)', (-0.05, 0.81), (0.05, 1.15), fontsize=7.5, color='#7d3c98',
                arrowprops=dict(arrowstyle='-', color='#7d3c98', lw=0.8))
    tx = edge + ppm.PLACE_TARGET_INSET
    ty = -ppm.PLACE_SLOT_OFFSET
    ax.plot(ty, tx, '+', ms=12, mew=2, c='#1f3a5f', zorder=5)
    ax.annotate('preferred slot for the 2nd object: 18 cm behind\nthe edge, 7.5 cm right of the table centre line', (ty, tx),
                (-0.05, 0.47), fontsize=7, color='#1f3a5f',
                arrowprops=dict(arrowstyle='-', color='#1f3a5f', lw=0.8))
    ax.annotate('', (0, edge - 0.02), (0, 0.42), arrowprops=dict(arrowstyle='->', color='#555555'))
    ax.text(0.38, 0.43, 'robot is below, looking up', fontsize=7.5, color='#555555')
    ax.set_xlim(0.45, -0.45)                     # robot's left on the left
    ax.set_ylim(0.40, 1.35)
    ax.set_aspect('equal')
    ax.set_xlabel('robot y [m] (left +)')
    ax.set_ylabel('robot x [m] (ahead)')
    ax.set_title('Place spot choice for the cup (corner table, robot frame)')
    ax.legend(loc='center left', bbox_to_anchor=(1.02, 0.5), fontsize=7, framealpha=0.9)
    fig.tight_layout()
    fig.savefig(os.path.join(IMG, 'fig_place_spots.png'), dpi=170)
    plt.close(fig)


if __name__ == '__main__':
    fig_place_spots()
    fig_map_annotated()
    fig_map_compare()
    fig_ekf()
    print('figures written to', IMG)
