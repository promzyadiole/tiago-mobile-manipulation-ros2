#!/usr/bin/env python3
"""Joint-motion figures for my_tiago_documentation.html (chapter "How each joint moves").

For every movable joint of the robot it draws the real robot meshes in 3D:
  - the PARENT link in blue,
  - the CHILD link (and everything carried by it) in orange,
  - the child again, transparent, after a positive move of the joint ("ghost"),
  - the joint axis as a black arrow, and the direction of a POSITIVE joint
    value: a curved arrow (right-hand rule) for rotation, a straight arrow
    for translation,
  - the joint frame (x red, y green, z blue).

Forward kinematics and meshes come straight from the URDF, so the figures
show exactly what robot_state_publisher computes.

Run:  python3 make_joint_figures.py
Inputs: ../../src/my_tiago_description/urdf/my_tiago_expanded_reference.urdf,
        STL meshes in ../../src/pal_description_overlay
Output: images/joint_*.png
"""
import math
import os
import struct
import xml.etree.ElementTree as ET

import matplotlib
import mpl_toolkits
# Ubuntu's old python3-matplotlib also installs an mpl_toolkits package; make
# sure the 3D toolkit that belongs to the matplotlib in use is imported
# (this has to happen before pyplot is imported).
mpl_toolkits.__path__ = [os.path.join(os.path.dirname(os.path.dirname(matplotlib.__file__)),
                                      'mpl_toolkits')]
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import FancyArrowPatch  # noqa: E402
from mpl_toolkits.mplot3d import proj3d  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
IMG = os.path.join(HERE, 'images')
WS = os.path.join(HERE, '..', '..')
URDF = os.path.join(WS, 'src', 'my_tiago_description', 'urdf', 'my_tiago_expanded_reference.urdf')
PKG_ROOT = os.path.join(WS, 'src', 'pal_description_overlay')

plt.rcParams.update({'font.family': 'DejaVu Sans', 'font.size': 9,
                     'axes.titleweight': 'bold'})

PARENT_C = np.array([0.20, 0.42, 0.75])
CHILD_C = np.array([0.93, 0.50, 0.13])
OTHER_C = np.array([0.78, 0.78, 0.78])

# Pose used for all figures: arm stretched out to the side and forward so that
# every link can be seen (all values inside the joint limits).
DISPLAY_Q = {'torso_lift_joint': 0.15,
             'arm_1_joint': 0.20, 'arm_2_joint': 0.0, 'arm_3_joint': -1.57,
             'arm_4_joint': 1.0, 'arm_5_joint': 0.0, 'arm_6_joint': 0.0,
             'arm_7_joint': 0.0,
             'gripper_left_finger_joint': 0.03, 'gripper_right_finger_joint': 0.03,
             'head_1_joint': 0.0, 'head_2_joint': 0.0}


# ---------------------------------------------------------------- URDF + FK
def rpy_matrix(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def transform(xyz, rpy):
    t = np.eye(4)
    t[:3, :3] = rpy_matrix(*rpy)
    t[:3, 3] = xyz
    return t


def axis_angle(axis, a):
    k = np.asarray(axis, float) / np.linalg.norm(axis)
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    t = np.eye(4)
    t[:3, :3] = np.eye(3) + math.sin(a) * kx + (1 - math.cos(a)) * kx @ kx
    return t


def floats(s, default):
    return [float(v) for v in s.split()] if s else default


class Robot:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.joints = {}
        self.children = {}
        for j in root.findall('joint'):
            o = j.find('origin')
            a = j.find('axis')
            lim = j.find('limit')
            jd = dict(name=j.get('name'), type=j.get('type'),
                      parent=j.find('parent').get('link'), child=j.find('child').get('link'),
                      origin=transform(floats(o.get('xyz') if o is not None else None, [0, 0, 0]),
                                       floats(o.get('rpy') if o is not None else None, [0, 0, 0])),
                      axis=np.array(floats(a.get('xyz') if a is not None else None, [1, 0, 0])),
                      lower=float(lim.get('lower', 0)) if lim is not None else 0.0,
                      upper=float(lim.get('upper', 0)) if lim is not None else 0.0)
            self.joints[jd['name']] = jd
            self.children.setdefault(jd['parent'], []).append(jd)
        self.visuals = {}
        for link in root.findall('link'):
            for v in link.findall('visual'):
                o = v.find('origin')
                vt = transform(floats(o.get('xyz') if o is not None else None, [0, 0, 0]),
                               floats(o.get('rpy') if o is not None else None, [0, 0, 0]))
                g = v.find('geometry')[0]
                tris = None
                if g.tag == 'mesh' and g.get('filename', '').endswith('.stl'):
                    tris = load_stl(g.get('filename'))
                    tris = tris * np.array(floats(g.get('scale'), [1, 1, 1]))
                elif g.tag == 'box':
                    tris = box_tris(floats(g.get('size'), [0.1] * 3))
                elif g.tag == 'cylinder' and float(g.get('radius')) > 0.015:
                    tris = cylinder_tris(float(g.get('radius')), float(g.get('length')))
                if tris is not None:
                    self.visuals.setdefault(link.get('name'), []).append((vt, tris))
        child_links = {j['child'] for j in self.joints.values()}
        self.root = next(lk.get('name') for lk in root.findall('link')
                         if lk.get('name') not in child_links)

    def fk(self, q):
        """Pose (4x4, base_footprint frame) of every link and every joint frame."""
        links, jframes = {self.root: np.eye(4)}, {}
        stack = [self.root]
        while stack:
            p = stack.pop()
            for j in self.children.get(p, []):
                jf = links[p] @ j['origin']               # joint frame (before motion)
                jframes[j['name']] = jf
                v = q.get(j['name'], 0.0)
                if j['type'] in ('revolute', 'continuous'):
                    m = axis_angle(j['axis'], v)
                elif j['type'] == 'prismatic':
                    m = np.eye(4)
                    m[:3, 3] = j['axis'] * v
                else:
                    m = np.eye(4)
                links[j['child']] = jf @ m
                stack.append(j['child'])
        return links, jframes

    def rigid_body(self, link):
        """`link` plus every link joined to it by fixed joints: they move as one
        body (e.g. suspension_left_link has no mesh, but is fixed to base_link)."""
        parent_of = {j['child']: j for j in self.joints.values()}
        while link in parent_of and parent_of[link]['type'] == 'fixed':
            link = parent_of[link]['parent']
        out, stack = set(), [link]
        while stack:
            lk = stack.pop()
            out.add(lk)
            stack += [j['child'] for j in self.children.get(lk, []) if j['type'] == 'fixed']
        return out

    def subtree(self, link):
        out, stack = set(), [link]
        while stack:
            lk = stack.pop()
            out.add(lk)
            stack += [j['child'] for j in self.children.get(lk, [])]
        return out


def resolve(url):
    pkg, rel = url[len('package://'):].split('/', 1)
    return os.path.join(PKG_ROOT, pkg, rel)


_STL_CACHE = {}


def load_stl(url):
    if url not in _STL_CACHE:
        with open(resolve(url), 'rb') as f:
            data = f.read()
        n = struct.unpack('<I', data[80:84])[0]
        rec = np.frombuffer(data[84:84 + 50 * n], dtype=np.dtype([
            ('n', '<f4', 3), ('v', '<f4', (3, 3)), ('a', '<u2')]))
        _STL_CACHE[url] = rec['v'].astype(float)
    return _STL_CACHE[url]


def box_tris(size):
    sx, sy, sz = [s / 2 for s in size]
    c = np.array([[x, y, z] for x in (-sx, sx) for y in (-sy, sy) for z in (-sz, sz)])
    faces = [(0, 1, 3, 2), (4, 6, 7, 5), (0, 4, 5, 1), (2, 3, 7, 6), (0, 2, 6, 4), (1, 5, 7, 3)]
    tris = []
    for a, b, cc, d in faces:
        tris += [[c[a], c[b], c[cc]], [c[a], c[cc], c[d]]]
    return np.array(tris)


def cylinder_tris(r, h, n=24):
    t = np.linspace(0, 2 * math.pi, n + 1)
    tris = []
    for i in range(n):
        p0 = [r * math.cos(t[i]), r * math.sin(t[i])]
        p1 = [r * math.cos(t[i + 1]), r * math.sin(t[i + 1])]
        a, b = [*p0, -h / 2], [*p1, -h / 2]
        c, d = [*p1, h / 2], [*p0, h / 2]
        tris += [[a, b, c], [a, c, d], [[0, 0, h / 2], d, c], [[0, 0, -h / 2], b, a]]
    return np.array(tris)


# ----------------------------------------------------------------- drawing
class Arrow3D(FancyArrowPatch):
    def __init__(self, p0, p1, **kw):
        super().__init__((0, 0), (0, 0), **kw)
        self._p = (p0, p1)

    def do_3d_projection(self, renderer=None):
        (x0, y0, z0), (x1, y1, z1) = self._p
        xs, ys, zs = proj3d.proj_transform([x0, x1], [y0, y1], [z0, z1], self.axes.M)
        self.set_positions((xs[0], ys[0]), (xs[1], ys[1]))
        return min(zs)


def arrow(ax, p0, p1, color, lw=2.5, scale=16, z=20):
    a = Arrow3D(p0, p1, arrowstyle='-|>', mutation_scale=scale, lw=lw, color=color, zorder=z)
    ax.add_artist(a)


def shaded(tris, color, alpha, light=np.array([0.4, -0.5, 0.8])):
    n = np.cross(tris[:, 1] - tris[:, 0], tris[:, 2] - tris[:, 0])
    n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
    k = 0.45 + 0.55 * np.abs(n @ (light / np.linalg.norm(light)))
    rgb = np.clip(color[None, :] * k[:, None], 0, 1)
    return np.hstack([rgb, np.full((len(tris), 1), alpha)])


def world_tris(robot, links, names):
    out = []
    for lk in names:
        for vt, tris in robot.visuals.get(lk, []):
            t = links[lk] @ vt
            out.append(tris @ t[:3, :3].T + t[:3, 3])
    return np.concatenate(out) if out else np.zeros((0, 3, 3))


def clip(tris, colours, box):
    """Keep the triangles whose centre is inside the view box (matplotlib does
    not clip 3D polygons at the axes limits)."""
    if box is None:
        return tris, colours
    lo, hi = box
    c = tris.mean(axis=1)
    keep = np.all((c >= lo) & (c <= hi), axis=1)
    return tris[keep], colours[keep]


def draw_robot(ax, robot, q, colours, ghost_q=None, ghost_links=(), box=None):
    """colours: {link: (rgb, alpha)}; links not listed are light grey."""
    links, _ = robot.fk(q)
    allt, allc = [], []
    for lk in robot.visuals:
        rgb, alpha = colours.get(lk, (OTHER_C, 0.18))
        t = world_tris(robot, links, [lk])
        allt.append(t)
        allc.append(shaded(t, rgb, alpha))
    if ghost_q is not None:
        glinks, _ = robot.fk(ghost_q)
        t = world_tris(robot, glinks, ghost_links)
        if len(t):
            allt.append(t)
            allc.append(shaded(t, CHILD_C, 0.22))
    tris, cols = clip(np.concatenate(allt), np.concatenate(allc), box)
    pc = Poly3DCollection(tris, facecolors=cols,
                          edgecolors='none', linewidths=0)
    ax.add_collection3d(pc)


def perpendicular(a):
    u = np.cross(a, [0, 0, 1]) if abs(a[2]) < 0.9 else np.cross(a, [1, 0, 0])
    return u / np.linalg.norm(u)


def rotation_arrow(ax, p, a, r, color='#c0392b'):
    """Curved arrow around axis a through p: the positive (right-hand) sense."""
    u = perpendicular(a)
    v = np.cross(a, u)
    t = np.linspace(0, 1.45 * math.pi, 60)
    pts = p + r * (np.outer(np.cos(t), u) + np.outer(np.sin(t), v))
    ax.plot(pts[:-2, 0], pts[:-2, 1], pts[:-2, 2], color=color, lw=2.6, zorder=21)
    arrow(ax, pts[-4], pts[-1], color, lw=2.6, scale=20, z=22)


def frame_triad(ax, t, size):
    for i, c in enumerate(['#d62728', '#2ca02c', '#1f77b4']):
        arrow(ax, t[:3, 3], t[:3, 3] + size * t[:3, i], c, lw=1.6, scale=9, z=19)


def set_view(ax, centre, half, elev, azim):
    c = np.asarray(centre)
    ax.set_xlim(c[0] - half, c[0] + half)
    ax.set_ylim(c[1] - half, c[1] + half)
    ax.set_zlim(c[2] - half, c[2] + half)
    ax.set_box_aspect((1, 1, 1))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()
    # compass: the robot's own directions (base_footprint axes)
    o = c + np.array([-0.85, -0.85, -0.85]) * half
    s = 0.22 * half
    for d, label in [((1, 0, 0), 'forward'), ((0, 1, 0), 'left'), ((0, 0, 1), 'up')]:
        d = np.array(d, float)
        arrow(ax, o, o + s * d, '#555555', lw=1.3, scale=8, z=18)
        ax.text(*(o + 1.25 * s * d), label, fontsize=6.5, color='#555555', zorder=30)


def view_along(axis_w):
    """(elev, azim) of a viewer near the TIP of the axis, tilted ~35° so the
    figure stays 3D: from there a positive rotation looks counter-clockwise."""
    a = axis_w / np.linalg.norm(axis_w)
    side = np.array([0.0, 0.0, 1.0]) if abs(a[2]) < 0.9 else np.array([0.75, -0.65, 0.0])
    side -= (side @ a) * a
    v = a + 0.7 * side / np.linalg.norm(side) + 0.25 * np.cross(a, side)
    v /= np.linalg.norm(v)
    return math.degrees(math.asin(v[2])), math.degrees(math.atan2(v[1], v[0]))


def direction_words(v):
    """Name the base_footprint direction closest to unit vector v."""
    names = {0: ('forward (+x)', 'backward (-x)'), 1: ('left (+y)', 'right (-y)'),
             2: ('up (+z)', 'down (-z)')}
    i = int(np.argmax(np.abs(v)))
    return names[i][0 if v[i] > 0 else 1]


# ------------------------------------------------------------- the figures
# name: (positive move shown, view centre, half-size of the view, elev, azim)
VIEWS = {
    'wheel_left_joint': (0.9, 'wheel', 0.33, None, None),
    'wheel_right_joint': (0.9, 'wheel', 0.33, None, None),
    'torso_lift_joint': (0.20, 'joint', 0.60, 12, -50),
    'arm_1_joint': (0.6, 'joint', 0.42, None, None),
    'arm_2_joint': (0.6, 'joint', 0.42, None, None),
    'arm_3_joint': (0.7, 'joint', 0.42, None, None),
    'arm_4_joint': (0.6, 'joint', 0.40, None, None),
    'arm_5_joint': (0.9, 'joint', 0.30, None, None),
    'arm_6_joint': (0.7, 'joint', 0.30, None, None),
    'arm_7_joint': (0.9, 'joint', 0.26, None, None),
    'gripper_left_finger_joint': (0.015, 'joint', 0.13, 30, -60),
    'gripper_right_finger_joint': (0.015, 'joint', 0.13, 30, -60),
    'head_1_joint': (0.6, 'joint', 0.30, None, None),
    'head_2_joint': (0.5, 'joint', 0.28, None, None),
}


def joint_figure(robot, name):
    j = robot.joints[name]
    delta, centre_kind, half, elev, azim = VIEWS[name]
    q = dict(DISPLAY_Q)
    if name.startswith('gripper'):
        q['gripper_left_finger_joint'] = q['gripper_right_finger_joint'] = 0.01
    links, jframes = robot.fk(q)
    jf = jframes[name]
    axis_w = jf[:3, :3] @ (j['axis'] / np.linalg.norm(j['axis']))
    p = jf[:3, 3]

    child_set = robot.subtree(j['child'])
    colours = {lk: (CHILD_C, 0.95) for lk in child_set}
    for lk in robot.rigid_body(j['parent']):
        colours[lk] = (PARENT_C, 0.55)
    ghost = dict(q)
    ghost[name] = q.get(name, 0.0) + delta
    centre = links[j['child']][:3, 3] if centre_kind == 'wheel' else p
    if elev is None:
        elev, azim = view_along(axis_w)
    box = (centre - half, centre + half)
    fig = plt.figure(figsize=(4.6, 4.4))
    ax = fig.add_subplot(projection='3d', computed_zorder=False)
    draw_robot(ax, robot, q, colours, ghost_q=ghost, ghost_links=child_set, box=box)

    L = half * 0.75
    arrow(ax, p - 0.35 * L * axis_w, p + L * axis_w, 'black', lw=2.0, scale=14, z=20)
    ax.text(*(p + 1.08 * L * axis_w), 'axis', fontsize=8, fontweight='bold', zorder=30)
    frame_triad(ax, jf, half * 0.28)
    if j['type'] in ('revolute', 'continuous'):
        rotation_arrow(ax, p + 0.55 * L * axis_w, axis_w, half * 0.30)
        motion = f'ghost: +{math.degrees(delta):.0f}°'
    else:
        off = perpendicular(axis_w) * half * 0.35
        arrow(ax, p + off, p + off + half * 0.55 * axis_w, '#c0392b', lw=4, scale=22, z=22)
        motion = f'ghost: +{delta * 100:g} cm'

    set_view(ax, centre, half, elev, azim)
    kind = {'revolute': 'revolute', 'continuous': 'continuous (no limits)',
            'prismatic': 'prismatic'}[j['type']]
    ax.set_title(f'{name}  ·  {kind}', fontsize=10, pad=2)
    # parent / child / axis text goes in the figure caption of the PDF
    fig.subplots_adjust(left=0, right=1, top=0.95, bottom=0)
    out = os.path.join(IMG, f'joint_{name}.png')
    fig.savefig(out, dpi=150, bbox_inches='tight', pad_inches=0.05)
    plt.close(fig)
    return motion, axis_w


def overview_figure(robot):
    """The whole robot with every movable joint axis."""
    q = dict(DISPLAY_Q)
    links, jframes = robot.fk(q)
    fig = plt.figure(figsize=(6.6, 6.0))
    ax = fig.add_subplot(projection='3d', computed_zorder=False)
    colours = {lk: (np.array([0.70, 0.74, 0.80]), 0.55) for lk in robot.visuals}
    draw_robot(ax, robot, q, colours)
    for name in VIEWS:
        j = robot.joints[name]
        jf = jframes[name]
        a = jf[:3, :3] @ (j['axis'] / np.linalg.norm(j['axis']))
        c = '#c0392b' if j['type'] != 'prismatic' else '#1e8449'
        arrow(ax, jf[:3, 3], jf[:3, 3] + 0.16 * a, c, lw=2.0, scale=11, z=20)
        ax.text(*(jf[:3, 3] + 0.19 * a), name.replace('_joint', ''), fontsize=6.3, zorder=30)
    set_view(ax, [0.22, -0.18, 0.66], 0.56, 18, -65)
    ax.set_title('All 14 movable joints: red = rotation axis, green = translation axis', fontsize=9.5)
    fig.subplots_adjust(left=0, right=1, top=0.96, bottom=0.0)
    fig.savefig(os.path.join(IMG, 'joint_overview.png'), dpi=150, bbox_inches='tight', pad_inches=0.05)
    plt.close(fig)


if __name__ == '__main__':
    robot = Robot(URDF)
    overview_figure(robot)
    for name in VIEWS:
        motion, a = joint_figure(robot, name)
        print(f'{name:28s} {motion:14s} axis in base_footprint at the display pose: '
              f'({a[0]:+.2f}, {a[1]:+.2f}, {a[2]:+.2f}) = {direction_words(a)}')
    print('figures written to', IMG)
