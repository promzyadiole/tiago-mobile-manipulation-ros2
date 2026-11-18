"""
Simple tabletop perception for Tiago's head RGB-D camera (numpy only).

Used by perception_demo.py and, through pick_place.py, by exercise2.py.

Pipeline (all in the robot frame ``base_footprint``):

  1. Grab one point cloud from ``/head_front_camera/depth/rgb/points`` and
     transform it from the camera optical frame to ``base_footprint`` (TF).
  2. Crop to the workspace in front of the robot.
  3. **Table**: RANSAC fit of the dominant horizontal plane -> table height and
     normal. PCA of the plane inliers gives the table's centre, orientation (yaw)
     and size (oriented bounding box).
  4. **Objects**: points above the table plane and inside its outline are
     grouped by 2-D connected-component clustering on a 1 cm grid.
  5. For each cluster:
       * position  : least-squares circle fit of the points (x, y) -> true axis
                     of the cylinder even though only its front half is visible;
                     z = table height + half the object height.
       * size      : radius from the circle fit, height from the highest point.
       * orientation (roll, pitch) from the table normal (object stands upright
                     on the table) and yaw from the handle direction when one is
                     visible (cup), otherwise facing the robot.
       * label     : from the mean colour (green -> bottle, red -> cup), with
                     the height as fallback.
     Together this gives the full 6-DOF pose of every object.
"""
import math
import time
from dataclasses import dataclass, field

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

from .robot import quat_from_rpy

CLOUD_TOPIC = '/head_front_camera/depth/rgb/points'
FRAME = 'base_footprint'


@dataclass
class Table:
    center: np.ndarray        # (x, y, z) of the table-top centre, z = top surface
    yaw: float                # orientation of the table's long axis
    size: tuple               # (length along yaw, width) of the top [m]
    normal: np.ndarray        # unit normal of the table plane
    near_x: float = 0.0       # distance (robot x) to the closest visible table edge
    y_range: tuple = (0.0, 0.0)  # visible extent of the top along robot y
    top_xy: np.ndarray = None    # (sub-sampled) table-top points, robot frame

    def edge_x_at(self, y, band=0.04):
        """x of the near table edge along the line robot-y = y (None if the
        table is not visible there)."""
        sel = np.abs(self.top_xy[:, 1] - y) < band
        if sel.sum() < 10:
            return None
        return float(np.percentile(self.top_xy[sel, 0], 2))

    def supports(self, x, y, margin=0.07, tol=0.025):
        """True if there is table top under (x, y) and `margin` around it in
        the four directions (so an object placed there does not tip over)."""
        for dx, dy in ((0, 0), (margin, 0), (-margin, 0), (0, margin), (0, -margin)):
            d = np.hypot(self.top_xy[:, 0] - (x + dx), self.top_xy[:, 1] - (y + dy))
            if d.min() > tol:
                return False
        return True


@dataclass
class DetectedObject:
    label: str                # 'bottle' / 'cup' / 'unknown'
    pose: PoseStamped         # 6-DOF pose: centre of the object, z axis = up
    radius: float             # estimated radius [m]
    height: float             # estimated height above the table [m]
    color: tuple              # mean (r, g, b) in 0..1
    n_points: int
    extra: dict = field(default_factory=dict)


# ----------------------------------------------------------------------------
# Point cloud conversion
# ----------------------------------------------------------------------------
def cloud_to_numpy(msg):
    """PointCloud2 -> (N,3) float xyz and (N,3) float rgb in 0..1, NaNs removed."""
    offsets = {f.name: f.offset for f in msg.fields}
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(-1, msg.point_step)
    xyz = np.stack([raw[:, offsets[k]:offsets[k] + 4].copy().view(np.float32)[:, 0]
                    for k in ('x', 'y', 'z')], axis=1)
    # 'rgb' is a float32 whose 4 bytes are B, G, R, (A)
    o = offsets['rgb']
    rgb = raw[:, [o + 2, o + 1, o]].astype(np.float32) / 255.0
    ok = np.isfinite(xyz).all(axis=1)
    return xyz[ok].astype(np.float64), rgb[ok]


def transform_matrix(tf):
    """geometry_msgs/TransformStamped -> 4x4 homogeneous matrix."""
    q = tf.transform.rotation
    t = tf.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = [t.x, t.y, t.z]
    return M


# ----------------------------------------------------------------------------
# Geometry algorithms
# ----------------------------------------------------------------------------
def ransac_horizontal_plane(pts, iters=300, thresh=0.008, min_nz=0.95, rng=None):
    """Find the horizontal plane with most inliers.
    Returns (normal, d, inlier_mask) with plane: normal . p + d = 0, normal z > 0."""
    rng = rng or np.random.default_rng(0)
    best = (None, None, np.zeros(len(pts), bool))
    if len(pts) < 3:
        return best
    for _ in range(iters):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        if n[2] < 0:
            n = -n
        if n[2] < min_nz:            # only accept (almost) horizontal planes
            continue
        d = -n.dot(a)
        inl = np.abs(pts @ n + d) < thresh
        if inl.sum() > best[2].sum():
            best = (n, d, inl)
    # Refine the plane with a least-squares fit on the inliers
    n, d, inl = best
    if n is not None and inl.sum() > 10:
        P = pts[inl]
        c = P.mean(axis=0)
        _, _, vt = np.linalg.svd(P - c, full_matrices=False)
        n = vt[2] if vt[2][2] > 0 else -vt[2]
        d = -n.dot(c)
        inl = np.abs(pts @ n + d) < thresh
    return n, d, inl


def largest_cluster_2d(xy, cell=0.02):
    """Mask of the biggest 2-D connected component (used to isolate the table
    top from other planes at the same height)."""
    labels = grid_clusters(xy, cell)
    if labels.max() < 0:
        return np.zeros(len(xy), bool)
    counts = np.bincount(labels[labels >= 0])
    return labels == counts.argmax()


def grid_clusters(xy, cell=0.01):
    """Connected-component clustering of 2-D points on a grid of `cell` metres
    (8-connectivity). Returns a label per point (-1 = none)."""
    if len(xy) == 0:
        return np.zeros(0, int)
    ij = np.floor((xy - xy.min(axis=0)) / cell).astype(int)
    cells = {}
    for k, key in enumerate(map(tuple, ij)):
        cells.setdefault(key, []).append(k)
    labels = np.full(len(xy), -1)
    cell_label = {}
    current = 0
    for start in cells:
        if start in cell_label:
            continue
        stack = [start]
        cell_label[start] = current
        while stack:                       # flood fill over neighbouring cells
            ci, cj = stack.pop()
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    nb = (ci + di, cj + dj)
                    if nb in cells and nb not in cell_label:
                        cell_label[nb] = current
                        stack.append(nb)
        current += 1
    for key, idx in cells.items():
        labels[idx] = cell_label[key]
    return labels


def fit_circle(xy):
    """Algebraic least-squares circle fit (Kasa). Returns (cx, cy, r)."""
    x, y = xy[:, 0], xy[:, 1]
    A = np.stack([x, y, np.ones_like(x)], axis=1)
    b = x * x + y * y
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy = sol[0] / 2, sol[1] / 2
    r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))
    return cx, cy, r


def classify(rgb_mean, height):
    """Label an object from its mean colour (and height as a fallback)."""
    r, g, b = rgb_mean
    if g > 1.4 * r and g > 1.2 * b:
        return 'bottle'      # green
    if r > 1.4 * g and r > 1.4 * b:
        return 'cup'         # red
    return 'bottle' if height > 0.16 else 'cup'


# ----------------------------------------------------------------------------
# ROS wrapper
# ----------------------------------------------------------------------------
class TabletopPerception:
    """Uses the TiagoRobot node (for TF and subscriptions)."""

    def __init__(self, robot):
        self.robot = robot
        self.marker_pub = robot.create_publisher(MarkerArray, '/detected_objects', 10)
        self._latest = None

    def _cb(self, msg):
        self._latest = msg

    def get_cloud(self, timeout=10.0):
        """Wait for a cloud captured *after* this call (camera has settled).
        The subscription only exists while waiting: the 640x480 RGB-D cloud is
        ~10 MB per message at 30 Hz, so a permanent subscription would load the
        ROS middleware for nothing."""
        t0 = self.robot.get_clock().now()
        self._latest = None
        sub = self.robot.create_subscription(PointCloud2, CLOUD_TOPIC, self._cb, 1)
        try:
            start = time.time()
            while time.time() - start < timeout:
                msg = self._latest
                if msg is not None and rclpy.time.Time.from_msg(msg.header.stamp) > t0:
                    return msg
                time.sleep(0.05)
        finally:
            self.robot.destroy_subscription(sub)
        raise TimeoutError('No point cloud received')

    def detect(self, max_range=2.0):
        """Run the full pipeline. Returns (Table or None, [DetectedObject])."""
        msg = self.get_cloud()
        tf = self.robot.tf_buffer.lookup_transform(
            FRAME, msg.header.frame_id, rclpy.time.Time.from_msg(msg.header.stamp),
            timeout=rclpy.duration.Duration(seconds=2.0))
        xyz, rgb = cloud_to_numpy(msg)
        M = transform_matrix(tf)
        pts = xyz @ M[:3, :3].T + M[:3, 3]

        # 2) Crop: in front of the robot, between knee and head height
        keep = ((pts[:, 0] > 0.2) & (pts[:, 0] < max_range) & (np.abs(pts[:, 1]) < 1.2) &
                (pts[:, 2] > 0.3) & (pts[:, 2] < 1.5))
        pts, rgb = pts[keep], rgb[keep]
        # Self-filter: remove points on the robot's own arm / gripper (and on
        # an object held in the gripper), so they are not detected as objects.
        keep = self._not_on_robot(pts)
        pts, rgb = pts[keep], rgb[keep]
        # Down-sample for speed (every 3rd point is plenty for RANSAC)
        sample = pts[::3]

        # 3) Table plane
        n, d, _ = ransac_horizontal_plane(sample)
        if n is None:
            self.robot.get_logger().warn('Perception: no table plane found')
            return None, []
        dist = pts @ n + d                       # signed height above the plane
        on_plane = np.abs(dist) < 0.01
        top = largest_cluster_2d(pts[on_plane, :2])
        top_pts = pts[on_plane][top]
        table = self._table_from_points(top_pts, n)
        # Sanity check: a table top is 0.6-0.9 m high and at least 25 cm wide
        # in both directions (rejects shelves, furniture tops, small patches).
        if not (0.6 < table.center[2] < 0.9) or min(table.size) < 0.25:
            self.robot.get_logger().warn(
                f'Perception: plane at z={table.center[2]:.2f} m, size {table.size[0]:.2f}x'
                f'{table.size[1]:.2f} m is not a table')
            return None, []

        # 4) Objects: above the plane, inside the (slightly shrunk) table outline
        above = (dist > 0.012) & (dist < 0.45) & self._inside_table(pts, table, margin=0.01)
        obj_pts, obj_rgb, obj_h = pts[above], rgb[above], dist[above]
        labels = grid_clusters(obj_pts[:, :2], cell=0.01)

        objects = []
        for lab in range(labels.max() + 1 if len(labels) else 0):
            m = labels == lab
            if m.sum() < 80:                     # ignore noise / tiny blobs
                continue
            if np.percentile(obj_h[m], 2) > 0.03:
                # Lowest points float above the table: not standing on it
                # (e.g. an object held in the gripper above the table).
                continue
            objects.append(self._make_object(obj_pts[m], obj_rgb[m], obj_h[m], table, msg))
        self.publish_markers(table, objects)
        names = ', '.join(f'{o.label}@({o.pose.pose.position.x:.2f},{o.pose.pose.position.y:.2f})'
                          for o in objects)
        self.robot.get_logger().info(
            f'Perception: table top z={table.center[2]:.3f} m, size {table.size[0]:.2f}x'
            f'{table.size[1]:.2f} m; objects: {names or "none"}')
        return table, objects

    # ------------------------------------------------------------- helpers
    # Arm chain used by the self-filter (points within ARM_RADIUS of the line
    # segments joining these frames are removed) and the held-object region.
    ARM_CHAIN = ['arm_3_link', 'arm_4_link', 'arm_5_link', 'arm_6_link', 'arm_7_link',
                 'arm_tool_link', 'gripper_grasping_frame']
    ARM_RADIUS = 0.09
    HAND_RADIUS = 0.14       # around the fingertips: gripper + held object

    def _not_on_robot(self, pts):
        keep = np.ones(len(pts), bool)
        try:
            chain = []
            for f in self.ARM_CHAIN:
                t = self.robot.lookup(FRAME, f, timeout=1.0).transform.translation
                chain.append(np.array([t.x, t.y, t.z]))
        except Exception as e:  # noqa: BLE001 - TF not available: no filtering
            self.robot.get_logger().warn(f'Self-filter disabled: {e}')
            return keep
        for a, b in zip(chain[:-1], chain[1:]):
            ab = b - a
            t = np.clip(((pts - a) @ ab) / max(ab @ ab, 1e-9), 0.0, 1.0)
            closest = a + t[:, None] * ab
            keep &= np.linalg.norm(pts - closest, axis=1) > self.ARM_RADIUS
        keep &= np.linalg.norm(pts - chain[-1], axis=1) > self.HAND_RADIUS
        return keep

    @staticmethod
    def _table_from_points(top_pts, normal):
        """Oriented bounding box of the table-top points via PCA."""
        xy = top_pts[:, :2]
        c = xy.mean(axis=0)
        _, _, vt = np.linalg.svd(xy - c, full_matrices=False)
        axis = vt[0]
        yaw = math.atan2(axis[1], axis[0])
        R = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
        local = (xy - c) @ R.T
        lo, hi = local.min(axis=0), local.max(axis=0)
        mid = (lo + hi) / 2
        center_xy = c + mid @ R          # centre of the box (not of the points)
        z = float(top_pts[:, 2].mean())
        # The camera often sees only part of the table, so the box size is not
        # reliable; the nearest edge (what the robot needs) is measured directly.
        near_x = float(np.percentile(top_pts[:, 0], 1))
        y_range = (float(np.percentile(top_pts[:, 1], 2)), float(np.percentile(top_pts[:, 1], 98)))
        return Table(center=np.array([center_xy[0], center_xy[1], z]), yaw=yaw,
                     size=(float(hi[0] - lo[0]), float(hi[1] - lo[1])), normal=normal,
                     near_x=near_x, y_range=y_range, top_xy=xy[::5].copy())

    @staticmethod
    def _inside_table(pts, table, margin=0.0):
        c, yaw = table.center, table.yaw
        dx, dy = pts[:, 0] - c[0], pts[:, 1] - c[1]
        u = dx * math.cos(yaw) + dy * math.sin(yaw)
        v = -dx * math.sin(yaw) + dy * math.cos(yaw)
        return ((np.abs(u) < table.size[0] / 2 - margin) &
                (np.abs(v) < table.size[1] / 2 - margin))

    def _make_object(self, pts, rgb, h, table, msg):
        height = float(np.percentile(h, 99))
        # Fit the circle on the main body only (ignore the top 20 %, e.g. a
        # bottle neck) to get the cylinder axis and radius.
        body = h < 0.8 * height
        body_xy = pts[body, :2] if body.sum() > 20 else pts[:, :2]
        cx, cy, _ = fit_circle(body_xy)
        # Radius from the silhouette: the width of the body points measured
        # perpendicular to the camera's line of sight is the full diameter
        # (more robust than the circle-fit radius when only the front is seen).
        ray = np.array([cx, cy]) / max(math.hypot(cx, cy), 1e-6)
        perp = np.array([-ray[1], ray[0]])
        proj = body_xy @ perp
        r = float(np.percentile(proj, 98) - np.percentile(proj, 2)) / 2
        rgb_mean = tuple(float(v) for v in rgb.mean(axis=0))
        label = classify(rgb_mean, height)

        # Yaw: direction of points sticking out of the circle (e.g. cup handle);
        # if there are none the object is rotationally symmetric -> face robot.
        radial = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy)
        outliers = pts[radial > r * 1.25]
        if len(outliers) > 15:
            yaw = math.atan2(outliers[:, 1].mean() - cy, outliers[:, 0].mean() - cx)
        else:
            yaw = math.atan2(cy, cx)
        # Roll / pitch: object stands on the table, so its z axis = table normal
        nx, ny, nz = table.normal
        roll = math.atan2(-ny, nz)
        pitch = math.atan2(nx, math.hypot(ny, nz))

        pose = PoseStamped()
        pose.header.frame_id = FRAME
        pose.header.stamp = msg.header.stamp
        pose.pose.position.x, pose.pose.position.y = float(cx), float(cy)
        pose.pose.position.z = float(table.center[2] + height / 2)
        pose.pose.orientation = quat_from_rpy(roll, pitch, yaw)
        return DetectedObject(label=label, pose=pose, radius=float(r), height=height,
                              color=rgb_mean, n_points=int(len(pts)))

    def publish_markers(self, table, objects):
        """Visualise the detections in RViz (topic /detected_objects)."""
        arr = MarkerArray()
        stamp = self.robot.now()
        m = Marker(action=Marker.DELETEALL)   # DELETEALL is an action, not a type
        arr.markers.append(m)
        if table is not None:
            t = Marker(type=Marker.CUBE, ns='table', id=0, action=Marker.ADD)
            t.header.frame_id, t.header.stamp = FRAME, stamp
            t.pose.position.x, t.pose.position.y = table.center[0], table.center[1]
            t.pose.position.z = table.center[2] - 0.01
            t.pose.orientation = quat_from_rpy(0, 0, table.yaw)
            t.scale.x, t.scale.y, t.scale.z = table.size[0], table.size[1], 0.02
            t.color.r, t.color.g, t.color.b, t.color.a = 0.6, 0.4, 0.2, 0.5
            arr.markers.append(t)
        for i, o in enumerate(objects):
            c = Marker(type=Marker.CYLINDER, ns='objects', id=i, action=Marker.ADD)
            c.header.frame_id, c.header.stamp = FRAME, stamp
            c.pose = o.pose.pose
            c.scale.x = c.scale.y = 2 * o.radius
            c.scale.z = o.height
            c.color.r, c.color.g, c.color.b = o.color
            c.color.a = 0.8
            arr.markers.append(c)
            txt = Marker(type=Marker.TEXT_VIEW_FACING, ns='labels', id=i, action=Marker.ADD)
            txt.header.frame_id, txt.header.stamp = FRAME, stamp
            txt.pose.position.x = o.pose.pose.position.x
            txt.pose.position.y = o.pose.pose.position.y
            txt.pose.position.z = o.pose.pose.position.z + o.height / 2 + 0.08
            txt.scale.z = 0.05
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            txt.text = o.label
            arr.markers.append(txt)
        self.marker_pub.publish(arr)
