"""Pick and place with perception, MoveIt and Nav2.

Used by pick_demo.py (pick one object and put it back) and exercise2.py (move
both objects to the corner table). The values below were tuned in Gazebo; the
comment next to each one says what went wrong without it.

Pick, for one object:
  look at the table -> detect table + objects (6-DOF poses) -> reposition the
  base if the object is not front-right -> MoveIt scene from the detections ->
  pre-grasp (MoveIt plan) -> straight approach (Cartesian) -> close gripper ->
  attach (Gazebo + MoveIt) -> lift (Cartesian) -> check with the camera that
  the object has left the table.

Place:
  look at the target table -> choose a free spot on the measured table top ->
  pre-place 10 cm above it -> straight down -> detach -> open -> retreat up,
  then back -> check with the camera that the object is on the table.
"""
import math

import numpy as np

from my_tiago_tasks.perception import TabletopPerception
from my_tiago_tasks.robot import TOOL_TO_GRASP, make_pose, yaw_from_quat

# ---------------------------------------------------------------------------
# Known map locations (map frame). Only where the tables are is assumed; the
# objects themselves are found by perception.
# ---------------------------------------------------------------------------
MIDDLE_TABLE = {'name': 'middle table', 'center': (2.25, 2.25), 'size': (0.8, 1.2),
                'approach': (1.25, 2.25, 0.0)}
# Corner table edge at x = 4.20: stopping 0.5 m before it keeps the place spots
# well inside the arm's reach (at 0.6 m the arm was nearly stretched out and
# the straight moves after releasing needed IK flips).
CORNER_TABLE = {'name': 'corner table', 'center': (4.55, 4.55), 'size': (0.7, 0.7),
                'approach': (3.70, 4.55, 0.0)}
TABLE_HEIGHT_GUESS = 0.75       # only used to aim the head before perceiving

OBJECT_ORDER = ['bottle', 'cup']

# Place spot search (see choose_place_spot), robot frame
PLACE_CLEARANCE = 0.14          # min distance to objects already there [m]
PLACE_INSET = 0.12              # spots from this far behind the table's near edge ...
PLACE_INSET_MAX = 0.30          # ... to this far [m]
PLACE_REACH = 0.85              # no spot further ahead than this (arm reach) [m]
PLACE_EDGE_MARGIN = 0.10        # free table all around the spot [m]
PLACE_TARGET_INSET = 0.18       # preferred distance behind the near edge [m]
# Side to side (robot y, left +). TIAGo's arm is mounted on its right, so the
# hand reaches every spot from the right. The first object goes slightly left
# of centre, each further one to the RIGHT of the objects already there:
# placing the cup to the left of the bottle made the arm reach across the
# standing bottle and knock it over.
# Preferred spots are fixed on the table (map frame, known table position), so
# they do not depend on how the robot stopped: the first object goes
# PLACE_SLOT_OFFSET left of the table's centre line, the second the same
# distance right of it. Measured from the robot instead, a 20 deg heading
# error put the bottle 7 cm from the table's side edge.
PLACE_SLOT_OFFSET = 0.075       # [m], slots 0.15 m apart
PLACE_RIGHT_OF = 0.10           # a spot must be at least this far right of every object [m]
PLACE_CORRIDOR = 0.10           # half-width of the gripper's approach path [m]

# Where the object should be relative to the base for a grasp: in front and
# slightly to the RIGHT, on the side of Tiago's arm. Reaching across the body
# to an object on the left shifted the robot's weight so far that it tipped
# onto one drive wheel in Gazebo and then spun around uncontrollably.
GRASP_REL_X = 0.70              # [m] ahead of the base
GRASP_REL_Y = -0.10             # [m] to the right
GRASP_REL_TOL = 0.12            # re-position the base if further off than this

# Grasp parameters
PREGRASP_DIST = 0.15            # stand-off before the straight approach [m]
LIFT_HEIGHT = 0.10              # lift after grasping [m]
# gripper_grasping_frame sits near the base of the finger pads; stopping this
# far short of the object axis puts the object mid-way along the pads instead
# of against the palm (which pushed light objects away).
GRASP_BACKOFF = 0.025
# Finger target relative to the object surface [m]: -0.004 = stop 4 mm short.
# In Gazebo nothing may touch or overlap the object while it is welded to the
# gripper: the solver resolves overlaps with violent impulses that jolted the
# robot and flung the light cup away. The weld holds the object.
GRIPPER_SQUEEZE = -0.004
# Speed scaling for planned arm moves while an object is held. A planned path
# can swing the outstretched arm in a wide arc; at the default speed (0.4) the
# reaction spun the whole robot once around on the spot while it held the cup.
HOLDING_VELOCITY = 0.15
# Release height of the object bottom above the table [m]. Small, so the
# object is not dropped (3 cm was a visible drop), but not zero: pushing a
# welded object into the table top has the same effect as squeezing it. The
# object is welded where it was grasped, so its bottom height is known.
RELEASE_GAP = 0.015
# After releasing, the open fingers rise this far above the object's top
# before the gripper pulls back [m].
RETREAT_CLEARANCE = 0.05


class PickAndPlace:
    def __init__(self, robot):
        self.robot = robot
        self.perception = TabletopPerception(robot)
        self.log = robot.get_logger()
        self.last_table = None
        self.holding = None          # label of the object welded to the gripper

    # ------------------------------------------------------------ helpers
    def tool_pose(self, point, yaw, back=0.0, dz=0.0):
        """arm_tool_link pose (base_footprint) that puts the gripper's grasping
        frame at `point` (x, y, z), approaching horizontally along `yaw`, with
        the fingers closing horizontally (roll = 90 deg). `back` moves the
        gripper backwards along the approach direction."""
        d = TOOL_TO_GRASP + back
        x = point[0] - d * math.cos(yaw)
        y = point[1] - d * math.sin(yaw)
        return make_pose('base_footprint', x, y, point[2] + dz, roll=math.pi / 2, yaw=yaw)

    def perceive(self, table_map_xy):
        """Look at a table (known map position) and run perception."""
        tx, ty = self.robot.map_to_base(*table_map_xy)
        # aim at the near half of the table, where the objects are reachable
        self.robot.look_at(0.5 * tx + 0.25, 0.5 * ty, TABLE_HEIGHT_GUESS)
        table, objects = self.perception.detect()
        if table is None:
            # retry once looking a bit lower, straight ahead
            self.robot.move_head(0.0, -0.75)
            table, objects = self.perception.detect()
        return table, objects

    def build_scene(self, table, objects, skip=None):
        """Collision objects for MoveIt from the perception result."""
        r = self.robot
        depth = 0.8
        z = table.center[2]
        width = max(table.size) + 0.3
        r.add_box('table', make_pose('base_footprint', table.near_x + depth / 2,
                                     table.center[1], z / 2, yaw=0.0),
                  (depth, width, z))
        for o in objects:
            if o.label == skip:
                continue
            p = o.pose.pose.position
            r.add_cylinder(o.label, make_pose('base_footprint', p.x, p.y, p.z),
                           o.height, o.radius + 0.01)

    def drive_to(self, pose):
        """Navigate; on failure back up and try once more."""
        if self.robot.go_to(*pose):
            return True
        self.log.warn('Navigation failed, backing up and retrying once')
        self.robot.back_up(0.25)
        return self.robot.go_to(*pose)

    def stow_arm(self):
        # Head up first: after looking down at a table, the tilted head is in
        # the elbow's way (MoveIt then sees arm_4_link touching head_2_link
        # and refuses to plan).
        self.robot.move_head(0.0, 0.0)
        if not self.robot.stow_arm():
            self.log.warn('Could not stow the arm')

    # --------------------------------------------------------------- pick
    def pick(self, label, table_info=MIDDLE_TABLE):
        """Pick `label` from the table. Returns (DetectedObject, grasp height
        above the table) or None."""
        r = self.robot
        table, objects = self.perceive(table_info['center'])
        self.last_table = table
        target = next((o for o in objects if o.label == label), None)
        if table is None or target is None:
            self.log.error(f'{label} not found on the {table_info["name"]}')
            return None
        p = target.pose.pose.position
        self.log.info(f'{label}: pose x={p.x:.3f} y={p.y:.3f} z={p.z:.3f} '
                      f'radius={target.radius:.3f} height={target.height:.3f}')

        # Position the base for this object: it should be in front and
        # slightly to the right (arm side). If not, drive to a pose computed
        # from the object's map position and perceive again.
        if abs(p.y - GRASP_REL_Y) > GRASP_REL_TOL:
            ox, oy = r.base_to_map(p.x, p.y)
            gx, gy = ox - GRASP_REL_X, oy - GRASP_REL_Y
            self.log.info(f'{label} is at y={p.y:+.2f} m: repositioning the base to '
                          f'({gx:.2f}, {gy:.2f}) for a front-right grasp')
            r.back_up(0.25)
            if not self.drive_to((gx, gy, 0.0)):
                self.log.error('Could not reposition for the grasp')
                return None
            table, objects = self.perceive(table_info['center'])
            self.last_table = table
            target = next((o for o in objects if o.label == label), None)
            if table is None or target is None:
                self.log.error(f'{label} not found after repositioning')
                return None
            p = target.pose.pose.position
            self.log.info(f'{label}: pose x={p.x:.3f} y={p.y:.3f} z={p.z:.3f}')

        r.clear_scene(OBJECT_ORDER + ['table'])
        self.build_scene(table, objects)

        # Grasp point: on the object axis, low on the body, but at least 7 cm
        # above the table (lower, the gripper body touches the table top).
        table_z = table.center[2]
        grasp_h = max(0.07, min(0.5 * target.height, 0.11))
        point = (p.x, p.y, table_z + grasp_h)

        # Approach direction from the object's 6-DOF pose: perception sets the
        # yaw to point from the object axis towards its handle (cup), or along
        # the line of sight for symmetric objects. Approaching along it keeps
        # the handle straight behind the object, away from the fingers.
        # Limited to +-0.5 rad around the line of sight (arm reachability).
        # If the handle seems to face the robot (more than 90 deg off), the
        # approach goes straight along the line of sight instead: turning
        # towards a handle on the near side brought the gripper in from the
        # side and knocked the cup over, and that reading was also wrong once
        # (stray points on the near side while the real handle was hidden
        # behind the cup).
        los = math.atan2(p.y, p.x)
        obj_yaw = yaw_from_quat(target.pose.pose.orientation)
        d = math.atan2(math.sin(obj_yaw - los), math.cos(obj_yaw - los))
        if abs(d) > math.pi / 2:
            preferred = los
        else:
            preferred = los + max(-0.5, min(0.5, d))
        self.log.info(f'{label}: object yaw {math.degrees(obj_yaw):.0f} deg, '
                      f'approach yaw {math.degrees(preferred):.0f} deg')

        r.open_gripper()
        # 1) pre-grasp (MoveIt plan, collision-aware) and 2) straight approach
        #    (Cartesian; the target is removed from the scene so the fingers
        #    may surround it). If the approach is refused (e.g. it would need
        #    an IK flip), the next approach direction is tried.
        los = math.atan2(p.y, p.x)
        yaw = None
        for cand in [preferred, los, 0.0, los + 0.3, los - 0.3]:
            if not r.move_to_pose(self.tool_pose(point, cand, back=PREGRASP_DIST),
                                  planning_time=8.0):
                self.log.warn(f'Pre-grasp at approach yaw {math.degrees(cand):.0f} deg '
                              'not reachable, trying another')
                continue
            r.remove_object(label)
            if r.move_cartesian([self.tool_pose(point, cand, back=GRASP_BACKOFF).pose],
                                velocity=0.2):
                yaw = cand
                break
            self.log.warn(f'Straight approach at yaw {math.degrees(cand):.0f} deg refused, '
                          'trying another')
            r.add_cylinder(label, make_pose('base_footprint', p.x, p.y, p.z),
                           target.height, target.radius + 0.01)
        if yaw is None:
            self.log.error('No approach direction worked')
            return None
        # 3) close the gripper, weld the object to it in Gazebo, and attach it
        #    to the gripper in MoveIt (MoveIt then plans with it in hand)
        r.close_gripper_on(2 * target.radius, squeeze=GRIPPER_SQUEEZE)
        r.sleep(1.0)
        r.attach_in_gazebo(label)
        self.holding = label
        r.add_cylinder(label, make_pose('base_footprint', p.x, p.y, p.z),
                       target.height, target.radius)
        r.attach_object(label)
        # 4) lift straight up (short Cartesian path); the move away from the
        #    table to the carry pose is then a normal collision-aware plan
        lift = self.tool_pose(point, yaw, back=GRASP_BACKOFF, dz=LIFT_HEIGHT).pose
        if not r.move_cartesian([lift], velocity=0.2, avoid_collisions=False):
            self.log.warn('Lift incomplete')
        # 5) check with the camera: the table must be seen, and the object
        #    must no longer be on it (the perception self-filter removes the
        #    hand and what it holds). Not seeing the table is not a success:
        #    it means the robot moved or the view is blocked.
        seen, still = self.perceive(table_info['center'])
        if seen is None or any(o.label == label for o in still):
            self.log.error(f'Grasp failed: the {label} is still on the table' if seen is not None
                           else 'Grasp not confirmed: the table is not visible any more')
            r.detach_in_gazebo(label)
            self.holding = None
            r.detach_object(label)
            r.open_gripper()
            r.move_cartesian([self.tool_pose(point, yaw, back=PREGRASP_DIST,
                                             dz=LIFT_HEIGHT).pose],
                             velocity=0.2, avoid_collisions=False)
            return None
        self.log.info(f'Grasped {label} (checked: no longer on the table)')
        return target, grasp_h

    def carry_pose(self):
        """Hold the object close to the body, upright, while driving."""
        r = self.robot
        # Close to the body (inside the base footprint Nav2 plans with, so it
        # cannot hit furniture while driving) and slightly to the right, so it
        # hides less of the table from the head camera. About 1 m high: above
        # table height, but low and far enough forward that the head can tilt
        # down without touching the arm (at 1.15 m / 0.28 m the tilted head
        # collided with the wrist).
        #
        # The hand is first moved there along a straight line (Cartesian, with
        # the IK-flip guard), so the arm cannot swing out in a wide arc. A
        # freely planned path did exactly that once: the outstretched arm
        # holding the cup swung round and its reaction spun the whole robot
        # once around on the spot. Only if no straight path is possible is
        # the move planned, and then slowly (HOLDING_VELOCITY).
        candidates = [(0.38, -0.18, 1.02), (0.36, -0.22, 1.00), (0.40, -0.12, 1.00)]
        for x, y, z in candidates:
            if r.move_cartesian([make_pose('base_footprint', x, y, z, roll=math.pi / 2).pose],
                                velocity=0.2, avoid_collisions=True):
                return True
        self.log.warn('No straight path to the carry pose; planning it (slowly)')
        for x, y, z in candidates:
            if r.move_to_pose(make_pose('base_footprint', x, y, z, roll=math.pi / 2),
                              ori_tol=0.1, planning_time=5.0, velocity=HOLDING_VELOCITY):
                return True
        self.log.warn('No carry pose reachable; keeping the current pose')
        return False

    # -------------------------------------------------------------- place
    def choose_place_spot(self, table, objects, table_info=CORNER_TABLE):
        """Free spot (x, y) on the table top, in the robot frame.

        Candidates lie on a grid over the near part of the table (PLACE_INSET
        to PLACE_INSET_MAX behind the near edge, at most PLACE_REACH ahead of
        the robot). A spot is valid if
          * it is inside the measured table outline with PLACE_EDGE_MARGIN
            all around (the object cannot tip over the edge),
          * it is PLACE_CLEARANCE away from every object already there, and
          * no object stands in the gripper's way to it: the gripper comes
            horizontally from the robot, so an object closer to the robot and
            within PLACE_CORRIDOR of the robot-spot line blocks the spot (a
            spot right behind the bottle made every place pose unreachable),
          * it is to the right of every object already there (PLACE_RIGHT_OF),
            because the arm reaches in from the robot's right.
        It must also lie PLACE_EDGE_MARGIN inside the known table (map). The
        valid spot closest to the preferred slot on the table is chosen: first
        object PLACE_SLOT_OFFSET left of the table's centre line, second object
        the same distance right of it, PLACE_TARGET_INSET behind the near edge: measured from the table, so
        it does not end up near the edge when the robot stops a little short.

        The table outline is used instead of individual table-top points:
        objects on the table and the object in the gripper hide parts of the
        top from the camera, and checking for points there rejected every
        spot on the small corner table once the bottle stood on it."""
        def blocked(cx, cy):
            d = math.hypot(cx, cy)
            ux, uy = cx / d, cy / d                      # robot -> spot direction
            for o in objects:
                ox, oy = o.pose.pose.position.x, o.pose.pose.position.y
                along = ox * ux + oy * uy
                across = abs(-ox * uy + oy * ux)
                if 0.0 < along < d and across < PLACE_CORRIDOR:
                    return True
            return False

        # Grid centred sideways on the measured table centre (the robot may
        # stop turned by up to 20 deg, so the table is not always straight ahead)
        candidates = []
        x = table.near_x + PLACE_INSET
        while x <= min(table.near_x + PLACE_INSET_MAX, PLACE_REACH) + 1e-6:
            y = table.center[1] - 0.40
            while y <= table.center[1] + 0.40 + 1e-6:
                candidates.append((x, y))
                y += 0.025
            x += 0.03
        (tx0, ty0), (sx, sy) = table_info['center'], table_info['size']

        def on_known_table(cx, cy):
            mx, my = self.robot.base_to_map(cx, cy)
            return (abs(mx - tx0) <= sx / 2 - PLACE_EDGE_MARGIN
                    and abs(my - ty0) <= sy / 2 - PLACE_EDGE_MARGIN)

        valid = []
        for cx, cy in candidates:
            pt = np.array([[cx, cy, table.center[2]]])
            # inside both the measured outline and the known table (map): the
            # measured outline can be too large when other surfaces are seen
            if not TabletopPerception._inside_table(pt, table, margin=PLACE_EDGE_MARGIN)[0]:
                continue
            if not on_known_table(cx, cy):
                continue
            if any(math.hypot(o.pose.pose.position.x - cx, o.pose.pose.position.y - cy)
                   < PLACE_CLEARANCE for o in objects):
                continue
            if blocked(cx, cy):
                continue
            if any(cy > o.pose.pose.position.y - PLACE_RIGHT_OF for o in objects):
                continue                                 # not to the right of all objects
            valid.append((cx, cy))
        self.log.info(f'Place spots: {len(valid)} of {len(candidates)} candidates are free '
                      f'(table near edge {table.near_x:.2f} m, {len(objects)} object(s) on it)')
        if not valid:
            return None
        # Closest to the preferred spot; between equally good spots (to the
        # centimetre), the one nearer the middle of the table, away from its edges.
        # Preferred slot on the known table (the robot faces the table along +x
        # of the map, so map +y is the robot's left), converted to the robot frame
        slot_y = ty0 + PLACE_SLOT_OFFSET - 2 * PLACE_SLOT_OFFSET * min(len(objects), 1)
        target_x, target_y = self.robot.map_to_base(tx0 - sx / 2 + PLACE_TARGET_INSET, slot_y)
        return min(valid, key=lambda c: (round(math.hypot(c[0] - target_x, c[1] - target_y), 2),
                                         abs(c[1] - table.center[1])))

    def retreat(self, point, yaw, clear):
        """Get the open gripper clear of a just released object.

        First straight UP by `clear` (until the finger pads are above the
        object's top), slowly, then backwards. Pulling back before that dragged the
        open fingers across the object: across the bottle's neck after a 12 cm
        lift, and with the fingertips hooking the light cup. If the straight
        lift is refused or cut short (IK flip), a MoveIt-planned move to the
        same pose is tried; if the gripper still cannot rise, it does not pull
        back at all. Returns True if the gripper got clear."""
        r = self.robot
        z0 = r.current_ee_pose().position.z
        above = self.tool_pose(point, yaw, back=GRASP_BACKOFF, dz=clear)
        r.move_cartesian([above.pose], velocity=0.1, avoid_collisions=False, min_fraction=0.3)
        if r.current_ee_pose().position.z - z0 < clear - 0.02:
            self.log.warn('Straight lift cut short; planning the lift with MoveIt')
            r.move_to_pose(above, planning_time=8.0)
        if r.current_ee_pose().position.z - z0 < clear - 0.02:
            self.log.error('The gripper could not rise above the object')
            return False
        r.move_cartesian([self.tool_pose(point, yaw, back=0.20, dz=clear).pose],
                         velocity=0.15, avoid_collisions=False, min_fraction=0.3)
        return True

    def place(self, label, target, grasp_h, table_info=CORNER_TABLE):
        """Put the held `label` (DetectedObject `target`) down on the table.
        Returns True if the camera sees it on the table afterwards."""
        r = self.robot
        table, objects = self.perceive(table_info['center'])
        if table is None:
            self.log.error(f'{table_info["name"]} not found')
            return False
        r.clear_scene(['table'] + [o for o in OBJECT_ORDER if o != label])
        self.build_scene(table, objects, skip=label)

        table_z = table.center[2]
        spot = self.choose_place_spot(table, [o for o in objects if o.label != label], table_info)
        if spot is None:
            self.log.error(f'No free place spot on the {table_info["name"]}')
            return False
        x, y = spot
        point = (x, y, table_z + grasp_h + RELEASE_GAP)
        self.log.info(f'Placing {label} at x={x:.2f} y={y:.2f} (robot frame)')

        # Pre-place 10 cm above the spot (MoveIt plan), then straight down
        # (Cartesian). If the move down is refused (e.g. it would need an IK
        # flip), the next approach direction is tried. The object is only
        # released once it is really down at the release height: releasing it
        # from the pre-place pose dropped it 10 cm onto the table.
        los = math.atan2(point[1], point[0])
        yaw = None
        for cand in [los, 0.0, los + 0.3, los - 0.3]:
            if not r.move_to_pose(self.tool_pose(point, cand, back=GRASP_BACKOFF, dz=0.10),
                                  planning_time=8.0, velocity=HOLDING_VELOCITY):
                self.log.warn(f'Pre-place at yaw {math.degrees(cand):.0f} deg not reachable, '
                              'trying another')
                continue
            if r.move_cartesian([self.tool_pose(point, cand, back=GRASP_BACKOFF).pose],
                                velocity=0.15, avoid_collisions=False):
                yaw = cand
                break
            self.log.warn(f'Move down at yaw {math.degrees(cand):.0f} deg refused, trying another')
        if yaw is None:
            self.log.error('Could not lower the object to the table; still holding it')
            return False
        r.detach_in_gazebo(label)
        self.holding = None
        r.open_gripper()
        r.detach_object(label)
        r.sleep(0.5)
        clear = target.height - grasp_h + RETREAT_CLEARANCE
        self.retreat(point, yaw, max(clear, 0.10))
        # The placed object becomes an obstacle where it now stands (only if
        # the gripper is clear of it: a start state inside an obstacle makes
        # MoveIt refuse to plan), and the table and the other objects stay in
        # the scene while the arm is stowed. With an empty scene MoveIt
        # planned the stow straight through the table and swept both objects
        # onto the floor.
        g = r.lookup('base_footprint', 'gripper_grasping_frame').transform.translation
        if math.hypot(g.x - x, g.y - y) > 0.08 or g.z > table_z + target.height + 0.03:
            r.add_cylinder(label, make_pose('base_footprint', x, y, table_z + target.height / 2),
                           target.height, target.radius + 0.01)
        else:
            self.log.warn('Gripper still next to the placed object; stowing without it in the scene')
        self.stow_arm()
        r.clear_scene(['table'] + OBJECT_ORDER)

        # Check with the camera
        _, objects = self.perceive(table_info['center'])
        ok = any(o.label == label for o in objects)
        self.log.info(f'{label} on the {table_info["name"]}: {"YES" if ok else "NOT DETECTED"}')
        return ok
