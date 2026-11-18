"""Python interface to my_tiago used by the task scripts.

It wraps the ROS 2 interfaces of the simulation so that a task script reads
as a sequence of steps:

  * MoveIt     /move_action              (moveit_msgs/action/MoveGroup)
               /compute_cartesian_path   (straight-line approach / retreat)
               /execute_trajectory       (execute a Cartesian path)
               /apply_planning_scene     (collision objects, attach / detach)
  * ros2_control trajectory controllers  (gripper, head; not planned)
  * Nav2       nav2_simple_commander     (drive the mobile base)
  * TF2                                  (frame transforms)
  * IFRA_LinkAttacher /ATTACHLINK, /DETACHLINK (hold grasped objects in Gazebo)

moveit_py (the official MoveIt Python API) is not released for ROS 2 Humble,
which is why the MoveIt action and service interfaces are used directly.

Needs: my_tiago_gazebo.launch.xml and my_tiago_moveit_sim.launch.xml running;
my_tiago_navigation.launch.xml as well for the navigation functions.
"""
import math
import threading
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

import tf2_ros
from builtin_interfaces.msg import Duration as DurationMsg
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Quaternion
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints,
                             JointConstraint, MoveItErrorCodes, OrientationConstraint,
                             PlanningScene, PlanningSceneWorld, PositionConstraint,
                             RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath
from linkattacher_msgs.srv import AttachLink, DetachLink
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint

# MoveIt planning group (torso + 7-DOF arm) and the link whose pose we command
ARM_GROUP = 'arm_torso'
EE_LINK = 'arm_tool_link'
# The gripper "grasping frame" (point between the fingertips) is 0.141 m in
# front of arm_tool_link along its x axis (measured with TF).
TOOL_TO_GRASP = 0.141

GRIPPER_JOINTS = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
GRIPPER_OPEN = 0.044         # per-finger opening [m] (limit 0.045)
HEAD_JOINTS = ['head_1_joint', 'head_2_joint']   # pan, tilt

# Arm folded close to the body (PAL's "home" pose), torso half up
HOME_JOINTS = {'torso_lift_joint': 0.15,
               'arm_1_joint': 0.20, 'arm_2_joint': -1.34, 'arm_3_joint': -0.20,
               'arm_4_joint': 1.94, 'arm_5_joint': -1.57, 'arm_6_joint': 1.37,
               'arm_7_joint': 0.0}
# Arm folded at the side with the torso raised (end of PAL's "home" motion):
# small footprint for driving, and the arm stays out of the camera view.
STOW_JOINTS = {'torso_lift_joint': 0.35,
               'arm_1_joint': 0.50, 'arm_2_joint': -1.34, 'arm_3_joint': -0.48,
               'arm_4_joint': 1.94, 'arm_5_joint': -1.49, 'arm_6_joint': 1.37,
               'arm_7_joint': 0.0}


# ----------------------------------------------------------------------------
# Small math helpers
# ----------------------------------------------------------------------------
def quat_from_rpy(roll, pitch, yaw):
    """Roll/pitch/yaw (rad) -> geometry_msgs Quaternion."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return Quaternion(x=sr * cp * cy - cr * sp * sy,
                      y=cr * sp * cy + sr * cp * sy,
                      z=cr * cp * sy - sr * sp * cy,
                      w=cr * cp * cy + sr * sp * sy)


def yaw_from_quat(q):
    """Yaw angle (rad) of a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def make_pose(frame, x, y, z, roll=0.0, pitch=0.0, yaw=0.0, stamp=None):
    """PoseStamped from a position and roll/pitch/yaw."""
    ps = PoseStamped()
    ps.header.frame_id = frame
    if stamp is not None:
        ps.header.stamp = stamp
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = float(x), float(y), float(z)
    ps.pose.orientation = quat_from_rpy(roll, pitch, yaw)
    return ps


def secs(t):
    """float seconds -> builtin_interfaces/Duration."""
    return DurationMsg(sec=int(t), nanosec=int((t % 1) * 1e9))


# ----------------------------------------------------------------------------
# Robot interface
# ----------------------------------------------------------------------------
class TiagoRobot(Node):
    """Robot actions needed by the task scripts, as blocking calls."""

    def __init__(self, name='my_tiago_task'):
        super().__init__(name)
        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        cb = ReentrantCallbackGroup()

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Latest joint positions
        self.joints = {}
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10,
                                 callback_group=cb)

        # MoveIt interfaces
        self.move_group = ActionClient(self, MoveGroup, '/move_action', callback_group=cb)
        self.exec_traj = ActionClient(self, ExecuteTrajectory, '/execute_trajectory',
                                      callback_group=cb)
        self.cartesian_srv = self.create_client(GetCartesianPath, '/compute_cartesian_path',
                                                callback_group=cb)
        self.scene_srv = self.create_client(ApplyPlanningScene, '/apply_planning_scene',
                                            callback_group=cb)
        # Gazebo link attacher (simulation only, see attach_in_gazebo)
        self.attach_srv = self.create_client(AttachLink, '/ATTACHLINK', callback_group=cb)
        self.detach_srv = self.create_client(DetachLink, '/DETACHLINK', callback_group=cb)
        # Localisation recovery (see recover_localisation)
        self.loc_manager = self.create_client(ManageLifecycleNodes,
                                              '/lifecycle_manager_localization/manage_nodes',
                                              callback_group=cb)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 1)
        # Direct controller interfaces (gripper and head are not planned)
        self.gripper = ActionClient(self, FollowJointTrajectory,
                                    '/gripper_controller/follow_joint_trajectory',
                                    callback_group=cb)
        self.head = ActionClient(self, FollowJointTrajectory,
                                 '/head_controller/follow_joint_trajectory',
                                 callback_group=cb)

        # Spin this node in a background thread so the public methods can block
        # while their callbacks (action results, TF, joint states) still run.
        self._executor = MultiThreadedExecutor()
        self._executor.add_node(self)
        self._spin_thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._spin_thread.start()

        for client, label in [(self.move_group, 'move_action'),
                              (self.exec_traj, 'execute_trajectory'),
                              (self.gripper, 'gripper controller'),
                              (self.head, 'head controller')]:
            self.get_logger().info(f'Waiting for {label}...')
            client.wait_for_server()
        self.cartesian_srv.wait_for_service()
        self.scene_srv.wait_for_service()

        # Nav2 helper. It is a separate node that the commander spins itself.
        self.nav = BasicNavigator(node_name=name + '_navigator')
        self.get_logger().info('Robot interface ready.')

    # ------------------------------------------------------------------ utils
    def _joint_cb(self, msg):
        self.joints.update(zip(msg.name, msg.position))

    def _wait(self, future, timeout=60.0):
        """Block until an rclpy future is done (the executor runs in a thread)."""
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                raise TimeoutError('ROS call timed out')
            time.sleep(0.01)
        return future.result()

    def _run_action(self, client, goal, timeout=120.0):
        """Send an action goal and block until its result arrives."""
        handle = self._wait(client.send_goal_async(goal), 10.0)
        if not handle.accepted:
            raise RuntimeError('Action goal rejected')
        return self._wait(handle.get_result_async(), timeout).result

    def now(self):
        return self.get_clock().now().to_msg()

    def sleep(self, seconds):
        """Sleep in simulated time."""
        end = self.get_clock().now() + Duration(seconds=seconds)
        while self.get_clock().now() < end:
            time.sleep(0.01)

    def lookup(self, target, source, timeout=5.0):
        """Latest transform target <- source."""
        return self.tf_buffer.lookup_transform(target, source, rclpy.time.Time(),
                                               timeout=Duration(seconds=timeout))

    # ------------------------------------------------------------- navigation
    def wait_for_nav2(self):
        """Wait until AMCL and the BT navigator are active.

        BasicNavigator.waitUntilNav2Active(localizer='amcl') is not used: it
        publishes an initial pose of (0, 0, 0) first, which would reset AMCL
        to the spawn point whenever a script starts after the robot moved.
        """
        self.nav._waitForNodeToActivate('amcl')
        self.nav._waitForNodeToActivate('bt_navigator')

    def go_to(self, x, y, yaw):
        """Drive the base to (x, y, yaw) in the map frame with Nav2."""
        if not self.localisation_ok():
            self.recover_localisation()
        goal = make_pose('map', x, y, 0.0, yaw=yaw, stamp=self.now())
        self.get_logger().info(f'Navigating to x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.0f} deg')
        self.nav.goToPose(goal)
        while not self.nav.isTaskComplete():
            time.sleep(0.2)
        ok = self.nav.getResult() == TaskResult.SUCCEEDED
        if not ok:
            # Nav2 can give up during the last small heading correction (the
            # simulated base turns poorly on the spot). Accept the result if
            # the robot is close: perception works relative to the robot.
            bx, by, byaw = self.base_pose()
            dyaw = abs(math.atan2(math.sin(byaw - yaw), math.cos(byaw - yaw)))
            if math.hypot(bx - x, by - y) < 0.25 and dyaw < 0.6:
                self.get_logger().info('Navigation ended close to the goal: accepted')
                ok = True
        self.get_logger().info('Navigation ' + ('succeeded' if ok else 'FAILED'))
        return ok

    def back_up(self, distance=0.35, speed=0.1):
        """Reverse straight back (Nav2 BackUp behaviour), e.g. to get clear of
        a table before turning towards the next goal."""
        self.nav.backup(backup_dist=distance, backup_speed=speed, time_allowance=20)
        while not self.nav.isTaskComplete():
            time.sleep(0.1)

    def base_pose(self):
        """Current robot (x, y, yaw) in the map frame.

        Composed from the latest map->odom (AMCL) and the latest
        odom->base_footprint (EKF) separately: a single map->base_footprint
        lookup fails when the two are far apart in time (AMCL stalled)."""
        mo = self.lookup('map', 'odom').transform
        ob = self.lookup('odom', 'base_footprint').transform
        myaw, oyaw = yaw_from_quat(mo.rotation), yaw_from_quat(ob.rotation)
        x = mo.translation.x + math.cos(myaw) * ob.translation.x - math.sin(myaw) * ob.translation.y
        y = mo.translation.y + math.sin(myaw) * ob.translation.x + math.cos(myaw) * ob.translation.y
        yaw = math.atan2(math.sin(myaw + oyaw), math.cos(myaw + oyaw))
        return x, y, yaw

    def localisation_ok(self, max_age=3.0):
        """True if AMCL has updated map->odom within the last `max_age` s
        (its stamps run up to transform_tolerance = 1 s ahead)."""
        try:
            st = self.lookup('map', 'odom').header.stamp
        except Exception:  # noqa: BLE001 - no transform at all
            return False
        age = (self.get_clock().now() - rclpy.time.Time.from_msg(st)).nanoseconds * 1e-9
        return age < max_age

    def recover_localisation(self):
        """Restart AMCL if it has stopped updating, keeping the robot's pose.

        In a few runs AMCL stopped publishing map->odom (no error message;
        its laser scans were apparently all dropped), Nav2 could then not
        place its goals, and the robot was lost. The current map pose is
        estimated from AMCL's last update plus the EKF's odometry since, AMCL
        is restarted through Nav2's lifecycle manager (fresh TF buffer), and
        the estimate is sent to it as its initial pose."""
        x, y, yaw = self.base_pose()
        self.get_logger().warn(f'AMCL has stopped updating: restarting it at x={x:.2f} '
                               f'y={y:.2f} yaw={math.degrees(yaw):.0f} deg')
        if not self.loc_manager.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Localisation lifecycle manager not available')
            return False
        for cmd in (ManageLifecycleNodes.Request.RESET, ManageLifecycleNodes.Request.STARTUP):
            res = self._wait(self.loc_manager.call_async(ManageLifecycleNodes.Request(command=cmd)), 90.0)
            if not res.success:
                self.get_logger().error(f'Lifecycle command {cmd} failed')
                return False
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.pose.pose = make_pose('map', x, y, 0.0, yaw=yaw).pose
        msg.pose.covariance[0] = msg.pose.covariance[7] = 0.05 ** 2
        msg.pose.covariance[35] = math.radians(5) ** 2
        for _ in range(30):                       # until AMCL publishes again
            msg.header.stamp = self.now()
            self.initial_pose_pub.publish(msg)
            self.sleep(0.5)
            if self.localisation_ok(max_age=2.0):
                self.get_logger().info('AMCL running again')
                return True
        self.get_logger().error('AMCL did not recover')
        return False

    def map_to_base(self, x, y):
        """Map-frame (x, y) -> base_footprint (x, y)."""
        bx, by, byaw = self.base_pose()
        dx, dy = x - bx, y - by
        return (math.cos(byaw) * dx + math.sin(byaw) * dy,
                -math.sin(byaw) * dx + math.cos(byaw) * dy)

    def base_to_map(self, x, y):
        """base_footprint (x, y) -> map-frame (x, y)."""
        bx, by, byaw = self.base_pose()
        return (bx + math.cos(byaw) * x - math.sin(byaw) * y,
                by + math.sin(byaw) * x + math.cos(byaw) * y)

    # ---------------------------------------------------------------- MoveIt
    def _motion_request(self, constraints, velocity=0.4, planning_time=5.0, attempts=5):
        goal = MoveGroup.Goal()
        req = goal.request
        req.group_name = ARM_GROUP
        req.num_planning_attempts = attempts
        req.allowed_planning_time = planning_time
        req.max_velocity_scaling_factor = velocity
        req.max_acceleration_scaling_factor = velocity
        req.goal_constraints = [constraints]
        req.start_state.is_diff = True          # start from the current state
        goal.planning_options.plan_only = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        return goal

    def move_to_pose(self, pose, link=EE_LINK, pos_tol=0.01, ori_tol=0.05, **kw):
        """Plan + execute so that `link` reaches `pose` (PoseStamped)."""
        c = Constraints()
        pc = PositionConstraint()
        pc.header = pose.header
        pc.link_name = link
        pc.constraint_region.primitives = [SolidPrimitive(type=SolidPrimitive.SPHERE,
                                                          dimensions=[pos_tol])]
        pc.constraint_region.primitive_poses = [pose.pose]
        pc.weight = 1.0
        oc = OrientationConstraint()
        oc.header = pose.header
        oc.link_name = link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance = ori_tol
        oc.absolute_z_axis_tolerance = ori_tol
        oc.weight = 1.0
        c.position_constraints = [pc]
        c.orientation_constraints = [oc]
        result = self._run_action(self.move_group, self._motion_request(c, **kw))
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS
        if not ok:
            self.get_logger().warn(f'move_to_pose failed (MoveIt error {result.error_code.val})')
        return ok

    def move_joints(self, joints, **kw):
        """Plan + execute to a joint configuration given as {name: value}."""
        c = Constraints()
        for name, value in joints.items():
            c.joint_constraints.append(JointConstraint(
                joint_name=name, position=float(value),
                tolerance_above=0.01, tolerance_below=0.01, weight=1.0))
        result = self._run_action(self.move_group, self._motion_request(c, **kw))
        ok = result.error_code.val == MoveItErrorCodes.SUCCESS
        if not ok:
            self.get_logger().warn(f'move_joints failed (MoveIt error {result.error_code.val})')
        return ok

    def home_arm(self, **kw):
        """Fold the arm close to the body, torso half up."""
        return self.move_joints(HOME_JOINTS, **kw)

    def stow_arm(self, **kw):
        """Fold the arm at the side, torso up (for driving and looking)."""
        kw.setdefault('planning_time', 10.0)
        return self.move_joints(STOW_JOINTS, **kw)

    def move_cartesian(self, waypoints, frame='base_footprint', velocity=0.2,
                       avoid_collisions=True, min_fraction=0.9,
                       revolute_jump=0.3, max_joint_travel=1.2):
        """Move EE_LINK along straight lines through a list of Poses.

        Two guards against IK flips (the elbow or wrist swinging round in the
        middle of a straight move): MoveIt stops the path where a joint jumps
        more than `revolute_jump` rad between two 1 cm steps, and the path is
        refused if any joint travels more than `max_joint_travel` rad in
        total. Without them a 12 cm approach to the cup became a whole-arm
        swing that tipped the robot onto one wheel and spun it 100 degrees."""
        req = GetCartesianPath.Request()
        req.header.frame_id = frame
        req.header.stamp = self.now()
        req.start_state.is_diff = True
        req.group_name = ARM_GROUP
        req.link_name = EE_LINK
        req.waypoints = waypoints
        req.max_step = 0.01
        # Absolute jump limits (rad per step for revolute joints, m for the
        # torso). The relative jump_threshold is not used: it cut short the
        # 10-15 cm straight moves used here.
        req.jump_threshold = 0.0
        req.revolute_jump_threshold = float(revolute_jump)
        req.prismatic_jump_threshold = 0.05
        req.avoid_collisions = avoid_collisions
        res = self._wait(self.cartesian_srv.call_async(req))
        if res.fraction < min_fraction:
            self.get_logger().warn(f'Cartesian path only {res.fraction * 100:.0f}% feasible')
            return False
        traj = res.solution
        pts = traj.joint_trajectory.points
        if len(pts) > 1:
            travel = [sum(abs(b.positions[i] - a.positions[i]) for a, b in zip(pts, pts[1:]))
                      for i in range(len(pts[0].positions))]
            worst = max(range(len(travel)), key=lambda i: travel[i])
            if travel[worst] > max_joint_travel:
                self.get_logger().warn(
                    f'Cartesian path refused: {traj.joint_trajectory.joint_names[worst]} would '
                    f'travel {travel[worst]:.2f} rad (IK flip)')
                return False
        # compute_cartesian_path returns a full-speed trajectory: slow it down
        for pt in traj.joint_trajectory.points:
            t = (pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9) / velocity
            pt.time_from_start = secs(t)
            pt.velocities = [v * velocity for v in pt.velocities]
            pt.accelerations = [a * velocity * velocity for a in pt.accelerations]
        result = self._run_action(self.exec_traj, ExecuteTrajectory.Goal(trajectory=traj))
        return result.error_code.val == MoveItErrorCodes.SUCCESS

    def current_ee_pose(self, frame='base_footprint'):
        """Current pose of EE_LINK as a geometry_msgs Pose."""
        t = self.lookup(frame, EE_LINK)
        p = Pose()
        p.position.x = t.transform.translation.x
        p.position.y = t.transform.translation.y
        p.position.z = t.transform.translation.z
        p.orientation = t.transform.rotation
        return p

    # ------------------------------------------------- gripper / head control
    def _trajectory_goal(self, joints, positions, duration):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = joints
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=[float(p) for p in positions], time_from_start=secs(duration))]
        return goal

    def set_gripper(self, finger_pos, duration=1.5):
        """Command both fingers to `finger_pos` (m from the centre, 0..0.045)."""
        self._run_action(self.gripper, self._trajectory_goal(
            GRIPPER_JOINTS, [finger_pos, finger_pos], duration))

    def open_gripper(self):
        self.set_gripper(GRIPPER_OPEN)

    def close_gripper_on(self, object_width, squeeze=0.0):
        """Close the fingers onto an object of width `object_width`. `squeeze`
        is how far inside the object surface each finger is commanded
        (negative = stop just short of it). The fingers are position-controlled
        in Gazebo, so any squeeze becomes a penetration impulse."""
        self.set_gripper(max(0.0, object_width / 2.0 - squeeze), duration=2.0)

    def gripper_opening(self):
        """Current distance between the fingers (sum of both joints) [m]."""
        return sum(self.joints.get(j, 0.0) for j in GRIPPER_JOINTS)

    def move_head(self, pan, tilt, duration=1.5):
        """Point the head camera (rad; negative tilt = look down)."""
        self._run_action(self.head, self._trajectory_goal(HEAD_JOINTS, [pan, tilt], duration))
        self.sleep(0.5)   # let the image settle

    def look_at(self, x, y, z):
        """Point the head camera at the point (x, y, z) in base_footprint."""
        h = self.lookup('base_footprint', 'head_1_link').transform.translation
        pan = math.atan2(y - h.y, x - h.x)
        tilt = -math.atan2(h.z - z, math.hypot(x - h.x, y - h.y))
        pan = max(-1.2, min(1.2, pan))              # inside the joint limits
        tilt = max(-0.95, min(0.7, tilt))
        self.move_head(pan, tilt)

    # ------------------------------------------------------- planning scene
    def _apply_scene(self, scene):
        scene.is_diff = True
        scene.robot_state.is_diff = True
        self._wait(self.scene_srv.call_async(ApplyPlanningScene.Request(scene=scene)))

    def add_box(self, name, pose, size):
        """Add a box collision object (pose: PoseStamped, size: (x, y, z))."""
        co = CollisionObject(id=name, header=pose.header, operation=CollisionObject.ADD)
        co.primitives = [SolidPrimitive(type=SolidPrimitive.BOX,
                                        dimensions=[float(s) for s in size])]
        co.primitive_poses = [pose.pose]
        self._apply_scene(PlanningScene(world=PlanningSceneWorld(collision_objects=[co])))

    def add_cylinder(self, name, pose, height, radius):
        """Add a vertical cylinder collision object centred at `pose`."""
        co = CollisionObject(id=name, header=pose.header, operation=CollisionObject.ADD)
        co.primitives = [SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                        dimensions=[float(height), float(radius)])]
        co.primitive_poses = [pose.pose]
        self._apply_scene(PlanningScene(world=PlanningSceneWorld(collision_objects=[co])))

    def remove_object(self, name):
        co = CollisionObject(id=name, operation=CollisionObject.REMOVE)
        self._apply_scene(PlanningScene(world=PlanningSceneWorld(collision_objects=[co])))

    def clear_scene(self, names):
        for n in names:
            self.remove_object(n)

    def attach_object(self, name):
        """Attach a world collision object to the gripper, so MoveIt plans
        with it in hand and ignores finger-object contacts."""
        aco = AttachedCollisionObject(link_name='gripper_link')
        aco.object.id = name
        aco.object.operation = CollisionObject.ADD
        aco.touch_links = ['gripper_link', 'gripper_left_finger_link',
                           'gripper_right_finger_link', 'gripper_grasping_frame', EE_LINK]
        self._apply_scene(PlanningScene(
            robot_state=RobotState(attached_collision_objects=[aco], is_diff=True)))

    def detach_object(self, name):
        """Detach the object from the gripper and remove it from the scene."""
        aco = AttachedCollisionObject(link_name='gripper_link')
        aco.object.id = name
        aco.object.operation = CollisionObject.REMOVE
        self._apply_scene(PlanningScene(
            robot_state=RobotState(attached_collision_objects=[aco], is_diff=True)))
        self.remove_object(name)

    # ------------------------------------------- simulation grasp helper
    # Gazebo's contact physics cannot hold an object between Tiago's
    # position-controlled fingers: it slips out when the arm lifts. Once the
    # gripper has closed around the object, the object is fixed to the gripper
    # with the IFRA_LinkAttacher plugin, and released before the gripper opens.
    # Gazebo merges the gripper links (fixed joints) into 'arm_7_link', so that
    # is the link to attach to. On a real robot these two calls are not needed.
    def attach_in_gazebo(self, model):
        if not self.attach_srv.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Link attacher not available')
            return False
        req = AttachLink.Request(model1_name='tiago', link1_name='arm_7_link',
                                 model2_name=model, link2_name='link')
        res = self._wait(self.attach_srv.call_async(req))
        return res.success

    def detach_in_gazebo(self, model):
        if not self.detach_srv.wait_for_service(timeout_sec=2.0):
            return False
        req = DetachLink.Request(model1_name='tiago', link1_name='arm_7_link',
                                 model2_name=model, link2_name='link')
        res = self._wait(self.detach_srv.call_async(req))
        return res.success

    # --------------------------------------------------------------- cleanup
    def shutdown(self):
        # Stop the background executor before destroying the nodes, so no
        # callback runs on a node that is being destroyed.
        self._executor.shutdown(timeout_sec=2.0)
        self.nav.destroy_node()
        self.destroy_node()
