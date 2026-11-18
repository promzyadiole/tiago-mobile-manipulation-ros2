#!/usr/bin/env python3
"""Before/after experiment for the link attacher (figures of section 15.1).

A bottle stands on a small pedestal inside the open gripper (fingers closing
sideways, as in pick_place.py). Three cases, each: close the fingers, lift the
gripper 12 cm (straight line), put everything back:
  A  fingers close to the bottle surface, no link attacher
  B  fingers squeeze 5 mm into the bottle (as a real gripper would), no attacher
  C  fingers stop 4 mm short (pick_place.py) + /ATTACHLINK, then /DETACHLINK
A camera model spawned in Gazebo takes a picture at each stage; the bottle's
true height is logged at 20 Hz (/gazebo/get_entity_state).

Run (Gazebo and MoveIt running, robot freshly spawned, nothing else moving it):
    python3 attach_experiment.py images/attach
Output: images/attach/{A,B,C}{1_ready,2_closed,3_lifted}.png, attach_experiment.json
Then:   python3 make_attach_figures.py
"""
import json
import math
import sys
import threading
import time

import numpy as np
import rclpy
from gazebo_msgs.srv import DeleteEntity, GetEntityState, SetEntityState, SpawnEntity
from geometry_msgs.msg import Pose, Point, Quaternion
from PIL import Image as PILImage
from sensor_msgs.msg import Image

from my_tiago_tasks.robot import TiagoRobot, make_pose

OUT = sys.argv[1]
R, H = 0.03, 0.26                       # bottle radius, height (world file)

CAM_SDF = """<sdf version='1.6'><model name='demo_cam'><static>true</static>
<link name='link'><sensor name='cam' type='camera'><update_rate>10</update_rate>
<camera><horizontal_fov>1.0</horizontal_fov><image><width>960</width><height>720</height></image>
<clip><near>0.05</near><far>20</far></clip></camera>
<plugin name='demo_cam_plugin' filename='libgazebo_ros_camera.so'><ros><namespace>/demo_cam</namespace></ros>
<camera_name>cam</camera_name><frame_name>demo_cam</frame_name></plugin></sensor></link></model></sdf>"""


def pedestal_sdf(h):
    return f"""<sdf version='1.6'><model name='pedestal'><static>true</static><link name='link'>
<collision name='c'><geometry><box><size>0.10 0.10 {h}</size></box></geometry></collision>
<visual name='v'><geometry><box><size>0.10 0.10 {h}</size></box></geometry>
<material><ambient>0.45 0.32 0.2 1</ambient><diffuse>0.55 0.4 0.25 1</diffuse></material></visual>
</link></model></sdf>"""


def tilt_deg(q):
    """Angle between the bottle's axis and the vertical [deg]."""
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (q.x * q.x + q.y * q.y)))))


def quat_yaw_pitch(yaw, pitch):
    cy, sy, cp, sp = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(pitch / 2), math.sin(pitch / 2)
    return Quaternion(w=cy * cp, x=-sy * sp, y=cy * sp, z=sy * cp)


class Exp:
    def __init__(self, robot):
        self.r = robot
        n = robot
        self.spawn = n.create_client(SpawnEntity, '/spawn_entity')
        self.get = n.create_client(GetEntityState, '/gazebo/get_entity_state')
        self.set = n.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.delete = n.create_client(DeleteEntity, '/delete_entity')
        for c in (self.spawn, self.get, self.set, self.delete):
            c.wait_for_service()
        self.img = None
        n.create_subscription(Image, '/demo_cam/cam/image_raw', self._img, 2)
        self.log_on = False
        self.samples = []

    def _img(self, m):
        self.img = m

    def call(self, client, req):
        return self.r._wait(client.call_async(req))

    def spawn_model(self, name, xml, pose):
        self.call(self.delete, DeleteEntity.Request(name=name))
        for _ in range(10):                     # the delete takes a moment
            time.sleep(1.0)
            res = self.call(self.spawn, SpawnEntity.Request(name=name, xml=xml, initial_pose=pose))
            self.r.get_logger().info(f'spawn {name}: {res.success} {res.status_message}')
            if res.success:
                return res
        raise RuntimeError(f'could not spawn {name}')

    def bottle_pose(self):
        res = self.call(self.get, GetEntityState.Request(name='bottle', reference_frame='world'))
        return res.state.pose

    def put_bottle(self, x, y, z):
        req = SetEntityState.Request()
        req.state.name = 'bottle'
        req.state.pose = Pose(position=Point(x=x, y=y, z=z), orientation=Quaternion(w=1.0))
        req.state.reference_frame = 'world'
        self.call(self.set, req)

    def snap(self, name):
        self.img = None
        t = time.time()
        while self.img is None and time.time() - t < 10:
            time.sleep(0.05)
        m = self.img
        a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.width, 3)
        PILImage.fromarray(a).save(f'{OUT}/{name}.png')

    def logger(self, case):
        t0 = time.time()
        while self.log_on:
            p = self.bottle_pose()
            g = self.r.lookup('base_footprint', 'gripper_grasping_frame').transform.translation
            self.samples.append(dict(case=case, t=time.time() - t0, bz=p.position.z,
                                     bx=p.position.x, by=p.position.y, gz=g.z,
                                     tilt=tilt_deg(p.orientation)))
            time.sleep(0.05)


def main():
    rclpy.init()
    r = TiagoRobot('attach_experiment')
    e = Exp(r)
    log = r.get_logger()
    r.clear_scene(['pedestal'])
    r.open_gripper()
    r.move_head(0.0, -0.3)
    ok = r.move_to_pose(make_pose('base_footprint', 0.58, -0.20, 0.82, roll=math.pi / 2), velocity=0.3)
    log.info(f'arm in grasp pose: {ok}')
    r.sleep(1.0)
    g = r.lookup('base_footprint', 'gripper_grasping_frame').transform.translation
    log.info(f'grasp frame at {g.x:.3f} {g.y:.3f} {g.z:.3f}')
    gx, gy, gz = g.x, g.y, g.z                 # robot at the world origin, yaw 0
    # the bottle model's origin is at its BASE: grasp 10 cm above the base
    ped_h = gz - 0.10
    bottle_z = ped_h + 0.001
    e.spawn_model('pedestal', pedestal_sdf(ped_h),
                  Pose(position=Point(x=gx, y=gy, z=ped_h / 2), orientation=Quaternion(w=1.0)))
    # camera on the robot's right, looking at the gripper
    # camera in front of the robot, to its right, looking back at the gripper
    cx, cy, cz = gx + 0.60, gy - 0.50, gz + 0.20
    cam = Pose(position=Point(x=cx, y=cy, z=cz),
               orientation=quat_yaw_pitch(math.atan2(gy - cy, gx - cx),
                                          math.atan2(cz - (gz + 0.01), math.hypot(gx - cx, gy - cy))))
    e.spawn_model('demo_cam', CAM_SDF, cam)
    r.sleep(2.0)

    cases = [('A', 'no attacher, fingers touch the bottle', R, False),
             ('B', 'no attacher, fingers squeeze 5 mm', R - 0.005, False),
             ('C', 'link attacher, fingers 4 mm short', R + 0.004, True)]
    results = {}
    for case, text, finger, attach in cases:
        log.info(f'===== case {case}: {text}')
        r.open_gripper()
        for k in range(8):                    # place it until it really stands there
            e.put_bottle(gx, gy, bottle_z)
            r.sleep(2.0)
            p = e.bottle_pose()
            tilt = tilt_deg(p.orientation)
            log.info(f'  bottle placed: z={p.position.z:.3f} tilt={tilt:.1f} (try {k + 1})')
            if abs(p.position.z - bottle_z) < 0.01 and tilt < 6:   # rests ~4 deg on a finger
                break
        e.snap(f'{case}1_ready')
        e.log_on = True
        th = threading.Thread(target=e.logger, args=(case,), daemon=True)
        th.start()
        r.set_gripper(finger, duration=2.0)
        r.sleep(1.0)
        if attach:
            log.info(f'attach: {r.attach_in_gazebo("bottle")}')
        e.snap(f'{case}2_closed')
        up = r.current_ee_pose()
        up.position.z += 0.12
        okc = r.move_cartesian([up], velocity=0.15, avoid_collisions=False)
        r.sleep(1.5)
        e.snap(f'{case}3_lifted')
        e.log_on = False
        th.join()
        p = e.bottle_pose().position
        results[case] = dict(text=text, cartesian=okc, bottle=(p.x, p.y, p.z))
        log.info(f'case {case}: bottle at {p.x:.3f} {p.y:.3f} {p.z:.3f} (start z {bottle_z:.3f})')
        # back down and release (open first if nothing is held: the closed
        # fingers would otherwise hit the bottle left on the pedestal)
        if not attach:
            r.open_gripper()
        down = r.current_ee_pose()
        down.position.z -= 0.12
        r.move_cartesian([down], velocity=0.15, avoid_collisions=False)
        if attach:
            r.detach_in_gazebo('bottle')
        r.open_gripper()
        r.sleep(1.0)
    results['bottle_start_z'] = bottle_z
    results['grasp'] = (gx, gy, gz)
    json.dump(dict(results=results, samples=e.samples), open(f'{OUT}/attach_experiment.json', 'w'))
    r.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
