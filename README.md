# my_tiago_ws: TIAGo mobile manipulation in ROS 2

Author: **Promise Emeziem Adiole**

ROS 2 Humble workspace for a ROS 2 simulation task in mobile manipulation:

- **Exercise 1:** TIAGo starts in a corner of a room, drives in front of the middle table (Nav2) and moves its arm right above the table (MoveIt).
- **Exercise 2:** TIAGo moves a bottle and a cup from the middle table to a table in the corner, autonomously, finding the objects' 6-DOF poses with its head RGB-D camera.

Report (5 pages: decisions, results, sources): **[report.pdf](report.pdf)**. Full documentation: **[docs/my_tiago_documentation.pdf](docs/my_tiago_documentation.pdf)**. Installation: **[INSTALL.md](INSTALL.md)**.
Demo video (one run of each exercise, 2 minutes): **[video/demo.mp4](video/demo.mp4)**.

## Results

| Task | Script | Result (fresh simulation each run) |
|---|---|---|
| Exercise 1 | `my_tiago_tasks/exercise1.py` | 5 of 5 runs: gripper pointing down, fingertips 0.29–0.31 m above the table top; 41–66 s |
| Exercise 2 | `my_tiago_tasks/exercise2.py` | 4 of 4 runs: bottle and cup upright on the corner table (Gazebo ground truth), cup to the right of the bottle, 15–20 cm apart, at least 14 cm inside the table edges; 7–9 min |

## What is pre-built and what is my own

| Part | Source |
|---|---|
| TIAGo robot model (URDF meshes and macros) | PAL Robotics' description packages, as source copies in `src/pal_description_overlay`; one option added (`use_mock_hardware`) |
| Gazebo, ros2_control, MoveIt, Nav2, slam_toolbox, robot_localization, twist_mux | ROS 2 Humble binary packages |
| Gazebo link attacher plugin (holds grasped objects in simulation) | IFRA_LinkAttacher, Cranfield University (Apache-2.0), built from source with a local patch (`patches/`) |
| Simulation world (room, two tables, bottle, cup, furniture) | own: `src/my_tiago_bringup/worlds/my_tiago_room.world` |
| Map of the room | own, built with slam_toolbox: `src/my_tiago_bringup/maps/` |
| Robot wrapper, MoveIt configuration, controller, EKF, twist_mux, SLAM and Nav2 configuration, launch files | own: `my_tiago_description`, `my_tiago_moveit_config`, `my_tiago_bringup` |
| Task code: robot interface, perception, pick and place, the two exercises | own: `src/my_tiago_tasks` (Python) |

## Packages

| Package | Contents |
|---|---|
| `my_tiago_description` | `my_tiago.urdf.xacro` (TIAGo with PAL gripper and Schunk F/T sensor), RViz model view |
| `pal_description_overlay` | `tiago_description`, `pmb2_description`, `pal_gripper_description`, `pal_urdf_utils` |
| `my_tiago_moveit_config` | SRDF (groups `arm`, `arm_torso`, `gripper`), kinematics, MoveIt controllers |
| `my_tiago_bringup` | launch files (Gazebo, MoveIt, mapping, navigation), controllers, EKF, twist_mux, SLAM and Nav2 parameters, world, map, RViz configurations |
| `my_tiago_tasks` | `robot.py`, `perception.py`, `pick_place.py`, `exercise1.py`, `exercise2.py`, test scripts |
| `IFRA_LinkAttacher` | Gazebo link attacher plugin and its service messages |

## Build

Requirements: Ubuntu 22.04, ROS 2 Humble, Gazebo 11 and the packages listed in [INSTALL.md](INSTALL.md), including the `~/.bashrc` settings for CycloneDDS.

```bash
cd ~/my_tiago_ws
colcon build --symlink-install
source install/setup.bash
```

## Run the exercises

Use normal Ubuntu terminals, each starting with `cd ~/my_tiago_ws && source install/setup.bash`. Start the simulation fresh (robot in its corner, objects in place) before each run.

```bash
# Terminal 1: simulation (wait for "Configured and activated ft_sensor_controller")
ros2 launch my_tiago_bringup my_tiago_gazebo.launch.xml

# Terminal 2: navigation (wait for "Managed nodes are active" twice)
ros2 launch my_tiago_bringup my_tiago_navigation.launch.xml

# Terminal 3: MoveIt (wait for "You can start planning now!")
ros2 launch my_tiago_bringup my_tiago_moveit_sim.launch.xml

# Terminal 4: one exercise
ros2 run my_tiago_tasks exercise1      # about 1 minute
ros2 run my_tiago_tasks exercise2      # about 7 minutes
```

Exercise 1 ends with `Exercise 1 complete: the gripper is above the table.`
Exercise 2 ends with `Exercise 2 result: bottle=placed, cup=placed`.
Both scripts exit with code 0 on success.

Test scripts: `ros2 run my_tiago_tasks robot_demo` (arm, head, gripper), `perception_demo` (detections compared with the world file), `pick_demo [bottle|cup]` (pick one object and put it back), `go_to_pose X Y [YAW_DEG]` (Nav2 goal).

## License

My own code and documentation are released under the [Apache License 2.0](LICENSE).
Third-party packages keep their own licences: PAL Robotics' description packages in `src/pal_description_overlay` (Apache-2.0 / BSD) and IFRA_LinkAttacher (Apache-2.0, see `src/IFRA_LinkAttacher/LICENSE.md`).
