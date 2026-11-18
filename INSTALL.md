# Installation: my_tiago_ws

Software needed to build and run this workspace, and how to install it on a fresh Ubuntu 22.04 machine.

## 1. Base system

| Component | Version |
|---|---|
| OS | Ubuntu 22.04.5 LTS |
| ROS 2 | Humble Hawksbill (`/opt/ros/humble`) |
| Gazebo | Gazebo Classic 11.10.2 |
| Python | 3.10 |

## 2. ROS packages (apt)

| Package | Version | Used for |
|---|---|---|
| ros-humble-gazebo-ros-pkgs | 3.9.0 | Gazebo ↔ ROS 2 |
| ros-humble-gazebo-ros2-control | 0.4.10 | ros2_control hardware inside Gazebo |
| ros-humble-pal-gazebo-plugins | 4.1.1 | Gazebo plugins referenced by the TIAGo URDF |
| ros-humble-ros2-controllers | 2.54.0 | diff drive, trajectory and broadcaster controllers |
| ros-humble-moveit | 2.5.10 | motion planning |
| ros-humble-navigation2, ros-humble-nav2-bringup | 1.1.20 | navigation, AMCL, Simple Commander |
| ros-humble-slam-toolbox | 2.6.10 | mapping |
| ros-humble-robot-localization | 3.5.4 | EKF odometry |
| ros-humble-twist-mux | 4.3.0 | velocity command priorities |
| ros-humble-teleop-twist-keyboard | 2.4.1 | driving by hand |
| ros-humble-joint-state-publisher-gui | 2.4.0 | joint sliders in the RViz model view |
| ros-humble-xacro | 2.1.1 | robot description |
| ros-humble-rmw-cyclonedds-cpp | 1.3.5 | DDS middleware |

On a fresh machine:

```bash
sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-pal-gazebo-plugins ros-humble-ros2-controllers ros-humble-moveit \
    ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-slam-toolbox \
    ros-humble-robot-localization ros-humble-twist-mux ros-humble-teleop-twist-keyboard \
    ros-humble-joint-state-publisher-gui ros-humble-xacro ros-humble-rmw-cyclonedds-cpp
```

The TIAGo description packages do not need to be installed: source copies are included in `src/pal_description_overlay`.

## 3. Built from source

### 3.1 IFRA_LinkAttacher

Gazebo's contact physics cannot hold an object between TIAGo's position-controlled gripper fingers: grasped objects slip out when the arm lifts. This Gazebo plugin creates a fixed joint between the gripper and the grasped object, and removes it when the object is released.

| Item | Value |
|---|---|
| Repository | https://github.com/IFRA-Cranfield/IFRA_LinkAttacher (Cranfield University, Apache-2.0) |
| Branch / commit | `humble` / `b056289ba93ccb549db98926dcbb9679642d0c8d` |
| Packages | `ros2_linkattacher` (Gazebo plugin `libgazebo_link_attacher.so`), `linkattacher_msgs` (services `/ATTACHLINK`, `/DETACHLINK`) |
| Location | `src/IFRA_LinkAttacher` |
| Loaded by | `src/my_tiago_bringup/worlds/my_tiago_room.world` |

The patched plugin is included in `src/IFRA_LinkAttacher` and is built with the rest of the workspace (section 5); nothing extra is needed. To recreate it from the original repository instead:

```bash
cd ~/my_tiago_ws/src
git clone -b humble https://github.com/IFRA-Cranfield/IFRA_LinkAttacher.git
cd IFRA_LinkAttacher && git checkout b056289ba93ccb549db98926dcbb9679642d0c8d
git apply ~/my_tiago_ws/patches/ifra_linkattacher_physics_lock.patch     # fix, see below
cd ~/my_tiago_ws && colcon build --symlink-install --packages-select linkattacher_msgs ros2_linkattacher
```

**Local patch** (`patches/ifra_linkattacher_physics_lock.patch`, already applied to the copy in `src/IFRA_LinkAttacher`): without it, gzserver could freeze or crash during a `/DETACHLINK` call. Two problems in `ros2_LinkAttacher/src/gazebo_link_attacher.cpp` are fixed:

1. The attach and detach services change joints from the ROS service thread while Gazebo's physics thread is stepping. Both now pause the world around the joint change (`world_->SetPaused(true)`, then the previous state), the mechanism `Model::RemoveJoint()` uses itself; holding the physics lock instead caused a deadlock.
2. Detaching removed the joint from Gazebo but not from the plugin's `GV_joints` list. Attaching the same object a second time and detaching it then used the old, deleted joint. The entry is now erased on detach.

## 4. Environment (~/.bashrc)

```bash
source /opt/ros/humble/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_LOCALHOST_ONLY=1
# CycloneDDS on loopback allows ~10 processes per domain by default; Gazebo + Nav2 + MoveIt need more
export CYCLONEDDS_URI='<CycloneDDS><Domain><Discovery><ParticipantIndex>auto</ParticipantIndex><MaxAutoParticipantIndex>100</MaxAutoParticipantIndex></Discovery></Domain></CycloneDDS>'
```

## 5. Build the workspace

```bash
cd ~/my_tiago_ws && colcon build --symlink-install && source install/setup.bash
```
