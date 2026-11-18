#!/usr/bin/env python3
"""Generate the Graphviz diagrams (SVG) used in my_tiago_documentation.html.

Run:  python3 make_diagrams.py      (needs the `dot` command, package graphviz)
Output: diagrams/*.svg
"""
import os
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, 'diagrams')

# Shared look: one font, soft fills, colour by role
NAVY = '#1f3a5f'
STYLE = f'''
  graph [fontname="Helvetica", fontsize=11, bgcolor="transparent", pad=0.2, nodesep=0.35, ranksep=0.45];
  node  [fontname="Helvetica", fontsize=10, shape=box, style="rounded,filled", fillcolor="#eef3f8", color="{NAVY}", penwidth=1.1];
  edge  [fontname="Helvetica", fontsize=9, color="#4a5868", arrowsize=0.7];
'''
# role colours
C_PAL = '#fde9cf'     # PAL / upstream package
C_OWN = '#d9f0e3'     # package written in this workspace
C_ROS = '#e6e9f5'     # ROS / Nav2 / MoveIt node
C_SIM = '#f6dede'     # Gazebo
C_SENS = '#fff6c9'    # sensor
C_TOPIC = '#ffffff'

DIAGRAMS = {}

# ---------------------------------------------------------------- 1. robot parts
DIAGRAMS['robot_kinematics'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.22; ranksep=0.32;
  base   [label="base_footprint / base_link\\nPMB2 mobile base", fillcolor="{C_PAL}"];
  wl     [label="wheel_left_link"];  wr [label="wheel_right_link"];
  cast   [label="4 casters, 2 suspensions\\n(fixed in URDF)", fillcolor="#f2f2f2", color="#99aaaa"];
  sens   [label="base sensors: laser (SICK 571),\\nIMU, 3 sonars", fillcolor="{C_SENS}"];
  torso_f[label="torso_fixed_link"];
  torso  [label="torso_lift_link"];
  a1 [label="arm_1_link"]; a2 [label="arm_2_link"]; a3 [label="arm_3_link"]; a4 [label="arm_4_link"];
  a5 [label="arm_5_link"]; a6 [label="arm_6_link"]; a7 [label="arm_7_link"];
  tool [label="arm_tool_link\\n(MoveIt tip)"];
  ft [label="wrist_ft_link\\n(Schunk F/T)", fillcolor="{C_SENS}"];
  grip [label="gripper_link"];
  fl [label="left finger"]; fr [label="right finger"];
  h1 [label="head_1_link"]; h2 [label="head_2_link"];
  cam[label="RGB-D camera\\n(Orbbec Astra)", fillcolor="{C_SENS}"];
  base -> wl [label="wheel_left_joint\\n(continuous)", color="#c0392b", fontcolor="#c0392b"];
  base -> wr [label="wheel_right_joint\\n(continuous)", color="#c0392b", fontcolor="#c0392b"];
  base -> cast [style=dashed]; base -> sens [style=dashed];
  base -> torso_f [style=dashed];
  torso_f -> torso [label=" torso_lift_joint\\n prismatic", color="#8e44ad", fontcolor="#8e44ad"];
  torso -> a1 [label=" arm_1", color="#1f6fb2", fontcolor="#1f6fb2"];
  a1 -> a2 [label=" arm_2", color="#1f6fb2", fontcolor="#1f6fb2"];
  a2 -> a3 [label=" arm_3", color="#1f6fb2", fontcolor="#1f6fb2"];
  a3 -> a4 [label=" arm_4", color="#1f6fb2", fontcolor="#1f6fb2"];
  a4 -> a5 [label=" arm_5", color="#1f6fb2", fontcolor="#1f6fb2"];
  a5 -> a6 [label=" arm_6", color="#1f6fb2", fontcolor="#1f6fb2"];
  a6 -> a7 [label=" arm_7", color="#1f6fb2", fontcolor="#1f6fb2"];
  a7 -> tool [style=dashed]; tool -> ft [style=dashed]; ft -> grip [style=dashed];
  grip -> fl [label=" left_finger\\n prismatic", color="#16a085", fontcolor="#16a085"];
  grip -> fr [label=" right_finger\\n prismatic", color="#16a085", fontcolor="#16a085"];
  torso -> h1 [label=" head_1 (pan)", color="#d35400", fontcolor="#d35400"];
  h1 -> h2 [label=" head_2 (tilt)", color="#d35400", fontcolor="#d35400"];
  h2 -> cam [style=dashed];
}}"""

# ---------------------------------------------------------------- 2. workspace packages
DIAGRAMS['workspace_packages'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.2; ranksep=0.4;
  subgraph cluster_ws {{ label="~/my_tiago_ws/src  (this project)"; style="rounded,dashed"; color="#2e7d4f"; fontcolor="#2e7d4f";
    tasks [label="my_tiago_tasks (ament_python)\\nrobot, perception, pick_place,\\nexercise1, exercise2", fillcolor="{C_OWN}"];
    ifra [label="IFRA_LinkAttacher (C++,\\nCranfield, + local patch)", fillcolor="{C_SIM}"];
    bringup [label="my_tiago_bringup\\nlaunch, controllers, EKF,\\ntwist_mux, SLAM, Nav2, maps", fillcolor="{C_OWN}"];
    moveit [label="my_tiago_moveit_config\\nSRDF, kinematics,\\nMoveIt controllers", fillcolor="{C_OWN}"];
    desc [label="my_tiago_description\\nmy_tiago.urdf.xacro", fillcolor="{C_OWN}"];
    subgraph cluster_ov {{ label="pal_description_overlay"; style="rounded"; color="#b9770e"; fontcolor="#b9770e";
      td [label="tiago_description\\n(+ use_mock_hardware)", fillcolor="{C_PAL}"];
      pmb2 [label="pmb2_\\ndescription", fillcolor="{C_PAL}"];
      grip [label="pal_gripper_\\ndescription", fillcolor="{C_PAL}"];
      utils [label="pal_urdf_utils", fillcolor="{C_PAL}"];
    }}
  }}
  subgraph cluster_opt {{ label="/opt/ros/humble (apt binaries)"; style="rounded,dashed"; color="#555555"; fontcolor="#555555";
    nav2 [label="Nav2 1.1.20", fillcolor="{C_ROS}"];
    slam [label="slam_toolbox", fillcolor="{C_ROS}"];
    rl [label="robot_\\nlocalization", fillcolor="{C_ROS}"];
    mux [label="twist_mux", fillcolor="{C_ROS}"];
    gz [label="gazebo_ros,\\ngazebo_ros2_control", fillcolor="{C_SIM}"];
    rc [label="ros2_control", fillcolor="{C_ROS}"];
    mi [label="MoveIt 2.5", fillcolor="{C_ROS}"];
  }}
  nav2 -> rc [style=invis]; slam -> mi [style=invis]; rl -> gz [style=invis];
  tasks -> nav2 [color="#99aaaa"]; tasks -> ifra [color="#99aaaa"]; bringup -> ifra [color="#99aaaa"];
  tasks -> bringup [style=invis];
  bringup -> moveit; bringup -> desc;
  bringup -> {{nav2 slam rl mux gz rc}} [color="#99aaaa"];
  moveit -> desc; moveit -> mi [color="#99aaaa"];
  desc -> td; td -> pmb2; td -> grip; td -> utils;
}}"""

# ---------------------------------------------------------------- 3. overlay
DIAGRAMS['overlay'] = f'''
digraph G {{ {STYLE}
  rankdir=LR;
  q [label="$(find tiago_description)\\nros2 pkg prefix tiago_description", shape=note, fillcolor="#ffffff"];
  ws [label="1st: ~/my_tiago_ws/install/tiago_description\\n(overlay, built from pal_description_overlay)", fillcolor="{C_OWN}"];
  opt [label="2nd: /opt/ros/humble/share/tiago_description\\n(underlay, apt package)", fillcolor="#eeeeee", fontcolor="#777777", color="#999999"];
  q -> ws [label=" found first in\\n AMENT_PREFIX_PATH", color="#2e7d4f", fontcolor="#2e7d4f", penwidth=1.6];
  q -> opt [label=" shadowed\\n (never used)", style=dashed, color="#999999", fontcolor="#999999"];
}}'''

# ---------------------------------------------------------------- 4. hardware selection
DIAGRAMS['hardware_selection'] = f'''
digraph G {{ {STYLE}
  rankdir=LR;
  x [label="my_tiago.urdf.xacro\\n+ xacro arguments", fillcolor="{C_OWN}"];
  m [label="mock_components/GenericSystem\\ncommands copied to states\\n(MoveIt tests, no physics)", fillcolor="{C_ROS}"];
  g [label="gazebo_ros2_control/GazeboSystem\\njoints driven by Gazebo physics", fillcolor="{C_SIM}"];
  r [label="robot_control/RobotControl\\nPAL real-robot hardware", fillcolor="#eeeeee"];
  x -> m [label=" use_mock_hardware:=true"];
  x -> g [label=" use_sim_time:=true"];
  x -> r [label=" (default: real robot)"];
  cm [label="controller_manager\\n+ the same 7 controllers\\n(my_tiago_controllers.yaml)", fillcolor="{C_OWN}"];
  m -> cm; g -> cm; r -> cm [style=dashed];
}}'''

# ---------------------------------------------------------------- 5. ros2_control
DIAGRAMS['ros2_control'] = f"""
digraph G {{ {STYLE}
  rankdir=LR; nodesep=0.12; ranksep=0.45;
  fjt [label="<controller>/follow_joint_trajectory\\n(action, used by MoveIt)", shape=ellipse, fillcolor="{C_TOPIC}"];
  cv [label="/mobile_base_controller/\\ncmd_vel_unstamped", shape=ellipse, fillcolor="{C_TOPIC}"];
  js [label="/joint_states", shape=ellipse, fillcolor="{C_TOPIC}"];
  wr [label="/ft_sensor_controller/wrench", shape=ellipse, fillcolor="{C_TOPIC}"];
  od [label="/mobile_base_controller/odom", shape=ellipse, fillcolor="{C_TOPIC}"];
  subgraph cluster_cm {{ label="controller_manager, 100 Hz\\n(inside gzserver)"; style="rounded"; color="{NAVY}";
    jsb [label="joint_state_broadcaster", fillcolor="{C_ROS}"];
    ftb [label="ft_sensor_controller", fillcolor="{C_ROS}"];
    arm [label="arm_controller", fillcolor="{C_ROS}"];
    tor [label="torso_controller", fillcolor="{C_ROS}"];
    grp [label="gripper_controller", fillcolor="{C_ROS}"];
    hd  [label="head_controller", fillcolor="{C_ROS}"];
    dd  [label="mobile_base_controller", fillcolor="{C_ROS}"];
  }}
  hw [label="Hardware interface\\nGazeboSystem\\n\\ncommand:\\nposition / velocity\\n\\nstate:\\nposition, velocity,\\neffort, F/T", fillcolor="{C_SIM}", shape=box3d, height=3.2];
  js -> jsb [dir=back]; wr -> ftb [dir=back];
  fjt -> arm; fjt -> tor; fjt -> grp; fjt -> hd;
  cv -> dd; od -> dd [dir=back];
  {{jsb ftb}} -> hw [dir=back, color="#999999"];
  {{arm tor grp hd dd}} -> hw [dir=both, color="#999999"];
}}"""

# ---------------------------------------------------------------- 6. gazebo launch graph
DIAGRAMS['gazebo_launch'] = f"""
digraph G {{ {STYLE}
  rankdir=LR; nodesep=0.12; ranksep=0.5;
  L [label="my_tiago_gazebo\\n.launch.xml", fillcolor="{C_OWN}", shape=folder];
  env [label="1  set_env GAZEBO_MODEL_PATH, GAZEBO_RESOURCE_PATH", shape=note, fillcolor="#ffffff"];
  xa  [label="   xacro my_tiago.urdf.xacro use_sim_time:=true is_public_sim:=true", shape=note, fillcolor="#ffffff"];
  gzs [label="2  gzserver (world) + controller_manager plugin", fillcolor="{C_SIM}"];
  gzc [label="2  gzclient (GUI)", fillcolor="{C_SIM}"];
  rsp [label="3  robot_state_publisher  ->  /robot_description, TF", fillcolor="{C_ROS}"];
  sp  [label="4  spawn_entity.py  (reads /robot_description, inserts 'tiago')", fillcolor="{C_ROS}"];
  spn [label="5  spawner: 7 controllers from my_tiago_controllers.yaml", fillcolor="{C_ROS}"];
  mux [label="6  twist_mux  (my_tiago_twist_mux.yaml)", fillcolor="{C_ROS}"];
  ekf [label="7  ekf_filter_node  (my_tiago_ekf.yaml)", fillcolor="{C_ROS}"];
  L -> {{env xa gzs gzc rsp sp spn mux ekf}};
}}"""

# ---------------------------------------------------------------- 7. velocity path
DIAGRAMS['cmd_vel'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.5; ranksep=0.45;
  nav [label="Nav2\\nvelocity_smoother, behavior_server", fillcolor="{C_ROS}"];
  tel [label="teleop_twist_keyboard\\n-r cmd_vel:=/key_vel", fillcolor="{C_ROS}"];
  mux [label="twist_mux\\nhighest active priority wins (timeout 0.5 s)", fillcolor="{C_OWN}"];
  dd  [label="mobile_base_controller (DiffDriveController)\\nspeed and acceleration limits", fillcolor="{C_ROS}"];
  wh  [label="wheel joints (Gazebo)", fillcolor="{C_SIM}"];
  out [label="/mobile_base_controller/cmd_vel_out\\n(the limited command actually applied)", shape=ellipse, fillcolor="{C_TOPIC}"];
  nav -> mux [label=" /cmd_vel   priority 10"];
  tel -> mux [label=" /key_vel   priority 90"];
  mux -> dd [label=" cmd_vel_out remapped to\\n /mobile_base_controller/cmd_vel_unstamped"];
  dd -> wh [label=" wheel velocities"];
  dd -> out [style=dashed];
}}"""

# ---------------------------------------------------------------- 8. SLAM
DIAGRAMS['slam'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.4; ranksep=0.35;
  laser [label="/scan_raw (laser, 10 Hz)", shape=ellipse, fillcolor="{C_SENS}"];
  tf [label="TF odom -> base_footprint (EKF)", shape=ellipse, fillcolor="{C_TOPIC}"];
  st [label="slam_toolbox (online_async)\\nscan matching + pose graph + loop closure (Ceres)", fillcolor="{C_ROS}"];
  map [label="/map  (OccupancyGrid, 0.05 m)", shape=ellipse, fillcolor="{C_TOPIC}"];
  tf2 [label="TF map -> odom", shape=ellipse, fillcolor="{C_TOPIC}"];
  ms [label="map_saver_cli -> maps/my_tiago_room.pgm + .yaml", fillcolor="{C_OWN}"];
  rv [label="RViz (mapping.rviz)", fillcolor="{C_ROS}"];
  laser -> st; tf -> st; st -> map; st -> tf2; map -> ms; map -> rv;
}}"""

# ---------------------------------------------------------------- 9. TF tree
DIAGRAMS['tf_tree'] = f'''
digraph G {{ {STYLE}
  rankdir=TB;
  node [shape=ellipse, fillcolor="#ffffff"];
  map -> odom [label="  AMCL (navigation)\\n  or slam_toolbox (mapping)", color="#c0392b", fontcolor="#c0392b", penwidth=1.6];
  odom -> base_footprint [label="  EKF  ekf_filter_node", color="#1f6fb2", fontcolor="#1f6fb2", penwidth=1.6];
  base_footprint -> base_link [label="  robot_state_publisher\\n  (fixed joints, /tf_static)", color="#555555", fontcolor="#555555"];
  base_link -> base_laser_link; base_link -> base_imu_link; base_link -> "wheel_*_link" [label=" RSP (from /joint_states)", fontcolor="#555555"];
  base_link -> torso_fixed_link -> torso_lift_link;
  torso_lift_link -> "arm_1_link ... arm_7_link" -> arm_tool_link -> wrist_ft_link -> wrist_ft_tool_link -> gripper_link -> "gripper_*_finger_link";
  torso_lift_link -> "head_1_link" -> "head_2_link" -> "head_front_camera_*";
}}'''

# ---------------------------------------------------------------- 10. EKF
DIAGRAMS['ekf'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.5; ranksep=0.4;
  odom [label="/mobile_base_controller/odom\\n(wheel odometry)", shape=ellipse, fillcolor="{C_TOPIC}"];
  imu  [label="/base_imu\\n(IMU, 100 Hz)", shape=ellipse, fillcolor="{C_SENS}"];
  ekf  [label="ekf_filter_node (robot_localization), 50 Hz, two_d_mode\\npredict with motion model, update with each measurement", fillcolor="{C_OWN}"];
  out  [label="/odometry/filtered", shape=ellipse, fillcolor="{C_TOPIC}"];
  tf   [label="TF odom -> base_footprint", shape=ellipse, fillcolor="{C_TOPIC}"];
  amcl [label="AMCL: laser + map -> TF map -> odom", fillcolor="{C_ROS}"];
  odom -> ekf [label=" vx, vy only\\n (wheels slip when turning)"];
  imu  -> ekf [label=" yaw, yaw rate\\n (relative to start)"];
  ekf -> out; ekf -> tf; tf -> amcl [label=" motion prediction"];
}}"""

# ---------------------------------------------------------------- 11. Nav2
DIAGRAMS['nav2'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.3; ranksep=0.4;
  goal [label="Goal: RViz 'Nav2 Goal' or go_to_pose.py", fillcolor="{C_OWN}"];
  subgraph cluster_loc {{ label="localization"; style="rounded"; color="{NAVY}";
    ms [label="map_server\\nmy_tiago_room.yaml", fillcolor="{C_ROS}"];
    amcl [label="amcl\\nparticle filter", fillcolor="{C_ROS}"];
  }}
  subgraph cluster_nav {{ label="navigation"; style="rounded"; color="{NAVY}";
    bt [label="bt_navigator\\nnavigate_to_pose.xml", fillcolor="{C_ROS}"];
    pl [label="planner_server\\nNavFn + global costmap", fillcolor="{C_ROS}"];
    ct [label="controller_server\\nDWB + local costmap", fillcolor="{C_ROS}"];
    bh [label="behavior_server\\nwait / backup / spin", fillcolor="{C_ROS}"];
    sm [label="velocity_smoother", fillcolor="{C_ROS}"];
  }}
  mux [label="twist_mux -> base controller", fillcolor="{C_OWN}"];
  goal -> bt [label=" NavigateToPose action"];
  ms -> amcl [label=" /map"];
  amcl -> bt [label=" TF map->odom", style=dashed];
  bt -> pl [label=" ComputePathToPose"]; bt -> ct [label=" FollowPath"]; bt -> bh [label=" recovery"];
  ct -> sm [label=" cmd_vel_nav"]; sm -> mux [label=" /cmd_vel"]; bh -> mux [label=" /cmd_vel"];
}}"""

# ---------------------------------------------------------------- 12. go_to_pose flow
DIAGRAMS['go_to_pose'] = f'''
digraph G {{ {STYLE}
  rankdir=TB;
  a [label="parse x, y, yaw (deg)\\nargparse", fillcolor="{C_OWN}"];
  b [label="BasicNavigator(node_name='go_to_pose')", fillcolor="{C_OWN}"];
  c [label="wait until 'amcl' and 'bt_navigator' are active\\n(NOT waitUntilNav2Active: it would reset AMCL to 0,0,0)", fillcolor="{C_OWN}"];
  d [label="make_pose(): PoseStamped in 'map'\\nq = (0, 0, sin(yaw/2), cos(yaw/2))", fillcolor="{C_OWN}"];
  e [label="goToPose(goal)  ->  NavigateToPose action", fillcolor="{C_ROS}"];
  f [label="isTaskComplete()?  (spins 0.1 s)\\nprint distance_remaining every ~1 s", fillcolor="{C_ROS}", shape=diamond, style="filled"];
  g [label="getResult(): SUCCEEDED / CANCELED / FAILED\\nexit code 0 or 1", fillcolor="{C_OWN}"];
  a -> b -> c -> d -> e -> f; f -> f [label=" no"]; f -> g [label=" yes"];
}}'''

# ---------------------------------------------------------------- 13. workflow
DIAGRAMS['workflow'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.18; ranksep=0.3; node [fillcolor="{C_OWN}", width=1.15];
  s1 [label="1. Robot model\\nxacro + PAL\\noverlay"]; s2 [label="2. Visualise\\nRViz + joint\\nsliders"];
  s3 [label="3. MoveIt\\nconfig"]; s4 [label="4. ros2_control\\nown controllers"];
  s5 [label="5. Gazebo\\nworld, spawn,\\ndrive"]; s6 [label="6. SLAM\\nmap the room"];
  s7 [label="7. Nav2\\nAMCL, twist_mux"]; s8 [label="8. EKF\\nIMU-aided\\nodometry"];
  s9 [label="9. Simple\\nCommander\\ngo_to_pose"]; s10 [label="10. MoveIt in\\nGazebo +\\nrobot.py"];
  s11 [label="11. Exercise 1\\narm above\\nthe table", fillcolor="#bfe3cc"]; s12 [label="12. Perception\\n6-DOF poses"];
  s13 [label="13. Grasping\\nlink attacher"]; s14 [label="14. Exercise 2\\nmove both\\nobjects", fillcolor="#bfe3cc"];
  {{rank=same; s1; s2; s3; s4; s5; s6; s7}}
  {{rank=same; s8; s9; s10; s11; s12; s13; s14}}
  s1 -> s2 -> s3 -> s4 -> s5 -> s6 -> s7; s8 -> s9 -> s10 -> s11 -> s12 -> s13 -> s14;
  s1 -> s8 [style=invis]; s2 -> s9 [style=invis]; s3 -> s10 [style=invis]; s4 -> s11 [style=invis];
  s5 -> s12 [style=invis]; s6 -> s13 [style=invis]; s7 -> s14 [style=invis];
  s7 -> s8 [constraint=false, color="#2e7d4f"];
}}"""

# ---------------------------------------------------------------- 14. robot interface
DIAGRAMS['robot_interface'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.15; ranksep=0.45;
  subgraph cluster_scripts {{ label="my_tiago_tasks (Python)"; style="rounded,dashed"; color="#2e7d4f"; fontcolor="#2e7d4f";
    ex1 [label="exercise1.py", fillcolor="{C_OWN}"];
    ex2 [label="exercise2.py", fillcolor="{C_OWN}"];
    pp  [label="pick_place.py\\nPickAndPlace", fillcolor="{C_OWN}"];
    per [label="perception.py\\nTabletopPerception", fillcolor="{C_OWN}"];
    demos [label="robot_demo, perception_demo,\\npick_demo, go_to_pose", fillcolor="{C_OWN}"];
  }}
  robot [label="robot.py\\nTiagoRobot (one node,\\nMultiThreadedExecutor\\nin a background thread)", fillcolor="{C_OWN}", shape=box3d];
  mg  [label="move_group (MoveIt)\\n/move_action\\n/compute_cartesian_path\\n/execute_trajectory\\n/apply_planning_scene", fillcolor="{C_ROS}"];
  ctl [label="gripper_controller, head_controller\\n.../follow_joint_trajectory", fillcolor="{C_ROS}"];
  nav [label="Nav2 (BasicNavigator)\\nNavigateToPose, BackUp", fillcolor="{C_ROS}"];
  tf  [label="TF2 buffer\\n/joint_states", shape=ellipse, fillcolor="{C_TOPIC}"];
  att [label="IFRA link attacher\\n/ATTACHLINK, /DETACHLINK", fillcolor="{C_SIM}"];
  cam [label="/head_front_camera/\\ndepth/rgb/points", shape=ellipse, fillcolor="{C_SENS}"];
  ex1 -> robot; ex2 -> pp; pp -> robot; pp -> per; demos -> robot; per -> robot [label=" TF, node"]; per -> cam [label=" one cloud"];
  robot -> mg; robot -> ctl; robot -> nav; robot -> tf; robot -> att;
}}"""

# ---------------------------------------------------------------- 15. perception pipeline
DIAGRAMS['perception'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.3; ranksep=0.28;
  a [label="1  one point cloud (640x480 RGB-D) -> base_footprint (TF)", fillcolor="{C_SENS}"];
  b [label="2  crop to the workspace; self-filter: drop points within 9 cm of the arm and 14 cm of the hand", fillcolor="{C_OWN}"];
  c [label="3  RANSAC: largest horizontal plane -> table height and normal", fillcolor="{C_OWN}"];
  d [label="4  PCA of the plane points -> table centre, direction, size, near edge;  check 0.6 < z < 0.9 m, >= 25 cm wide", fillcolor="{C_OWN}"];
  e [label="5  points 1.2-45 cm above the plane, inside the outline -> clusters on a 1 cm grid", fillcolor="{C_OWN}"];
  f [label="6  per cluster: circle fit -> axis (x, y), radius;  highest point -> height;  mean colour -> label;\\nhandle (points outside the circle) -> yaw;  table normal -> roll, pitch", fillcolor="{C_OWN}"];
  g [label="DetectedObject: label, 6-DOF pose, radius, height   +   markers on /detected_objects", fillcolor="{C_ROS}"];
  a -> b -> c -> d -> e -> f -> g;
}}"""

# ---------------------------------------------------------------- 16. pick sequence
DIAGRAMS['pick'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.3; ranksep=0.25;
  p1 [label="look at the table, detect table + objects", fillcolor="{C_OWN}"];
  p2 [label="object front-right of the base?\\n(0.70 m ahead, 0.10 m right, +-0.12 m)", shape=diamond, style="filled", fillcolor="#ffffff"];
  p2b [label="back up, drive to a pose computed from the\\nobject's map position, detect again", fillcolor="{C_ROS}"];
  p3 [label="MoveIt scene: table box + objects (cylinders)", fillcolor="{C_OWN}"];
  p4 [label="for each approach direction (handle-aware, then line of sight, 0, +-17 deg):\\npre-grasp 15 cm before the object (MoveIt plan)\\n-> straight approach (Cartesian, IK-flip guard)", fillcolor="{C_OWN}"];
  p5 [label="close the fingers 4 mm short of the surface\\n-> weld in Gazebo (/ATTACHLINK) -> attach in MoveIt", fillcolor="{C_SIM}"];
  p6 [label="lift 10 cm (Cartesian)", fillcolor="{C_OWN}"];
  p7 [label="camera check: table seen and object gone?", shape=diamond, style="filled", fillcolor="#ffffff"];
  ok [label="picked: (object, grasp height)", fillcolor="{C_ROS}"];
  no [label="detach, open, back off -> retry (up to 3 attempts)", fillcolor="#fbe7e5"];
  p1 -> p2; p2 -> p3 [label=" yes"]; p2 -> p2b [label=" no"]; p2b -> p3;
  p3 -> p4 -> p5 -> p6 -> p7; p7 -> ok [label=" yes"]; p7 -> no [label=" no"];
}}"""

# ---------------------------------------------------------------- 17. place sequence
DIAGRAMS['place'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.3; ranksep=0.25;
  q1 [label="look at the corner table, detect table + objects already there", fillcolor="{C_OWN}"];
  q2 [label="choose a spot: inside the outline (10 cm margin), 14 cm from objects,\\nnot behind an object, closest to 18 cm behind the edge, slightly right", fillcolor="{C_OWN}"];
  q3 [label="for each approach direction: pre-place 10 cm above (MoveIt)\\n-> straight down to 1.5 cm above the table (Cartesian)", fillcolor="{C_OWN}"];
  q4 [label="release: /DETACHLINK -> open -> detach in MoveIt", fillcolor="{C_SIM}"];
  q5 [label="retreat: straight up until the fingers are 5 cm above the object's top,\\nthen back 20 cm (MoveIt plan if the straight lift is refused)", fillcolor="{C_OWN}"];
  q6 [label="placed object becomes an obstacle; head up; stow the arm\\nwith the scene still in place; then clear the scene", fillcolor="{C_OWN}"];
  q7 [label="camera check: object on the table?", fillcolor="{C_ROS}"];
  q1 -> q2 -> q3 -> q4 -> q5 -> q6 -> q7;
}}"""

# ---------------------------------------------------------------- 18. exercise 2
DIAGRAMS['exercise2'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.35; ranksep=0.3; node [fillcolor="{C_OWN}"];
  s [label="start\\n(room corner)\\nstow arm", fillcolor="{C_ROS}"];
  a [label="drive to the\\nmiddle table"];
  b [label="pick\\n(section 15)"];
  c [label="carry pose,\\nback up"];
  d [label="drive to the\\ncorner table"];
  e [label="place\\n(section 15)"];
  f [label="next object?", shape=diamond, style="filled", fillcolor="#ffffff"];
  g [label="result:\\nbottle=placed,\\ncup=placed", fillcolor="{C_ROS}"];
  {{rank=same; a; b; c}} {{rank=same; d; e; f}}
  s -> a -> b -> c; c -> d; d -> e -> f; f -> a [label=" cup (back up first)"]; f -> g [label=" done"];
}}"""


# ---------------------------------------------------------------- 19. system overview (report)
DIAGRAMS['system_overview'] = f"""
digraph G {{ {STYLE}
  rankdir=LR; nodesep=0.18; ranksep=0.42;
  subgraph cluster_sim {{ label="Gazebo Classic 11"; style="rounded"; color="#b03a2e"; fontcolor="#b03a2e";
    world [label="my_tiago_room.world\\nroom, 2 tables,\\nbottle, cup", fillcolor="{C_SIM}"];
    robot [label="TIAGo (PAL URDF)\\nros2_control: 7 controllers\\nlaser, IMU, RGB-D camera", fillcolor="{C_SIM}"];
    att [label="link attacher\\n(patched)", fillcolor="{C_SIM}"];
  }}
  ekf [label="EKF\\nwheels + IMU", fillcolor="{C_ROS}"];
  nav [label="Nav2 + AMCL\\non the SLAM map", fillcolor="{C_ROS}"];
  mux [label="twist_mux", fillcolor="{C_ROS}"];
  mg [label="MoveIt\\nmove_group", fillcolor="{C_ROS}"];
  subgraph cluster_py {{ label="my_tiago_tasks (Python)"; style="rounded,dashed"; color="#2e7d4f"; fontcolor="#2e7d4f";
    rb [label="robot.py", fillcolor="{C_OWN}"];
    per [label="perception.py\\n6-DOF poses", fillcolor="{C_OWN}"];
    pp [label="pick_place.py", fillcolor="{C_OWN}"];
    ex [label="exercise1.py\\nexercise2.py", fillcolor="{C_OWN}"];
  }}
  robot -> ekf [label=" odom, IMU"]; ekf -> nav [label=" TF"]; robot -> nav [label=" laser"];
  nav -> mux -> robot [label=" base"];
  mg -> robot [label=" arm, torso,\\n gripper"];
  robot -> per [label=" point cloud"];
  ex -> pp -> rb; ex -> rb; pp -> per;
  rb -> nav; rb -> mg; rb -> att; rb -> robot [label=" head, gripper", style=dashed];
}}"""


# ------------------------------------------------- manual start of the Gazebo launch
DIAGRAMS['manual_launch'] = f"""
digraph G {{ {STYLE}
  rankdir=LR; nodesep=0.14; ranksep=0.55;
  subgraph cluster_cmd {{ label="terminal commands (section 7.4)"; style="rounded,dashed"; color="#8899aa"; fontsize=10;
    t1 [label="T1  gzserver my_tiago_room.world\\n-s libgazebo_ros_init/factory/force_system", fillcolor="{C_SIM}"];
    t2 [label="T2  gzclient", fillcolor="{C_SIM}"];
    t3 [label="T3  ros2 run robot_state_publisher\\n-p robot_description:=\\"$(xacro ...)\\"", fillcolor="{C_ROS}"];
    t4 [label="T4  ros2 run gazebo_ros spawn_entity.py\\n-topic robot_description -entity tiago", fillcolor="{C_ROS}"];
    t5 [label="T5  ros2 run controller_manager spawner\\n7 controllers -p my_tiago_controllers.yaml", fillcolor="{C_ROS}"];
    t6 [label="T6  ros2 run twist_mux twist_mux", fillcolor="{C_ROS}"];
    t7 [label="T7  ros2 run robot_localization ekf_node", fillcolor="{C_ROS}"];
    {{ rank=same; t1; t2; t3; t4; t5; t6; t7; }}
    t1 -> t2 -> t3 -> t4 -> t5 -> t6 -> t7 [style=invis];
  }}
  node [shape=ellipse, fillcolor="#ffffff"];
  gz   [label="/gazebo\\n(+ /spawn_entity service)", fillcolor="{C_SIM}"];
  cm   [label="/controller_manager\\n(gazebo_ros2_control plugin,\\nstarts after the spawn)", fillcolor="{C_SIM}"];
  rsp  [label="/robot_state_publisher"];
  ctrl [label="7 controllers:\\njoint_state_broadcaster, arm, torso,\\ngripper, head, mobile_base, ft_sensor", fillcolor="{C_SIM}"];
  mux  [label="/twist_mux"];
  ekf  [label="/ekf_filter_node"];
  node [shape=box, style="filled", fillcolor="{C_TOPIC}", color="#8899aa", fontsize=9];
  rd [label="/robot_description"]; js [label="/joint_states"]; tf [label="/tf, /tf_static"];
  clk [label="/clock"]; imu [label="/base_imu"]; cv [label="/mobile_base_controller/\\ncmd_vel_unstamped"];
  kv [label="/cmd_vel, /key_vel"]; od [label="/mobile_base_controller/odom"]; of [label="/odometry/filtered"];
  t1 -> gz [style=dashed]; t2 -> gz [style=dashed, label="GUI of"]; t3 -> rsp [style=dashed];
  t4 -> gz [style=dashed, label="calls /spawn_entity"]; t5 -> ctrl [style=dashed, label="load + activate"];
  t6 -> mux [style=dashed]; t7 -> ekf [style=dashed];
  rsp -> rd; rd -> t4 [label="URDF"]; gz -> cm [label="robot spawned"]; cm -> ctrl;
  gz -> clk; gz -> imu; ctrl -> js; js -> rsp; rsp -> tf;
  kv -> mux; mux -> cv; cv -> ctrl; ctrl -> od; od -> ekf; imu -> ekf; ekf -> of; ekf -> tf [label="odom->base_footprint"];
}}
"""


# ------------------------------------------------- link attacher patch: threads
DIAGRAMS['attacher_threads'] = f"""
digraph G {{ {STYLE}
  rankdir=TB; nodesep=0.25; ranksep=0.32; newrank=true;
  subgraph cluster_a {{ label="(a) BEFORE the patch: two threads change the same joints"; fontcolor="#c0392b";
    style="rounded"; color="#c0392b";
    a1 [label="ROS service thread\\n/DETACHLINK\\n-> joint->Detach(), RemoveJoint()", fillcolor="{C_ROS}"];
    a2 [label="Gazebo physics thread\\nstep: reads every joint\\n(1000 steps per second)", fillcolor="{C_SIM}"];
    a3 [label="the joint is deleted\\nin the middle of a step", shape=box, style="rounded,filled", fillcolor="#f6dede", color="#c0392b"];
    a4 [label="gzserver freezes or crashes", shape=octagon, style="filled", fillcolor="#c0392b", fontcolor="white"];
    a1 -> a3; a2 -> a3; a3 -> a4;
  }}
  subgraph cluster_b {{ label="(b) FIRST FIX (rejected): hold the physics mutex"; fontcolor="#d35400";
    style="rounded"; color="#d35400";
    b1 [label="service thread\\nholds the physics mutex,\\ncalls RemoveJoint()", fillcolor="{C_ROS}"];
    b2 [label="RemoveJoint() calls SetPaused():\\nwaits for the world-update lock", fillcolor="{C_ROS}"];
    b3 [label="physics thread holds the\\nworld-update lock, waits\\nfor the physics mutex", fillcolor="{C_SIM}"];
    b4 [label="DEADLOCK:\\neach waits for the other", shape=octagon, style="filled", fillcolor="#d35400", fontcolor="white"];
    b1 -> b2; b2 -> b3 [label=" waits for"]; b3 -> b1 [label=" waits for"]; b3 -> b4 [style=dashed];
  }}
  subgraph cluster_c {{ label="(c) THE PATCH: pause the world around the joint change"; fontcolor="#1e8449";
    style="rounded"; color="#1e8449";
    c1 [label="was_paused = world_->IsPaused()", fillcolor="{C_OWN}"];
    c2 [label="world_->SetPaused(true)\\nreturns only after the\\ncurrent physics step has ended", fillcolor="{C_OWN}"];
    c3 [label="create the joint (attach) or\\nDetach() + RemoveJoint() (detach)\\nwhile no step runs", fillcolor="{C_OWN}"];
    c4 [label="world_->SetPaused(was_paused)\\nphysics continues", fillcolor="{C_OWN}"];
    c1 -> c2 -> c3 -> c4;
  }}
  {{ rank=same; a1; b1; c1; }}
}}
"""

# ------------------------------------------------- link attacher patch: joint list
DIAGRAMS['attacher_gvjoints'] = f"""
digraph G {{ {STYLE}
  rankdir=LR; nodesep=0.18; ranksep=0.35;
  subgraph cluster_b {{ label="AFTER: the entry is erased on detach"; fontcolor="#1e8449"; style="rounded"; color="#1e8449";
    y1 [label="1st attach\\nJ1 created, stored", fillcolor="{C_OWN}"];
    y2 [label="detach\\nRemoveJoint(J1) +\\nGV_joints.erase(J1)", fillcolor="{C_OWN}"];
    y3 [label="2nd attach\\nnew joint J2, stored;\\ndetach removes J2", fillcolor="{C_OWN}"];
    y1 -> y2 -> y3;
  }}
  subgraph cluster_a {{ label="BEFORE: the list keeps a deleted joint"; fontcolor="#c0392b"; style="rounded"; color="#c0392b";
    x1 [label="1st attach\\njoint J1 created,\\nstored in GV_joints", fillcolor="{C_ROS}"];
    x2 [label="detach\\nJ1 removed from Gazebo,\\nbut still in GV_joints", fillcolor="#f6dede"];
    x3 [label="2nd attach, then detach\\ngetJoint() finds the OLD J1\\n-> works on a deleted joint", fillcolor="#f6dede"];
    x1 -> x2 -> x3;
  }}
}}
"""


def main():
    os.makedirs(OUT, exist_ok=True)
    for name, src in DIAGRAMS.items():
        with open(os.path.join(OUT, name + '.dot'), 'w') as f:
            f.write(src)
        subprocess.run(['dot', '-Tsvg', '-o', os.path.join(OUT, name + '.svg')],
                       input=src.encode(), check=True)
        print('wrote', name + '.svg')


if __name__ == '__main__':
    main()
