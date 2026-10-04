# Strata Cobot — Progress Log

Read with `STRATA_COBOT_PROJECT_CONTEXT.md` (the project brief). Last updated 2026-10-05.

## Done

- **Repo**: official `eYRC_26-27_Strata-Cobot` copied into this folder, git history kept
  (upstream commit `4559a79`). `core.autocrlf=false` set locally so scripts stay LF
  (CRLF breaks the `#!/usr/bin/env python3` shebang).
- **Nodes written** (boilerplates filled, structure kept), registered in `algorithms/setup.py`:
  | Task | File | Run |
  |---|---|---|
  | 1A ore perception | `algorithms/scripts/task1a/task1a.py` | `ros2 run algorithms task1a.py` |
  | 1B arm waypoints | `algorithms/scripts/task1b/task1b.py` | `ros2 run algorithms task1b.py` |
  | 1C eBot navigation | `algorithms/scripts/task1c/task1c.py` | `ros2 run algorithms task1c.py` |
- **Offline checks** (no ROS needed), `tools/`:
  - `sim_task1b.py` — kinematic UR7e from `initial_positions.yaml`: all 5 waypoints held,
    done ~49 s, min |elbow| 0.083 rad, max joint speed 0.63 rad/s. **PASS**
  - `sim_task1c.py` — unicycle eBot + synthetic ±90° lidar, 3 rock layouts: in-order, no
    collisions. **PASS**
  - `sim_task1a.py` — synthetic frame with .dae ore colours at 3 brightness levels: all six
    found, ore_package ignored. **PASS** (run in WSL)

## Design notes (why things are the way they are)

- **1A**: HSV ranges from the ore `.dae` diffuse colours (azurite H~113, malachite H~63,
  vanadinite H~1 wraps, ore_package purple H~150 excluded). Camera is a mast RealSense at
  base_link (-1.152, -0.02, 1.238), pitched ~48° down, so the surface→centre correction is
  along the view ray using the ore box (0.1016×0.1016×0.0762). IDs: `_1` = nearer the arm
  base (matches spawn table), then tracked by nearest pixel. `SAVE_FRAME=/tmp/f.png` dumps a
  frame for HSV tuning. `use_sim_time` added.
- **1B**: twist interface only, switched once in `__init__` (STRICT/BEST_EFFORT retries).
  Tool z-axis → straight down (all 5 waypoints verified reachable by IK). Rotate first
  (translation gated while axis error > 0.35 rad — arm starts with tool horizontal). Caps
  0.12 m/s / 0.30 rad/s (servo drops >0.15/0.35 whole). Accelerate-only ramp 0.05 m/s²
  (waypoint 3 is near elbow singularity; a ramp on braking caused overshoot through it).
  Hold 2.5 s counted only while inside 1 cm / 0.05 rad. In this description `base_link`
  equals the DH base frame (base_link_inertia rpy = 0).
- **1C**: `/ebot_path` QoS RELIABLE + TRANSIENT_LOCAL. Lidar offset from TF
  (odom child frame ← scan frame), not typed in. Corridor heading search: half-width
  0.245 + 0.16 = 0.405 m (> 0.395 m corner sweep when turning on the spot — 0.12 collided
  in sim). Linear and angular scaled together to keep the arc. Waypoint tol 0.25 m, final
  0.15 m. Whole cycle in try/except → zero cmd.

## Environment state

- WSL `Ubuntu-24.04`, user **root**, WSLg available, 7 GB RAM.
- ROS 2 Jazzy `desktop-full` **installed** (log `/root/ros_install.log` ends `ROS_INSTALL_DONE`;
  `ros2 pkg list` works, OpenCV 4.6). `requirements.sh` and workspace build not done yet.

## Next steps

1. In WSL: `source /opt/ros/jazzy/setup.bash` (add to `~/.bashrc`).
2. Workspace:
   ```bash
   mkdir -p ~/strata_ws/src
   ln -s /mnt/c/Users/Developers/Strata-Cobot ~/strata_ws/src/eYRC_26-27_Strata-Cobot
   cd ~/strata_ws/src/eYRC_26-27_Strata-Cobot && ./requirements.sh
   cd ~/strata_ws && colcon build && source install/setup.bash
   echo "source ~/strata_ws/install/setup.bash" >> ~/.bashrc
   ```
3. Re-run `tools/sim_task1*.py` after any controller change.
4. Task 0: `ros2 launch eyantra_kepler_colony task0.launch.py`; inspect nodes/topics.
5. 1A in sim: save a frame, tune HSV; check TFs in RViz vs ore spawn poses.
6. 1B in sim: watch `/arm_status`, `/arm_status_detail`; confirm switch service + motion.
7. 1C in sim: confirm route received, lidar TF found, tune tolerances/margins.
8. Evaluator per task: `./eyrc-sc-evaluator --task 1X --team-id <ID>`, start node after `Ready`.
9. Package per theme-book instructions; never touch `result.zip`.
10. Commit a checkpoint (nothing committed yet).

## Open questions (need from user)

- **Team ID** (headers say `[ Team-ID ]`; evaluator needs it).
- **Task 1A/1B/1C theme-book pages**: exact submission names/files, evaluator tolerances,
  whether 1B requires a particular tool orientation (assumed pointing down), 1C waypoint radius.
