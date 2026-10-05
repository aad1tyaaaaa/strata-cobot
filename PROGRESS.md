# Strata Cobot — Progress Log

Read with `STRATA_COBOT_PROJECT_CONTEXT.md` (the project brief). Last updated 2026-10-05 (evaluator: 1A 20/20, 1B 40/40, 1C 40/40). Team ID **3660**.

## Done

- **Repo**: official `eYRC_26-27_Strata-Cobot` copied into this folder, git history kept
  (upstream commit `4559a79`). `core.autocrlf=false`; the working tree had been checked out
  with CRLF before that, which broke `requirements.sh` (`bash`), so all `i/lf w/crlf`
  files were converted back to LF (content identical to git).
- **Workspace** (WSL): `~/strata_ws/src/eYRC_26-27_Strata-Cobot` -> symlink to this folder,
  `requirements.sh` run, `colcon build --symlink-install` OK (4 packages). Both
  `/opt/ros/jazzy/setup.bash` and `~/strata_ws/install/setup.bash` are in `~/.bashrc`.
  With `--symlink-install`, edits to the scripts take effect without a rebuild.
- **Nodes**, registered in `algorithms/setup.py`, all **verified in Gazebo (headless)**:
  | Task | File | Run | Sim result (2026-10-05) |
  |---|---|---|---|
  | 0 | — | `ros2 launch eyantra_kepler_colony task0.launch.py` | arena, arm, camera, controllers up |
  | 1A | `algorithms/scripts/task1a/task1a.py` | `ros2 run algorithms task1a.py` | **evaluator 20/20** (twice, random layouts); ~1 mm vs Gazebo top-face centre |
  | 1B | `algorithms/scripts/task1b/task1b.py` | `ros2 run algorithms task1b.py` | **evaluator 40/40** — all 5 waypoints within 0.03 m, held |
  | 1C | `algorithms/scripts/task1c/task1c.py` | `ros2 run algorithms task1c.py` | **evaluator 40/40** — all 10 waypoints within 0.3 m, in order |
- **Offline checks** (no ROS needed, run in WSL), `tools/`:
  - `sim_task1a.py` — `detect_ores` on a real sim frame (`tools/frame_task1a.png`): two of
    each ore on the conveyor, terrain ignored. **PASS**
  - `sim_task1b.py` — kinematic UR7e: all 5 waypoints, done ~49 s. **PASS**
  - `sim_task1c.py` — unicycle eBot + synthetic lidar, 3 rock layouts. **PASS**

## Design notes (why things are the way they are)

- **1A**: HSV ranges **measured on a sim frame** (not the .dae colours): azurite H105,
  malachite H74, vanadinite is orange H10 S252 — the Mars terrain is H~8 but S~160, so
  vanadinite is split from terrain by saturation (S>=220). Ores are **randomised per run**
  (position + yaw), so spawn poses are not ground truth; read them from
  `gz topic -e -t /world/eyantra_kepler_world/pose/info` (base_link = `arm_mount` at world
  (-0.5973, 1.8653, 0.8825), yaw 0). Position = centroid of the blob's **top face** (all
  blob pixels deprojected, keep those within 6 mm of the highest), **no** half-height drop:
  the evaluator scores the top-face centre (subtracting 3.81 cm gave 18/20 — outside the
  strict zone; the boilerplate's "named by its middle" hint is misleading here). Pixel
  centres are index + 0.5 because Gazebo's cx, cy = 320, 240 (fixed a ~1.8 mm bias).
  The evaluator pairs `_1`/`_2` itself (they are identical models placed at random), so any
  stable numbering scores. `SAVE_FRAME=/tmp/f.png` dumps a frame.
- **1B**: twist interface only, switched once in `__init__` (STRICT/BEST_EFFORT retries).
  Tool z-axis → straight down (all 5 waypoints verified reachable by IK). Rotate first
  (translation gated while axis error > 0.35 rad — arm starts with tool horizontal). Caps
  0.12 m/s / 0.30 rad/s (servo drops >0.15/0.35 whole). Accelerate-only ramp 0.05 m/s²
  (waypoint 3 is near elbow singularity; a ramp on braking caused overshoot through it).
  Hold 2.5 s counted only while inside 1 cm / 0.05 rad. In this description `base_link`
  equals the DH base frame (base_link_inertia rpy = 0). In sim `/arm_status` briefly reads
  11 (a WARNING_* code) near waypoint 3 and clears; harmless. rclpy forbids one log call
  site at two severities (ValueError) — that crashed the node once.
- **1C**: `/ebot_path` QoS RELIABLE + TRANSIENT_LOCAL. Lidar offset from TF
  (odom child frame ← scan frame), not typed in. Corridor heading search: half-width
  0.245 + 0.16 = 0.405 m (> 0.395 m corner sweep when turning on the spot — 0.12 collided
  in sim). Linear and angular scaled together to keep the arc. Waypoint tol 0.25 m, final
  0.15 m. Whole cycle in try/except → zero cmd.

## Environment state

- WSL `Ubuntu-24.04`, user **root**, WSLg available, 7 GB RAM. ROS 2 Jazzy desktop-full +
  task deps installed, workspace built (see Done).
- Headless sim: add `gui:=false rviz:=false` to any task launch.
- From Git Bash on Windows, call WSL with `MSYS_NO_PATHCONV=1 wsl -d Ubuntu-24.04 -u root -- ...`.

## Evaluator runs / submission

- Evaluator: launch the task (`gui:=false rviz:=false` is fine), run
  `./eyrc-sc-evaluator --task 1X --team-id 3660`, start the node after `Ready`, `q`/Ctrl-C
  when done (1A stops on its own). Kill any leftover `ur_arm_controller` from a previous
  sim first, or the pre-check fails with "more than one node is publishing".
- Results in `SC-3660-task-1{A,B,C}/` (gitignored): `result.zip` (signed, untouched),
  node copy `task1X.py`, and `SC#3660_task1X.zip` = those two files — named per the README
  example; **verify archive/file names against each task page before uploading**.
- If a node changes, re-run its evaluator and rebuild that archive.

## Open questions (need from user)

- Task 1A/1B/1C theme-book submission pages: confirm archive and file names
  (`SC#3660_task1X.zip`, `task1X.py` assumed from the README).
