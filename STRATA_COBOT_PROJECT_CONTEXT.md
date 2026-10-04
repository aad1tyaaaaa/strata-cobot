# eYRC 2026–27 — Strata Cobot (SC) Project Context & Agent Build Brief

## 0. Purpose of this document

This document is the persistent project context for an AI coding/engineering agent working on the **eYRC 2026–27 Strata Cobot (SC)** theme.

The goal is to take the project from the official e-Yantra starter repository and theme-book instructions through:

1. Correct software/environment setup.
2. Task 0 verification.
3. Task 1A — ore perception.
4. Task 1B — UR7e arm waypoint control.
5. Task 1C — eBot navigation.
6. Integration, testing, debugging, evaluator runs, and submission preparation.

The agent must work from the **official repository and the user's current theme-book/dashboard instructions**, not from assumptions or old eYRC versions.

---

# 1. User / Team Context

- User: **Aaditya Jaiswar**
- Competition: **eYRC 2026–27**
- Selected theme: **Strata Cobot (SC)**
- The user's eYRC dashboard also shows **Hola The Explorer (HE)** as the other locked theme, but this document is specifically for **Strata Cobot**.
- The user's eYRC dashboard showed:
  - Task 0: **2/3 done**
  - Instructions: completed
  - Theme: locked
  - Submission: pending
  - Locked themes: **Hola The Explorer (HE)** and **Strata Cobot (SC)**
- The dashboard screenshot showed Task 0 deadlines as **14 September, 11:59 PM** and the status as Closed. If submission status matters, verify it against the current portal rather than assuming it can still be submitted.
- Only the **team leader** can submit from the dashboard.
- A team member named **Prathamesh Rajesh Aarya** was shown as not having joined the Discord forum; the portal indicated this was required from Task 1 onward.

Important: The dashboard information above is based on the user's screenshot. Do not invent current portal status. If the user asks about current deadlines/submission availability, check the live portal if accessible or ask for a fresh screenshot.

---

# 2. Official Project Repository

Official repository:

https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Strata-Cobot

Current repository description:

**eYRC 2026-27 Strata-Cobot Theme**

Official stack:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

Repository top-level structure currently includes:

```text
eYRC_26-27_Strata-Cobot/
├── algorithms/
├── ebot_description/
├── eyantra_kepler_colony/
├── ur_description/
├── eyrc-sc-evaluator/
├── .gitignore
├── README.md
└── requirements.sh
```

The official repository README states:

- `requirements.sh` installs task dependencies on top of an existing ROS 2 Jazzy installation.
- It does NOT install ROS itself.
- Build with `colcon build`.
- Source with `source install/setup.bash`.
- Add the source command to `~/.bashrc`.
- Each task has its own launch file.
- Launch files do NOT start the student's solution node.
- Student code goes in the `algorithms` package.
- The evaluator produces a signed `result.zip`.
- Do not unpack, rebuild, rename, or modify the evaluator-generated `result.zip`.

---

# 3. Official Task Launch Files

The current official README lists:

```bash
ros2 launch eyantra_kepler_colony task0.launch.py
```

Task 0 — whole arena.

```bash
ros2 launch eyantra_kepler_colony task1a.launch.py
```

Task 1A — ore perception.

```bash
ros2 launch eyantra_kepler_colony task1b.launch.py
```

Task 1B — arm waypoints.

```bash
ros2 launch eyantra_kepler_colony task1c.launch.py
```

Task 1C — eBot navigation.

Any of these can be launched with:

```bash
rviz:=false
```

if RViz should not start.

Example:

```bash
ros2 launch eyantra_kepler_colony task1a.launch.py rviz:=false
```

---

# 4. Core Architecture

The overall system is conceptually:

```text
                    STRATA COBOT
                         |
        +----------------+----------------+
        |                                 |
     eBot Mobile Base                 UR7e Arm
        |                                 |
   Navigation (1C)                  Manipulation (1B)
        |                                 |
        +------------ Mission ------------+
                         |
                  Perception (1A)
                         |
                       Camera
                         |
                  Ore detection
```

The project should be developed incrementally.

Do not try to implement everything simultaneously.

Recommended order:

```text
Environment
   ↓
Task 0
   ↓
Understand ROS topics/nodes
   ↓
Task 1A
   ↓
Task 1B
   ↓
Task 1C
   ↓
Integration
   ↓
Evaluator
   ↓
Submission archive
```

---

# 5. Environment Requirements

Recommended development environment:

```text
Windows 11
   ↓
WSL2
   ↓
Ubuntu 24.04
   ↓
ROS 2 Jazzy
   ↓
Gazebo Harmonic
   ↓
Strata Cobot workspace
```

If native Ubuntu 24.04 is available, it can be used instead.

Do not install the Strata Cobot stack against ROS 2 Humble. The official SC repository specifies ROS 2 Jazzy.

Do not confuse this with the HE theme. HE uses ROS 2 Humble + MuJoCo, while SC uses ROS 2 Jazzy + Gazebo Harmonic.

---

# 6. Initial Environment Setup

For a clean Ubuntu environment:

```bash
lsb_release -a
```

Confirm Ubuntu 24.04.

After ROS 2 Jazzy is installed:

```bash
printenv ROS_DISTRO
```

Expected:

```text
jazzy
```

Create the workspace:

```bash
mkdir -p ~/strata_ws/src
cd ~/strata_ws/src
```

Clone the official repository:

```bash
git clone https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Strata-Cobot.git
```

Enter the repository:

```bash
cd ~/strata_ws/src/eYRC_26-27_Strata-Cobot
```

Install task dependencies:

```bash
chmod +x requirements.sh
./requirements.sh
```

Build:

```bash
cd ~/strata_ws
colcon build
```

Source:

```bash
source install/setup.bash
```

Persist the workspace source:

```bash
echo "source ~/strata_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If dependencies or packages fail, diagnose them before proceeding. Do not blindly install unrelated packages.

---

# 7. Task 0 — Setup Verification

Task 0 is primarily the software installation/setup task.

First launch:

```bash
ros2 launch eyantra_kepler_colony task0.launch.py
```

Expected result:

- Gazebo opens.
- Strata Cobot arena loads.
- Robot/environment is visible.
- ROS 2 nodes and topics are active.

Task 0 is not the place to write the final robot algorithms.

Useful inspection commands:

```bash
ros2 node list
```

```bash
ros2 topic list
```

```bash
ros2 service list
```

```bash
ros2 action list
```

Use these to understand the simulation before implementing Task 1.

---

# 8. ROS 2 Development Rules

The agent must understand that a ROS 2 node is only useful if:

1. It is placed in the correct package.
2. It is registered in the package setup.
3. The workspace is rebuilt.
4. The workspace is sourced.
5. The executable can be discovered.
6. The simulator is running.
7. The node uses the correct topics, message types, QoS, and simulation time.

After adding an executable to `algorithms/setup.py`, build:

```bash
cd ~/strata_ws
colcon build --packages-select algorithms
source install/setup.bash
```

Check installed executables:

```bash
ros2 pkg executables algorithms
```

Run an executable:

```bash
ros2 run algorithms <executable_name>.py
```

Important official repository warning:

- A script is not installed until it is included in the `SCRIPTS` list in `algorithms/setup.py` and the package is rebuilt.
- `chmod +x` is required for executable scripts.
- `ros2 run` starts the installed copy, so rebuild and re-source after edits.

---

# 9. Algorithms Package

Student algorithm code belongs in:

```text
algorithms/
```

The official README gives the pattern:

```bash
cd algorithms
cp boilerplate/task1b_boilerplate.py scripts/task1b/arm_waypoints.py
chmod +x scripts/task1b/arm_waypoints.py
```

Then add the script path to `algorithms/setup.py`.

Example:

```python
SCRIPTS = [
    'scripts/task1b/arm_waypoints.py',
]
```

Build:

```bash
cd ~/strata_ws
colcon build --packages-select algorithms
source install/setup.bash
```

Run:

```bash
ros2 run algorithms arm_waypoints.py
```

The agent should preserve the official boilerplate structure and fill the intended sections rather than unnecessarily replacing the whole file.

---

# 10. Task 1A — Ore Perception

Official launch:

```bash
ros2 launch eyantra_kepler_colony task1a.launch.py
```

Goal:

Build the perception solution for detecting/identifying ore according to the official SC theme-book specification.

Conceptual pipeline:

```text
Camera / sensor data
        ↓
ROS image topic
        ↓
OpenCV / image processing
        ↓
Preprocessing
        ↓
Ore segmentation/detection
        ↓
Feature extraction
        ↓
Ore position/classification
        ↓
Required ROS output
```

Do not invent topic names or required output messages.

First inspect the actual task environment:

```bash
ros2 topic list
```

Then inspect relevant camera/image topics:

```bash
ros2 topic info <topic> -v
```

```bash
ros2 topic echo <topic>
```

Use:

```bash
ros2 interface show <message_type>
```

to understand message structures.

The exact required topics, outputs, evaluator expectations, file name, and submission format must be taken from the current official Task 1A theme-book page.

Recommended development approach:

1. Launch Task 1A.
2. Identify the camera topic.
3. Capture sample frames.
4. Understand image dimensions/encoding.
5. Implement simple robust segmentation first.
6. Add noise handling.
7. Determine ore center/geometry.
8. Publish exactly what the task specification requires.
9. Test against multiple scene states.
10. Run the evaluator.

Do not add unnecessary machine learning if classical computer vision is sufficient. Prefer deterministic, explainable processing unless the task specifically requires another method.

---

# 11. Task 1B — UR7e Arm Waypoints

Official launch:

```bash
ros2 launch eyantra_kepler_colony task1b.launch.py
```

The official boilerplate uses these ROS interfaces:

### Publishers

```text
/delta_twist_cmds
/delta_joint_cmds
```

### Subscribers

```text
/tcp_pose_raw
/joint_states
/arm_status
```

### Service

The boilerplate uses:

```text
<servo_ns>/switch_controller
```

through:

```text
control_msgs / controller_manager_msgs
```

The official boilerplate defines these tool-position waypoints in `base_link`:

```text
(-0.4085, -0.5379, 0.1967)
(-0.8000, -0.0005, 0.3967)
(-0.7430,  0.5280, 0.1967)
(-0.4097,  0.5280, 0.1967)
(-0.0763,  0.5280, 0.1967)
```

These are given in the official Task 1B boilerplate and should not be silently changed.

Important: `base_link` is the UR7e's own frame, not the Gazebo world frame.

The arm command interfaces use VELOCITIES, not direct position commands.

The official boilerplate says:

- Use `TwistStamped` for end-effector velocity commands.
- Use `JointJog` for joint velocity commands.
- Only one command interface is active at a time.
- Publish commands on every control tick, including zero.
- Never sleep inside the control callback.
- Use proportional control toward the target.
- Cap velocity by scaling the entire velocity vector.
- Command orientation as well as position.
- Track waypoints in order.
- Hold each waypoint for at least two seconds according to the boilerplate/task requirement.
- Account for servo ramp-down and settling.
- Consider the actual robot path, joint limits, and configurations.
- Match joint angles by joint name rather than assuming a fixed order.

Conceptual controller:

```text
Current TCP pose
       ↓
Target waypoint
       ↓
Position error
       ↓
Velocity command
       ↓
Orientation error
       ↓
Angular velocity
       ↓
Servo controller
       ↓
UR7e
```

Useful development values:

```python
v = Kp * (target - current)
```

Cap:

```text
if |v| > cap:
    v = v * cap / |v|
```

For orientation, the boilerplate suggests using the rotation-vector style calculation from the current tool axis to desired tool axis.

Do not hard-code assumptions about joint order.

Do not command the arm with blocking sleeps.

---

# 12. Task 1C — eBot Navigation

Official launch:

```bash
ros2 launch eyantra_kepler_colony task1c.launch.py
```

The official Task 1C boilerplate uses:

### Subscriber topics

```text
/ebot_path
/odom
/scan
```

### Publisher

```text
/cmd_vel
```

Message types:

```text
/ebot_path → nav_msgs/msg/Path
/odom      → nav_msgs/msg/Odometry
/scan      → sensor_msgs/msg/LaserScan
/cmd_vel   → geometry_msgs/msg/Twist
```

The official limits are:

```text
max linear speed  = 0.5 m/s
max angular speed = 1.0 rad/s
```

The route on `/ebot_path` is published once before the solution node starts and held for late subscribers.

Therefore the route subscriber must use compatible QoS, specifically the transient-local behavior described by the boilerplate.

Conceptual pipeline:

```text
/ebot_path
    ↓
Waypoints
    ↓
Current /odom pose
    ↓
Heading + distance error
    ↓
Controller
    ↓
Obstacle handling using /scan
    ↓
/cmd_vel
    ↓
eBot
```

Basic steering idea from the official boilerplate:

```text
distance = hypot(tx - x, ty - y)

heading =
    atan2(ty - y, tx - x) - yaw
```

Wrap heading:

```text
atan2(sin(error), cos(error))
```

Then:

```text
angular = Kp * heading
```

Drive slower when the heading error is large.

The route waypoints must be followed in order.

Obstacle avoidance is required. A pure line-following controller that ignores the laser scan will not be enough.

The official boilerplate suggests:

- Use valid laser ranges only.
- Consider a forward cone.
- Compare available space on both sides.
- Choose a route around obstacles.
- Account for the actual eBot geometry.
- Do not assume perfect on-the-spot turning.
- Verify actual motion using `/odom`, not only commands sent to `/cmd_vel`.
- Stop the robot after completing the route.
- Avoid uncaught exceptions because a dead node can leave the base coasting.

Important:

Laser measurements are relative to the LIDAR mounting location, not automatically the center of the chassis. Use TF if a geometric offset is required; do not guess the offset.

---

# 13. ROS Debugging Checklist

When something does not work, follow this order.

### Is ROS running?

```bash
printenv ROS_DISTRO
```

### Is the workspace sourced?

```bash
echo $AMENT_PREFIX_PATH
```

### Does the package exist?

```bash
ros2 pkg list | grep algorithms
```

### Is the executable installed?

```bash
ros2 pkg executables algorithms
```

### Are nodes running?

```bash
ros2 node list
```

### Are topics present?

```bash
ros2 topic list
```

### What type is a topic?

```bash
ros2 topic type /topic_name
```

### What publishers/subscribers exist?

```bash
ros2 topic info /topic_name -v
```

### Can data be observed?

```bash
ros2 topic echo /topic_name
```

### What is the message definition?

```bash
ros2 interface show <package>/msg/<Message>
```

### Is TF working?

```bash
ros2 run tf2_tools view_frames
```

and use the relevant TF tools available in the installed ROS 2 environment.

---

# 14. Simulation Time

The official boilerplate explicitly uses:

```python
use_sim_time = True
```

For nodes running in simulation, preserve simulation-time behavior.

Do not mix wall-clock assumptions with Gazebo simulation time.

Use:

```python
self.get_clock().now()
```

when generating timestamps.

---

# 15. Evaluator

The official repository contains:

```text
eyrc-sc-evaluator/
```

The official README gives the pattern:

```bash
./eyrc-sc-evaluator --task 1B --team-id 1455
```

The actual team ID must be replaced with the user's team's ID. Never invent it.

Arguments can be omitted and the evaluator will ask for them.

The evaluator:

1. Checks setup.
2. Prints `Ready`.
3. Records the run.
4. The student's solution should be started only after `Ready`.
5. Press `q` when the run is finished, or `Ctrl-C` where interactive input is unavailable.
6. Produces a `result.zip` in a team/subtask folder.

CRITICAL:

```text
result.zip
```

is signed.

Do NOT:

- unpack it,
- edit it,
- rename files inside it,
- rebuild it,
- modify its contents.

---

# 16. Submission

The official repository README gives this example pattern:

```text
SC#1455_task1B.zip
├── result.zip
└── task1B.py
```

However, the README explicitly warns that the exact:

- archive name,
- filenames,
- required files,
- number of files

vary by subtask.

Therefore, the agent MUST use the current official Task 1A/1B/1C theme-book submission instructions for the exact final archive.

Never assume the Task 1B example applies unchanged to Task 1A or 1C.

---

# 17. Development Strategy

Use the following engineering sequence.

## Stage A — Environment

Goal:

```text
Ubuntu 24.04
ROS 2 Jazzy
Gazebo Harmonic
colcon
official SC repo
```

Verify everything before writing task code.

## Stage B — Task 0

Goal:

```text
task0.launch.py
```

must launch successfully.

Inspect:

```bash
ros2 node list
ros2 topic list
```

## Stage C — Task 1A

Goal:

Get perception working independently.

Do not move to Task 1B until:

- image topic is known,
- detector works,
- required output is published,
- evaluator can see the expected behavior.

## Stage D — Task 1B

Goal:

Move UR7e through the required waypoints.

Start with:

1. Receive TCP pose.
2. Receive joint state.
3. Receive arm status.
4. Select correct controller.
5. Move to first waypoint.
6. Stop/hold.
7. Move through remaining waypoints.
8. Verify orientation.
9. Verify actual path.
10. Run evaluator.

## Stage E — Task 1C

Goal:

Navigate eBot through the provided route while avoiding obstacles.

Start with:

1. Correctly receive `/ebot_path`.
2. Read `/odom`.
3. Read `/scan`.
4. Drive toward first waypoint.
5. Tune waypoint acceptance radius.
6. Add heading control.
7. Add obstacle handling.
8. Test the complete route.
9. Verify actual path with odometry.
10. Run evaluator.

## Stage F — Integration

Only integrate after each component works independently.

---

# 18. Coding Quality Requirements

The agent should write code that is:

- deterministic,
- readable,
- modular,
- commented where logic is non-obvious,
- compatible with Python 3 / ROS 2 Jazzy,
- safe under simulation,
- robust against missing messages,
- protected from divide-by-zero and invalid sensor values,
- non-blocking inside timer callbacks,
- careful about QoS,
- careful about frame transforms,
- easy to tune.

Avoid:

- arbitrary hard-coded offsets,
- unexplained magic numbers,
- `time.sleep()` inside ROS callbacks,
- blocking service calls from active timer callbacks,
- assuming topic QoS,
- assuming joint order,
- assuming laser data is always valid,
- sending velocity commands without a stop condition,
- rewriting official infrastructure unnecessarily.

---

# 19. Agent Rules

The AI agent working from this document must follow these rules.

### Rule 1 — Official source first

For task requirements, use the official eYRC theme book/repository.

Official repository:

https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Strata-Cobot

Do not rely on random tutorials for competition-specific requirements.

### Rule 2 — Never invent missing requirements

If the current theme-book page is needed and not available, explicitly say:

> "I need the current Task X specification/theme-book page before implementing this requirement."

Do not fabricate:

- topic names,
- message types,
- evaluator behavior,
- waypoint coordinates,
- submission names,
- marks,
- deadlines.

### Rule 3 — Inspect before coding

Before implementing a ROS algorithm, inspect:

```bash
ros2 topic list
ros2 node list
ros2 topic info <topic> -v
ros2 interface show <type>
```

when appropriate.

### Rule 4 — Preserve official boilerplate intent

Fill the sections marked for student implementation unless there is a concrete reason to restructure.

### Rule 5 — Test incrementally

Every change should follow:

```text
edit
 ↓
colcon build
 ↓
source install/setup.bash
 ↓
launch simulation
 ↓
run node
 ↓
inspect ROS data
 ↓
test behavior
```

### Rule 6 — Do not claim success without evidence

A task is not "done" just because the code compiles.

Use:

- logs,
- topic output,
- simulation behavior,
- evaluator output,

to establish correctness.

### Rule 7 — Keep backups

Before major algorithm changes, preserve a working version using Git.

Recommended:

```bash
git status
git add .
git commit -m "working checkpoint"
```

### Rule 8 — Submission artifacts are sacred

Never modify evaluator-generated `result.zip`.

---

# 20. Suggested Repository Workflow

Use a branch/workflow like:

```text
main
│
├── task1a-development
├── task1b-development
└── task1c-development
```

Suggested commits:

```text
setup: initialize Strata Cobot workspace
task0: verify simulation environment
task1a: add ore perception baseline
task1a: improve detection robustness
task1b: add arm controller baseline
task1b: tune waypoint controller
task1c: add path follower
task1c: add obstacle avoidance
integration: combine tested components
submission: prepare evaluator artifacts
```

Do not commit generated build artifacts unless the official repository requires them.

---

# 21. Useful Commands

Clone:

```bash
git clone https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Strata-Cobot.git
```

Update:

```bash
git pull
```

Build everything:

```bash
colcon build
```

Build algorithms only:

```bash
colcon build --packages-select algorithms
```

Source:

```bash
source install/setup.bash
```

Launch Task 0:

```bash
ros2 launch eyantra_kepler_colony task0.launch.py
```

Launch Task 1A:

```bash
ros2 launch eyantra_kepler_colony task1a.launch.py
```

Launch Task 1B:

```bash
ros2 launch eyantra_kepler_colony task1b.launch.py
```

Launch Task 1C:

```bash
ros2 launch eyantra_kepler_colony task1c.launch.py
```

List nodes:

```bash
ros2 node list
```

List topics:

```bash
ros2 topic list
```

List services:

```bash
ros2 service list
```

List actions:

```bash
ros2 action list
```

List executables:

```bash
ros2 pkg executables algorithms
```

---

# 22. Current Known Official Task 1B Interfaces

From the official `task1b_boilerplate.py`:

```text
Publish:
    /delta_twist_cmds
    /delta_joint_cmds

Subscribe:
    /tcp_pose_raw
    /joint_states
    /arm_status

Service:
    <servo_ns>/switch_controller
```

Task 1B waypoint coordinates:

```text
1: (-0.4085, -0.5379, 0.1967)
2: (-0.8000, -0.0005, 0.3967)
3: (-0.7430,  0.5280, 0.1967)
4: (-0.4097,  0.5280, 0.1967)
5: (-0.0763,  0.5280, 0.1967)
```

---

# 23. Current Known Official Task 1C Interfaces

From the official `task1c_boilerplate.py`:

```text
Subscribe:
    /ebot_path
    /odom
    /scan

Publish:
    /cmd_vel
```

Message types:

```text
/ebot_path → nav_msgs/msg/Path
/odom      → nav_msgs/msg/Odometry
/scan      → sensor_msgs/msg/LaserScan
/cmd_vel   → geometry_msgs/msg/Twist
```

Limits:

```text
linear.x  ≤ 0.5 m/s
angular.z ≤ 1.0 rad/s
```

The route is published once and held for late subscribers, so QoS compatibility is essential.

---

# 24. Recommended Agent Behavior When Starting Work

When this file is supplied to an AI coding agent, the agent should first:

1. Read this entire file.
2. Inspect the current local project directory.
3. Determine whether the official Strata repository is already cloned.
4. Determine whether Ubuntu/WSL/ROS 2/Gazebo are installed.
5. Check:
   ```bash
   printenv ROS_DISTRO
   ```
6. Check:
   ```bash
   ros2 --version
   ```
7. Check:
   ```bash
   colcon --version
   ```
8. Check repository status:
   ```bash
   git status
   ```
9. Inspect the actual repository tree.
10. Compare local files against the current official repository if network access is available.
11. Read the current theme-book/task instructions before implementing competition-specific behavior.
12. Build and launch Task 0.
13. Only after Task 0 works, start Task 1A/1B/1C.

If the agent has access to the internet, it should verify the current official repository before making major changes because eYRC can update task files.

---

# 25. Important distinction: "Build the project" vs "Submit the competition task"

There are two goals:

### Goal A — Engineering

Make the Strata Cobot simulation and algorithms actually work.

### Goal B — Competition submission

Produce exactly the files and evaluator artifacts required by eYRC.

They are related but not identical.

The agent must optimize for both.

A solution can work in Gazebo but still fail submission packaging if the wrong files are submitted.

Likewise, a correctly packaged submission is useless if the algorithm does not work.

---

# 26. Sources / Verification

Primary source:

Official Strata Cobot repository:

https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Strata-Cobot

Official repository README confirms:

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic
- `requirements.sh`
- `colcon build`
- Task launch files
- algorithms package workflow
- evaluator workflow
- signed `result.zip`
- submission archive rules

Official Task 1B boilerplate:

https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Strata-Cobot/blob/main/algorithms/boilerplate/task1b_boilerplate.py

Official Task 1C boilerplate:

https://github.com/eYantra-Robotics-Competition/eYRC_26-27_Strata-Cobot/blob/main/algorithms/boilerplate/task1c_boilerplate.py

These sources were checked while preparing this project context.

---

# 27. Final Mission for the Agent

The final objective is:

> Build, test, debug, and prepare the eYRC 2026–27 Strata Cobot project for Tasks 1A, 1B, and 1C using the official ROS 2 Jazzy + Gazebo Harmonic environment, while preserving the official repository structure and satisfying the current theme-book/evaluator requirements.

The agent should not merely generate code.

It should:

```text
UNDERSTAND
    ↓
INSPECT
    ↓
IMPLEMENT
    ↓
BUILD
    ↓
RUN
    ↓
OBSERVE
    ↓
DEBUG
    ↓
TEST
    ↓
EVALUATE
    ↓
PACKAGE
```

Do not mark a task complete until its behavior has been tested in the simulator and, where applicable, verified with the official evaluator.

