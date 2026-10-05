# Setup Guide — Strata Cobot (eYRC 2026-27)

This guide takes a fresh machine to a working simulation that runs all three Task 1
solutions and the official evaluator. Follow the steps in order.

**Stack:** Ubuntu 24.04 · ROS 2 Jazzy (desktop-full) · Gazebo Harmonic · x86-64 (amd64) CPU.

Other Ubuntu releases and other ROS distros (Humble, for example) will not work. On ARM
machines, including Apple Silicon, the prebuilt arm controller and plugins will not run.

---

## 1. Get Ubuntu 24.04

**Option A: native Ubuntu 24.04.** Skip to step 2.

**Option B: Windows 10/11 with WSL2.** Run this in an admin PowerShell:

```powershell
wsl --install -d Ubuntu-24.04
```

Reboot if Windows asks you to, open "Ubuntu 24.04" from the Start menu and create your
user. Windows 11 includes WSLg, so the Gazebo and RViz windows open on your desktop. Give
WSL at least 8 GB of RAM if you can; the simulation is heavy.

Check the release:

```bash
lsb_release -a        # must say 24.04
```

## 2. Install ROS 2 Jazzy

These are the steps from the official
[ROS 2 Jazzy install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
Check that page if anything below fails.

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop-full     # large download, about 20-40 min
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Check the install:

```bash
printenv ROS_DISTRO   # jazzy
ros2 pkg list | head  # prints packages, no errors
```

## 3. Get the code into a colcon workspace

```bash
mkdir -p ~/strata_ws/src
cd ~/strata_ws/src
git clone https://github.com/aad1tyaaaaa/strata-cobot.git eYRC_26-27_Strata-Cobot
```

> **Windows users who keep the repo on the Windows side** (for example
> `C:\Users\<you>\Strata-Cobot`): before you clone, turn off line-ending conversion, or the
> shell scripts fail with `/usr/bin/env: 'bash\r': No such file or directory`:
>
> ```bash
> git config --global core.autocrlf false
> ```
>
> Then link the folder into the workspace instead of cloning a second copy:
>
> ```bash
> ln -s /mnt/c/Users/<you>/Strata-Cobot ~/strata_ws/src/eYRC_26-27_Strata-Cobot
> ```

## 4. Install the task dependencies

```bash
cd ~/strata_ws/src/eYRC_26-27_Strata-Cobot
chmod +x requirements.sh eyrc-sc-evaluator
./requirements.sh            # add --check to only report what is missing
```

The script installs ros2_control, the Gazebo bridges, OpenCV and the other task
dependencies on top of ROS 2 Jazzy. It finishes with `all task dependencies are installed`.

## 5. Build

```bash
cd ~/strata_ws
colcon build --symlink-install
source install/setup.bash
echo "source ~/strata_ws/install/setup.bash" >> ~/.bashrc
```

`--symlink-install` links the Python nodes instead of copying them, so edits to
`algorithms/scripts/**` take effect without a rebuild. You only need to rebuild after
adding a new script to `algorithms/setup.py`.

Check that the three solution nodes are installed:

```bash
ros2 pkg executables algorithms
# algorithms task1a.py
# algorithms task1b.py
# algorithms task1c.py
```

## 6. Check the simulation (Task 0)

```bash
ros2 launch eyantra_kepler_colony task0.launch.py
```

Gazebo should open with the Kepler arena, the UR7e arm and its mast camera. In a second
terminal:

```bash
ros2 node list
ros2 topic list
```

The log shows one error, `A controller named 'forward_position_controller' was already
loaded`. It comes from the official launch file and you can ignore it.

Launch options that work on every task:

| Option | Effect |
|---|---|
| `rviz:=false` | Don't open RViz |
| `gui:=false` | Run Gazebo headless (no window); useful on slow machines or over SSH |

## 7. Run the Task 1 solutions

Use two terminals: one runs the simulation and one runs the node. Each launch file only
starts the world; the node is the solution.

| Task | Simulation | Solution node | What it does |
|---|---|---|---|
| 1A: ore perception | `ros2 launch eyantra_kepler_colony task1a.launch.py` | `ros2 run algorithms task1a.py` | Detects the 6 ores with the mast camera and broadcasts a TF per ore (`azurite_ore_1` …) in `base_link` |
| 1B: arm waypoints | `ros2 launch eyantra_kepler_colony task1b.launch.py` | `ros2 run algorithms task1b.py` | Drives the UR7e tool through the 5 waypoints with twist commands, holding each one |
| 1C: eBot navigation | `ros2 launch eyantra_kepler_colony task1c.launch.py` | `ros2 run algorithms task1c.py` | Follows `/ebot_path` and steers around rocks using `/scan` |

Wait until the simulation has fully started (about 30–45 s, until the arm or eBot is
spawned and the ores are on the conveyor) before you start the node.

A 1A tuning aid: `SAVE_FRAME=/tmp/f.png ros2 run algorithms task1a.py` saves the camera
frame so you can check the HSV ranges.

## 8. Offline checks (no simulator needed)

Each controller has a fast self-check. Run these after changing a node:

```bash
cd ~/strata_ws/src/eYRC_26-27_Strata-Cobot
python3 tools/sim_task1a.py   # ore detector on a real saved sim frame
python3 tools/sim_task1b.py   # kinematic UR7e through all 5 waypoints
python3 tools/sim_task1c.py   # unicycle eBot + synthetic lidar, 3 rock layouts
```

Each one prints `ok` lines, or fails with an assertion.

## 9. Run the evaluator

With the task's simulation running (terminal 1), start the evaluator in terminal 2 from the
repo folder:

```bash
cd ~/strata_ws/src/eYRC_26-27_Strata-Cobot
./eyrc-sc-evaluator --task 1A --team-id <YOUR_TEAM_ID>    # or 1B / 1C
```

It runs its checks and prints `Ready`. **Only then** start the node in terminal 3. When
the run is over, press `q` (or `Ctrl-C` on a non-interactive terminal); the 1A evaluator
stops by itself. It prints your marks and writes `SC-<team>-task-1X/result.zip`.

Scores from our runs: **1A 20/20, 1B 40/40, 1C 40/40.**

`result.zip` is signed. Do not unpack, rename, rebuild or edit it.

## 10. Package the submission

Put a copy of the node next to the `result.zip` the evaluator wrote, renamed, and zip the
two files together:

```
SC#<team>_task1B.zip
  ├── result.zip      from the evaluator, untouched
  └── task1B.py       copy of algorithms/scripts/task1b/task1b.py
```

The names above follow the official README example. **Check the archive name and the file
list on each task's page before uploading.** They can differ between subtasks.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `/usr/bin/env: 'bash\r'` or `'python3\r'` | Windows line endings. Run `git config core.autocrlf false`, then re-clone the repo (or convert the files back to LF). |
| `requirements.sh` says ROS is not installed | Install `ros-jazzy-desktop-full` (step 2) and open a new terminal. |
| `ros2 run algorithms task1X.py` → "No executable found" | Run `source ~/strata_ws/install/setup.bash`. If you added a new script, check that it is listed in `algorithms/setup.py`, rebuild, and confirm it is executable (`chmod +x`). |
| Evaluator: "More than one node is publishing the arm's own topics" | A previous simulation is still running. Close all sim terminals, run `pkill -f ur_arm_controller; pkill -f "gz sim"`, then relaunch. |
| Evaluator: "Arm is at its start pose" fails (1B) | The arm has already moved. Relaunch the simulation, and start the node only after `Ready`. |
| 1A finds no vanadinite, or a TF lands on the ground | HSV ranges are off for your rendering. Save a frame with `SAVE_FRAME` and check the ore colours. |
| Gazebo is very slow or the window is black under WSL | Run with `gui:=false rviz:=false`. Make sure WSL has enough RAM (`.wslconfig`) and that your GPU driver is up to date. |
| Calling WSL from Git Bash mangles `/mnt/...` paths | Prefix the command with `MSYS_NO_PATHCONV=1`. |

For how and why the nodes work the way they do, see `PROGRESS.md`. The original brief is in
`STRATA_COBOT_PROJECT_CONTEXT.md`.
