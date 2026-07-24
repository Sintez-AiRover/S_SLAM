# Swarm-SLAM Workspace — S3E Dataset + AiRover Simulation

A ROS 2 Humble workspace running [Swarm-SLAM (cslam)](https://github.com/lajoiepy/Swarm-SLAM)
decentralized multi-robot SLAM in two independent tracks:

| Track | What it does | Domain |
|---|---|---|
| **S3E** | Replays the S3E multi-robot dataset (3 robots, stereo + lidar) through cslam, with ground-truth evaluation and map saving | `42` |
| **Rover sim** | Runs cslam live on a 3-rover Ignition Fortress crop-field simulation | `43` |

The two tracks share the `cslam` core but are otherwise separate: different
launch files, configs, RViz layouts and ROS domains. **They can run at the same
time** — that is why they use different `ROS_DOMAIN_ID`s.

---

## Contents

- [What is in this repository](#what-is-in-this-repository)
- [Prerequisites](#prerequisites)
- [Getting the code](#getting-the-code)
- [Build](#build)
- [Track A — S3E dataset](#track-a--s3e-dataset)
- [Track B — Rover simulation](#track-b--rover-simulation)
- [Tools reference](#tools-reference)
- [Package layout](#package-layout)
- [Configuration reference](#configuration-reference)
- [Troubleshooting](#troubleshooting)

---

## What is in this repository

This repo holds the **integration work**, not the upstream SLAM stack and not the
datasets. Cloning it gives you roughly 11 MB:

| Tracked here | |
|---|---|
| `src/rover_slam/` | all rover↔cslam glue: configs, launches, RViz layout |
| `src/rover_description/` | AiRover model, Ignition world, teleop |
| `src/livox_msgs/`, `src/cslam_swarm_msgs/` | message packages |
| `start_*.sh`, `rover_drive_pattern.py`, `rover_slam_eval.py`, `check_saved_map.py` | run scripts and tools |
| `gt_eval_run*.txt` | results from the S3E tuning campaign |

| Deliberately **not** tracked | why | how to get it |
|---|---|---|
| `src/cslam*/` | separate upstream git repos; committing them would embed broken sub-repos | [clone them](#getting-the-code) |
| `**/data/S3E_*/`, `*.db3` | ~6.3 GB in `src/`, ~36 GB installed; GitHub rejects files >100 MB | `data/download.sh` |
| `*.pth` | CosPlace/NetVLAD weights | ships with `cslam` |
| `build/`, `install/`, `log/` | build artifacts | `colcon build` |
| `saved_maps/`, `drive_gt_log.csv` | run outputs | regenerated per run |

---

## Prerequisites

- **ROS 2 Humble** (`source /opt/ros/humble/setup.bash`)
- **rtabmap_ros** — provides `icp_odometry`, used for lidar odometry in both tracks
- **Ignition Fortress** + `ros_gz_sim` / `ros_gz_bridge` — rover track only
- **NVIDIA GPU** strongly recommended (see [GPU rendering](#gpu-rendering))

Python packages used by the cslam nodes:

```bash
pip install numpy scipy networkx sortedcontainers
```

Verify the ROS-side dependencies are present:

```bash
for p in rtabmap_odom rtabmap_slam ros_gz_sim ros_gz_bridge; do
    ros2 pkg prefix $p >/dev/null 2>&1 && echo "$p: OK" || echo "$p: MISSING"
done
```

---

## Getting the code

> Paths below assume the workspace at `~/ros2_ws`. In the project container the
> user is `root`, so that is the same directory as `/root/ros2_ws`, which is what
> the absolute paths elsewhere in this README refer to.

```bash
# 1. this repository
git clone https://github.com/Sintez-AiRover/S_SLAM.git ~/ros2_ws
cd ~/ros2_ws

# 2. the upstream Swarm-SLAM packages (not tracked here)
cd src
git clone https://github.com/lajoiepy/cslam.git
git clone https://github.com/lajoiepy/cslam_interfaces.git
git clone https://github.com/lajoiepy/cslam_experiments.git
git clone https://github.com/lajoiepy/cslam_visualization.git
cd ..

# 3. the S3E dataset (S3E track only -- skip for the rover sim)
bash src/cslam_experiments/data/download.sh
```

> ### ⚠ The upstream clones need local modifications
>
> A pristine clone of the four `cslam*` repos will **not** reproduce the results
> in this README. The working versions carry fixes made in this project that are
> not yet published anywhere:
>
> | Package | Modification | Without it |
> |---|---|---|
> | `cslam` | `stereo_handler.cpp` image subscriptions set to RELIABLE QoS | ~1 image ever received → zero keyframes |
> | `cslam` | null-logger guard in `decentralized_pgo.cpp` | segfault on the first keyframe |
> | `cslam` | `latest_local_transform_` for received keyframes | inter-robot edges rotated ~90° |
> | `cslam` | `save_map_callback` + `latest_optimized_aggregate_` | no map saving |
> | `cslam_experiments` | S3E camera pipeline, GT eval, C++ map accumulator, swarm status | no S3E pipeline, no evaluation |
> | `cslam_visualization` | `CMakeLists.txt`: `rtabmap_ros` → `rtabmap_msgs` + `rtabmap_conversions` | does not build on Humble |
>
> Publishing these is an open task — either fork the four repos under the project
> org and add them here as submodules, or vendor them into this repo. Until then,
> the modified sources live only in the working container.

---

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Two build details that matter:

**1. `cslam_experiments` must be built Release.** It contains the C++ map
accumulator; at `-O0` that node is roughly **30× slower** (13.9 s renders vs
~0.5 s), which starves the whole pipeline.

```bash
colcon build --packages-select cslam_experiments --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**2. `--symlink-install` is not optional in practice.** With it, edits to
`.py`, `.yaml`, `.rviz` and `.sdf` files under `src/` take effect on the next
launch with **no rebuild**. Every config path in this README assumes that.

> **Always build from `/root/ros2_ws`.** The Bash working directory persists
> between commands; a stray `cd src/cslam` before `colcon build` creates a
> nested workspace at `src/cslam/build` and the real `install/` never receives
> your change. If a fix seems to have no effect, check the binary's mtime and
> `/proc/<pid>/exe`.

---

## Track A — S3E dataset

### Data

Bags live in the install data directory (they are **not** in `src/`, so a clean
rebuild does not delete them, but a wiped `install/` does):

```
install/cslam_experiments/share/cslam_experiments/data/
├── S3E_Playground/     # default sequence
├── S3E_Campus_Road/    # 31.7 GB, far more inter-robot overlap
└── download.sh
```

Ground-truth files (`alpha_gt.txt`, `bob_gt.txt`, `carol_gt.txt` = robots 0/1/2,
TUM format, UTM coordinates) sit next to the bags and are read by the evaluator.

### Run

```bash
cd /root/ros2_ws
./start_s3e.sh sequence:=Campus_Road force_3dof:=true rate:=0.5
```

This is the **recommended command** — see [settled config](#s3e-settled-config).
`start_s3e.sh` kills leftover nodes from previous runs first (leftovers publish
on the same topics and corrupt the new map), then exports `ROS_DOMAIN_ID=42` and
launches everything: bag playback, camera pipeline, lidar odometry, cslam
front/back end, visualization and RViz.

**Never start the bag yourself** — the launch plays it. Two bag players means
two `/clock` publishers, which makes RViz reset thousands of times a second.

### Launch arguments

| Argument | Default | Notes |
|---|---|---|
| `sequence` | `Playground` | or `Campus_Road` |
| `rate` | `0.5` | bag playback rate. **The single biggest accuracy lever** — 0.25 gave 4–5× better odometry because ICP processes nearly every lidar frame instead of skipping ~60% |
| `max_nb_robots` | `3` | |
| `force_3dof` | `false` | passes `--Reg/Force3DoF true` to `icp_odometry`; strongly recommended, the sequences are flat |
| `flatten_map_z` | `false` | renders all keyframes at z=0. Cosmetic only, does not affect evaluation |
| `enable_visualization` | `true` | RViz + GT paths + merged map |
| `enable_swarm_status` | `true` | per-robot status + aggregated swarm table |
| `designated_origin_robot_id` | `0` | |

> `force_3dof` is applied **internally** by rtabmap. `ros2 param get` will still
> report `false` — trust the log line `Update parameter "Reg/Force3DoF"="true"
> from arguments`. Passing `-p Reg/Force3DoF:=true` instead **crashes** the node
> (string-vs-bool type clash).

### Evaluation

`s3e_gt_eval.py` runs automatically behind `enable_visualization`. It compares
the merged pose graph against RTK ground truth and appends to
`/root/ros2_ws/gt_eval_latest.txt`:

- **solo ATE** — per-robot fit, drift only
- **joint ATE** — one shared alignment across all merged robots, so it *includes*
  inter-robot merge error. The joint-minus-solo gap is exactly "how good is the merge"

Aligned GT is published as `nav_msgs/Path` on `/cslam/viz/gt_path_rN`.

> Alignment is **yaw + translation only**, never full 3D Kabsch: on a near-straight
> road a full-3D fit has a rotational gauge freedom and the aligned path spins
> about the travel axis.

Archived results from the tuning campaign are the `gt_eval_run*.txt` files.

### Saving a map

```bash
export ROS_DOMAIN_ID=42 && source install/setup.bash
ros2 run cslam_experiments map_saver_trigger.py \
    --dir ~/ros2_ws/saved_maps/my_run --optimizer-id 0 --max-nb-robots 3
```

Writes `poses.txt`, `edges.txt`, `pose_graph.g2o`, `manifest.json` and the
per-robot descriptor databases.

**Timing matters:** trigger *after the bag ends* but *before Ctrl-C* — the map
only exists in memory. The log must say `(optimized)`; `(UNOPTIMIZED initial
estimates)` means no optimization had completed and each robot's trajectory is
still in its own local frame.

Check a saved map:

```bash
python3 check_saved_map.py ~/ros2_ws/saved_maps/my_run --plot out.png
```

`PROBLEMS` are real bugs; `WARNINGS` are expected states (e.g. a robot that had
not merged at save time).

### S3E settled config

In `src/cslam_experiments/config/s3e_stereo_and_lidar.yaml` (comments there
document the full run-by-run history):

| Parameter | Value | Why |
|---|---|---|
| `pnp_min_inliers` | 15 | up from 12 |
| `max_waiting_time_sec` | 120 | up from 60; lets late bad edges be out-voted |
| `intra_loop_min_inbetween_keyframes` | 20 | tested at 10 → **catastrophic**, reverted |
| `inter_robot_loop_closure_budget` | 4 | tested at 8 → worse |
| `similarity_threshold` | 0.7 | tested looser → worse |
| `Icp/MaxCorrespondenceDistance` | 3.0 | in `rtabmap_s3e_lidar_odometry.launch.py` |

Best verified result: solo ATE 4.2 / 9.2 / 2.3 m, joint 9.5 / 11.0 / 8.4 m, no
catastrophic outlier, 100% cloud coverage end to end.

> **Run-to-run variance is real.** Two runs with *identical* config and binaries
> differed by 5–11 m joint ATE and ~25 loop closures, purely from timing-dependent
> loop-closure detection under CPU load. Treat any single-run "improvement" below
> that noise floor as unproven.

---

## Track B — Rover simulation

Three rovers in an Ignition crop field, running cslam live on simulated lidar —
no bag replay. Everything here is **additive**: the `cslam*` packages are not
modified by this track.

### Run

Three terminals, all on `ROS_DOMAIN_ID=43`.

**1. Simulation** (must start first — it provides `/clock`):

```bash
cd /root/ros2_ws
./start_rover_sim.sh                      # Gazebo GUI
./start_rover_sim.sh headless:=true       # server only
```

**2. SLAM** (ICP odometry + cslam + visualization + RViz):

```bash
./start_rover_slam.sh
./start_rover_slam.sh max_nb_robots:=2 enable_visualization:=false
```

**3. Drive.** Either by hand:

```bash
export ROS_DOMAIN_ID=43 && source /root/ros2_ws/install/setup.bash
ros2 run rover_description rover_teleop.py --rovers rover_0 rover_1 rover_2
```

or with the repeatable scripted pattern (preferred for any measurement):

```bash
python3 rover_drive_pattern.py --laps 3 --speed 0.35
```

### The field

```
   y
 +3.0  ● ● ● ● ● ● ● ●     crop row
 +2.4  ─────────────────   lane C  (1.2 m wide)
 +1.8  ● ● ● ● ● ● ● ●
  0.0  ─────────────────   lane B  (3.6 m wide — the only one allowing a 180°)
 -1.8  ● ● ● ● ● ● ● ●
 -2.4  ─────────────────   lane A  (1.2 m wide)
 -3.0  ● ● ● ● ● ● ● ●
       x=1.5 ........ 9.9
```

Plants are 1.2 m cylinders on an exact 1.2 m grid. Thirteen **perimeter
landmarks** (heights 1.5–3.2 m, all different, at deliberately non-grid
positions) ring the field — they exist because the bare crop grid is perfectly
periodic and scan-based place recognition cannot distinguish one position from
another without them. If you add more, keep them taller than the 1.2 m crop,
each a different height, off the 1.2 m grid, and outside the drivable area.

Rovers are small: chassis 0.234 × 0.207 m, DiffDrive capped at 1.0 m/s / 1.0 rad/s.

### Scripted drive

`rover_drive_pattern.py` drives a fixed circuit so that runs are **comparable** —
hand teleop is not reproducible, so "did that change help?" is otherwise
unanswerable. All three rovers circulate one shared ring in the *same direction*,
spaced apart, threading all three lanes; each lap revisits their own path
(intra-robot loop closures) and covers ground the others mapped (inter-robot).

It is closed-loop on **Ignition ground truth**, not odometry, and refuses to keep
driving a rover that is tilting, stalled, off its lane, or too close to an
obstacle. Obstacles are parsed from the world SDF, so landmarks you add are
picked up automatically.

```bash
python3 rover_drive_pattern.py --laps 3 --speed 0.35 --log drive_gt_log.csv
```

| Option | Default | |
|---|---|---|
| `--laps` | 3 | circuits of the ring |
| `--speed` | 0.35 | m/s |
| `--log` | `drive_gt_log.csv` | ground-truth trace |
| `--timeout` | 1800 | seconds |

It refuses to start if any rover is already tilted (stuck on something) — restart
the sim to reset them.

### Evaluation

```bash
python3 rover_slam_eval.py --duration 300 --rate 2
```

Samples ground truth and cslam estimates in the same loop, so the pairs are
simultaneous without any keyframe/sim-time sync, then reports **solo ATE**,
**joint ATE** and **inter-robot baseline error** (the last needs no alignment at
all, so it is the most assumption-free number and is directly comparable across
runs). Appends to `rover_eval_latest.txt`.

Run it *while the rovers are moving* — a stationary trajectory makes the
alignment ill-conditioned.

### Rover config

`src/rover_slam/config/rover_lidar.yaml` (symlinked into `install/`, so edits
need only a SLAM restart). Each value carries an in-file comment with the
measurement behind it.

| Parameter | Value | Why |
|---|---|---|
| `registration_min_inliers` | 45 | 60 → nothing verified, rover_1 could never merge. 35 → false loop closures warped a *parked* rover by 6 m |
| `keyframe_generation_ratio_distance` | 0.3 | at 0.5 the whole fleet built only ~53 keyframes — too few for PGO to average out a bad edge |
| `intra_loop_min_inbetween_keyframes` | 10 | at 20, with 12–26 keyframes per robot, intra-robot loop closure was *structurally impossible* (measured: exactly 0) |
| `similarity_threshold` | 0.7 | untouched — the next lever if false candidates persist |

> **Known open issue.** Merge quality in the rover field is still being tuned.
> Inlier count alone does not cleanly separate true from false matches in a
> repetitive crop field; the perimeter landmarks were added to attack that at
> the source. Judge changes with `rover_slam_eval.py`, never by eye.

### Not implemented for rovers

- **Dense merged point-cloud map.** RViz shows `cslam_visualization`'s live
  `/cslam/viz/cloudmarker`, which vanishes on shutdown. The dense accumulator
  (`s3e_map_accumulator`, proven on S3E) has not been ported.
- **Descriptor-DB saving.** `map_saver_trigger.py` works against the rover stack
  (it targets `/r{i}/cslam/...`, which is where the rover cslam nodes live) and
  the *pose graph* saves correctly — but the descriptor DB fails with
  `'ScanContextMatching' object has no attribute 'save'`. `save()` was only ever
  added to `NearestNeighborsMatching`; the rover config uses `scancontext`.

---

## Tools reference

| Tool | Purpose |
|---|---|
| `start_s3e.sh` | clean-start the S3E experiment (domain 42) |
| `start_rover_sim.sh` | Ignition rover simulation (domain 43) |
| `start_rover_slam.sh` | cslam + RViz for the rover sim (domain 43) |
| `rover_drive_pattern.py` | repeatable scripted drive, ground-truth closed-loop with safety guards |
| `rover_slam_eval.py` | rover accuracy vs Ignition ground truth |
| `check_saved_map.py` | validate + plot a saved map |
| `ros2 run cslam_experiments map_saver_trigger.py` | trigger a map save |
| `ros2 run rover_description rover_teleop.py` | manual keyboard driving |

---

## Package layout

```
src/
├── cslam/                  # Swarm-SLAM core (C++ back end + Python front end)
├── cslam_common_interfaces/
├── cslam_swarm_msgs/       # RobotStatus.msg, SwarmTable.msg (additive)
├── cslam_experiments/      # launches, configs, S3E pipeline, evaluation, map saving
├── cslam_visualization/    # upstream viz (patched: rtabmap_ros → rtabmap_msgs)
├── rover_description/      # AiRover model, Ignition world, teleop  (from Sintez)
├── livox_msgs/             # lidar msgs                              (from Sintez)
└── rover_slam/             # ALL rover↔cslam glue: configs, launches, RViz
```

---

## Configuration reference

| File | Controls |
|---|---|
| `src/cslam_experiments/config/s3e_stereo_and_lidar.yaml` | S3E cslam tuning |
| `src/cslam_experiments/launch/odometry/rtabmap_s3e_lidar_odometry.launch.py` | S3E ICP odometry |
| `src/cslam_experiments/config/s3e.rviz` | S3E RViz layout |
| `src/rover_slam/config/rover_lidar.yaml` | rover cslam tuning |
| `src/rover_slam/launch/rtabmap_rover_lidar_odometry.launch.py` | rover ICP odometry |
| `src/rover_slam/config/rover_swarm.rviz` | rover RViz layout (fixed frame `world`) |
| `src/rover_description/worlds/crop_field.sdf` | the simulated field + landmarks |
| `src/rover_description/config/swarm.yaml` | fleet size and spawn poses |

---

## Troubleshooting

### ROS domains

`42` = S3E, `43` = rover sim, `0` = **contaminated** (a Gazebo stack on another
LAN machine publishes `/clock` there; DDS auto-discovery will pull it in).

Any terminal inspecting an experiment — including `ros2 topic echo` — must export
the matching domain or it will see the wrong system entirely.

Do **not** set `ROS_LOCALHOST_ONLY=1`: with CycloneDDS it caps discovery at ~10
participants per host, and these launches spawn ~25 processes. The excess die
with `rmw_create_node: failed to create domain`.

### RViz "blinking" / constant resets

Always caused by **two `/clock` publishers**. Check first:

```bash
ROS_DOMAIN_ID=42 ros2 topic info /clock --verbose
```

Causes seen: a rogue Gazebo on domain 0, and starting `ros2 bag play` manually
in addition to the launch's own player.

### GPU rendering

Both tracks need PRIME offload, already set inside the start scripts:

```bash
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
```

Without it the GLX/dri3 path falls back to software: Ignition sensor rendering
drops the lidar from ~4.8 Hz to ~1.8 Hz (RTF 0.35 → 0.93) and RViz runs at ~1 fps
on llvmpipe. **A lidar rate near 4.8 Hz is the quick check that this is working.**

### `ros2` CLI hangs on domain 43

Frequent — the daemon wedges, and `ros2 daemon stop` hangs too:

```bash
pkill -9 -f ros2-daemon
```

Prefer short `rclpy` scripts for diagnostics. Note `Twist` fields need explicit
`float()`.

### Killing processes safely

`pkill -f "pattern"` **kills its own shell** when the pattern appears in its own
command line. Use the bracket trick and verify afterwards:

```bash
pkill -9 -f "[i]cp_odometry"
```

When both tracks are running, kill by domain rather than by name:

```bash
for pid in $(pgrep -f 'icp_odometry|pose_graph_manager'); do
    tr '\0' '\n' < /proc/$pid/environ 2>/dev/null | grep -q '^ROS_DOMAIN_ID=43$' && kill -9 $pid
done
```

`start_s3e.sh` kills globally by name — it will take down the rover track too.

### `ros2 launch` does not exit when the bag ends

Nodes idle forever. Always kill the previous launch before starting a new one,
or duplicate publishers will poison message-filter synchronization.

### Rover ground truth

```bash
ign model -m rover_0 -p           # authoritative, but ~5 s per call
```

Far too slow for a control loop — stream `/world/rover_world/dynamic_pose/info`
instead (that is what the drive and eval tools do).

**Wheel odometry lies.** A rover stuck against an obstacle reports metres of
travel. Always judge position against ground truth.

### Rovers stuck / tilted

A rover climbed onto a crop plant sits at pitch ≈ 42° and z ≈ 0.077 (nominal is
0.0606), its lidar pointing at sky and ground, so its scans match nothing and it
never merges. It cannot be freed by driving — **restart the sim**. Use the
scripted drive, which stops a rover before this happens.

### Topic names that cost debugging time

- The pose graph is on the **global** `/cslam/viz/pose_graph`, published by all
  robots (filter by `msg.robot_id`) — *not* `/r{i}/cslam/viz/pose_graph`.
- `/cslam/viz/cloudmarker` is a single `Marker`, not a `MarkerArray`.
  `cloudmarker_array` has no publisher.
- Viz publishers only publish while something is subscribed.
- cslam **hardcodes** the `/r{robot_id}` namespace for every inter-robot channel.
  Running its nodes under any other namespace breaks robot-to-robot silently:
  keyframes flow and edges grow, but pose graphs stay stuck at 1 value.

### Launch-configuration leakage

Wrap **every** `IncludeLaunchDescription` in
`PushLaunchConfigurations()` / `PopLaunchConfigurations()`. Without it an
argument declared inside one include leaks into the next — which once silently
handed the visualization node the wrong config file, disabling point clouds with
no error anywhere. Diagnose by checking the node's real command line:
`pgrep -fa <node>` and read `--params-file`.
