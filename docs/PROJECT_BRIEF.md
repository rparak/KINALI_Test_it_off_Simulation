# KINALI Test it off – Automated PCBA Testing Cell (MuJoCo) – Project Brief for Claude Code

> Put this file in the project root (e.g. as `docs/PROJECT_BRIEF.md`) and reference it from `CLAUDE.md`.
> Work language in code, comments, commit messages and docs: **English**. The user communicates in Czech/Slovak.

## 1. Context

This project simulates the robotic workcell **"Test it off"** by KINALI (https://www.testitoff.cz/en/about-test-it-off/),
a concept for **Automated PCBA Testing for Electronics Manufacturers**. The cell contains:

| Element | Model | Role |
|---|---|---|
| Robot | **AUBO i5** (MJCF from github.com/ian-chuang/Manipulator-Mujoco, MIT) on the robot pedestal | moves the PCB, opens/closes the tester |
| End effector | **OnRobot VGC10** (4 suction cups D15) + a **fork ("opener")** on the side | vacuum for the PCB, fork for the tester handle |
| Tester | **MG06** – manual ICT/FCT test fixture (lid, handle with toggle, press plate, lock) | the PCB is tested inside |
| Loader | single-purpose machine: 2 linear axes **QF12 S420 (Y)** + **QF12 S300 (Z)** + gripper **SCHUNK EGI 040-EI** | inserts the PCB into MG06 and takes it out |
| Storage | two blue crates (Box_Type_A): **input = left**, **output = right** (seen from the front of the cell) | PCB source / sink |
| PCB | UDCAA_63900_PCB, 120 × 85 × 1.8 mm, 60 g | the part |

The **physics and the realism of the cell are finished and tested** (see §3). The current robot motion is produced by an
IK-based planner (`tools/plan_cell.py`). **Phase 1 replaces that with a user-friendly, keypoint-based robot program.**

## 2. Goals

### Phase 1 (now) – keypoint program, terminal → MuJoCo GUI
1. **Keypoints in a configuration file** (YAML): named robot poses (joint values and/or TCP pose), loader axis positions,
   gripper and vacuum states. Human readable, editable, commented.
2. **Program file** (YAML), read **top to bottom**, describing the whole process from picking the PCB in the input crate to
   placing it into the output crate: `movej`, `movel`, loader moves, gripper open/close, vacuum on/off, wait, MG06 fork
   path. No planning algorithm – the path is exactly the listed keypoints; collision freedom is **checked**, not searched.
3. **Python CLI** (run from the terminal, opens the native MuJoCo viewer `mujoco.viewer`):
   - show the whole cell posed at **one keypoint** (robot + loader + jaws + PCB where it would be),
   - step through keypoints / program steps,
   - generate the trajectory and **play it** with physics in the viewer,
   - run the validation checks headless.
4. **Output**: one trajectory file with time-sampled commands for **all** elements (robot joints, loader Y/Z, jaws,
   vacuum, step labels), which reproduces the full process.
5. **PCB colour: green** (realistic solder-mask green), not orange.
6. Clean project structure, README, tests.

### Phase 2 (later) – PyQt6 application
MuJoCo rendered in a **PyQt6** window via OpenGL (`QOpenGLWidget` + `mujoco.MjrContext`/`mujoco.Renderer`), with controls:
keypoint list and editing (jog joints / TCP), program list, play / pause / step, timeline, cameras, check results.
Design Phase 1 so the core (model, keypoints, program, trajectory, simulation, checks) has **no GUI dependency** and can be
reused by the Qt app unchanged.

## 3. Current state (verified)

Files currently in the project root (copied from `mg06_cell_mujoco`):

| File / folder | What it is |
|---|---|
| `cell.py` | builds the full cell with `MjSpec`: `scene.xml` + AUBO i5 + VGC10/fork tool + crates (floor + 4 walls) + actuators + vacuum + look/cameras. `build_model()` returns `MjModel` |
| `scene.xml` | finished scene: workstation, MG06, loader, PCB (free body). Built earlier from Blender – **do not regenerate** |
| `cell_scene.xml` | the whole cell incl. robot as one MJCF (viewer only), written by `python cell.py` |
| `test_cell_cycle.py` | physics run of the whole cycle from `cell_plan.npz`, prints all checks (regression test) |
| `cell_plan.npz` | current plan: robot joint trajectory (dt 4 ms), loader moves, vacuum events, phase labels |
| `tools/plan_cell.py` | current IK-based planner (reference for timings, keypoints, special moves) |
| `robot_kin.py` | DLS IK for the suction TCP and the flange, collision check of a robot configuration |
| `pcb_meta.json` | PCB poses (CAD / place target / in the jaws) and loader axis positions |
| `mg06/assets`, `machine/assets`, `workstation/assets` | meshes (visual + collision) |
| `robot/aubo_i5/` | robot MJCF + meshes + licence |
| `robot_tool/` | `vgc10_opener_free.xml` + meshes, `fork_close_open_path.npz` (recorded flange path that closes/opens MG06) |

Run: `python test_cell_cycle.py` (≈1–2 min), viewer: `python cell.py && python -m mujoco.viewer --mjcf=cell_scene.xml`.

Reference result of the current cycle (must stay valid as a regression baseline):
```
cycle time 65.8 s
  3.28s vacuum ON  (4 of 4 cups touch the PCB)
  9.52s vacuum OFF
 54.76s vacuum ON  (4 of 4 cups touch the PCB)
 62.44s vacuum OFF
PCB in MG06 (t=16.4s): error 0.30 mm (dx 0.04 dy -0.29 dz 0.06), tilt 0.02 deg
MG06 closed with the robot away (t=31.6s): lid -0.5 deg, handle 28.7 deg
PCB at the end: x 406.3 y -752.4 z 4.9 mm (right crate centre 406.2 -751.6, floor 4.0); tilt 0.00 deg
robot joint tracking error max 0.0126 rad, PCB slip in the cups max 1.32 mm, loader axis lag max 22.6 mm
max contact force fork -> handle bar: 294 N
deepest PCB penetration: MG06 holder -0.20 mm, crate -0.08 mm, loader jaws -0.11 mm
robot <-> MG06 / workstation / crates             : no contact
robot tool <-> handle (outside closing / opening) : no contact
robot <-> loader                                  : no contact
loader <-> MG06 / workstation                     : no contact
PCB <-> lid / press plate                         : no contact
MG06 at the end: lid -40.0 deg, handle -0.0 deg
```

### Process (current, 65.8 s)
1. Robot picks the PCB with vacuum from the **left (input) crate**.
2. Robot puts it into the open jaws of the loader (loader home), loader closes, vacuum off, robot leaves.
3. Loader: up → towards MG06 → down into the holder → open → up → park (y −199.5 mm, z +20 mm).
4. Robot closes MG06 with the fork (lid down, handle to ~29°), leaves, comes back, opens MG06.
5. Loader: back → down → close (pick) → up → home.
6. Robot takes the PCB (vacuum on, loader opens), puts it into the **right (output) crate**, goes home.

## 4. Model reference

### Frames
Everything is in the **MG06 fixture frame**: origin = centre of the MG06 footprint on the table top, **Z up**, front of MG06 = **−Y**,
units **m / rad**. The robot base is at `(0.42072, −0.33497, 0.162)` (top of `deska_robot`), yaw **−135°**.
The cassette / PCB plane in MG06 is tilted **8.94° about X**.

### qpos layout (from `cell.build_model()`)
| qpos | joint |
|---|---|
| 0 / 1 / 2 | `lid_hinge` (0 closed, −0.6974 open), `press_slide`, `handle_hinge` (0 up, ~0.50 = 29° closed) |
| 3 / 4 | loader `y_axis` (−0.1995 … 0.2205), `z_axis` (−0.0035 … 0.2965) |
| 5 / 6 | `jaw_L`, `jaw_R` (0 … 0.040, coupled by equality) |
| 7–13 | `pcb_free` (pos + quat) |
| 14–19 | `aubo/shoulder_pan_joint` … `aubo/wrist_3_joint` (limits ±3.04) |
| 20–23 | `aubo/tool/cup0..3_bellows` (passive springs of the suction cups) |

### Actuators (ctrl)
| idx | name | meaning |
|---|---|---|
| 0, 1 | `y_servo`, `z_servo` | loader axis position (m); first-order lag τ = 30 ms (`filterexact`, `act` state – initialise `act` = start position) |
| 2 | `gripper` | jaw target (m); force-limited to 40 N/jaw. **Clamp = target 12 mm behind the part** (`jaw_grip − 0.012`), open = 0.035 |
| 3–8 | `robot_*` | joint position servos (kp 5000/5000/4000/2000/2000/1500, kv 150/150/100/25/25/15, armature, gravcomp). Use **velocity feed-forward**: `ctrl = q_ref + (kv/kp)·v_ref` |
| 9–12 | `vacuum0..3` | **adhesion** actuators on the 4 cups, ctrl 0/1, 7 N each (40 kPa × D15). Holds only when the cup touches |

Sites: `aubo/tool/suction_tcp` (cup face, 260.6 mm from the flange, tool +Z = cup direction), `aubo/tool/tool_flange`,
`aubo/tool/fork_center` (slot centre of the fork), `handle_grasp` (handle bar centre). Cameras: `cam_pcb`, `cam_tool`
(`targetbody`). Timestep 0.5 ms, elliptic cones, impratio 10, noslip 5.

### Key numbers
- Loader: home y −0.1000 / z 0; place y +0.19666 / z +0.06722; travel height z +0.07922; park y −0.1995 / z +0.020;
  jaws open 0.035, grip 0.01775 (clamp command 0.00575).
- PCB in the loader jaws (loader home): pos `(0.0057, −0.30829, 0.14961)`, quat `(0.99696, 0.07794, 0, 0)` (8.94° tilt);
  placed in MG06: pos `(0.0057, −0.011636, 0.216826)`, same quat (see `pcb_meta.json`).
- PCB in a crate: flat, board centre at crate centre, z = 0.004 + 0.0009. Input crate centre (0.15720, −0.75160),
  output crate centre (0.40620, −0.75160).
- Suction TCP relative to the PCB body frame: `(−0.00125, 0.0046, 0.0075)`, tool axes = diag(1, −1, −1) in PCB frame.
- Robot speed: AUBO i5 datasheet 220°/s (J1–J4), 230°/s (J5–J6); plan uses **60 %**; acceleration limit (assumed)
  3 rad/s² (J1–J3), 6 rad/s² (J4–J6). Free MoveL 200–250 mm/s, last 15 mm before contact 30 mm/s.

### Lessons learned (do not break these)
- **Vacuum:** press the cups **1.5 mm** onto the PCB before vacuum on (bellows). At the **loader handover do NOT press**:
  the board must sit 0.3 mm above the finger ledges, otherwise the jaws clamp on the ledge corners.
- **Robot IK families:** PCB handling uses "wrist up" (wrist_2 > 0), the MG06 fork path uses "wrist down" (wrist_2 < 0).
  Switch between them only in free space (via `via_mg06_high`).
- **MG06 fork path:** the recorded close/open path is tight (bar on the slot bottom). Leaving/returning uses a separately
  planned path with the bar centred in the slot (5 mm play), based on the closed handle angle **28.65°** and lid **−0.5°**
  (`H_CLOSED`, `LID_CLOSED`). Joining the opening path must start where the recorded handle angle is already below the real
  closed angle, so the handle is **never pushed down** before opening.
- MuJoCo 3.16: `gap` is not applied to `includemargin` – don't use `margin/gap` on the PCB/cups (things float).
  PCB and cup contacts use `solref 0.002`, `solimp 0.99 0.999`, `solmix 100`.
- MuJoCo **resets the state on instability** – the run must detect `d.time` going backwards and fail (already in
  `test_cell_cycle.py`), never loop silently.
- CAD deviations already compensated in `scene.xml`: loader shifted +100 mm in Y and +0.95 mm in X, rear-right holder
  support lowered by 0.8 mm, electrical cabinet 200 mm deep instead of 300 mm. Keep them; they are documented for the user.

## 5. Phase 1 – target design

### 5.1 Proposed structure
```
KINALI_Test_it_off_Simulation/
├─ CLAUDE.md                     # short: how to work in this repo, links to docs
├─ README.md                     # user-facing (use the writer-readme skill)
├─ pyproject.toml                # package + console script `kinali-sim`
├─ config/
│  ├─ cell.yaml                  # robot base, speed %, accel limits, loader limits, vacuum, tolerances
│  ├─ keypoints.yaml             # named keypoints (robot / loader / gripper / vacuum)
│  └─ program.yaml               # the process, top to bottom
├─ models/
│  ├─ scene.xml                  # finished scene (do not regenerate)
│  └─ cell_scene.xml             # generated by `kinali-sim export-xml`
├─ assets/                       # mg06/, machine/, workstation/, robot/aubo_i5/, robot_tool/  (paths updated)
├─ data/
│  ├─ pcb_meta.json
│  └─ fork_close_open_path.npz
├─ src/kinali_sim/
│  ├─ __init__.py
│  ├─ model.py                   # from cell.py (build_spec / build_model / export_xml / style)
│  ├─ kinematics.py              # from robot_kin.py (FK, IK, collision query)
│  ├─ config.py                  # load + validate YAML (pydantic or dataclasses), units, defaults
│  ├─ keypoints.py               # keypoint resolution: joints | tcp(+seed) → joints; pose the whole cell
│  ├─ program.py                 # program steps → timeline of segments/events
│  ├─ trajectory.py              # movej (min-jerk, speed/accel limits), movel (Cartesian, IK per sample), path follow
│  ├─ devices.py                 # loader axes, gripper, vacuum command profiles
│  ├─ simulate.py                # physics run of a trajectory (servo feed-forward, events, instability guard)
│  ├─ checks.py                  # collisions, PCB pose errors, cups in contact, penetration, forces → report
│  ├─ viewer.py                  # mujoco.viewer: show keypoint, step, play trajectory
│  └─ cli.py                     # `kinali-sim ...`
├─ output/                       # trajectory.npz / trajectory.csv / report.txt (git-ignored)
├─ tests/                        # pytest: config validation, IK round-trip, short segments, full-cycle regression (slow)
└─ legacy/                       # plan_cell.py, test_cell_cycle.py, cell_plan.npz – kept as reference until replaced
```

### 5.2 Keypoint file (proposal)
```yaml
units: {length: m, angle: deg}          # angles in deg for humans; convert internally
robot_keypoints:
  home:
    joints: [45.24, 39.59, 105.04, -24.56, 90.00, 0.24]
  above_input_pcb:
    tcp: {pos: [0.15595, -0.74700, 0.13240], rpy: [180, 0, 0]}   # or joints
    seed: wrist_up                         # IK family / seed keypoint
  ...
loader_keypoints:
  home:   {y: -0.1000, z: 0.0}
  travel_place: {y: 0.19666, z: 0.07922}
  place:  {y: 0.19666, z: 0.06722}
  park:   {y: -0.1995, z: 0.020}
gripper: {open: 0.035, clamp: 0.00575}
```
Rules: every robot keypoint is stored/resolved to joints once (IK with the given family/seed) and the resolved joints are
written back or cached, so playback is deterministic. A keypoint may be defined relative to another one
(`relative_to: pick_input_pcb, offset: [0, 0, 0.12]`).

### 5.3 Program file (proposal)
```yaml
- {step: "Pick PCB from input crate", do: [ {movej: above_input_pcb}, {movel: input_pcb_approach, speed: 0.20},
                                             {movel: input_pcb_press, speed: 0.03}, {vacuum: on}, {wait: 0.3},
                                             {movel: above_input_pcb, speed: 0.20} ]}
- {step: "Hand over to the loader", do: [ {movej: via_input_to_loader}, {movej: above_loader},
                                          {movel: loader_approach, speed: 0.20}, {movel: loader_handover, speed: 0.03},
                                          {gripper: clamp}, {wait: 0.25}, {vacuum: off}, ... ]}
- {step: "Loader inserts PCB into MG06", parallel: false, do: [ {loader: {z: travel_place.z}}, ... ]}
- {step: "Close MG06", do: [ {movej: via_mg06_high}, {movej: fork_start}, {path: fork_close}, ... ]}
...
```
Supported commands (Phase 1): `movej`, `movel`, `path` (recorded flange path segment), `loader` (y / z, duration or
speed), `gripper` (open / clamp), `vacuum` (on / off), `wait`. Optional `blend` / parallel execution of loader and robot
can come later – first keep steps sequential.

### 5.4 Trajectory output
`output/trajectory.npz` (+ optional CSV): `t`, `robot_q[6]`, `robot_qd[6]`, `loader_y`, `loader_z`, `gripper`,
`vacuum`, `step_index`, `step_label`, sampled at 4 ms (same as now). `simulate.py` plays it with physics and produces the
report of §3. The trajectory alone must be enough to reproduce the whole process.

### 5.5 CLI (proposal)
```
kinali-sim show home                 # viewer, cell posed at keypoint "home"
kinali-sim show --step 5             # cell at the end of program step 5
kinali-sim keypoints --check         # IK + collision clearance for all keypoints
kinali-sim build                     # program → output/trajectory.npz (+ checks of every segment)
kinali-sim play                      # viewer, physics playback of output/trajectory.npz
kinali-sim run                       # headless physics + report (regression)
kinali-sim export-xml                # models/cell_scene.xml
```

### 5.6 Acceptance criteria for Phase 1
- `kinali-sim run` with the default config reproduces the reference report (§3) within tolerances:
  no robot/loader collisions, 4/4 cups at both picks, PCB in MG06 error < 0.5 mm & tilt < 0.2°, PCB in the output crate
  < 2 mm from the centre, MG06 ends open, no instability.
- Every keypoint can be shown in the viewer; invalid keypoints (IK fails, collision < 3 mm) are reported clearly.
- PCB is green.
- README explains: installation, CLI, how to edit keypoints/program, file formats.

## 6. Initial keypoints (extracted from the verified plan)

Robot joints in **rad** (order: pan, lift, elbow, wrist1, wrist2, wrist3), TCP = `aubo/tool/suction_tcp` in m.

| name | joints (rad) | TCP / flange (m) | note |
|---|---|---|---|
| `home` | 0.7896, 0.6910, 1.8333, −0.4286, 1.5708, 0.0042 | tcp (0.3000, −0.5200, 0.4000) | tool down |
| `above_input_pcb` | 0.4649, −0.1335, 2.0022, 0.5650, 1.5708, −0.3205 | tcp (0.1559, −0.7470, 0.1324) | 120 mm above |
| `input_pcb_approach` | 0.4649, −0.3261, 2.1191, 0.8743, 1.5708, −0.3205 | tcp z 0.0274 | 15 mm above |
| `input_pcb_press` | 0.4649, −0.3633, 2.1304, 0.9229, 1.5708, −0.3205 | tcp z 0.0109 | cups 1.5 mm pressed → vacuum on |
| `via_input_to_loader` | 0.0465, 0.0565, 1.4637, −0.1952, 1.4994, −0.7367 | tcp (0.0164, −0.5608, 0.3958) | free-space via |
| `above_loader` | −0.3720, 0.1896, 1.7926, −0.0310, 1.4280, −1.1529 | tcp (0.0045, −0.3298, 0.3158) | 160 mm along the plate normal |
| `loader_approach` | −0.4242, 0.1057, 2.1193, 0.3873, 1.4249, −1.2055 | tcp (0.0045, −0.3072, 0.1726) | 15 mm along normal |
| `loader_handover` | −0.4296, 0.0867, 2.1469, 0.4347, 1.4246, −1.2110 | tcp (0.0045, −0.3049, 0.1578) | PCB at the hold pose, **no press** |
| `loader_take_press` | −0.4302, 0.0847, 2.1496, 0.4395, 1.4246, −1.2116 | tcp (0.0045, −0.3047, 0.1563) | 1.5 mm press → vacuum on |
| `loader_out_of_jaws` | −0.4187, 0.1225, 2.0906, 0.3409, 1.4252, −1.2000 | tcp (0.0044, −0.3096, 0.1873) | 30 mm along normal |
| `via_mg06_high` | −0.0138, 0.2020, 0.7805, −1.6993, −0.0164, 0.4996 | flange (0.1251, −0.3218, 0.9331) | wrist-up ↔ wrist-down switch |
| `above_handle` | −0.8205, 0.2056, 0.9667, −2.3805, −1.6059, 1.1519 | flange (0.1550, −0.2074, 0.8531) | |
| `fork_start` | −1.1328, 0.0136, 1.1340, −2.0213, −1.9182, 1.1519 | flange (0.0550, −0.1074, 0.7531) | start/end of the recorded path |
| `fork_closed_end` | −0.5157, 0.0227, 2.4098, −0.7545, −1.3011, 1.1518 | flange (0.0550, −0.2840, 0.3154) | end of `path: fork_close` |
| `fork_bar_centred` | −0.5118, 0.0507, 2.3961, −0.7961, −1.2972, 1.1518 | flange (0.0550, −0.2850, 0.3259) | bar in slot centre, 2 mm off bottom |
| `fork_out_of_bar` | −0.8243, 0.0582, 2.2746, −0.9252, −1.6097, 1.1518 | flange (0.0550, −0.2028, 0.3625) | 90 mm along the slot |
| `fork_retreat_up` | −0.8243, 0.2137, 2.0471, −1.3081, −1.6097, 1.1518 | flange (0.0550, −0.2028, 0.4825) | +120 mm, wait here (MG06 closed) |
| `fork_open_join` | −0.5034, 0.0542, 2.3958, −0.8000, −1.2887, 1.1520 | flange (0.0550, −0.2872, 0.3269) | join point of `path: fork_open` |
| `via_loader_to_output` | 0.7093, 0.7032, 1.8349, −0.4944, 1.5158, −0.0734 | tcp (0.2680, −0.4831, 0.3958) | free-space via |
| `above_output` | 1.0463, 0.0611, 2.2028, 0.5709, 1.5708, 0.2609 | tcp (0.4050, −0.7470, 0.1334) | |
| `output_approach` | 1.0462, −0.1697, 2.3385, 0.9374, 1.5708, 0.2609 | tcp z 0.0284 | |
| `output_place` | 1.0462, −0.2120, 2.3510, 0.9922, 1.5708, 0.2609 | tcp z 0.0134 | PCB 1 mm above the crate floor → vacuum off |

`path: fork_close` = rows of `fork_close_open_path.npz` with phase `CLOSE` (≈6.1 s, after the first rows that bring the
fork from `fork_start`); `path: fork_open` = rows with phase `OPEN` after the join row (handle ≤ 28.45°). The file stores
`t, lid, press, handle, flange_pos[3], flange_quat[4]` per row; follow it with IK on the flange site (see
`Plan.replay` in `tools/plan_cell.py`). A later task may approximate these paths by a few MoveL keypoints – only if the
physics checks still pass (force on the bar, final lid/handle angles).

## 7. Work plan (Phase 1)

Use the project skills in `.claude/skills` (`feature-manager` for planning/tracking features, `project-optimization`
for structure and code quality, `writer-readme` for README/docs, and any other relevant skill there).

1. **Restructure** into §5.1 (package `kinali_sim`, `config/`, `models/`, `assets/`, `data/`, `legacy/`). Update all
   asset paths. `kinali-sim run` must still reproduce the reference report using the *legacy* `cell_plan.npz`.
2. **PCB green**: `pcb_vis` rgba in `models/scene.xml` (now `0.91 0.45 0.03 1`) → e.g. `0.05 0.42 0.18 1`; regenerate
   `cell_scene.xml`. Optional: split the PCB mesh into board and components (z above the board top) and colour
   components grey/black.
3. **Config + keypoints**: schema, loader, validation, IK resolution (families/seeds), `kinali-sim show` and
   `kinali-sim keypoints --check`. Seed `keypoints.yaml` with §6.
4. **Program + trajectory**: commands of §5.3, min-jerk MoveJ with speed/accel limits, Cartesian MoveL, `path`, loader /
   gripper / vacuum profiles; `kinali-sim build` writes `output/trajectory.npz`; segment checks (3 mm clearance, carried
   PCB included).
5. **Simulation + checks + viewer playback**: move the physics loop and the report from `test_cell_cycle.py` into
   `simulate.py` / `checks.py`; `kinali-sim play` (passive viewer, real-time) and `kinali-sim run`.
6. **Regression**: the default `program.yaml` must reproduce §3 (acceptance §5.6). Then remove `legacy/`.
7. **Docs**: README (writer-readme), `CLAUDE.md` with conventions and commands.

Keep commits small and per feature. After each step run the regression and report the numbers.

## 8. Phase 2 outline (PyQt6)
- `QOpenGLWidget` hosting MuJoCo rendering (`MjvScene`, `MjrContext` in the widget's GL context) or offscreen
  `mujoco.Renderer` → `QImage` as a fallback; mouse camera control.
- Panels: keypoints (table, jog joints/TCP, IK family, save), program steps (list, run from step, step-by-step),
  timeline slider, check results, camera selector (`free`, `cam_pcb`, `cam_tool`).
- Simulation in a worker thread / QTimer with fixed real-time stepping; the core library from Phase 1 unchanged.
