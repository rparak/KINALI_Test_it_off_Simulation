# Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo — Specification

Repository: `KINALI_Test_it_off_Simulation`.

Status: Approved 2026-10-24

A number marked *measured* names what it was measured on; where a test pins it, the test is named.
This specification is derived from the owner's project brief (git `427e076:docs/PROJECT_BRIEF.md`,
removed from the tree when this SPEC replaced it; §15) and from the verified state of the code.

## 1. Goal

**Project context** (owner, 2026-10-24): An internal research project of JIC Smart Factory, part of its internal research supporting small and medium-sized enterprises in advanced simulation. Developed in collaboration with KINALI for the development of the Test-it-off system, an automated workcell for testing assembled printed circuit boards (PCBA). The project has no external or public
funding. Brand: JIC Smart Factory, in collaboration with KINALI, product Test-it-off, referenced by name only (design-decisions).

**Test-it-off Simulation** (design-decisions: the name) simulates the robotic workcell **Test-it-off** by KINALI (automated PCBA testing for electronics
manufacturers, https://www.testitoff.cz/en/about-test-it-off/) in MuJoCo, so that a robot program for
the whole process - PCB from the input crate, through the MG06 tester, to the output crate - can be
written, checked and played back before it is run anywhere else.

The physics and the realism of the cell are finished and verified (§4.6). The robot motion is today
produced by an IK-based planner (`legacy/plan_cell.py`, kept in git tag `legacy-planner-reference`;
its plan is `Data/Reference/cell_plan.npz`). **Phase 1 replaces it with a keypoint-based
robot program** (named poses in a configuration file, a program read top to bottom, a trajectory file,
a viewer); Phase 2 adds a PyQt6 application on the same core. Phase 1 is done when the default program
reproduces the reference cycle within the acceptance criteria of §10.1.

## 2. Scope

Phase 1 (brief §7, in this order):

1. **Restructure** into the RPX layout (§6) with every path resolved from the code's location. *Done
   in the brief's layout (package `kinali_sim`); the RPX migration is `FEATURES.md` F3.*
2. **Green PCB**: the visual geom `pcb_vis` in solder-mask green `0.05 0.42 0.18 1`. *Done.* Optional
   follow-up: split the PCB mesh into board and components (`KNOWN_ISSUES.md`).
3. **Configuration and keypoints**: a YAML configuration (§8) with named keypoints - robot (joints, or
   TCP pose + IK family/seed, or relative to another keypoint), loader axes, gripper, vacuum - resolved
   to joints once so that playback is deterministic; a viewer that poses the whole cell at one
   keypoint; a check of every keypoint (IK, collision clearance ≥ 3 mm).
4. **Program and trajectory**: a program file read top to bottom with the commands `movej`, `movel`,
   `path` (recorded flange path), `loader` (y / z), `gripper` (open / clamp), `vacuum` (on / off),
   `wait`; min-jerk MoveJ with speed and acceleration limits, Cartesian MoveL (IK per sample), path
   following; a trajectory file (§9); a clearance check of every segment, carried PCB included. No
   planning algorithm: collision freedom is checked, not searched. Steps are sequential.
5. **Simulation, checks, playback**: the physics run of a trajectory with the report of §4.6, a
   real-time viewer playback, a headless run.
6. **Regression**: the default program reproduces §4.6 within §10.1; then it replaces `Data/Reference/cell_plan.npz`
   as the regression input. (The legacy planner and cycle test were removed in F4; git tag `legacy-planner-reference`.)
7. **Docs**: the README (readme-writer) and `CLAUDE.md`.

Phase 2 (later): a PyQt5 or PyQt6 window (open, see design-decisions: *Phase 2 GUI toolkit*) rendering MuJoCo through
OpenGL (`QOpenGLWidget` + `mujoco.MjrContext` / `mujoco.Renderer`, or an
offscreen `mujoco.Renderer` into a `QImage` as a fallback), with the keypoint list and editing (jog
joints / TCP, IK family, save), the program list (run from a step, step by step), play / pause / step,
a timeline, camera selection (`free`, `cam_pcb`, `cam_tool`) and the check results, with mouse camera
control; the simulation runs in a worker thread or a QTimer with fixed real-time stepping. The Phase 1
core has no GUI dependency, so the Qt application reuses it unchanged. The application carries the project's
brand (design-decisions: the brand) and the window title **Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo**: the JIC Smart Factory and KINALI logos in black or white SVG
(`Media/`), no funding band. The brief's outline, verbatim:

- `QOpenGLWidget` hosting MuJoCo rendering (`MjvScene`, `MjrContext` in the widget's GL context) or offscreen
  `mujoco.Renderer` → `QImage` as a fallback; mouse camera control.
- Panels: keypoints (table, jog joints/TCP, IK family, save), program steps (list, run from step, step-by-step),
  timeline slider, check results, camera selector (`free`, `cam_pcb`, `cam_tool`).
- Simulation in a worker thread / QTimer with fixed real-time stepping; the core library from Phase 1 unchanged.

Work rule (brief §7): keep commits small and per feature; after each step run the regression and report
the numbers. The brief's work plan with its original command names (superseded by the `Example/`
scripts, §16.4) is kept verbatim:

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

## 3. Out of scope

- Regenerating `scene.xml` from Blender: the scene is finished (§4.7).
- Changing the verified physics: timestep, solver options, any solref/solimp/solmix/margin/gap,
  friction, masses, geometry, actuator gains and ranges, the vacuum/adhesion setup, the bellows springs,
  the CAD deviations (§4.7). A change to any of these is a feature with its own report.
- Path planning (search): the program lists the path; the tool checks it.
- Blending and parallel robot/loader execution in Phase 1 (possible later).
- Driving a real robot or machine now. A transfer to the real cell is planned later (§17).

## 4. Measured facts this specification rests on

### 4.1 The cell

| Element | Model | Role |
|---|---|---|
| Robot | **AUBO i5**, MJCF from github.com/ian-chuang/Manipulator-Mujoco (MIT), on the robot pedestal | moves the PCB, opens and closes the tester |
| End effector | **OnRobot VGC10** (4 suction cups D15) + a **fork ("opener")** on the side | vacuum for the PCB, fork for the tester handle |
| Tester | **MG06** manual ICT/FCT fixture (lid, handle with toggle, press plate, lock) | the PCB is tested inside |
| Loader | 2 linear axes **QF12 S420 (Y)** + **QF12 S300 (Z)** + gripper **SCHUNK EGI 040-EI** | inserts the PCB into MG06 and takes it out |
| Storage | two blue crates (Box_Type_A): **input = left**, **output = right**, seen from the front | PCB source and sink |
| PCB | UDCAA_63900_PCB, 120 × 85 × 1.8 mm, 60 g | the part |

### 4.2 Frames and units

Everything is in the **MG06 fixture frame**: origin = centre of the MG06 footprint on the table top,
**Z up**, front of MG06 = **−Y**; units **m / rad / s** in code. The robot base is at
(0.42072, −0.33497, 0.162) m (top of `deska_robot`, the CAD origin of the cell), yaw **−135°**. The
cassette / PCB plane in MG06 is tilted **8.94° about X**.

### 4.3 Compiled model (*measured*, MuJoCo 3.16.0, 2026-10-24; `test/Scene/data/baseline_model_fingerprint.json`)

nq 24, nv 23, nu 13, na 2, nbody 25, ngeom 509, nmesh 438, neq 2; timestep 0.5 ms, elliptic cones,
impratio 10, noslip iterations 5, solver Newton, integrator Euler (the scene's; §14, `KNOWN_ISSUES.md`).

| qpos | joint |
|---|---|
| 0 / 1 / 2 | `lid_hinge` (0 closed, −0.6974 open), `press_slide`, `handle_hinge` (0 up, about 0.50 = 29° closed) |
| 3 / 4 | loader `y_axis` (−0.1995 … 0.2205), `z_axis` (−0.0035 … 0.2965) |
| 5 / 6 | `jaw_L`, `jaw_R` (0 … 0.040, coupled by an equality) |
| 7–13 | `pcb_free` (position + quaternion) |
| 14–19 | `aubo/shoulder_pan_joint` … `aubo/wrist_3_joint` (limits ±3.04) |
| 20–23 | `aubo/tool/cup0..3_bellows` (passive springs of the suction cups) |

| ctrl | name | meaning |
|---|---|---|
| 0, 1 | `y_servo`, `z_servo` | loader axis position [m]; first-order lag τ = 30 ms (`filterexact`; its `act` state starts at the start position) |
| 2 | `gripper` | jaw target [m], force-limited to 40 N per jaw; clamp = 12 mm behind the part (`jaw_grip − 0.012`), open = 0.035 |
| 3–8 | `robot_*` | joint position servos, kp 5000/5000/4000/2000/2000/1500, kv 150/150/100/25/25/15, armature, gravity compensation; driven with velocity feed-forward `ctrl = q_ref + (kv/kp)·v_ref` |
| 9–12 | `vacuum0..3` | adhesion on the 4 cups, ctrl 0/1, 7 N each (40 kPa × D15); holds only where a cup touches |

Sites: `aubo/tool/suction_tcp` (cup face, 260.6 mm from the flange, tool +Z = cup direction),
`aubo/tool/tool_flange`, `aubo/tool/fork_center`, `handle_grasp`. Cameras `cam_pcb`, `cam_tool`
(`targetbody`).

### 4.4 Key numbers

- Loader: home y −0.1000 / z 0; place y +0.19666 / z +0.06722; travel height z +0.07922; park
  y −0.1995 / z +0.020; jaws open 0.035, grip 0.01775 (clamp command 0.00575). Source: now
  `Configuration/Keypoints.yaml` (`Loader`, `Gripper`; formerly `pcb_meta.json`), `legacy/plan_cell.py` (git
  tag `legacy-planner-reference`).
- PCB in the loader jaws (loader home): position (0.0057, −0.30829, 0.14961), quaternion
  (0.99696, 0.07794, 0, 0) (8.94° tilt); placed in MG06: (0.0057, −0.011636, 0.216826), same quaternion.
- PCB in a crate: flat, board centre at the crate centre, z = 0.004 + 0.0009. Input crate centre
  (0.15720, −0.75160), output crate centre (0.40620, −0.75160).
- Suction TCP in the PCB body frame: (−0.00125, 0.0046, 0.0075); tool axes diag(1, −1, −1).
- Robot speed: AUBO i5 datasheet 220°/s (J1–J4), 230°/s (J5–J6); the plan uses 60 %. Acceleration
  limits 3 rad/s² (J1–J3), 6 rad/s² (J4–J6) are an **assumption**, not in the datasheet. Free MoveL
  200–250 mm/s, the last 15 mm before contact 30 mm/s.
- Initial keypoints: §4.8; the recorded fork path: §4.9.

### 4.5 Process of the reference cycle (65.8 s)

1. The robot picks the PCB with vacuum from the left (input) crate.
2. It puts it into the open loader jaws (loader home); the loader closes, vacuum off, the robot leaves.
3. Loader: up → towards MG06 → down into the holder → open → up → park (y −199.5 mm, z +20 mm).
4. The robot closes MG06 with the fork (lid down, handle to about 29°), leaves, comes back, opens MG06.
5. Loader: back → down → close (pick) → up → home.
6. The robot takes the PCB (vacuum on, loader opens), puts it into the right (output) crate, goes home.

### 4.6 Reference result (*measured*, `test/Simulation/data/baseline_report.txt`)

The reference plan `Data/Reference/cell_plan.npz` (written by the legacy planner) run with physics, 2026-10-24, on macOS 26.6.2, Apple M2 Pro.
Baseline environment: Python 3.11.2, numpy 2.4.6, scipy 1.17.1, MuJoCo 3.16.0 (`install.sh`). First
measured with Python 3.14.7 / numpy 2.5.4; the two reports are byte-identical:

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
deepest PCB penetration: MG06 holder -0.20 mm (t=16.14), crate -0.14 mm (t=62.46), loader jaws -0.11 mm (t=9.13)
robot <-> MG06 / workstation / crates             : no contact
robot tool <-> handle (outside closing / opening) : no contact
robot <-> loader                                  : no contact
loader <-> MG06 / workstation                     : no contact
PCB <-> lid / press plate                         : no contact
MG06 at the end: lid -40.0 deg, handle -0.0 deg
```

Run time 15–16 s on that machine. The brief's original reference (environment not recorded) differs in
one value, the crate penetration (`KNOWN_ISSUES.md`); verbatim from brief §3:

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

The brief's file table of that state (brief §3, verbatim; every file's new place is in `FEATURES.md`
F1 and F3 *Files: old → new*):

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

The legacy planner regenerated `cell_plan.npz` to within 1.2e-14 rad (2026-10-24, through the ported
Kinematics and Collision code; F3.4).

### 4.7 Lessons learned and deliberate deviations (do not break)

- **Vacuum**: the cups are pressed **1.5 mm** onto the PCB before vacuum on (bellows). At the **loader
  handover there is no press**: the board sits 0.3 mm above the finger ledges, otherwise the jaws clamp
  on the ledge corners.
- **IK families**: PCB handling uses "wrist up" (wrist_2 > 0), the MG06 fork path "wrist down"
  (wrist_2 < 0); the switch happens only in free space (via `via_mg06_high`, §4.8).
- **MG06 fork path**: the recorded close/open path is tight (bar on the slot bottom). Leaving and
  returning use a separately planned path with the bar centred in the slot (5 mm play), from the closed
  handle angle 28.65° and lid −0.5° (`H_CLOSED`, `LID_CLOSED` in `legacy/plan_cell.py`, git tag `legacy-planner-reference`). The opening path is joined where its recorded handle angle is
  already below the real closed angle, so the handle is never pushed down before opening.
- **MuJoCo 3.16**: `gap` is not applied to `includemargin`, so no `margin`/`gap` on the PCB or the cups
  (parts would float 2–4 mm apart). PCB and cup contacts use `solref 0.002`, `solimp 0.99 0.999`,
  `solmix 100`.
- **Instability**: MuJoCo resets the state on instability; a run detects `d.time` going backwards and
  fails, never loops silently.
- **CAD deviations compensated in `scene.xml`** (design-decisions): loader shifted +100 mm in Y and
  +0.95 mm in X; rear-right holder support (`holder_10`) lowered by 0.8 mm; electrical cabinet 200 mm
  deep instead of 300 mm.

### 4.8 Initial keypoints (extracted from the verified plan; brief §6, verbatim)

In `Configuration/Keypoints.yaml` at full precision: each keypoint's joints are the reference plan's stationary
segment end (joint speed 0; `q_home` for home), rounded to 1e-6 deg; within 9.3e-5 rad of this table, FK
within 0.081 mm of its positions (measured 2026-10-24; `test/Configuration/test_configuration_files.py`). Their source, the planner, is `legacy/plan_cell.py` at git tag
`legacy-planner-reference` (`SEED`, the `solve` calls of its `main`).

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

### 4.9 The recorded MG06 fork path (brief §6, verbatim)

`fork_close_open_path.npz` is now `Data/Paths/fork_close_open_path.npz`, and
`tools/plan_cell.py` became `legacy/plan_cell.py`, removed in F4 and kept in git: the fork-path joins,
the leave / return path around the closed handle and the other special moves are looked up with
`git show legacy-planner-reference:legacy/plan_cell.py` (step 4 builds the `path` command from them).


`path: fork_close` = rows of `fork_close_open_path.npz` with phase `CLOSE` (≈6.1 s, after the first rows that bring the
fork from `fork_start`); `path: fork_open` = rows with phase `OPEN` after the join row (handle ≤ 28.45°). The file stores
`t, lid, press, handle, flange_pos[3], flange_quat[4]` per row; follow it with IK on the flange site (see
`Plan.replay` in `tools/plan_cell.py`). A later task may approximate these paths by a few MoveL keypoints – only if the

### 4.10 Model details (from the previous Czech README of `mg06_cell_mujoco`; verified state)

Lineage: this project continues `mg06_loader_mujoco` and the full `mg06_cell_mujoco`. The scripts that
built the scene (Blender → meshes → `scene.xml`, the loader's `MACHINE_DX` shift, the MG06 lock `GAP`) and
the video scripts are in those projects, not here.

**Robot (AUBO i5).** Base yaw −135° so that every task lies within about ±65° of the pan joint's zero (the
reference plan uses −64.7° … +60.2°). Torque limits 180/180/150/60/60/60 Nm and armature
0.2/0.2/0.15/0.04/0.04/0.04 kg·m² are **estimates** (AUBO publishes neither); gravity compensation and
velocity feed-forward on every joint; the resulting tracking lag is at most 0.013 rad (§4.6). The legacy
planner timed each MoveJ as min-jerk with T = max(0.5 s, maxⱼ 1.875·Δqⱼ / v_maxⱼ, maxⱼ √(5.774·Δqⱼ / a_maxⱼ)),
v_max = 60 % of the datasheet speeds, a_max of §4.4, sampled every 4 ms; where a straight joint move
collided it inserted a via point lifted above the move's midpoint; every robot segment kept ≥ 3 mm to the
scene, the carried PCB included, in the scene state of its time.

**Suction cups (VGC10).** Each cup is its own body on a slide joint (the bellows: 3 mm travel, 3000 N/m,
damping 20 N·s/m) with its own adhesion actuator, so a cup holds only where it touches: a tilted or shifted
pick loses cups as in reality. The cup's collision is a square pad of the cup's area (flat face, clean
normals), friction 2.0, `solref 0.002`. Measured: all 4 cups touch at both picks; the PCB slips at most
1.3 mm in the cups, about 0.4 mm of it contact compliance during transfers, the rest the loader jaws
centring the board as they close. The cups hold the PCB on the top of its component area (in collision the
components are one box).

**PCB.** In collision: the board box (1.8 mm, `condim 4`), two rear corner strips and one component box.
Placed in MG06 it does not bounce (tilt 0.02°, §4.6).

**MG06 mechanism.** A real mechanism, its angles change continuously: the lid has over-centre gas springs
(a spatial tendon with `stiffness` and `springlength`) - below −8° gravity keeps it closed, above it the
springs keep it open; the handle has a toggle spring with its dead point at about 31°, friction 0.6 N·m
and damping; the joint ranges end in stops (`solreflimit`); the press plate is coupled to the handle by a
third-degree polynomial `equality`; the lock is a hook on the handle and a pin in the base with their own
contact bits, radial play 1.5 mm (design-decisions). The robot therefore really pushes the handle: up to
294 N between the fork and the handle bar (§4.6).

**Loader.** Axis ranges `y_axis` −199.5 … +220.5 mm (QF12 S420, towards MG06), `z_axis` −3.5 … +296.5 mm
(QF12 S300, vertical), jaws 0 … 40 mm each (SCHUNK EGI 040-EI, coupled, 40 N per jaw); 0 is the CAD
position. Home y −100 mm, z 0 (the gripper where the CAD has it); jaws open 35 mm at the start (the robot
brings the PCB). Fingers KS0010175 with a ledge and front stop KS0010278, tilted 8.94° like the cassette:
the PCB lies on the ledges by its edges and is clamped over its front ~38 mm. Masses are **estimates**:
Y carriage 4 kg, vertical carriage with the gripper 1.6 kg, PCB 60 g.

**Leaving and returning to the MG06 handle.** The recorded close/open path (MG06 project) leaves with the
bar on the bottom of the fork slot and assumes the handle at exactly 29.3°; with the robot the closed
handle stays at 28.6° and the lid rises 0.5° in the lock's play, and the fork tooth hit the bar. The
replacement path: (1) move the fork so the bar is centred in the slot (5 mm play each side) and 2 mm off
its bottom; (2) out 90 mm along the slot, then 120 mm up; (3) return the same way in reverse; (4) join the
recorded opening path only where its handle angle already points up below the real closed angle, so the
slot wall meets the bar moving up and the handle is never pushed down (it stays at 28.6° until opening
starts). Measured fork-to-bar clearance while leaving and returning: 3.8–4.7 mm.

**Look.** The studio look (F7; the default NVIDIA Isaac Sim / Isaac Lab environment, approximated), applied
in code by `Scene.Core._Apply_Look` (`CONST_LOOK_*`), visual only:

- sky: gradient from the zenith (0.20 0.22 0.25) to the nadir (0.64 0.68 0.73), so the horizon (their mean in
  MuJoCo) is 0.42 0.45 0.49; haze and fog in that horizon colour; `fogstart` 1.5, `fogend` 6 (multiples of
  the extent: 3.9–15.6 m), `haze` 0.6;
- floor: the existing visual-only plane `floor` (z −0.873 m, the workstation feet; contype = conaffinity = 0),
  20 × 20 m, a dark grid (base 0.23 0.24 0.26, lines 0.40 0.42 0.45; 200 px flat texture with an edge mark,
  `texuniform`, `texrepeat` 2 2 = one line per 1 m on the world axes), reflectance 0.05, specular 0.2,
  shininess 0.3;
- lights: headlight ambient 0.35 0.36 0.38, diffuse 0.25, specular 0.05; the key spot light `key_light` at
  (1.6, 1.2, 4.0) m, direction (−0.4, −0.3, −1), cutoff 75°, diffuse 0.65 0.64 0.62, casting shadows (hard:
  MuJoCo's shadow maps have no penumbra); the directional fill `fill_light`, direction (0.5, 0.4, −0.6),
  diffuse 0.18 0.19 0.21, no shadows; `shadowsize` 8192, `offsamples` 8;
- `statistic`: centre (0.178, −0.353, 0.208) m and extent 2.6 m, from the cell's bounding box at the default
  qpos, robot included (half diagonal 1.351 m): the default free camera (distance 1.5 × extent = 3.9 m, fovy
  45°) holds the whole cell with a 10 % margin;
- fog is a render flag, not a model field: on in `Render.Core`'s offscreen images and passive viewer
  (`CONST_LOOK_FLAGS`: fog and haze); the blocking viewer (`show_cell.py`, `python -m mujoco.viewer`) starts
  with fog off - left panel, Rendering > Fog (Haze is on by default).

Materials: aluminium and metal `specular` 0.75, plastic 0.25, dark parts 0.15. Cameras `cam_pcb` (follows the
PCB) and `cam_tool` (follows the tool from the side opposite the robot), both `targetbody`.

## 5. Platform

- macOS 26 on Apple silicon (primary, verified on the development machine, Apple M2 Pro) and Ubuntu
  Linux (unverified: the installer's Linux path is tested only against stubs).
- **Python 3.11**, called explicitly: `/opt/homebrew/bin/python3.11` on macOS, `python3.11` on Linux.
  Prerequisite, presence-checked by the installer: `brew install python@3.11`; Ubuntu `python3.11`
  and `python3.11-venv` (deadsnakes on older releases).
- Direct dependencies pinned `==` in `requirements.txt` (§11, design-decisions): `mujoco==3.16.0`,
  `numpy==2.4.6`, `scipy==1.17.1`, `PyYAML==6.0.3`, `trimesh==5.1.1`, `rtree==1.4.1`,
  `coacd==1.0.14`, `matplotlib==3.11.2`, `pytest==9.1.1`. The groups without a user yet are marked
  *planned* (§14).
- **The verified baseline environment is recorded as a full `pip freeze`** (owner, 2026-10-24): a record of
  what the baseline (§4.6) ran with, not an install input (`install.sh` installs `requirements.txt` only). It
  is replaced, with its date, whenever the baseline is re-measured. Recorded 2026-10-24, Python 3.11.2,
  macOS 26.6.2 arm64 (Apple M2 Pro), the `.venv` of `./install.sh --recreate` in which the baseline was
  re-measured (`.venv/bin/python -m pip freeze --all`):

  ```
  absl-py==2.5.1
  coacd==1.0.14
  contourpy==1.3.3
  cycler==0.12.1
  etils==1.14.0
  fonttools==4.67.0
  fsspec==2026.9.0
  glfw==2.11.0
  iniconfig==2.3.1
  kiwisolver==1.5.1
  matplotlib==3.11.2
  mujoco==3.16.0
  numpy==2.4.6
  packaging==26.3
  pillow==13.0.0
  pip==26.2.2
  pluggy==1.7.0
  Pygments==2.21.0
  PyOpenGL==3.1.10
  pyparsing==3.3.3
  pytest==9.1.1
  python-dateutil==3.0.0
  PyYAML==6.0.3
  rtree==1.4.1
  scipy==1.17.1
  setuptools==65.6.3
  six==1.17.0
  trimesh==5.1.1
  typing_extensions==4.16.0
  tzdata==2026.5
  websockets==17.3
  zipp==4.1.2
  ```
- The viewer: `python -m mujoco.viewer` (blocking `launch`) runs under `python` on macOS;
  `launch_passive` needs `mjpython` on macOS.

## 6. Repository structure

The RPX layout (`FEATURES.md` F3, done 2026-10-24):

```
README.md                 the public page (stub until step 7)
CLAUDE.md                 short: where the internal docs and the skills are, the commands, the language
LICENSE                   MIT, Copyright (c) 2026 JIC Smart Factory (headers: Author Roman Parak)
install.sh                macOS (verified) + Ubuntu (unverified), OS detection, Python 3.11
requirements.txt          pinned
pytest.ini                import mode and the slow marker (root exception)
.gitignore                .venv, caches, .DS_Store, logger/, Data/Generated/, any SKILL.md under .claude/skills/
.claude/settings.json     commit attribution off
.claude/project-docs/     SPEC.md, KNOWN_ISSUES.md, design-decisions.md, FEATURES.md (the tracker)
.claude/skills/<skill>/references/   the project layer of each personal skill (no SKILL.md tracked)
models/scene.xml          the finished scene (workstation, MG06, loader, PCB), from Blender; never regenerated
models/cell_scene.xml     the whole cell in one MJCF for the viewer, written by Example/Scene/export_cell_scene.py
models/mg06/ machine/ workstation/   the scene's meshes (visual + collision)
models/aubo_i5/           vendored robot MJCF + meshes + LICENSE_Manipulator-Mujoco
models/robot_tool/        VGC10 + fork MJCF and meshes
Configuration/Cell.yaml, Configuration/Keypoints.yaml   the cell configuration and the keypoints (§8)
Data/PCB/pcb_meta.json    PCB poses
Data/Paths/fork_close_open_path.npz   recorded flange path that closes / opens MG06
Data/Reference/cell_plan.npz   the verified IK plan, the regression input until step 6
Data/Generated/<Kind>/    generated artefacts (git-ignored): Reports/, Images/; step 4 adds Trajectories/
Media/<Logo>_Black.svg, Media/<Logo>_White.svg   the logos for light / dark grounds, written by Example/Media/make_logo_variants.py
Media/Original/           the owner's logo originals, kept untouched for regeneration
src/Configuration/Core.py the repository paths, the PCB meta reader, the MuJoCo log route
src/Scene/Core.py         the cell builder (MjSpec), its constants, the look, the MJCF export
src/Kinematics/Core.py    site poses, DLS IK for the suction TCP and the flange
src/Collision/Core.py     robot contacts and clearance, contact classification of a run
src/Render/Core.py        offscreen image, viewer start
src/Keypoints/Core.py     the keypoints: posing the cell, the check, the IK resolution (§7.7)
src/Simulation/Core.py    the reference plan reader, the physics run with the instability guard, the report, the acceptance checks
Example/Simulation/run_cycle.py        the physics run of a plan + report + acceptance (exit 0 / 1 / 2)
Example/Scene/export_cell_scene.py     writes models/cell_scene.xml
Example/Scene/model_fingerprint.py     compares (default) or writes the compiled model's fingerprint
Example/Render/show_cell.py            opens the viewer on the cell
Example/Render/render_image.py         offscreen image from a camera into Data/Generated/Images/
Example/Media/make_logo_variants.py    the black / white SVG logos + Data/Generated/Images/logo_check.png
Example/Keypoints/check_keypoints.py   the keypoint check table (exit 0 / 1)
Example/Keypoints/resolve_keypoint.py  IK of a Relative_To keypoint: prints the Joints line to paste
Example/Keypoints/show_keypoint.py     poses the cell at a keypoint: viewer (mjpython), --headless, --image
Example/Keypoints/step_keypoints.py    steps through the keypoints in the viewer (mjpython): Page Down / . / Page Up
test/<Domain>/test_core.py     mirroring src/; test/Example/<Domain>/ mirroring Example/; test/Install/, test/Docs/;
                               reference data (the baselines) in test/<Domain>/data/
logger/                   run logs and MuJoCo's log (git-ignored, created at run time)
```

`Example/` has one folder per `src/` domain its scripts belong to (`Simulation`, `Scene`, `Render`, `Media`,
`Keypoints`; later `Program`). `Data/` is ordered by purpose. The step 3 configuration (cell, keypoints,
program YAML) is in the root folder `Configuration/` (the style skill's `<config folder>/<file>.yaml`, as
in the template's `Configuration/Cell_SF1.yaml`).

`src/` layering: `Configuration`, `Render` import nothing of the project; `Scene` → `Configuration`;
`Kinematics`, `Collision` → `Scene` (and `Configuration`); `Simulation` → `Configuration`, `Scene`, `Collision`;
`Keypoints` → `Configuration`, `Scene`, `Kinematics`, `Collision`. Scripts reach
`src/` through a path built from `__file__`.

## 7. Domains

### 7.1 Configuration

The repository's folders and files as constants built from the module's location, never from the
working directory; the reader of `Data/PCB/pcb_meta.json`; the route of MuJoCo's log file into `logger/`.
From step 3 on, the only reader of the YAML configuration (§8).

### 7.2 Scene

Builds the cell from `models/scene.xml` with MjSpec: the crates' collision as floor + 4 walls, the AUBO
i5 at its base pose, the VGC10/fork tool rigidly on the flange with one body and one adhesion actuator
per cup, the robot servos, the loader drive lag, the look (lights, materials, cameras). Exports the cell
as one MJCF with mesh paths relative to the file. The compiled model is pinned by its fingerprint
(§10.2).

### 7.3 Kinematics

Site poses of a robot configuration, damped-least-squares IK on the suction TCP or the flange (with a
null-space pull to a reference), the TCP pose that holds the PCB at a given pose.

### 7.4 Collision

Robot contacts of a configuration (penetration below a margin), the clearance between robot geoms and
everything else (bounding-sphere filtered `mj_geomDistance`), planning margin 3 mm (6 mm geom margin on
robot and PCB geoms of the planning model). The classification of a run's contacts into the checked
categories of §4.6.

### 7.5 Render

An offscreen image from a named camera or a free camera, with the studio look's fog and haze flags on;
starting the viewer (the passive one with the same flags). No project imports.

### 7.6 Simulation

Reads a plan in the legacy planner's format (`Data/Reference/cell_plan.npz`: dt, robot joint trajectory, loader moves, vacuum events, phase
labels); runs it with physics (servo set-point with velocity feed-forward, min-jerk loader profiles,
vacuum events); fails with `Simulation_Error_Cls` when MuJoCo resets the state; returns the measured
values; formats the report of §4.6; checks §10.1.

### 7.7 Keypoints

The keypoints at work (`src/Keypoints/Core.py`): the target of a `Relative_To`, the MG06 state of a `Fixture`
(the press plate on the model's coupling), posing the whole cell at a keypoint (robot, bellows at rest,
loader axes, jaws, PCB at its location, MG06), the keypoint check and the IK resolution of a keypoint's joints
(printed, never written). Playback and posing read `Joints` only.

The check (per keypoint, one table row; FAIL on any rule, WARN when wrist_2 is within the family margin):
joint limits (the model); the family (wrist_2's sign); `Check` within `Clearance.Check_Position`; a
`Relative_To` target within `Stale_Position` / `Stale_Angle`; the clearance - **exact** distances
(`Collision.Close_Pairs`, `mj_geomDistance`) from the robot's geoms, and from the PCB's when the cups carry it,
to every collidable geom: every pair closer than `Clearance.Robot` fails unless its group is in
`Allow_Contact` and no deeper than `Allowed_Depth`. Groups: `cups_pcb` (a cup or the cup block on the PCB, not
the fork), `handle_bar` (the tool on the MG06 handle), `pcb_crate` (the carried PCB in a crate) - only groups a
keypoint observes; any other pair within the clearance, the carried PCB on the jaws included, fails. Not checked:
the robot's mount (its base on the pedestal plate, distance 0 by construction; a code constant, as the
template's base–stand exclusion). Exact distances because MuJoCo's box–box collider generates no contact for a
near pair (the carried PCB 1.0 mm above the crate floor), whatever the geom margin
(`test/Collision/test_core.py`).

## 8. Configuration — `Configuration/Cell.yaml`, `Configuration/Keypoints.yaml`

Both files are read only by `src/Configuration/Core.py` (`Load_Cell_Configuration`, `Load_Keypoints`): every
field REQUIRED (no optional fields; `Relative_To` may be `null`, `Allow_Contact` an empty list), every problem
collected and reported at once with its file and key path, unknown keys returned as WARNING notes that the
scripts print, angles in degrees converted to radians once. Tested in `test/Configuration/`.

| `Cell.yaml` field | Unit | Source | Read by |
|---|---|---|---|
| `Robot.Base_Position`, `Robot.Base_Yaw` | m, deg | CAD, §4.2 | `Scene` |
| `Vacuum.Pressure`, `Vacuum.Cup_Diameter` (force derived) | Pa, m | VGC10 datasheet, §4.3 | `Scene` |
| `MG06.Closed_Lid`, `MG06.Closed_Handle` | deg | §4.7 (`LID_CLOSED`, `H_CLOSED`) | posing, the keypoint check |
| `Clearance.Robot` (planning margin = 2 x, derived), `Allowed_Depth.<group>`, `Check_Position`, `Stale_Position`, `Stale_Angle`, `Family_Margin` | m, deg | §2, the owner (0.5 mm), measured (F6) | `Collision`, the keypoint check |
| `Acceptance.*` | m, deg | §10.1 | `Simulation.Check_Acceptance` |
| `Simulation.Phase_Steps`, `Simulation.Cup_Touch` | -, m | the legacy run (F3.5) | `Simulation`, `Collision` |

Not configured, on purpose: joint limits, loader axis ranges, the MG06 lid's open stop (the model holds them:
the vendored robot MJCF, `models/scene.xml`; read from the compiled model); joint speeds and accelerations
(step 4, with the trajectory, owner 2026-10-24).

`Keypoints.yaml`: `Robot.<name>` (`Joints` [deg] - the source of truth; `Family`; `Check` - a site and the
position its FK must reach; `Relative_To` - null or a target `pcb_hold@<location>` / a keypoint with a frame
and an offset [m]; `Fixture` - `Open`, `Closed` or the recorded `{Lid, Handle, Source}` of the fork path;
`Context` - `Loader`, `Gripper`, `PCB` location; `Allow_Contact` - contact groups; `Note`), `Loader.<position>`
(`Y`, `Z` [m]: home, travel_place, place, park - the only source of the loader positions, moved from
`pcb_meta.json`), `Gripper.<state>` [m] (open, grip, clamp). The 22 keypoints are §4.8 at full precision.

## 9. Data formats and units

| Quantity | Canonical unit in code | Where another unit appears |
|---|---|---|
| length | [m] | the report prints mm |
| angle | [rad] | the report prints deg; the keypoint file (step 3) will carry deg, converted at load |
| time | [s] | - |
| force | [N] | - |

| Format | Read and written by |
|---|---|
| `models/scene.xml` and the vendored / tool MJCFs | `Scene` (MjSpec), never written |
| `models/cell_scene.xml` | written by `Scene` (`Export_Cell_Scene`); read by MuJoCo's viewer only |
| `Data/PCB/pcb_meta.json` | `Configuration` (`Read_PCB_Meta`), written by hand: the PCB poses (`cad_pos`, `place_pos`, `hold_pos`, `quat`) only |
| `Configuration/Cell.yaml`, `Configuration/Keypoints.yaml` | `Configuration` (`Load_Cell_Configuration`, `Load_Keypoints`), written by hand (§8) |
| `Data/Paths/fork_close_open_path.npz` | no reader now (the legacy planner, git tag `legacy-planner-reference`, read it); step 4 adds one in `src/` |
| `Data/Reference/cell_plan.npz` | `Simulation` (`Read_Legacy_Plan`); written by the legacy planner (git tag `legacy-planner-reference`); never written now |
| the report (text) | `Simulation` (`Format_Report`), printed by `Example/Simulation/run_cycle.py`, copied to `--out`/`report.txt` (default `Data/Generated/Reports/`) |
| the logo variants `Media/<Logo>_<Black|White>.svg` | written by `Example/Media/make_logo_variants.py` from `Media/Original/`; read by the README (and the Phase 2 app) |
| the model fingerprint (JSON) | `Example/Scene/model_fingerprint.py` (`Model_Fingerprint`, `Compare_Fingerprints`) |
| `logger/<script>.log`, `logger/MUJOCO_LOG.TXT` | written by `Configuration` (`Script_Log_Cls`, `Configure_MuJoCo_Log`) and MuJoCo; never read |
| the trajectory file (step 4) | to be specified in step 4: `t`, `robot_q[6]`, `robot_qd[6]`, `loader_y`, `loader_z`, `gripper`, `vacuum`, `step_index`, `step_label`, sampled at 4 ms |

Scripts that write take `--out DIR`.

## 10. Tests

Layout per §6; `pytest.ini` with `--import-mode=importlib` and the `slow` marker. Everyday
`.venv/bin/python -m pytest -m "not slow"` (99 tests, about 15 s), full `.venv/bin/python -m pytest`
before a commit (103 tests, about 45 s; Apple M2 Pro, 2026-10-24).
Expected numbers come from §4 (measured) or the brief's criteria, never from the code under test.

### 10.1 Acceptance criteria (brief §5.6)

A cycle run passes when:

1. no robot / loader collision: the five contact categories of §4.6 report no contact;
2. 4 of 4 cups touch the PCB at both picks;
3. PCB in MG06: position error < 0.5 mm and tilt < 0.2°;
4. PCB in the output crate: < 2 mm from the crate centre, horizontally;
5. MG06 ends open: lid within 2° of its open stop (−39.96°) and handle within 2° of 0° (this reading
   of "ends open" is a decision, design-decisions);
6. no instability;
7. deepest PCB penetration ≤ 0.5 mm in every group (MG06 holder, crate, loader jaws).

Further Phase 1 criteria: every keypoint can be shown in the viewer, and an invalid keypoint (IK fails,
clearance < 3 mm) is reported clearly; the PCB is green; the README explains installation, the
commands, editing keypoints and program, the file formats.

### 10.2 Model and report equivalence

- The compiled model's fingerprint equals `test/Scene/data/baseline_model_fingerprint.json` except
  `geoms/pcb_vis/geom_rgba` (the green PCB).
- The cycle run's printed report diffs empty against `test/Simulation/data/baseline_report.txt`.
- Pinned by `test/Scene/test_core.py` (fingerprint), `test/Simulation/test_core.py` (slow: report, §10.1, raw values)
  and `test/Example/Simulation/test_run_cycle.py` (slow: the script's stdout).

## 11. Installer — `install.sh`

Steps, cheapest check first (project-style-rpx references/shell.md): detect the OS and architecture;
find Python 3.11 among explicit candidates only (macOS `/opt/homebrew/bin/python3.11`,
`/usr/local/bin/python3.11`, `python3.11` on PATH; Linux `python3.11` on PATH, `/usr/bin/python3.11`;
never a bare `python3`) or fail with the install command; create `.venv` if absent, reuse a working one
(a broken one is reported, deleted only with `--recreate`); upgrade pip and install `requirements.txt`;
verify the install (import mujoco, numpy, scipy, yaml and print the versions; mujoco must be 3.16.0);
create `logger/` and `Data/Generated/`; print the next commands. `--detect-only` stops after the plan.
Re-runnable, `set -euo pipefail`, no sudo, no global installs. Verified with `--recreate` on macOS
26.6.2, Apple M2 Pro (2026-10-24). Ubuntu path: written, tested only against stubs, unverified. Pinned
by `test/Install/test_install.py`.

## 12. Skills

The personal skills project-style-rpx, feature-manager, project-optimize and readme-writer, each with
this project's reference in `.claude/skills/<skill>/references/project.md` (project-style-rpx also
`check_style.json` and `mjcf.md`). The repository holds only those files; a `SKILL.md` under
`.claude/skills/` is git-ignored.

## 13. Risks and how they are handled

| # | Risk | Handling |
|---|---|---|
| 1 | A restructure silently changes the model | the fingerprint of the compiled model (§10.2) |
| 2 | A restructure silently changes the physics result | the report diffs empty against the baseline (§10.2); the slow regression (§10.1) |
| 3 | Instability hidden by MuJoCo's state reset | `d.time` going backwards fails the run (exit 2) |
| 4 | Platform drift of the numbers (Python / numpy version) | pinned requirements; re-baseline in the pinned environment (`KNOWN_ISSUES.md`) |
| 5 | The ported physics loop drifts from the verified legacy run | the report of `Data/Reference/cell_plan.npz` diffs empty against the baseline and its raw values are pinned (`test/Simulation`); the legacy run is in git tag `legacy-planner-reference` |

## 14. Open questions

1. The real-cell validation procedure (§17, `KNOWN_ISSUES.md`): not defined yet.

Answered 2026-10-24 (owner): simulation now, real-cell transfer planned (§17); scipy, trimesh, rtree,
coacd stay pinned as *planned*; the F3 decisions (`FEATURES.md`).

## 15. Sources

- Owner's project brief (every section mapped in `FEATURES.md` > Brief coverage) "KINALI Test-it-off – Automated PCBA Testing Cell (MuJoCo)", git
  `427e076:docs/PROJECT_BRIEF.md` (2026-10-24).
- The previous Czech description of the cell (git `legacy-planner-reference:docs/README_cs_mg06_cell.md`,
  originally `5e5ee62:README.md`); its content is in §4.10, KNOWN_ISSUES and design-decisions (F4.2).
- AUBO i5 datasheet (joint speeds, payload 5 kg, reach 886.5 mm), as quoted in `src/Scene/Core.py`.
- OnRobot VGC10 datasheet (20–60 % vacuum recommended), as quoted in `src/Scene/Core.py`.
- KINALI Test-it-off: https://www.testitoff.cz/en/about-test-it-off/

## 16. Proposed file formats and commands (brief §5.2–§5.5, verbatim)

The owner's proposals for steps 3–4, kept as the starting point; step 3 and 4 settle them (this
section is then replaced by the formats in §8 and §9). Quoted verbatim, so they keep the brief's paths: its
output folder is now `Data/Generated/` (design-decisions), the trajectory `Data/Generated/Trajectories/`. The CLI of §16.4 is superseded by the
`Example/` scripts (design-decisions: RPX over brief §5.1); its subcommands map to `run_cycle.py`
(`run`), `export_cell_scene.py` (`export-xml`), and the scripts of steps 3–5 (`show`, `keypoints --check`,
`build`, `play`).

### 16.1 Keypoint file (proposal)

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

### 16.2 Program file (proposal)

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

### 16.3 Trajectory output

`output/trajectory.npz` (+ optional CSV): `t`, `robot_q[6]`, `robot_qd[6]`, `loader_y`, `loader_z`, `gripper`,
`vacuum`, `step_index`, `step_label`, sampled at 4 ms (same as now). `simulate.py` plays it with physics and produces the
report of §3. The trajectory alone must be enough to reproduce the whole process.

### 16.4 CLI (proposal, superseded)

```
kinali-sim show home                 # viewer, cell posed at keypoint "home"
kinali-sim show --step 5             # cell at the end of program step 5
kinali-sim keypoints --check         # IK + collision clearance for all keypoints
kinali-sim build                     # program → output/trajectory.npz (+ checks of every segment)
kinali-sim play                      # viewer, physics playback of output/trajectory.npz
kinali-sim run                       # headless physics + report (regression)
kinali-sim export-xml                # models/cell_scene.xml
```

## 17. Future scope: real-cell transfer

The project is simulation only today. Keypoints and programs are intended to be transferred to the real
AUBO i5 later (owner, 2026-10-24); a test on the real cell is planned. Consequences:

1. **Controller-friendly formats.** The keypoint, program and trajectory formats (steps 3–4) state
   joint values in a defined unit and order (rad in code, the file's unit declared; order shoulder pan,
   shoulder lift, elbow, wrist 1, wrist 2, wrist 3), the TCP definition they refer to (`suction_tcp`:
   the cup face, 260.6 mm from the flange along tool +Z; the flange; §4.3), and speeds as a percentage of
   the AUBO i5 datasheet joint speeds (§4.4).
2. **Simulation-only deviations are checked against the real cell before any transfer**: the CAD shifts
   compensated in `scene.xml` (§4.7: loader +100 mm Y and +0.95 mm X, `holder_10` −0.8 mm, cabinet
   200 mm), the assumed acceleration limits (§4.4), the adhesion model of the vacuum (§4.3).
3. **Outputs are not validated for the real cell.** Before any use on real hardware they go through the
   real-cell validation procedure (to be defined, §14, `KNOWN_ISSUES.md`), at reduced speed and with the
   robot controller's own safety limits active.

