# KINALI_Test_it_off_Simulation — Specification

Status: Draft for the owner's approval

A number marked *measured* names what it was measured on; where a test pins it, the test is named.
This specification is derived from the owner's project brief (git `427e076:docs/PROJECT_BRIEF.md`,
removed from the tree when this SPEC replaced it; §15) and from the verified state of the code. §6
describes the layout **after** the RPX migration (`FEATURES.md` F3); until F3 is approved and done, the
layout in git is the one recorded there under *Current*.

## 1. Goal

Simulate the robotic workcell **"Test it off"** by KINALI (automated PCBA testing for electronics
manufacturers, https://www.testitoff.cz/en/about-test-it-off/) in MuJoCo, so that a robot program for
the whole process - PCB from the input crate, through the MG06 tester, to the output crate - can be
written, checked and played back before it is run anywhere else.

The physics and the realism of the cell are finished and verified (§4.6). The robot motion is today
produced by an IK-based planner (`legacy/plan_cell.py`). **Phase 1 replaces it with a keypoint-based
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
6. **Regression**: the default program reproduces §4.6 within §10.1; then `legacy/` is removed.
7. **Docs**: the README (readme-writer) and `CLAUDE.md`.

Phase 2 (later): a PyQt6 window rendering MuJoCo through OpenGL (`QOpenGLWidget` + `MjrContext`, or an
offscreen `mujoco.Renderer` into a `QImage` as a fallback), with the keypoint list and editing (jog
joints / TCP, IK family, save), the program list (run from a step, step by step), play / pause / step,
a timeline, camera selection (`free`, `cam_pcb`, `cam_tool`) and the check results. The Phase 1 core
has no GUI dependency, so the Qt application reuses it unchanged.

## 3. Out of scope

- Regenerating `scene.xml` from Blender: the scene is finished (§4.7).
- Changing the verified physics: timestep, solver options, any solref/solimp/solmix/margin/gap,
  friction, masses, geometry, actuator gains and ranges, the vacuum/adhesion setup, the bellows springs,
  the CAD deviations (§4.7). A change to any of these is a feature with its own report.
- Path planning (search): the program lists the path; the tool checks it.
- Blending and parallel robot/loader execution in Phase 1 (possible later).
- Driving a real robot or machine; whether the program will ever be exported to a real AUBO is an open
  question (§14).

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

### 4.3 Compiled model (*measured*, MuJoCo 3.16.0, 2026-10-24; `docs/baseline_model_fingerprint.json`)

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
  y −0.1995 / z +0.020; jaws open 0.035, grip 0.01775 (clamp command 0.00575). Source:
  `Data/pcb_meta.json`, `legacy/plan_cell.py`.
- PCB in the loader jaws (loader home): position (0.0057, −0.30829, 0.14961), quaternion
  (0.99696, 0.07794, 0, 0) (8.94° tilt); placed in MG06: (0.0057, −0.011636, 0.216826), same quaternion.
- PCB in a crate: flat, board centre at the crate centre, z = 0.004 + 0.0009. Input crate centre
  (0.15720, −0.75160), output crate centre (0.40620, −0.75160).
- Suction TCP in the PCB body frame: (−0.00125, 0.0046, 0.0075); tool axes diag(1, −1, −1).
- Robot speed: AUBO i5 datasheet 220°/s (J1–J4), 230°/s (J5–J6); the plan uses 60 %. Acceleration
  limits 3 rad/s² (J1–J3), 6 rad/s² (J4–J6) are an **assumption**, not in the datasheet. Free MoveL
  200–250 mm/s, the last 15 mm before contact 30 mm/s.
- Initial keypoints (joints in rad, TCP / flange in m): the brief's table §6, extracted from the
  verified plan; seeded into the keypoint file in step 3.

### 4.5 Process of the reference cycle (65.8 s)

1. The robot picks the PCB with vacuum from the left (input) crate.
2. It puts it into the open loader jaws (loader home); the loader closes, vacuum off, the robot leaves.
3. Loader: up → towards MG06 → down into the holder → open → up → park (y −199.5 mm, z +20 mm).
4. The robot closes MG06 with the fork (lid down, handle to about 29°), leaves, comes back, opens MG06.
5. Loader: back → down → close (pick) → up → home.
6. The robot takes the PCB (vacuum on, loader opens), puts it into the right (output) crate, goes home.

### 4.6 Reference result (*measured*, `docs/baseline_report.txt`)

The legacy plan `legacy/cell_plan.npz` run with physics, 2026-10-24, MuJoCo 3.16.0, Python 3.14.7,
numpy 2.5.4, macOS 26.6.2 on an Apple M2 Pro (re-baseline in the pinned Python 3.11 environment
pending, `KNOWN_ISSUES.md`):

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

Run time 15–16 s on that machine. The brief's original reference differs in one value (crate
penetration −0.08 mm, `KNOWN_ISSUES.md`). The legacy planner regenerates `cell_plan.npz` to within
1.2e-14 rad (2026-10-24).

### 4.7 Lessons learned and deliberate deviations (do not break)

- **Vacuum**: the cups are pressed **1.5 mm** onto the PCB before vacuum on (bellows). At the **loader
  handover there is no press**: the board sits 0.3 mm above the finger ledges, otherwise the jaws clamp
  on the ledge corners.
- **IK families**: PCB handling uses "wrist up" (wrist_2 > 0), the MG06 fork path "wrist down"
  (wrist_2 < 0); the switch happens only in free space (via `via_mg06_high`).
- **MG06 fork path**: the recorded close/open path is tight (bar on the slot bottom). Leaving and
  returning use a separately planned path with the bar centred in the slot (5 mm play), from the closed
  handle angle 28.65° and lid −0.5°. The opening path is joined where its recorded handle angle is
  already below the real closed angle, so the handle is never pushed down before opening.
- **MuJoCo 3.16**: `gap` is not applied to `includemargin`, so no `margin`/`gap` on the PCB or the cups
  (parts would float 2–4 mm apart). PCB and cup contacts use `solref 0.002`, `solimp 0.99 0.999`,
  `solmix 100`.
- **Instability**: MuJoCo resets the state on instability; a run detects `d.time` going backwards and
  fails, never loops silently.
- **CAD deviations compensated in `scene.xml`** (design-decisions): loader shifted +100 mm in Y and
  +0.95 mm in X; rear-right holder support (`holder_10`) lowered by 0.8 mm; electrical cabinet 200 mm
  deep instead of 300 mm.

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
- The viewer: `python -m mujoco.viewer` (blocking `launch`) runs under `python` on macOS;
  `launch_passive` needs `mjpython` on macOS.

## 6. Repository structure

Target of `FEATURES.md` F3 (the RPX migration, pending approval):

```
README.md                 the public page (stub until step 7)
CLAUDE.md                 short: where the internal docs and the skills are, the commands, the language
LICENSE                   MIT, Copyright (c) 2026 Roman Parak
install.sh                macOS (verified) + Ubuntu (unverified), OS detection, Python 3.11
requirements.txt          pinned
pytest.ini                import mode and the slow marker (root exception)
.gitignore                .venv, caches, .DS_Store, logger/, output/, any SKILL.md under .claude/skills/
.claude/settings.json     commit attribution off
.claude/project-docs/     SPEC.md, KNOWN_ISSUES.md, design-decisions.md, FEATURES.md (the tracker)
.claude/skills/<skill>/references/   the project layer of each personal skill (no SKILL.md tracked)
models/scene.xml          the finished scene (workstation, MG06, loader, PCB), from Blender; never regenerated
models/cell_scene.xml     the whole cell in one MJCF for the viewer, written by Example/export_cell_scene.py
models/mg06/ machine/ workstation/   the scene's meshes (visual + collision)
models/aubo_i5/           vendored robot MJCF + meshes + LICENSE_Manipulator-Mujoco
models/robot_tool/        VGC10 + fork MJCF and meshes
Data/pcb_meta.json        PCB poses and loader axis positions
Data/fork_close_open_path.npz   recorded flange path that closes / opens MG06
src/Configuration/Core.py the repository paths, the PCB meta reader, the MuJoCo log route
src/Scene/Core.py         the cell builder (MjSpec), its constants, the look, the MJCF export
src/Kinematics/Core.py    site poses, DLS IK for the suction TCP and the flange
src/Collision/Core.py     robot contacts and clearance, contact classification of a run
src/Render/Core.py        offscreen image, viewer start
src/Simulation/Core.py    the legacy plan reader, the physics run with the instability guard, the report, the acceptance checks
Example/run_cycle.py      the physics run of a plan + report + acceptance (exit 0 / 1 / 2)
Example/export_cell_scene.py   writes models/cell_scene.xml
Example/show_cell.py      opens the viewer on the cell
Example/render_image.py   offscreen image from a camera into output/
Example/model_fingerprint.py   writes or compares the compiled model's fingerprint
legacy/                   plan_cell.py, test_cell_cycle.py, cell_plan.npz, and the shims cell.py, robot_kin.py (until step 6)
docs/README_cs_mg06_cell.md    the previous Czech README (until step 7; root exception)
test/<Domain>/test_core.py     mirroring src/; test/Example/, test/Install/, test/Docs/; reference data in test/<Domain>/data/
logger/                   run logs and MuJoCo's log (git-ignored, created at run time)
output/                   generated reports and images (git-ignored)
```

`src/` layering: `Configuration`, `Render` import nothing of the project; `Scene` → `Configuration`;
`Kinematics`, `Collision` → `Scene`; `Simulation` → `Configuration`, `Scene`, `Collision`. Scripts reach
`src/` through a path built from `__file__`.

*Current* (until F3): `src/kinali_sim/` (package), `models/` (MJCFs), `assets/` (meshes), `data/`,
`config/`, `output/`, `legacy/`, `tests/`, `tools/`, `docs/`, `pyproject.toml`.

## 7. Domains

### 7.1 Configuration

The repository's folders and files as constants built from the module's location, never from the
working directory; the reader of `Data/pcb_meta.json`; the route of MuJoCo's log file into `logger/`.
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

An offscreen image from a named camera; starting the viewer. No project imports.

### 7.6 Simulation

Reads a legacy plan (`cell_plan.npz`: dt, robot joint trajectory, loader moves, vacuum events, phase
labels); runs it with physics (servo set-point with velocity feed-forward, min-jerk loader profiles,
vacuum events); fails with `Simulation_Error_Cls` when MuJoCo resets the state; returns the measured
values; formats the report of §4.6; checks §10.1.

## 8. Configuration

Not applicable yet: no configuration file exists. Step 3 adds `Configuration/` with the cell and
keypoint YAML, read only by `src/Configuration/Core.py`, every field REQUIRED with unit and source
(project-style-rpx references/yaml.md). Until then the cell's values are code constants with their
sources in `src/Scene/Core.py`.

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
| `Data/pcb_meta.json` | `Configuration` (`Read_PCB_Meta`), written by hand |
| `Data/fork_close_open_path.npz` | `legacy/plan_cell.py` only, until step 4 moves its reader into `src/` |
| `legacy/cell_plan.npz` | `Simulation` (`Read_Legacy_Plan`); written by `legacy/plan_cell.py`; the frozen `legacy/test_cell_cycle.py` keeps its own read until step 6 |
| the report (text) | `Simulation` (`Format_Report`), printed by `Example/run_cycle.py`, copied to `output/report.txt` |
| the model fingerprint (JSON) | `Example/model_fingerprint.py` |
| the trajectory file (step 4) | to be specified in step 4: `t`, `robot_q[6]`, `robot_qd[6]`, `loader_y`, `loader_z`, `gripper`, `vacuum`, `step_index`, `step_label`, sampled at 4 ms |

Scripts that write take `--out DIR`.

## 10. Tests

Layout per §6; `pytest.ini` with `--import-mode=importlib` and the `slow` marker. Everyday
`.venv/bin/python -m pytest -m "not slow"`, full `.venv/bin/python -m pytest` before a commit.
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

- The compiled model's fingerprint equals `docs/baseline_model_fingerprint.json` except
  `geoms/pcb_vis/geom_rgba` (the green PCB).
- The cycle run's printed report diffs empty against `docs/baseline_report.txt`.

## 11. Installer — `install.sh`

Steps, cheapest check first (project-style-rpx references/shell.md; the details are in `FEATURES.md`
F3 until implemented): detect the OS and architecture; find Python 3.11 or fail with the install
command; create `.venv` if absent (a broken one is reported); upgrade pip and install
`requirements.txt`; verify the install (import mujoco, numpy, scipy, yaml; mujoco is 3.16.0); create
`logger/` and `output/`; print the next commands. `--detect-only` stops after the plan. Re-runnable,
no sudo, no global installs. Ubuntu path: written, tested only against stubs, unverified.

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
| 5 | Two copies of the physics loop (frozen legacy and `Simulation`) drift | both run on the same plan and must print the same report until step 6 removes `legacy/` |

## 14. Open questions

1. Does any output of this project ever drive the real AUBO or the loader (a program export)? If so,
   the safety sentence of `KNOWN_ISSUES.md` becomes a rule with safe first-run conditions.
2. The requirement groups without a user yet (scipy, trimesh, rtree, coacd; matplotlib until the
   image script uses it): keep them pinned as *planned*, or drop them until a feature needs them.
3. The items of `FEATURES.md` F3 *Decisions needed*.

## 15. Sources

- Owner's project brief "KINALI Test it off – Automated PCBA Testing Cell (MuJoCo)", git
  `427e076:docs/PROJECT_BRIEF.md` (2026-10-24).
- The previous Czech description of the cell, `docs/README_cs_mg06_cell.md` (git `5e5ee62:README.md`).
- AUBO i5 datasheet (joint speeds, payload 5 kg, reach 886.5 mm), as quoted in `src/kinali_sim/model.py`.
- OnRobot VGC10 datasheet (20–60 % vacuum recommended), as quoted in `src/kinali_sim/model.py`.
- KINALI "Test it off": https://www.testitoff.cz/en/about-test-it-off/
