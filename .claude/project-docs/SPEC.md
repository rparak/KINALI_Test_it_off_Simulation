# Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo — Specification

Repository: `KINALI_Test_it_off_Simulation`.

Status: Approved 2026-10-24

A number marked *measured* names what it was measured on; where a test pins it, the test is named.
This specification is derived from the owner's project brief (git `427e076:docs/PROJECT_BRIEF.md`; §15) and from the
verified state of the code.

## 1. Goal

**Project context** (owner, 2026-10-24): An internal research project of JIC Smart Factory, part of its internal research supporting small and medium-sized enterprises in advanced simulation. Developed in collaboration with KINALI for the development of the Test-it-off system, an automated workcell for testing assembled printed circuit boards (PCBA). The project has no external or public
funding. Brand: JIC Smart Factory, in collaboration with KINALI, product Test-it-off, referenced by name only (design-decisions).

**Test-it-off Simulation** (design-decisions: the name) simulates the robotic workcell **Test-it-off** by KINALI (automated PCBA testing for electronics
manufacturers, https://www.testitoff.cz/en/about-test-it-off/) in MuJoCo, so that a robot program for
the whole process - PCB from the input crate, through the MG06 tester, to the output crate - can be
written, checked and played back before it is run anywhere else.

The physics and the realism of the cell are finished and verified (§4.7, §10.2). The robot motion is a keypoint-based
robot program (named poses in configuration files, a program read top to bottom, a trajectory file, the MuJoCo viewer),
in two versions - a taught program and an optimized one (§2 item 4) - built into trajectories (§9) and run with physics
(§7.6).

## 2. Scope

Scope (the brief's steps, §7, in this order):

1. **Layout**: the RPX layout (§6) with every path resolved from the code's location.
2. **Green PCB**: the visual geom `pcb_vis` in solder-mask green `0.05 0.42 0.18 1`. Optional follow-up: split the
   PCB mesh into board and components (`KNOWN_ISSUES.md`).
3. **Configuration and keypoints**: a YAML configuration (§8) with named keypoints - robot (joints, or
   TCP pose + IK family/seed, or relative to another keypoint), loader axes, gripper, vacuum - resolved
   to joints once so that playback is deterministic; a viewer that poses the whole cell at one
   keypoint; a check of every keypoint (IK, collision clearance ≥ 3 mm).
4. **Program and trajectory**: a program file read top to bottom with the commands `movej`, `movel`,
   `path` (recorded flange path), `loader` (y / z), `gripper` (open / clamp), `vacuum` (on / off),
   `wait`; min-jerk MoveJ with speed and acceleration limits, Cartesian MoveL (IK per sample), path
   following; a trajectory file (§9); a clearance check of every segment, carried PCB included. Two
   versions of the same process (owner, 2026-10-24; design-decisions: *two programs*): the **default
   program** (`FEATURES.md` F8) is taught as a human operator would - no planning algorithm, collision
   freedom is checked, not searched, steps are sequential, no blending, the robot and the loader never move
   at the same time; the **optimized program** (F9) is planned - the free-space transfers between the
   process-critical (locked) segments are searched, smoothed and time-optimally timed, and the robot and the
   loader move in parallel under interlocks (§7.11). Both versions pass the same checks, kinematic (§7.9) and physical
   (§10.1).
5. **Simulation, checks, playback**: the physics run of a trajectory with its report (§4.6, §7.6), a real-time viewer
   playback, a headless run.
6. **Regression**: the two programs' physics baselines (§10.2).
7. **Docs**: the README (readme-writer) and `CLAUDE.md`.

## 3. Out of scope

- Regenerating `scene.xml` from Blender: the scene is finished (§4.7).
- Changing the verified physics: timestep, solver options, any solref/solimp/solmix/margin/gap,
  friction, masses, geometry, actuator gains and ranges, the vacuum/adhesion setup, the bellows springs,
  the CAD deviations (§4.7). A change to any of these is a feature with its own report.
- Path planning (search) for the default program: it lists the path; the tool checks it. The optimized
  program (§2 item 4) searches only its free-space transfers.
- Blending (possible later); parallel robot/loader execution in the default program (the optimized program has it,
  under interlocks).
- Driving a real robot or machine now. A transfer to the real cell is planned later (§17).
- A GUI application (owner, 2026-10-25: no PyQt GUI, design-decisions). The interface is the scripts of `Example/`, the
  MuJoCo viewer (`show_program.py`, `show_physics.py`, the keypoint viewers) and the offscreen renders.

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
impratio 10, noslip iterations 5, solver Newton, integrator Euler (the scene's; `KNOWN_ISSUES.md`).

| qpos | joint |
|---|---|
| 0 / 1 / 2 | `lid_hinge` (0 closed, −0.6974 open), `press_slide`, `handle_hinge` (0 up, about 0.50 = 29° closed) |
| 3 / 4 | loader `y_axis` (−0.1995 … 0.2205), `z_axis` (−0.0035 … 0.2965) |
| 5 / 6 | `jaw_L`, `jaw_R` (0 … 0.040, synchronized by a stiff equality: solref 0.001, solimp 0.99 / 0.999 - the EGI's toothed belt) |
| 7–13 | `pcb_free` (position + quaternion) |
| 14–19 | `aubo/shoulder_pan_joint` … `aubo/wrist_3_joint` (limits ±3.04) |
| 20–23 | `aubo/tool/cup0..3_bellows` (passive springs of the suction cups) |

| ctrl | name | meaning |
|---|---|---|
| 0, 1 | `y_servo`, `z_servo` | loader axis position [m]; first-order lag τ = 30 ms (`filterexact`; its `act` state starts at the start position) |
| 2 | `gripper` | jaw target [m], force-limited to 70 N, the gripping force as SCHUNK states it (the sum of both jaws: 35 N per jaw; EGI 040-EI 25-70 N); clamp = 11.79 mm behind the part (`jaw_grip − 0.012`), open = 0.035 |
| 3–8 | `robot_*` | joint position servos, kp 5000/5000/4000/2000/2000/1500, kv 150/150/100/25/25/15, armature, gravity compensation; driven with velocity feed-forward `ctrl = q_ref + (kv/kp)·v_ref` |
| 9–12 | `vacuum0..3` | adhesion on the 4 cups, ctrl 0/1, 7 N each (40 kPa × D15); holds only where a cup touches |

Sites: `aubo/tool/suction_tcp` (cup face, 260.6 mm from the flange, tool +Z = cup direction),
`aubo/tool/tool_flange`, `aubo/tool/fork_center`, `handle_grasp`. Cameras `cam_pcb`, `cam_tool`
(`targetbody`).

### 4.4 Key numbers

- Loader: home y −0.1000 / z 0; place y +0.19666 / z +0.06759 (the PCB released 0.15 mm above its seat); fetch z +0.06725
  (the re-grip); travel height z +0.07922; park y −0.1995 / z +0.020; jaws open 0.035, grip 0.01754 (the physics grip with
  the synchronized jaws; clamp command 0.00575). Source: `Configuration/Constants.yaml` (`Loader_Positions`,
  `Gripper_Commands`) and `Simulation.yaml Jaw_Actual`.
- PCB in the loader jaws (loader home): position (0.0057, −0.30829, 0.14961), quaternion
  (0.99696, 0.07794, 0, 0) (8.94° tilt); placed in MG06: (0.0057, −0.011636, 0.216826), same quaternion.
- PCB in a crate: flat, board centre at the crate centre, z = 0.004 + 0.0009. Input crate centre
  (0.15720, −0.75160), output crate centre (0.40620, −0.75160).
- Suction TCP in the PCB body frame: (−0.00125, 0.0046, 0.0075); tool axes diag(1, −1, −1).
- Robot speed: AUBO i5 datasheet 220°/s (J1–J4), 230°/s (J5–J6); MoveJ runs at 60 % (`Motion.Speed_Fraction`). Acceleration
  limits 3 rad/s² (J1–J3), 6 rad/s² (J4–J6) are an **assumption**, not in the datasheet. Free MoveL
  200–250 mm/s, the last 15 mm before contact 30 mm/s.
- Initial keypoints: §4.8; the recorded fork path: §4.9.

### 4.5 The process

1. The robot picks the PCB with vacuum from the left (input) crate.
2. It puts it into the open loader jaws (loader home); the loader closes, vacuum off, the robot leaves.
3. Loader: up → towards MG06 → down into the holder → open → up → park (y −199.5 mm, z +20 mm).
4. The robot closes MG06 with the fork (lid down, handle to about 29°), leaves, comes back, opens MG06.
5. Loader: back → down → close (pick) → up → home.
6. The robot takes the PCB (vacuum on, loader opens), puts it into the right (output) crate, goes home.

The programs' cycle times: 70.4 s taught, 58.7 s optimized (the physics baselines, §10.2).

### 4.6 The physics report

The lines a run's report starts with (`Simulation.Report.Format_Report`; a program's report adds its sections, §7.6), here
with the values of the legacy reference run (2026-10-24, macOS 26.6.2, Apple M2 Pro), which the report's format tests use
(`test/Simulation/test_report.py`). The programs' reports are the physics baselines,
`test/Simulation/data/baseline_report_<taught|optimized>.txt` (§10.2).

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

### 4.7 Lessons learned and deliberate deviations (do not break)

- **Vacuum**: the cups are pressed **1.5 mm** onto the PCB before vacuum on (bellows). At the **loader
  handover there is no press**: the board sits 0.3 mm above the finger ledges, otherwise the jaws clamp
  on the ledge corners. **At the take from the loader, relieve the 1.5 mm press (back to touch,
  `P_LOADER_TAKE_TOUCH`) before the jaws open**; the stored bellows force would otherwise throw the board off the
  cups (owner, 2026-10-24, F11; KNOWN_ISSUES.md).
- **Loader gripper** (owner, 2026-10-25, B work): The loader gripper centres the PCB; the handover pose must put the PCB on the jaw
  centreline, otherwise the clamp pulls it off the cups. Release the PCB into MG06 slightly above its seat, never pressed. The
  re-grip from MG06 uses its own height L_FETCH with the same 0.3 mm ledge rule.
- **IK families**: PCB handling uses "wrist up" (wrist_2 > 0), the MG06 fork path "wrist down"
  (wrist_2 < 0); the switch happens only in free space (via `P_VIA_HOME_MG06`, §4.8).
- **MG06 fork path**: the recorded close/open path is tight (bar on the slot bottom). Leaving and
  returning use a separately planned path with the bar centred in the slot (5 mm play), from the closed
  handle angle 28.65° and lid −0.5° (`Cell.yaml MG06.Closed_Handle`, `Closed_Lid`). The opening path is joined where its recorded handle angle is
  already below the real closed angle, so the handle is never pushed down before opening.
- **MuJoCo 3.16**: `gap` is not applied to `includemargin`, so no `margin`/`gap` on the PCB or the cups
  (parts would float 2–4 mm apart). PCB and cup contacts use `solref 0.002`, `solimp 0.99 0.999`,
  `solmix 100`.
- **Instability**: MuJoCo resets the state on instability; a run detects `d.time` going backwards and
  fails, never loops silently.
- **CAD deviations compensated in `scene.xml`** (design-decisions): loader shifted +100 mm in Y and
  +0.95 mm in X; rear-right holder support (`holder_10`) lowered by 0.8 mm; electrical cabinet 200 mm
  deep instead of 300 mm.

### 4.8 Initial keypoints (extracted from the verified plan; brief §6)

The points the legacy planner solved (`legacy/plan_cell.py` at commit `f754e2d0379b06ffb43286f4fb7aabf88e25ca5f`: `SEED`, the
`solve` calls of its `main`), each a stationary segment end of its plan (joint speed 0; `q_home` for home). They are in
`Configuration/Constants.yaml` (`Points`) at full precision, rounded to 1e-6 deg: within 1e-4 rad of this table, the five
handover points re-taught 0.4 mm along the jaw axis within 2e-3 rad
(`test/Configuration/test_program_parser.py::test_the_points_are_the_spec_table`); the points the programs add are
`P_INPUT_CLEAR`, `P_LOADER_TAKE_TOUCH` and `P_LOADER_WAIT` (§8).

Robot joints in **rad** (order: pan, lift, elbow, wrist1, wrist2, wrist3), TCP = `aubo/tool/suction_tcp` in m.

| name | joints (rad) | TCP / flange (m) | note |
|---|---|---|---|
| `P_HOME` | 0.7896, 0.6910, 1.8333, −0.4286, 1.5708, 0.0042 | tcp (0.3000, −0.5200, 0.4000) | tool down |
| `P_INPUT_ABOVE` | 0.4649, −0.1335, 2.0022, 0.5650, 1.5708, −0.3205 | tcp (0.1559, −0.7470, 0.1324) | 120 mm above |
| `P_INPUT_APPROACH` | 0.4649, −0.3261, 2.1191, 0.8743, 1.5708, −0.3205 | tcp z 0.0274 | 15 mm above |
| `P_INPUT_PICK` | 0.4649, −0.3633, 2.1304, 0.9229, 1.5708, −0.3205 | tcp z 0.0109 | cups 1.5 mm pressed → vacuum on |
| `P_VIA_INPUT_LOADER` | 0.0465, 0.0565, 1.4637, −0.1952, 1.4994, −0.7367 | tcp (0.0164, −0.5608, 0.3958) | free-space via |
| `P_LOADER_ABOVE` | −0.3720, 0.1896, 1.7926, −0.0310, 1.4280, −1.1529 | tcp (0.0045, −0.3298, 0.3158) | 160 mm along the plate normal |
| `P_LOADER_APPROACH` | −0.4242, 0.1057, 2.1193, 0.3873, 1.4249, −1.2055 | tcp (0.0045, −0.3072, 0.1726) | 15 mm along normal |
| `P_LOADER_PLACE` | −0.4296, 0.0867, 2.1469, 0.4347, 1.4246, −1.2110 | tcp (0.0045, −0.3049, 0.1578) | PCB at the hold pose, **no press** |
| `P_LOADER_PICK` | −0.4302, 0.0847, 2.1496, 0.4395, 1.4246, −1.2116 | tcp (0.0045, −0.3047, 0.1563) | 1.5 mm press → vacuum on |
| `P_LOADER_EXIT` | −0.4187, 0.1225, 2.0906, 0.3409, 1.4252, −1.2000 | tcp (0.0044, −0.3096, 0.1873) | 30 mm along normal |
| `P_VIA_HOME_MG06` | −0.0138, 0.2020, 0.7805, −1.6993, −0.0164, 0.4996 | flange (0.1251, −0.3218, 0.9331) | wrist-up ↔ wrist-down switch |
| `P_MG06_ABOVE` | −0.8205, 0.2056, 0.9667, −2.3805, −1.6059, 1.1519 | flange (0.1550, −0.2074, 0.8531) | |
| `P_MG06_FORK_START` | −1.1328, 0.0136, 1.1340, −2.0213, −1.9182, 1.1519 | flange (0.0550, −0.1074, 0.7531) | start/end of the recorded path |
| `P_MG06_CLOSE_END` | −0.5157, 0.0227, 2.4098, −0.7545, −1.3011, 1.1518 | flange (0.0550, −0.2840, 0.3154) | end of `PATH_FORK_CLOSE` |
| `P_MG06_BAR_CENTRE` | −0.5118, 0.0507, 2.3961, −0.7961, −1.2972, 1.1518 | flange (0.0550, −0.2850, 0.3259) | bar in slot centre, 2 mm off bottom |
| `P_MG06_SLOT_EXIT` | −0.8243, 0.0582, 2.2746, −0.9252, −1.6097, 1.1518 | flange (0.0550, −0.2028, 0.3625) | 90 mm along the slot |
| `P_MG06_WAIT` | −0.8243, 0.2137, 2.0471, −1.3081, −1.6097, 1.1518 | flange (0.0550, −0.2028, 0.4825) | +120 mm, wait here (MG06 closed) |
| `P_MG06_OPEN_JOIN` | −0.5034, 0.0542, 2.3958, −0.8000, −1.2887, 1.1520 | flange (0.0550, −0.2872, 0.3269) | join point of `PATH_FORK_OPEN` |
| `P_VIA_LOADER_OUTPUT` | 0.7093, 0.7032, 1.8349, −0.4944, 1.5158, −0.0734 | tcp (0.2680, −0.4831, 0.3958) | free-space via |
| `P_OUTPUT_ABOVE` | 1.0463, 0.0611, 2.2028, 0.5709, 1.5708, 0.2609 | tcp (0.4050, −0.7470, 0.1334) | |
| `P_OUTPUT_APPROACH` | 1.0462, −0.1697, 2.3385, 0.9374, 1.5708, 0.2609 | tcp z 0.0284 | |
| `P_OUTPUT_PLACE` | 1.0462, −0.2120, 2.3510, 0.9922, 1.5708, 0.2609 | tcp z 0.0134 | PCB 1 mm above the crate floor → vacuum off |

### 4.8.1 (Removed: the brief's names and their constants, renamed 2026-10-24; in git `e7739f4:.claude/project-docs/SPEC.md`.)

### 4.9 The recorded MG06 fork path (brief §6)

`Data/Paths/fork_close_open_path.npz`, recorded in the MG06 project, stores `t, lid, press, handle, flange_pos[3],
flange_quat[4]` per row, each row with its phase. `PATH_FORK_CLOSE` is the rows with phase `CLOSE` (≈6.1 s, after the first
rows that bring the fork from `P_MG06_FORK_START`); `PATH_FORK_OPEN` the rows with phase `OPEN` after the join row (handle
≤ 28.45°). `Robot.MovePath` follows them with IK on the flange site (§7.9). The legacy planner's joins, the leave / return
path around the closed handle and its other special moves: `git show f754e2d0379b06ffb43286f4fb7aabf88e25ca5f:legacy/plan_cell.py`.

### 4.10 Model details (verified state; from the previous Czech README of `mg06_cell_mujoco`)

Lineage: this project continues `mg06_loader_mujoco` and the full `mg06_cell_mujoco`. The scripts that
built the scene (Blender → meshes → `scene.xml`, the loader's `MACHINE_DX` shift, the MG06 lock `GAP`) and
the video scripts are in those projects, not here.

**Robot (AUBO i5).** Base yaw −135° so that every task lies within about ±65° of the pan joint's zero (the
reference plan uses −64.7° … +60.2°). Torque limits 180/180/150/60/60/60 Nm and armature
0.2/0.2/0.15/0.04/0.04/0.04 kg·m² are **estimates** (AUBO publishes neither); gravity compensation and
velocity feed-forward on every joint; the resulting tracking lag is at most 0.013 rad (§4.6). MoveJ's timing (the
legacy planner's min-jerk rule): §7.9.

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
contact bits, radial play 1.5 mm (design-decisions). The robot therefore really pushes the handle (the fork's force on the
bar: §10.1 items 14-15, the physics baselines).

**Loader.** Axis ranges `y_axis` −199.5 … +220.5 mm (QF12 S420, towards MG06), `z_axis` −3.5 … +296.5 mm
(QF12 S300, vertical), jaws 0 … 40 mm each (SCHUNK EGI 040-EI, ID 1474382: stroke 40 mm per jaw, gripping force 25-70 N as
the sum of both jaws, jaws synchronized by a toothed belt, max. speed 100 mm/s, closing time 1.0 s - schunk.com product page and
data sheet, read 2026-10-25; modelled at 70 N, 35 N per jaw, synchronized stiffly, design-decisions); 0 is the CAD
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
starts). Measured fork-to-bar clearance while leaving and returning: 3.8–4.7 mm (on the legacy plan).

**Look.** The studio look (the default NVIDIA Isaac Sim / Isaac Lab environment, approximated), applied
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
  diffuse 0.18 0.19 0.21, no shadows; `shadowsize` 8192, `offsamples` 8; the offscreen buffer 2560 x 1440 (1080p and 1440p
  video frames; `models/scene.xml` has 1280 x 960);
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
  `coacd==1.0.14`, `matplotlib==3.11.2`, `pillow==13.0.0`, `imageio-ffmpeg==0.6.0` (the comparison's media, §7.12),
  `pytest==9.1.1`, `pytest-xdist==3.8.0`. The groups without a user yet are marked *planned* in `requirements.txt`.
- **The verified baseline environment is recorded as a full `pip freeze`** (owner, 2026-10-24): a record of
  what the physics baselines (§10.2) ran with, not an install input (`install.sh` installs `requirements.txt` only). It
  is replaced, with its date, whenever the baselines are re-measured. Recorded 2026-10-24, Python 3.11.2, macOS 26.6.2 arm64
  (Apple M2 Pro), the `.venv` of `./install.sh --recreate`; the same packages captured the baselines of 2026-10-25
  (`baseline_environment.json`); `execnet` and `pytest-xdist` (the parallel test runner, no effect on a result) added
  2026-10-25, `imageio-ffmpeg` (the media's encoder, no effect on a result) added 2026-10-25
  (`.venv/bin/python -m pip freeze --all`):

  ```
  absl-py==2.5.1
  coacd==1.0.14
  contourpy==1.3.3
  cycler==0.12.1
  etils==1.14.0
  execnet==2.1.2
  fonttools==4.67.0
  fsspec==2026.9.0
  glfw==2.11.0
  imageio-ffmpeg==0.6.0
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
  pytest-xdist==3.8.0
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

The RPX layout:

```
README.md                 the public page
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
Configuration/Cell.yaml   the cell's physical description (§8)
Configuration/Constants.yaml, Simulation.yaml, Program.yaml   the cell program: its named values, the simulation's data, the program (§8)
Configuration/Optimization.yaml   the optimized program's settings: the FREE segments, the planner (§8, F9)
Data/Optimized/Program_Optimized.yaml   the optimized program, written by Example/Program/optimize_program.py only (committed, §8)
Data/PCB/pcb_meta.json    PCB poses
Data/Paths/fork_close_open_path.npz   recorded flange path that closes / opens MG06
Data/Experiments/<name>/  a not-adopted experiment: its .patch, its script, README.md (the only copy of its commit)
Data/Generated/<Kind>/    generated artefacts (git-ignored): Reports/, Images/, Trajectories/; Reports/Comparison/ (the comparison, §7.12)
Media/<Logo>_Black.svg, Media/<Logo>_White.svg   the logos for light / dark grounds, written by Example/Media/make_logo_variants.py
Media/Original/           the owner's logo originals, kept untouched for regeneration
src/<Domain>/Core.py      each domain's primary module; a second concern in a topic module, shared constants in Const.py
                          (the module layout: style project reference > Layout, design-decisions)
src/Configuration/Core.py the repository paths, the YAML reader, Cell.yaml's loader, the PCB meta reader, the MuJoCo log route
src/Configuration/Program_Parser.py   the cell program's loader: Constants, Simulation, Program -> the typed IR (§8)
src/Configuration/Const.py   the root, Configuration/ and Data/, the contact groups, the Process keys (shared by both)
src/Scene/Core.py         the cell builder (MjSpec), its constants, the look, the MJCF export
src/Kinematics/Core.py    site poses, DLS IK for the suction TCP and the flange
src/Collision/Core.py     robot contacts and clearance, contact classification of a run
src/Render/Core.py        offscreen image, viewer start
src/Keypoints/Core.py     the points: posing the cell, the check, the IK resolution (§7.7)
src/Simulation/Core.py    the physics run with the instability guard and its measured values (§7.6)
src/Simulation/Report.py  a run's acceptance checks (§10.1), warnings and report
src/Simulation/Const.py   the constants the run and its report share
src/Trajectory/Core.py    the trajectory file format: the only writer and reader (npz + CSV export, §9)
src/Program/Core.py       the constants' model checks, the program build (motion laws, world state, the loader's lane) (§7.9)
src/Program/Check.py      the program check and its report, posing at a sample, the loader's actuator model (§7.9)
src/Program/Const.py      the constant the build and the check share
src/Planner/Core.py       joint-space planning: the spline, its time-optimal timing (TOPP), RRT-Connect, the shortcut, the refit (§7.10)
src/Optimizer/Core.py     the optimized program: the FREE transfers planned, the program written (§7.11)
src/Optimizer/Measure.py  its verification: the LOCKED parts, the interlocks, the tilt, the comparison's metrics (§7.11)
src/Report/Core.py        the comparison: the physics run recorded, the cell's geometry over time, custody, critical path, I8 vs loader timing (§7.12)
src/Report/Media.py       the comparison's media: labelled frames, the H.264 video, the GIF preview, the contact sheet (§7.12)
Example/Simulation/run_program.py      the physics run of a program + report + acceptance (exit 0 / 1 / 2)
Example/Simulation/show_physics.py     the physics run in the viewer (mjpython), --headless --time, --image
Example/Simulation/physics_baseline.py the programs' physics baselines: compare (bounds and drift; exit 0 / 1 / 2) or write
Example/Scene/export_cell_scene.py     writes models/cell_scene.xml
Example/Scene/model_fingerprint.py     compares (default) or writes the compiled model's fingerprint
Example/Render/show_cell.py            opens the viewer on the cell
Example/Render/render_image.py         offscreen image from a camera into Data/Generated/Images/
Example/Media/make_logo_variants.py    the black / white SVG logos + Data/Generated/Images/logo_check.png
Example/Keypoints/check_keypoints.py   the point check table (exit 0 / 1)
Example/Keypoints/resolve_keypoint.py  IK of a point: prints the Joints and Pose lines to paste
Example/Keypoints/show_keypoint.py     poses the cell at a point: viewer (mjpython), --headless, --image
Example/Keypoints/step_keypoints.py    steps through the points in the viewer (mjpython): Page Down / . / Page Up
Example/Program/build_program.py       a program -> its trajectory (npz + csv) and check report (exit 0 / 1); --program (an optimized one refused when
                                       stale or its schedule differs)
Example/Program/optimize_program.py    the optimized program (Data/Optimized/Program_Optimized.yaml) and its report (exit 0 / 1)
Example/Program/compare_programs.py    taught vs optimized: a table and a figure (exit 0 / 1)
Example/Report/compare_report.py       taught vs optimized in full: Data/Generated/Reports/Comparison/Comparison.md + figures (exit 0 / 1 / 2)
Example/Report/make_media.py           videos (MP4), GIF previews and the key points' contact sheet (Media/), the collision image
Example/Program/golden_program.py      the taught program's golden data: compare (exit 0 / 1) or write
Example/Program/show_program.py        plays a trajectory in the viewer (mjpython; Home / Page Down / . / Page Up / End; the overlay shows the
                                       segment and its current line), --version default | optimized, --step / --time / --image, --watchdog S (debug)
test/<Domain>/test_<module>.py mirroring src/ (none for a Const.py); test/Example/<Domain>/ mirroring Example/; test/Install/, test/Docs/;
                               reference data (the baselines) in test/<Domain>/data/; test/conftest.py (src/ on the path,
                               the shared fixtures)
logger/                   run logs and MuJoCo's log (git-ignored, created at run time)
```

`Example/` has one folder per `src/` domain its scripts belong to (`Simulation`, `Scene`, `Render`, `Media`, `Report`,
`Keypoints`, `Program`). `Data/` is ordered by purpose. The configuration (the cell, the
program's three files) is in the root folder `Configuration/` (the style skill's `<config folder>/<file>.yaml`, as
in the template's `Configuration/Cell_SF1.yaml`).

`src/` layering, by domain (within a domain: `Const` ← `Core` ← the topic modules): `Configuration`, `Render`, `Planner`
import nothing of the project (`Planner`: numpy, scipy); `Scene` → `Configuration`; `Kinematics` → `Scene`; `Collision` →
`Configuration`, `Scene`; `Trajectory` → `Configuration` (no MuJoCo); `Simulation` → `Configuration`, `Scene`, `Collision`,
`Trajectory`; `Keypoints` → `Configuration`, `Scene`, `Kinematics`, `Collision`; `Program` → `Configuration`, `Scene`,
`Kinematics`, `Collision`, `Keypoints`, `Trajectory`, `Planner`; `Optimizer` → `Configuration`, `Kinematics`, `Collision`,
`Keypoints`, `Planner`, `Program`, `Trajectory`; `Report` → `Configuration`, `Kinematics`, `Collision`, `Simulation`, `Program`,
`Optimizer`, `Trajectory`, `Render` (the top of the order). No import cycle (checked 2026-10-25). Scripts reach
`src/` through a path built from `__file__`.

## 7. Domains

### 7.1 Configuration

The repository's folders and files as constants built from the repository root (`Const.py`, from its own location),
never from the working directory; the reader of `Data/PCB/pcb_meta.json`; the route of MuJoCo's log file into `logger/`.
The only reader of the YAML configuration (§8): `Core` reads `Cell.yaml` and holds the strict YAML reader (`Read_YAML`);
`Program_Parser` reads the cell program's three files into the typed intermediate representation (`Program_Str`) that
the other domains read.

### 7.2 Scene

Builds the cell from `models/scene.xml` with MjSpec: the crates' collision as floor + 4 walls, the AUBO
i5 at its base pose, the VGC10/fork tool rigidly on the flange with one body and one adhesion actuator
per cup, the robot servos, the loader drive lag, the look (lights, materials, cameras). `Compile_Spec` compiles a spec
with MuJoCo's one known warning (the robot MJCF's integrator, `KNOWN_ISSUES.md`) shown as one INFO line on stderr. Exports the cell
as one MJCF with mesh paths relative to the file. The compiled model is pinned by its fingerprint
(§10.2).

### 7.3 Kinematics

Site poses of a robot configuration, damped-least-squares IK on the suction TCP or the flange (with a
null-space pull to a reference), the TCP pose that holds the PCB at a given pose.

### 7.4 Collision

Robot contacts of a configuration (penetration below a margin), the clearance between robot geoms and
everything else (bounding-sphere filtered `mj_geomDistance`; `Close_Pairs` with a vectorised broad phase, identical to
the per-pair loop), planning margin 3 mm (6 mm geom margin on robot and PCB geoms of the planning model). The planning
model is also the check model: it carries the table top as two boxes (`CONST_TABLE_BOXES`, measured from the table's
visual mesh; never in the physics model, design-decisions). The classification of a run's contacts into the checked
categories of §4.6.

### 7.5 Render

An offscreen image from a named camera or a free camera, with the studio look's fog and haze flags on;
starting the viewer (the passive one with the same flags); the playback loop of a passive viewer (`Playback_Cls`, the
timeline by the wall clock with queued key actions; `Run_Viewer_Loop`, the mujoco 3.16 viewer rules: `viewer.lock()` only
while posing, `set_texts` outside it and at most every 0.25 s unless the segment changes, `sync` every frame; `Watchdog_Cls`
for `--watchdog`). No project imports. The playback has a speed against the wall clock and a `frame` action (one
sample on, paused), and the loop a `prepare` hook called every frame before the lock (`Example/Simulation/show_physics.py` steps
its physics run there, at most 50 ms of wall time per frame, and only copies the state under the lock).

### 7.6 Simulation

Runs a program's trajectory with physics (servo set-point with velocity feed-forward, the loader's and the gripper's commands,
vacuum events); fails with `Simulation_Error_Cls` when MuJoCo resets the state; returns the measured values; formats the
report of §4.6 and the program report; checks §10.1 (the acceptance and the report: `Simulation/Report.py`).

The physics run (`Physics_Run_Cls`, `Run_Physics`) takes the drive `Trajectory_Drive_Cls` (a program's 4 ms trajectory: set-points linear
between samples with the stored `robot_qd` as feed-forward, the loader and gripper commands, the vacuum events, the steps as
labels; the fork may touch the handle in the steps whose `Allow_Contact` has `handle_bar`). Per step it measures the tracking
error, the PCB's slip in the cups (from the first cup contact after vacuum on until vacuum off, also after the cups lose it),
the tool's tilt and the TCP's acceleration, the deepest contacts and, in a parallel step, the closest robot-loader distance.

**The PCB lost** (F11, owner 2026-10-24): with the vacuum on, no cup touches the PCB for 20 ms (`CONST_LOST_TIME`), or the PCB
is 20 mm or more below the table top (`CONST_PCB_FLOOR`; the physics model has no table and its floor does not collide, so a
dropped PCB falls through). The run stops at once with the fault `PCB lost at t, step, line: why; last held at t by k cups,
slip, tool tilt, TCP acceleration` (an acceptance failure, exit 1). An instability (MuJoCo resets the state) raises
`Simulation_Error_Cls` with the time, the phase, the line and the PCB's last position (exit 2); neither hangs.

**A program's physics run** (F11; `Example/Simulation/run_program.py`): `Trajectory_Drive_Cls(trajectory, program, free_steps)`,
every 4 ms per step the closest robot - loader distance (the carriages and jaws, and in a FREE step the PCB where the program has
it in the jaws: in the LOCKED handover and take the cups touch it by design, the cups_pcb group) and carried PCB - loader
distance (at the handover and the take it touches the jaws by design, pcb_jaws; a line under the table says both);
`Format_Program_Report` (SPEC §4.6's lines, the vacuum holds, a row per step - FREE or LOCKED, parallel, tracking, slip, the
loader distances, the deepest contact -, each carrying free transfer's slip with the tool's tilt and the TCP's acceleration
then and their maxima, the warnings, the 28 checks of §10.1).

### 7.7 Keypoints

The robot points at work (`src/Keypoints/Core.py`): a point's tool site
(`Simulation.yaml Tool_Site`), the target of a derived point (its `Reference` moved by its `Offset`, or its `PCB_Hold`),
the MG06 state of a point's `Fixture` (the press plate on the model's coupling), posing the whole cell at a point in its
scene (robot, bellows at rest, loader axes, jaws at the gripper command's `Jaw_Actual`, PCB at its location, MG06), the
point check and the IK resolution of a point's joints (the `Joints` and `Pose` lines printed, never written). Playback and
posing read `Joints` only.

The check (per point, one table row; FAIL on any rule, WARN when wrist_2 is within the family margin):
joint limits (the model); the family (wrist_2's sign, shown); `Pose` against FK(`Joints`, `Tool`) within 1e-5 m /
1e-3° (the `check` column); a derived point against its target within `Stale_Position` / `Stale_Angle` (the `stale`
column); the clearance - **exact** distances
(`Collision.Close_Pairs`, `mj_geomDistance`) from the robot's geoms, and from the PCB's when the cups carry it,
to every collidable geom of the planning model (the table included): every pair closer than `Clearance.Robot` fails
unless its group is in the point's `Contacts` (`Simulation.yaml Points`) and no deeper than `Allowed_Depth`. Groups: `cups_pcb` (a cup or the cup block on
the PCB, not the fork), `handle_bar` (the tool on the MG06 handle), `pcb_crate` (the PCB on a crate, and on the table
only while the PCB is inside a crate), `pcb_jaws` (the PCB on the loader's jaws) - only groups a point or segment
observes; any other pair within the clearance fails. Not checked:
the robot's mount (its base on the pedestal plate, distance 0 by construction; a code constant, as the
template's base–stand exclusion). Exact distances because MuJoCo's box–box collider generates no contact for a
near pair (the carried PCB 1.0 mm above the crate floor), whatever the geom margin
(`test/Collision/test_core.py`).

### 7.8 Trajectory

The trajectory file (`src/Trajectory/Core.py`, F8): `Trajectory_Str`, `Write_Trajectory` (the `.npz` and the CSV export
beside it), `Read_Trajectory` (refuses a missing array, a wrong shape or another format version). It knows nothing of
MuJoCo; the program build fills it, the viewer and the physics run (§7.6) read it.

### 7.9 Program

The program at work (`src/Program/Core.py`; the check and posing: `src/Program/Check.py`; `FEATURES.md` F8, F10). `Validate_Constants` checks `Constants.yaml` against the
model (every tool's `Flange_Offset` against its site, every `Pose` against FK, every derived point against its target, the
loader positions and gripper commands within the model's ranges). `Build_Program` runs it first, then turns the IR into a
trajectory: sample 0 is the start state (`Simulation.yaml Start`), then each executed command's samples (4 ms) - the robot's
lines in order, a `Run` expanded into its job's lines (each labelled `<Run line> > <job line>`), `End_Settle` last - every
motion ending at rest. The loader's lane (F9): `Loader.Start(J_)` starts a job that then takes one sample per robot sample
while the robot's lines go on (a robot `Wait`, `WaitDI` or motion advances it too); `Loader.WaitDone(J_)` holds the robot
until the job's last sample; a `Run` gives the same samples as Start + WaitDone (tested).

- **MoveJ**: min-jerk to the target's stored `Joints`, `T = max(Movej_Min_Duration, maxⱼ 1.875·|Δqⱼ|/v_maxⱼ,
  maxⱼ √(5.774·|Δqⱼ|/a_maxⱼ))` (the legacy planner's rule), speeds analytic; `v_max` = the datasheet speeds × `v=` (omitted:
  `Motion.Speed_Fraction`).
- **MoveSpline** (an optimized program only): the cubic spline through its waypoints over the chord length (clamped), timed
  time-optimally (TOPP, §7.10) within its `v=` of the datasheet speeds and the acceleration limits from rest to rest on the
  file's TOPP grid, stretched to whole samples; speeds analytic; ends exactly on its end point's `Joints`; it may change the IK
  family (a joint-space motion); MG06 moves to the end point's `Fixture` along it.
- **MoveL**: the target point's tool site (`T_SUCTION`: `suction_tcp`, `T_FLANGE`: the flange) on a straight
  line, orientation slerp, min-jerk time law over `T = max(Movel_Min_Duration, 1.875·L/v)`; IK per sample (pure DLS,
  1e-8 m) seeded from the previous; fails on non-convergence, wrist_2 within `Family_Margin` or a family change; stretched
  (noted) while a sampled joint speed or acceleration exceeds its limit; ends exactly on the target's stored joints (the
  same IK branch, within the stale tolerance).
- **MovePath**: `PATH_FORK_CLOSE` = rows 1-305 from `P_MG06_FORK_START` to `P_MG06_CLOSE_END`; `PATH_FORK_OPEN` = the `OPEN`
  rows after the first with handle ≤ `Closed_Handle − Open_Join_Margin` (row 663), from `P_MG06_OPEN_JOIN` to
  `P_MG06_FORK_START`; flange IK per row, the recorded times interpolated onto the grid, ends snapped; `PATH_FORK_OPEN` starts
  moving (39.1 mm/s, noted).
- **MoveY / MoveZ / Gripper** (loader jobs): one channel, min-jerk over its `t=`, ending exactly on its target (the gripper
  command; the jaws' real position is its `Jaw_Actual`); **SetDO** drives the output's actuator (an event); **WaitDI** holds
  until the input comes, its `Signal_Response` after its cause (`DI_VACUUM_OK`: the last `SetDO(DO_VACUUM, ON)`), so a WaitDI
  whose input came already costs nothing (F11); one whose cause is off is a build error (the wait would time out); **Wait** /
  **Test** / `End_Settle` hold (`Test` needs MG06 settled closed and the PCB in it).
- **Speeds**: analytic in MoveJ; central differences elsewhere; zero at sample 0, in every hold and at every MoveL / Path end.
- **World state**: the PCB's location changes only by the vacuum and gripper rules (vacuum on with the cups on the PCB in a
  crate or the jaws; clamping on it in MG06; opening at MG06; vacuum off with it clamped in the jaws or within 2 mm of its
  place on a crate floor; anything else fails). The carried PCB is the nominal held pose set back along the tool axis by the
  bellows' compression left (1.5 mm at a press, relaxing as the TCP moves away). MG06 is the recorded rows during a `Path`;
  during a MoveJ / MoveL it moves from its state at the motion's start to the target point's `Fixture` along the motion's
  time law (the handle settles as the fork lets it go); it holds otherwise. The jaws stand at `G_CLAMP`'s `Jaw_Actual` while
  the PCB is between them.

`Pose_Sample` poses the whole cell at a sample (no IK, no physics). `Check_Trajectory` (the program check, on the planning
model): every sample where a robot geom, the carried PCB, the loader, the jaws or MG06 may have moved more than
`Clearance.Check_Step` since the last measured one (and every segment end) is measured exactly (`Close_Pairs`): the robot,
the PCB on the cups (to everything else), the PCB in the jaws (to the workstation: table, crates); a pair within
`Clearance.Robot` fails unless the segment allows its group (`Simulation.yaml Allow_Contact`) and it is no deeper than `Allowed_Depth`; every allowed group must
occur; per segment and group the closest pair (up to 10 mm) and its sample and time are reported. Also: the joint limits; the
peak joint speed and acceleration of every MoveJ and MoveSpline (at its `v=`; a MoveJ's rounding to whole samples allowed: 1 + 0.5 / n) and MoveL within
`Motion` (finite differences, 0.1 % tolerance), the recorded paths' peaks reported, not limited; the handover's no-press
rule (the PCB-ledge gap with the jaws at grip within `Handover.Ledge_Gap` ± `Ledge_Tolerance` at `P_LOADER_PLACE` and while
clamped, never smaller along the MoveL into it - there only its height above the finger ledges counts: the gap at the point plus
the PCB's height above it along the plate normal; owner 2026-10-25) and the PCB centre at most `Handover.Lateral_Max` from the jaw centreline at
`P_LOADER_PLACE` (along the jaw axis: the jaws centre the PCB as they clamp); the posed jaws are the command limited by the part
(`Jaw_Posing_Faults`: every gripper operation monotonic, never closer than grip with the PCB in the jaws) and every Gripper
command ramp no faster than `Motion.Jaw_Speed_Max` (`Jaw_Speed_Faults`); the re-grip from MG06 (`Regrip_Check_Str`, item 5):
the PCB-ledge gap with the jaws at grip about the PCB at its MG06 rest pose within the handover's rule, at `L_FETCH`, and the open
jaws not touching the PCB or the MG06 holder over the descent before it; the point check of every point the program uses.
`Format_Check_Report` prints it. The loader moving in parallel is part of every measured sample's scene, so the check also
measures the robot against the moving loader (I8).

**Room around the loader** (F11, owner 2026-10-24): in every free transfer (`Configuration.Program_Parser.Free_Steps`: the taught
program's `Optimization.yaml Free_Segments`, the optimized program's MoveSpline segments) the check measures every sample: the
robot and the carried PCB keep `Clearance.Loader_Free` + `Tracking_Margin` (10 + 4 mm) from the loader's carriages and jaws and
the PCB in them, the loader posed where its actuators put it (`Loader_Actual`: the 30 ms filter, the position servo, the axis'
damping, mass and gravity load, stepped at the model's timestep - within 0.17 mm (y) and 1.04 mm (z) of physics, where the
command is up to 22.8 mm off). The report gives each free transfer's closest loader pair. The work at the loader (the handover,
the take) keeps `Clearance.Robot` (the pass above).

### 7.10 Planner

Joint-space planning, independent of the cell (`src/Planner/Core.py`, numpy and scipy only; F9): `Spline_Path_Cls` (scipy
`CubicSpline`, clamped, over the chord length); `Topp` - TOPP by reachability (Pham & Pham 2018) on a uniform grid with
first-order interpolation (each interval's constant `s̈` meets the acceleration limits at both of its ends), solved per
interval in closed form (every lower and upper bound on `s̈` is linear in `ṡ²`); `Time_Path` - the timing sampled every 4 ms,
stretched to whole samples and further while a sample exceeds a limit (B3: the closed-form trapezoid and triangle within a
relative 1e-3, measured < 2e-7); `Rrt_Connect`, `Shortcut`, `Prune` and `Smooth` (the spline checked at `Check_Step` and
refitted by polyline midpoints), every random choice from the caller's seeded generator.

### 7.11 Optimizer

The optimized program (`src/Optimizer/Core.py`; what it is measured by: `src/Optimizer/Measure.py`; F9). `Free_Transfers`: the FREE segments of `Optimization.yaml`, consecutive
ones merged across segments that only run loader jobs (those jobs then run in parallel). `Free_Space_Cls`: a configuration's
validity in the transfer's scene (the taught program's at its start) - exact distances (`Close_Pairs`) of the robot and a
carried PCB (hung at the cup face) at least the clearance plus a margin from everything and `Loader_Free` + `Tracking_Margin`
plus the margin from the loader and the PCB in its jaws (F11), the carried PCB's tilt at most `Carry_Max_Tilt` (B4). A transfer with a waiting point (`Optimization.yaml Waiting_Points`) has two legs: to the waiting point
at once (after its first job's start), and on to its end at the slot the search finds (after a job's start, so after the
WaitDone of the job before it; never after a Wait, F15). `Plan_Transfer`: RRT-Connect
(seed `Seed` + the transfer's index + 1000 x the leg) and the shortcut at the planner
margin, the prune, the spline at `Spline.Margin`. `Optimize_Program`: plans every transfer at the first margin it can be
planned with, writes the program (`Optimized_Text`: LOCKED segments verbatim from `Program.yaml`, renumbered; a transfer as
`Loader.Start` / `Robot.MoveSpline` / `Loader.WaitDone` lines), loads, builds and checks it; a transfer that meets the loader
(I8: closer than `Loader_Free` + `Tracking_Margin` to the loader at its actuator model's position, F11) or that breaks the gating
rule (below) moves its spline after the next job's start (`Gate`; past the last job's start it fails, naming the remedies: split
the job or gate on a loader input), any other failure takes the next margin; finally writes the schedule. `Interlocks` (I1-I9
with their times), `Compare_Locked` (each
LOCKED part against the taught program, byte by byte from its own start), `Carried_Tilt`, `Stale_Inputs` (the inputs' and the
planning model's sha256 against the file's record), `Schedule_Differences`, `Measure` (the comparison's metrics).

**LOCKED and FREE.** `Optimization.yaml Free_Segments` lists the FREE segments by name; every other segment is LOCKED. A
LOCKED segment's samples relative to its own start are byte-identical in both programs (its start time may move); a FREE
segment's motion is planned. Consecutive FREE segments with only loader jobs between them are one transfer, from the end
of the previous LOCKED robot segment to the start of the next; their intermediate points (`P_HOME`, the vias) are dropped.

| Transfer / part | From → to | Taught segments | Carries the PCB | Family |
|---|---|---|---|---|
| FREE | `P_HOME` → `P_INPUT_ABOVE` | `S1_Approach_Input` | no | up |
| LOCKED | the crate pick: approach, press 1.5 mm, vacuum, lift | `S2_Pick_Input` | from vacuum on | up |
| FREE | `P_INPUT_ABOVE` → `P_LOADER_ABOVE` | `S3_To_Loader` | yes | up |
| LOCKED | the handover: no press, clamp, vacuum off, up | `S4_Handover` | until vacuum off | up |
| FREE | `P_LOADER_ABOVE` → `P_MG06_FORK_START` (`P_HOME` dropped) | `S5_Home_After_Handover`, `S8_To_MG06` | no | up → down (the switch in free space) |
| LOCKED | `PATH_FORK_CLOSE` and the bar centred, the leave, the test, the return, the open join and `PATH_FORK_OPEN` | `S9_Close_MG06` … `S13_Open_MG06` | no | down |
| FREE | `P_MG06_FORK_START` → `P_LOADER_ABOVE` (`P_HOME` dropped; via the waiting point `P_LOADER_WAIT`) | `S14_Home_After_MG06`, `S17_To_Take` | no | down → up |
| LOCKED | the take: approach, press 1.5 mm, vacuum, gripper open, out of the jaws, up | `S18_Take_From_Loader` | from vacuum on | up |
| FREE | `P_LOADER_ABOVE` → `P_OUTPUT_ABOVE` | `S19_To_Output` | yes | up |
| LOCKED | the place: approach, the last 15 mm at 30 mm/s, vacuum off, up | `S20_Place_Output` | until vacuum off | up |
| FREE | `P_OUTPUT_ABOVE` → `P_HOME` | `S21_Home_End` | no | up |
| LOCKED | the loader jobs (shape and durations; their start times move) | `S6_Loader_Insert`, `S7_Loader_Park`, `S15_Loader_Fetch`, `S16_Loader_Return` | - | - |
| LOCKED | the end settle (`Simulation.yaml End_Settle`) | - | - | - |

Every MoveL near contact, both fork paths and their joins and the MG06 closed-state rules sit inside LOCKED segments; no
MoveL is FREE, so no MoveL crosses the family switch.

**Interlocks** (`Interlocks` verifies each on the built trajectory, with the times that prove it):

| # | Interlock | Why |
|---|---|---|
| I1 | `S4_Handover` starts with the loader at `L_HOME` and the gripper at `G_OPEN` | the PCB enters open jaws |
| I2 | `J_INSERT`'s first loader move after `S4_Handover` ends | the loader moves only after the robot left the jaw zone |
| I3 | `S9_Close_MG06` starts after `J_PARK`'s last loader move (parked, gripper open) | MG06 is closed only with the PCB released and the loader parked |
| I4 | `S11_Test` starts after `S10_Leave_MG06` ends (MG06 closed, the fork out of the bar) | the tester runs closed, the robot away |
| I5 | `S12_Return_MG06` starts after `S11_Test` ends | no motion at MG06 during the test |
| I6 | `J_FETCH`'s first sample after `S13_Open_MG06` ends (MG06 open, the fork off the handle) | the loader enters MG06 only when it is open and the fork is gone |
| I7 | `S18_Take_From_Loader` starts after `J_RETURN_Z`'s last loader move (the loader at `L_HOME`, the PCB clamped) | the robot enters the jaw zone only when the loader is home |
| I8 | the robot (with a carried PCB) at least `Clearance.Loader_Free` + `Tracking_Margin` from the loader (with the PCB in its jaws, posed where its actuators put it) at every sample of the parts where both may move | parallel motion is safe only where checked |
| I9 | the robot's motion to the take (its last before `S18_Take_From_Loader`) starts after `J_RETURN_Y` is done: after its `Loader.WaitDone` (or `Loader.Run`) and after its last loader move (F15, owner 2026-10-25) | the robot enters the loader's way home only when the loader's Y is home, whatever the loader's speed |

I1-I7 and I9 hold by program order (`Loader.Start` / `Loader.WaitDone` lines); I8 is the exact robot-loader check, whose failure
moves a spline's slot (`Gate`).

**The gating rule** (F15, owner 2026-10-25; `Program.Check.Gating_Faults`): a robot motion that runs while a parallel loader job
may run (after its `Loader.Start`, before its `Loader.WaitDone`) must not come within `Clearance.Loader_Free` of any pose the job
sweeps (the loader where its actuators put it, every 20 ms; `Swept_Distance`, exact) - otherwise its separation rests on timing
and it must be gated by the job's `Loader.WaitDone` (or an input from the loader), never by a `Wait`. The optimizer moves such a
spline to the next slot and refuses a program that still breaks it; `optimize_program.py` reports it. A motion that starts
inside a job's volume and only leaves it (To_MG06's spline from `P_LOADER_ABOVE` during `J_INSERT`: 1.52 s ahead of the loader)
is reported with its lead, not refused. `J_RETURN` is two jobs for it: `J_RETURN_Y` (up to the travel height and home along Y)
and `J_RETURN_Z` (down home, the hold), the same motion and timing; the taught program runs both with `Loader.Run` (its
trajectory byte-identical, the golden's labels re-captured).

### 7.12 Report

The comparison of the taught and the optimized program (`src/Report/Core.py`, `src/Report/Media.py`; F14, owner 2026-10-25). It
changes no program, point, model or limit; it measures.

- **The recording** (`Record_Physics`): a program's physics run (`Physics_Run_Cls`, stepped as `Run_Physics` steps it) with its
  state read at every trajectory sample (4 ms): positions, velocities, the cups touching the PCB and whether the jaws, the
  MG06 holder or a crate touch it (closer than `Cup_Touch`), the vacuum, the TCP. It never writes the run's data: the run's report
  is byte-identical to `run_program.py`'s (tested).
- **The geometry over time** (`Cell_Geometry_Cls`, `Geometry_Series`), on the planning model posed at a recording's positions or a
  trajectory's sample: the exact distance (`Close_Pairs`, cut 100 mm) of each pair of `CONST_PAIRS` - the robot to the
  environment (its mount excluded), to MG06's lid / handle / press plate, to the loader, to the PCB and to its own links (the
  links not neighbours and not excluded by the model); the PCB to the environment, the loader and MG06; the loader to the
  environment and MG06 - every 20 ms, each pair's minimum refined on every sample within 20 ms, the contacts by design left
  out (`By_Design`: a step's `Allow_Contact` group, the PCB resting in a crate or MG06, the PCB between the jaws and the holder
  while the loader is down at MG06). The objects' extents and the zones' occupancy from their collision geoms' points (a
  mesh's vertices, a primitive's box corners).
- **The shared zones** (owner 2026-10-25): the jaw zone, the box of the loader's jaws at `L_HOME` (open and clamped); the MG06 zone,
  the box of MG06 with its lid, handle and press plate over every state of both trajectories; each grown by `Loader_Free`.
- **The PCB** (`Handovers`, `Custody_Gaps`, `Overhang`, `Tilt`, `Location_Spans`): each location change of the trajectory checked
  in physics (pose error, both holders, none), the spans without a holder, the board beyond the cups and the jaws.
- **The timeline** (`Critical_Path`, `Robot_Waits`, `Signal_Order`, `Wrist_Approach`): the robot's program is the cell's
  timeline; each segment is spent on its motion or on what it waits for (a `Wait` in a step with a parallel loader job: the
  loader by its time).
- **The loader slowed** (`Stretch_Job`, `Stretched_Distance`, `Clear_Height`): a parallel job's loader lane slowed by a factor, the
  robot's lines as programmed and holding at its WaitDone, I8's distance on it (the actuator model, exact, every 4 ms).
- **The media** (`Media.py`): frames from one fixed free camera with the studio look and a label; H.264 MP4 through
  imageio-ffmpeg's pinned ffmpeg (CRF 18, preset slow, yuv420p); GIF previews with one palette, kept under a size; a contact
  sheet.

`Example/Report/compare_report.py` writes `Data/Generated/Reports/Comparison/Comparison.md` (every number tagged with its source: [K] the
trajectory, [P] the physics run, [C] the configuration, [M] the model), five figures (timeline, joints, paths, clearance,
comparison; the dataviz palette, the optimized program's series in the JIC purple) and both physics reports.
`Example/Report/make_media.py` writes the videos (1920 x 1080 or 2560 x 1440, 60 fps, real time) and the point images and collision
image into `Data/Generated/Reports/Comparison/`, and the GIF previews (960 px, 4 x speed, 15 fps, at most 10 MB) and the key points' contact
sheet into `Media/` (committed; the videos become release assets only with the owner's approval).

## 8. Configuration — `Configuration/Cell.yaml` and the cell program (`Constants.yaml`, `Simulation.yaml`, `Program.yaml`)

Four hand-written files, split by who uses each value (F10, design-decisions): `Cell.yaml` the cell's physical description;
`Constants.yaml` every named value the cell's controllers use; `Program.yaml` what the controllers run; `Simulation.yaml`
what only the simulation needs. `Cell.yaml` is read by `src/Configuration/Core.py` (`Load_Cell_Configuration`), the other
three by `src/Configuration/Program_Parser.py` (`Load_Constants`, `Load_Simulation`, `Load_Program` -> `Program_Str`, the
typed IR). All four go through one strict YAML reader (`Read_YAML`: duplicate keys rejected, only `true` / `false` are
booleans, each string keeps its line). Every field REQUIRED (no optional fields; `null` and `[]` where the file's header
allows them), every problem collected and reported at once with its file, line and key path (a program line also with its
segment or job and its text), unknown keys or sections returned as WARNING notes that the scripts print, angles in degrees
converted to radians once. Tested in `test/Configuration/`; the checks against the model are `Program.Validate_Constants`
(§7.9).

| `Cell.yaml` field | Unit | Source | Read by |
|---|---|---|---|
| `Robot.Base_Position`, `Robot.Base_Yaw` | m, deg | CAD, §4.2 | `Scene` |
| `Vacuum.Pressure`, `Vacuum.Cup_Diameter` (force derived) | Pa, m | VGC10 datasheet, §4.3 | `Scene` |
| `MG06.Closed_Lid`, `MG06.Closed_Handle` | deg | §4.7 (`LID_CLOSED`, `H_CLOSED`) | posing, the keypoint check |
| `Clearance.Robot` (planning margin = 2 x, derived), `Allowed_Depth.<group>` (four groups), `Stale_Position`, `Stale_Angle`, `Family_Margin`, `Check_Step` | m, deg | §2, the owner (0.5 mm), measured (F6), F8 | `Collision`, the keypoint and program checks |
| `Clearance.Loader_Free` (0.010), `Tracking_Margin` (0.004) | m | the owner (F11, 2026-10-24); measured in physics (F11, 2026-10-25: the robot's 2.51 mm deviation in the free transfers + the loader model's 1.04 mm) | the program check's free transfers, the planner, `Delay`, I8 |
| `MG06.Open_Join_Margin` | deg | the legacy planner (28.45°, §4.9) | `Robot.MovePath(PATH_FORK_OPEN)` |
| `Handover.Ledge_Gap`, `Handover.Ledge_Tolerance` | m | §4.7, the owner (B1, F8) | the program check (no press at the handover) |
| `Handover.Lateral_Max` (0.0005) | m | the owner, 2026-10-25 (B work, item 1) | the program check (the PCB centre's offset from the jaw centreline at `P_LOADER_PLACE`) |
| `Motion.Jaw_Speed_Max` (0.1) | m/s per jaw | SCHUNK EGI 040-EI data sheet, max. speed 100 mm/s (2026-10-25) | the program check (every Gripper ramp's peak; the loader jobs' Gripper t = 0.6 s: 1.875 x 29.25 mm / 0.6 s = 91.4 mm/s) |
| `Motion.Speed_Fraction` (MoveJ's default `v=`), `Motion.Datasheet_Speeds`, `Motion.Acceleration_Limits` (*assumed*), `Motion.Movej_Min_Duration`, `Motion.Movel_Min_Duration` | -, deg/s, deg/s², s | §4.4, §4.10, the legacy planner | MoveJ, MoveL |
| `Process.Test_Duration` (a placeholder) | s | the owner, 2026-10-24 | `Test(Process.Test_Duration)` |
| `Acceptance.*` | m, deg | §10.1 | `Simulation.Check_Acceptance` |
| `Acceptance.Loader_Distance` (0.001), `Slip_Max` (0.00198), `Tracking_Max` (0.0189), `Fork_Net_Force_Max` (290), `Handle_Load_Max` (13.5) | m, m, rad, N, N m | the owner (F11: 1 mm; 1.5 x the legacy run's 1.32 mm and 0.0126 rad; step 6, 2026-10-25: the fork's worst mode 247.3 N / 11.45 N m x 1.15, rounded up) | `Simulation.Check_Program_Acceptance`, `Program_Warnings` |
| `Acceptance.Place_Holder_Force` (1.0), `Release_Penetration` (0.0001), `Rest_Penetration` (0.0000002), `Settle_Margin` (0.00001), `Seat_Time` (0.2), `Seat_Supports` (3), `Seat_Height` (0.00005), `Seat_Speed` (0.001), `Seat_Tilt` (0.1) | N, m, m, m, s, -, m, m/s, deg | the owner, 2026-10-25 (B work, items 7 and 9); `Rest_Penetration` measured (B.1) | `Simulation.Place_Checks` (§10.1 items 18-25) |
| `Simulation.Phase_Steps`, `Simulation.Cup_Touch` | -, m | the legacy run (F3.5); their sensitivity: design-decisions | `Simulation`, `Collision` |

Not configured, on purpose: joint limits, loader axis ranges, the MG06 lid's open stop (the model holds them:
the vendored robot MJCF, `models/scene.xml`; read from the compiled model); the loader and gripper move times (each
job line carries its own `t=`, with its source in a comment).

`Constants.yaml` (`FEATURES.md` F10): six sections; every name UPPERCASE with its section's prefix, declared once in the file,
with a one-line `Description`, and used by `Program.yaml` or `Simulation.yaml` (an unused one is an error).

| Section | Prefix | Fields | Unit, frame |
|---|---|---|---|
| `Tools` | `T_` | `Flange_Offset` [x, y, z, R, P, Y] | m, deg; the tool frame in the flange frame (`T_SUCTION`, `T_FLANGE`) |
| `Points` | `P_` | `Tool`, `Reference`, `Offset`, `Joints`, `Pose` | `Joints` [deg] in the model's joint order - authoritative; `Pose` [x, y, z m; R, P, Y deg] the tool frame in the robot base frame, R = Rz(Y)·Ry(P)·Rx(R), each angle written in (-180, 180], FK(`Joints`, `Tool`) within 1e-5 m / 1e-3° as a rotation; `Reference: <P_> \| null`, `Offset: [dx, dy, dz] \| null` [m] in the Reference point's tool frame, both null or both set, no cycle, FK within `Stale_Position` / `Stale_Angle` |
| `Loader_Positions` | `L_` | `Y`, `Z` | m, the model's axis zeros (`L_HOME`, `L_TRAVEL`, `L_PLACE`, `L_FETCH`, `L_PARK`) |
| `Gripper_Commands` | `G_` | `Jaw` | m per jaw, the command (`G_OPEN`, `G_CLAMP`) |
| `Paths` | `PATH_` | `File`, `Phase` (`CLOSE` / `OPEN`), `Start`, `End` (points) | the recorded flange path (§4.9) |
| `Signals` | `DO_` / `DI_` | `Port` | a placeholder (`KNOWN_ISSUES.md`) |

25 points: the 22 of §4.8 at full precision, the taught program's `P_INPUT_CLEAR` (F8.5) and `P_LOADER_TAKE_TOUCH` (the
take's touch, §4.7), and the optimized program's waiting point `P_LOADER_WAIT` (50 mm beyond `P_LOADER_ABOVE` along the plate
normal).
Approach and exit points are `Reference` + `Offset` of their place; the fork points and the vias are roots.

`Simulation.yaml`: `Start` (`Robot` a point, `Loader` a position, `Gripper` a command, `Vacuum` `ON` / `OFF`, `PCB` a location
- `input_crate`, `cups`, `jaws`, `mg06`, `output_crate` -, `MG06` `Open` / `Closed`); `Tool_Site` (a model site per tool);
`Actuators` (the simulated actuator per output: `vacuum`); `Signal_Response` [s] per input; `Vacuum_OK_Min_Cups` [cups, 1-4]
(the simulated vacuum switch: in physics `DI_VACUUM_OK` needs that many cups on the PCB for its response time, F11; 4);
`Jaw_Actual` [m] (where the jaws
really stop on the PCB per closing command); `End_Settle` [s] (a hold after the last segment, simulation only);
`Allow_Contact` (contact groups per segment, by its name without the `S<n>_` prefix; an unlisted segment is contact-free);
`Points` (per point: `Context` {`Loader`, `Gripper`, `PCB`}, `Fixture` `Open` / `Closed` / the recorded `{Lid, Handle,
Source}`, `Contacts` the groups allowed there, `PCB_Hold` null or {`Location`, `Offset` [m] along the tool axes} - the
target of a point placed against the PCB).

`Program.yaml`: `Program` (segments `S<n>_<Name>`, n = 1, 2, ... without gaps, `<Name>` unique, each a list of lines ending
with the robot at rest at a named point), `Loader_Jobs` and `Tester_Jobs` (`J_<NAME>` -> lines). One call per line, parsed
with `ast` and never evaluated; the only literal values are speeds, durations, waits and timeouts:

| Line | Where | Meaning |
|---|---|---|
| `Robot.MoveJ(P_, v=, a=, r=)` | Program | joint move to the point's `Joints`; `v` a fraction of the datasheet joint speeds in (0, 1] (omitted: `Motion.Speed_Fraction`) |
| `Robot.MoveL(P_, v=, a=, r=)` | Program | straight tool line to the point; `v` the peak tool speed [m/s], required |
| `Robot.MovePath(PATH_)` | Program | the recorded path; the robot must stand at its `Start`, it ends at its `End` |
| `Robot.SetDO(DO_, ON\|OFF)`, `Robot.WaitDI(DI_, timeout=)` | Program | set an output; wait for an input [s] (its simulated response must be within the timeout) |
| `Loader.Run(J_)`, `Tester.Run(J_)` | Program | run a station's job and wait until it is done |
| `Wait(t)` | Program, jobs | the program holds [s]: the robot in its program (a started loader job goes on), the loader in a job |
| `MoveY(L_, t=)`, `MoveZ(L_, t=)`, `Gripper(G_, t=)` | Loader_Jobs | one axis / the jaws to the constant's value in `t` [s] |
| `Test(Process.Test_Duration)` | Tester_Jobs | the MG06 test for `Cell.yaml`'s duration |

Reserved: `a=` and `r=` take only 0 or nothing (no blending yet); `Tester.Start` / `Tester.WaitDone` are refused (the
tester's job runs whole). Checked at load besides the grammar: every constant and job declared and used (a point declared in
`Optimization.yaml Waiting_Points` counts as used, the taught program may not use it, the optimized program must); the IK family changes
only across `P_VIA_HOME_MG06` and never in a MoveL; a MovePath starts where the robot stands; the segment names of
`Simulation.yaml` exist.

The **optimized program** `Data/Optimized/Program_Optimized.yaml` (F9) is written by `Example/Program/optimize_program.py`
only and committed (generated data). It has the `Program.yaml` format plus:
`Robot.MoveSpline(W_<NAME>, v=)` (a planned spline, `v` a fraction of the datasheet speeds, required), `Loader.Start(J_)` /
`Loader.WaitDone(J_)` (a job started while none runs and waited for before the next Start, Run or the end); `Waypoints`
(`W_<NAME>`: `From`, `To` points, `Points` the interior waypoints [deg]); and `Optimized` (`Inputs`: the sha256 of
`Program.yaml`, `Constants.yaml`, `Simulation.yaml`, `Cell.yaml` and `Optimization.yaml` - each of its loaded content
(`Configuration.Core.YAML_Hash`: a comment-only edit keeps it) -, of the fork path's bytes and of the compiled planning model; `Platform`; `Schedule`: each segment's start [s]; `Topp_Grid_Points`). These are refused in `Program.yaml`; its constants
are shared, so not all need be used. A build refuses it when an input changed or the schedule is not reproduced.

`Optimization.yaml` (F9; read by `Load_Optimization`, every field REQUIRED):

| Field | Unit | Source | Read by |
|---|---|---|---|
| `Speed_Fraction` | - | equal to `Cell.yaml Motion.Speed_Fraction` (a fair comparison) | the splines' `v=` |
| `Seed` | - | any fixed value (B5) | RRT-Connect, the shortcut |
| `Free_Segments` | segment keys | the approved LOCKED / FREE table (§7.11); MoveJ lines only | `Free_Transfers` |
| `Rrt.Max_Iterations`, `Extend_Step`, `Edge_Step`, `Goal_Bias` | -, deg, deg, - | chosen 2026-10-24 | `Rrt_Connect` |
| `Shortcut_Iterations` | - | chosen | `Shortcut` |
| `Spline.Check_Step`, `Margin`, `Max_Refits` | deg, m, - | chosen; measured | `Smooth` |
| `Topp_Grid_Points` | - | measured (B3) | `Time_Path` |
| `Margins` | m (increasing) | measured 2026-10-24 | `Plan_Transfer` |
| `Carry_Max_Tilt` | deg | the owner: 20 deg (no time cost), ASSUMED; held in simulation only (`KNOWN_ISSUES.md`) | `Free_Space_Cls` (B4) |
| `Waiting_Points` | transfer -> point | the robot-loader analysis (F9.8) | the transfer's legs; Constants.yaml's completeness (the point is the optimized program's only) |

## 9. Data formats and units

| Quantity | Canonical unit in code | Where another unit appears |
|---|---|---|
| length | [m] | the report prints mm |
| angle | [rad] | the report prints deg; `Cell.yaml`, `Constants.yaml` and `Simulation.yaml` carry deg, converted at load |
| time | [s] | - |
| force | [N] | - |

| Format | Read and written by |
|---|---|
| `models/scene.xml` and the vendored / tool MJCFs | `Scene` (MjSpec), never written |
| `models/cell_scene.xml` | written by `Scene` (`Export_Cell_Scene`); read by MuJoCo's viewer only |
| `Data/PCB/pcb_meta.json` | `Configuration` (`Read_PCB_Meta`), written by hand: the PCB poses (`cad_pos`, `place_pos`, `hold_pos`, `quat`) only |
| `Configuration/Cell.yaml` | `Configuration` (`Load_Cell_Configuration`), written by hand (§8) |
| `Configuration/Constants.yaml`, `Simulation.yaml`, `Program.yaml` | `Configuration.Program_Parser` (`Load_Constants`, `Load_Simulation`, `Load_Program`), written by hand (§8); `Example/Keypoints/resolve_keypoint.py` prints lines to paste, nothing writes them |
| `Configuration/Optimization.yaml` | `Configuration.Program_Parser` (`Load_Optimization`), written by hand (§8) |
| `Data/Optimized/Program_Optimized.yaml` | written by `Optimizer` (`Optimized_Text`, via `Example/Program/optimize_program.py` only), read by `Configuration.Program_Parser` (`Load_Program`) (§8) |
| the optimizer's report, the comparison table and figure | `Example/Program/optimize_program.py` (`Data/Generated/Reports/optimize_program.txt`), `Example/Program/compare_programs.py` (`Reports/compare_programs.txt`, `Images/compare_programs.png`) |
| the comparison `Data/Generated/Reports/Comparison/Comparison.md`, its figures, the videos, the point and collision images | written by `Example/Report/compare_report.py` and `make_media.py` (§7.12); never read |
| the README previews `Media/Cycle_<Taught|Optimized>.gif`, `Media/Key_Points.png` | written by `Example/Report/make_media.py` (§7.12); read by the README |
| `Data/Paths/fork_close_open_path.npz` | `Program` (`Robot.MovePath`, §7.9); recorded in the MG06 project, never written |
| a program's physics report (text) | `Simulation.Report` (`Format_Program_Report`), printed by `Example/Simulation/run_program.py`, written to `--out`/`<program>_physics.txt` (default `Data/Generated/Reports/`) |
| the physics baselines `test/Simulation/data/baseline_report_<program>.txt`, `baseline_metrics_<program>.json`, `baseline_environment.json` | `Example/Simulation/physics_baseline.py` (writes and compares) |
| the logo variants `Media/<Logo>_<Black|White>.svg` | written by `Example/Media/make_logo_variants.py` from `Media/Original/`; read by the README |
| the model fingerprint (JSON) | `Example/Scene/model_fingerprint.py` (`Model_Fingerprint`, `Compare_Fingerprints`) |
| `logger/<script>.log`, `logger/MUJOCO_LOG.TXT` | written by `Configuration` (`Script_Log_Cls`, `Configure_MuJoCo_Log`) and MuJoCo; never read |
| the trajectory file `Data/Generated/Trajectories/<program>.npz` (+ `.csv`, a write-only export) | `Trajectory` (`Write_Trajectory`, `Read_Trajectory`); written by the program build (F8). Per sample (4 ms, `CONST_DT`): `t` [s], `robot_q[6]` [rad], `robot_qd[6]` [rad/s], `loader_y`, `loader_z`, `gripper` [m, commands], `vacuum` [0/1], `step_index`, `step_label` (a step is a segment `S<n>_<Name>`, or `End_Settle`), `segment_id` (one executed command), `pcb_location` (index into the five locations), `pcb_pose[7]` [m, quaternion w first], `mg06_lid`, `mg06_handle` [rad], `jaw` [m, the jaws' real position]; per segment (one command): `segment_step`, `segment_command`, `segment_start`, `segment_end` (sample indices, end exclusive); `step_names`; `events` and `meta` (JSON: format version 1, dt, the program, the inputs' sha256 - program, constants, simulation, cell (their loaded
content: `Program.Core.Input_Hash`), fork path (its bytes) -, git commit, units, joint order, TCP). Deterministic: the same trajectory gives the same bytes |

Scripts that write take `--out DIR`.

## 10. Tests

Layout per §6; `pytest.ini` with `--import-mode=importlib` and the `slow` marker; pytest-xdist runs the suite in parallel
(`-n 8`), and it passes serially too. Per commit `.venv/bin/python -m pytest -n 8 -m "not slow"` (426 tests, about 35 s;
2 min serially); per accepted feature the full `.venv/bin/python -m pytest -n 8` (446 tests, about 3 min; 7.5 min serially;
Apple M2 Pro, 2026-10-25). The two tiers of checks: `CLAUDE.md`.
Expected numbers come from §4 (measured) or the brief's criteria, never from the code under test.

### 10.1 Acceptance criteria (brief §5.6)

A cycle run passes when:

1. no robot / loader collision: the five contact categories of §4.6 report no contact;
2. 4 of 4 cups touch the PCB at both picks;
3. PCB in MG06: position error < 0.5 mm and tilt < 0.2°;
4. PCB in the output crate: < 2 mm from the crate centre, horizontally;
5. MG06 ends open: lid within 2° of its open stop (−39.96°) and handle within 2° of 0° (this reading
   of "ends open" is a decision, design-decisions);
6. no instability (exit 2 with the evidence), and the PCB is never lost (§7.6; the run stops at the loss);
7. deepest PCB penetration ≤ 0.5 mm in every group (MG06 holder, crate, loader jaws).

A program's run (F11, owner 2026-10-24; `Simulation.Check_Program_Acceptance`) also passes when:

8. every vacuum hold keeps 4 of 4 cups on the PCB from vacuum on to off - after the jaws open at the take and in every carry
   (the report gives each hold's minimum and when);
9. the PCB is never lost (§7.6);
10. every `WaitDI(DI_VACUUM_OK)` gets its input in time: the simulated vacuum switch is true once `Vacuum_OK_Min_Cups` cups have
    touched the PCB for its `Signal_Response`; checked where the WaitDI ends on the trajectory's timeline (physics follows it),
    a false input stops the run (`vacuum not OK at t, step, line: ...`, exit 1);
11. in every free transfer the robot and the carried PCB stay at least `Acceptance.Loader_Distance` (1 mm) from the loader:
    the hard floor;
12. the PCB's slip in the cups at most `Acceptance.Slip_Max` (1.98 mm) and 13. the robot's joint tracking error at most
    `Tracking_Max` (0.0189 rad) - 1.5 x the legacy run's (step 6, owner 2026-10-25; a step above them is also a warning);
14. the fork's net contact force on the handle bar at most `Fork_Net_Force_Max` (290 N) and 15. the handle hinge load at most
    `Handle_Load_Max` (13.5 N m) - the worst of the force's two modes x 1.15 (design-decisions); the summed normal force is
    reported only (it adds the squeeze between the slot's walls). Both are **regression bounds derived from the simulation**,
    not safety limits: no rating of the real handle, fork or VGC10 is known (owner 2026-10-25: model-based until AUBO and
    OnRobot answer; KNOWN_ISSUES.md). The report names them so (`(sim. regression bound)`) and says it under the table;
16. in every free transfer the robot and the carried PCB at least `Clearance.Loader_Free` (10 mm) from the loader (closer is
    also a warning);
17. no robot actuator saturated: each joint's largest torque below 99.9 % of the model's actuator limit (180 / 180 / 150 /
    60 / 60 / 60 N m, the vendored model's estimates; the AUBO i5's official limits are not known, KNOWN_ISSUES.md) - also a
    model-based bound, not a safety limit.

The loader's place into MG06 and the gripper operations around it (B work, owner 2026-10-25; `Simulation.Place_Checks`, the
bounds in `Cell.yaml Acceptance`):

18. from reaching `L_PLACE` to the open command the jaw gap never shrinks (by more than 0.01 mm, their contact compliance;
    measured 0.0000-0.0002 mm): the jaws only open;
19. while the jaws are closed at `L_PLACE` the PCB - MG06 holder contact force at most `Place_Holder_Force` (1.0 N): no press;
20. after the jaws release the PCB (their last contact with it) the deepest PCB - holder penetration at most
    `Release_Penetration` (0.1 mm: the margin runs reach 0.068 mm, the old pressed place 0.157 / 0.181 mm) while it drops, and 21. settled at most `Rest_Penetration` (0.0002 mm, measured at rest under
    its own weight) + `Settle_Margin` (0.01 mm);
22. from the travel height to the release the PCB touches none of the holder's side guides and pins (every holder geom but its
    four supports; the closest clearance is reported);
23. seated: `Seat_Time` (0.2 s) after the release the PCB rests on at least `Seat_Supports` (3) of the holder's four supports
    (`col_holder_03`, `04`, `06`, `10`: the holder geoms the centred PCB touches at its static seat), at least that many of them
    within `Seat_Height` (0.05 mm), slower than `Seat_Speed` (1 mm/s), tilted less than `Seat_Tilt` (0.1°) from its place pose,
    touching no side guide and no jaw;
24. the handover clamp's centring of the PCB along the jaw axis at most `Handover.Lateral_Max` (0.5 mm);
25. the re-grip's PCB-ledge gap (0.2 s after its clamp) within `Handover.Ledge_Gap` ± `Ledge_Tolerance`.

Further criteria: every keypoint can be shown in the viewer, and an invalid keypoint (IK fails,
clearance < 3 mm) is reported clearly; the PCB is green; the README explains installation, the
commands, editing keypoints and program, the file formats.

### 10.2 Model and report equivalence

- The compiled model's fingerprint equals `test/Scene/data/baseline_model_fingerprint.json` except
  `geoms/pcb_vis/geom_rgba` (the green PCB).
- Pinned by `test/Scene/test_core.py` (fingerprint).
- **Golden data of the taught program** (`test/Program/data/`: the trajectory arrays' sha256, the cycle time, the events, the
  segments, every command's values as float hex, the check report, the point table), written and compared by
  `Example/Program/golden_program.py` (`--out DIR --source TEXT` re-captures), pinned by `test/Program/test_golden.py`. A
  change to the program or its build re-captures it in its own commit, listing what changed.
- **Physics baselines of the programs** (step 6, owner 2026-10-25): `test/Simulation/data/baseline_report_taught.txt` and
  `baseline_report_optimized.txt` (the full report of `Example/Simulation/run_program.py`), `baseline_metrics_<program>.json`
  (the compared metrics) and `baseline_environment.json` (the platform, the versions, the commit), written and compared by
  `Example/Simulation/physics_baseline.py` (`--out DIR` re-captures, in its own commit with the reason), pinned by
  `test/Simulation/test_baselines.py` (slow). A run FAILS on any §10.1 criterion and on a metric beyond its drift tolerance:
  the cycle time, the fault, the cups at vacuum on and the fewest cups per hold exact; the vacuum events and the holds +-1
  sample (4 ms); the PCB's placed error, final position and penetrations +-0.05 mm, its placed and final tilt +-0.02 deg; MG06's
  closed and end angles +-0.1 deg; the slip +-0.2 mm; the tracking error, the loader lag, the fork's net force, the handle hinge
  load and the joint torques +-10 %; the free transfers' closest loader distance +-0.5 mm; the place into MG06's metrics
  (`physics_baseline.CONST_TOLERANCES` lists every metric's tolerance). The report's diff is printed for
  information and never fails (contact physics changes with sub-micrometre differences, design-decisions).
- **The fork's envelope** (owner, 2026-10-25): the fork's contact with the handle bar is
  chaotic, so its net force, the handle hinge load and the J2 / J3 torques are compared with an envelope, not with the
  baseline run: each program's metrics hold `envelope` (per metric [min, max]) and `envelope_runs` over `CONST_PERTURBATIONS`
  - the PCB's start along x -5 / -1 / 0 / +1 / +5 µm without and -5 / 0 / +5 µm with a 0.1 s settle wait before the closing
  path, and the MG06 handle hinge ±1 µrad (10 runs; `--out` refuses to write when one of them fails §10.1). A run passes inside its program's
  envelope ± 10 %, and always within §10.1 (the fork's 290 N / 13.5 N m, the actuators below 99.9 %). The contact count
  (`fork_points`) is reported only. The runs are independent, so the script runs them in a pool of processes; the results,
  in the runs' order, are byte-identical to running them one after the other.
- **Baselines by run** (owner, 2026-10-24, B6; 2026-10-25): one baseline per program, measured in the pinned environment.
- **Determinism** (B5): a program build gives byte-identical trajectory files for the same inputs and commit on one
  machine and platform (`test/Program/test_core.py::test_the_build_is_deterministic`); across platforms results agree
  within numeric tolerance only. The trajectory's metadata records the platform and the versions.

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

The personal skills and their project layers: `CLAUDE.md` > Skills.

## 13. Risks and how they are handled

| # | Risk | Handling |
|---|---|---|
| 1 | A restructure silently changes the model | the fingerprint of the compiled model (§10.2) |
| 2 | A restructure silently changes a result | the fingerprint, the kinematic golden and the physics baselines (§10.2); the outputs compared byte for byte (project-optimize reference) |
| 3 | Instability hidden by MuJoCo's state reset | `d.time` going backwards fails the run (exit 2) |
| 4 | Platform drift of the numbers (Python / numpy version) | pinned requirements; re-baseline in the pinned environment (§5, §10.2) |
| 5 | The physics loop drifts | the programs' physics baselines: SPEC §10.1 and the metrics' drift tolerances (§10.2); a refactor proven by an empty report diff on one machine; the legacy run: commits `f754e2d0379b06ffb43286f4fb7aabf88e25ca5f` and `7a8560c8c16806739a02da8e989c2a2f65d1e41d` |

## 14. Open questions

1. The real-cell validation procedure (§17, `KNOWN_ISSUES.md`): not defined yet.

## 15. Sources

- Owner's project brief "KINALI Test-it-off – Automated PCBA Testing Cell (MuJoCo)" (its sections mapped to this SPEC: git
  `e7739f4:.claude/project-docs/FEATURES.md` > Brief coverage), git
  `427e076:docs/PROJECT_BRIEF.md` (2026-10-24).
- The previous Czech description of the cell (git `f754e2d0379b06ffb43286f4fb7aabf88e25ca5f:docs/README_cs_mg06_cell.md`,
  originally `5e5ee62:README.md`); its content is in §4.10, KNOWN_ISSUES and design-decisions.
- AUBO i5 datasheet (joint speeds, payload 5 kg, reach 886.5 mm); the joint speeds as quoted in `Configuration/Cell.yaml` (`Motion`).
- OnRobot VGC10 datasheet (20–60 % vacuum recommended), as quoted in `Configuration/Cell.yaml` (`Vacuum`).
- KINALI Test-it-off: https://www.testitoff.cz/en/about-test-it-off/

## 16. (Removed: the brief's proposed file formats and CLI, settled in §8, §9 and the `Example/` scripts; the brief's text is
in git `427e076:docs/PROJECT_BRIEF.md` §5.2–§5.5.)

## 17. Future scope: real-cell transfer

The project is simulation only today. Keypoints and programs are intended to be transferred to the real
AUBO i5 later (owner, 2026-10-24); a test on the real cell is planned. Consequences:

1. **Controller-friendly formats.** The keypoint, program and trajectory formats state
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
4. **The program a controller runs is separate from the simulation's data**: `Program.yaml` and `Constants.yaml`
   hold only what the controllers run and the named values they use; contacts, scenes, simulated signal times and the end
   settle are in `Simulation.yaml`. Open for the transfer (`KNOWN_ISSUES.md`): the I/O map (the signals' `Port` is a
   placeholder), the `Pose` RPY convention against the AUBO controller's, station user frames, and the recorded paths'
   accelerations.

