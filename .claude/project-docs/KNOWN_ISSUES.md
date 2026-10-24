# Known issues

**Outputs of this project (keypoints, programs, trajectories) are simulation results and are NOT
validated for driving the real cell.** Before any use on real hardware they must go through the
real-cell validation procedure (to be defined), with reduced speed and the robot controller's own safety
limits active.

This file lists what the project does not model or has not verified. It contains only what has been
decided or measured.

## Real-cell validation procedure not defined

A transfer of keypoints and programs to the real AUBO i5 is planned (SPEC §17), but no procedure exists
to validate an output on the real cell.

- **Impact:** no output may be run on the real cell; the simulation's agreement with the real cell is
  unknown.
- **Evidence:** SPEC §17; owner, 2026-10-24.
- **What would close it:** a defined procedure covering the calibration of the robot base, MG06 and
  loader positions against the model (the CAD deviations of SPEC §4.7 included), the real speeds and
  accelerations (the acceleration limits of SPEC §4.4 are assumed), the vacuum and gripper behaviour
  (the adhesion model, the jaw force), and the collision margins (the 3 mm planning clearance), run
  first at reduced speed with the controller's safety limits active.

## Real-cell verification pending (no access)

There is no access to the real cell; the project is simulation only for now (owner, 2026-10-24). Once access exists,
these are compared on the real AUBO i5 controller, and this entry is removed when the comparison is done:

- **The pose of one point** - `P_HOME` read on the controller (its joints and its pose display) against
  `Configuration/Constants.yaml`'s `Pose`: the RPY convention. This project writes orientation as fixed-axis roll, pitch,
  yaw in degrees, R = Rz(Y)·Ry(P)·Rx(R), each angle in (-180, 180] (`Kinematics.RPY_To_Matrix`, F10); the controller's
  axis order, intrinsic or extrinsic reading and units are not checked, so the same three numbers may mean another
  orientation there.
- **The pendant's TCP setting** against `T_SUCTION` (the cup face 260.6 mm from the flange along its Z, SPEC §4.3).
- **The joint values** - each joint's zero and positive direction and the joint order: the vendored MJCF
  (ian-chuang/Manipulator-Mujoco) defines them (shoulder lift, wrist 1 and wrist 3 about −Y, SPEC §4.3); whether they equal
  the controller's convention is not checked (F6).
- **The robot base pose** - `Cell.yaml` `Robot.Base_Position` and `Base_Yaw` against the real mounting.

- **Impact:** a point copied to the controller as joint values or as a pose could command a different pose (SPEC §17).
  The simulation itself is not affected: FK of the joints is checked against `Pose` with this project's convention.
- **What would close it:** the four comparisons above, recorded in SPEC §8 (the convention, or a conversion) and §4.2.

## The crate penetration is −0.14 mm in this environment, −0.08 mm in the brief's reference

The brief's reference result (produced on the original machine of `mg06_cell_mujoco`, environment not
recorded) gives the deepest PCB penetration into the crate as −0.08 mm, the previous Czech README (git `legacy-planner-reference:docs/README_cs_mg06_cell.md`) −0.07 mm;
every run here gives −0.14 mm at t = 62.46 s. Every other number of the report is identical.

- **Impact:** none on acceptance (bound 0.5 mm, SPEC §10.1); it shows that the report is reproducible
  only per environment, so the baseline is per environment.
- **Evidence:** `test/Simulation/data/baseline_report.txt` against git `427e076:docs/PROJECT_BRIEF.md` §3 (SPEC §4.6).
- **What would close it:** the original environment's versions; not expected to be available.

## MuJoCo warns about an integrator conflict when the robot is attached

Every build of the cell makes MuJoCo log, print on stderr and raise as a Python UserWarning:

```
WARNING: Attach conflict when attaching 'aubo_i5' to 'mg06_loader_scene', policy is 'warning'
integrator: parent has 0 (default), child has 3, keeping parent value
```

The cause is `<option integrator="implicitfast"/>` (line 4) of the vendored robot MJCF
`models/aubo_i5/aubo_i5.xml`, while `models/scene.xml` sets no integrator (Euler, the default). MuJoCo
keeps the parent's Euler; the verified physics was run that way, so the warning describes the verified
state. Not fixed on purpose: changing either file changes the physics or the vendored model. MuJoCo's log
file goes to `logger/MUJOCO_LOG.TXT` (`Configuration.Core.Configure_MuJoCo_Log`, called by every script
and the test suite), never into the working directory.

- **Impact:** noise on stderr and in the test summary's warnings; not a fault.
- **Evidence:** `MjLogConfig` probe 2026-10-24: the message is logged at the attach in the scene build.
- **Pinned by:** `test/Configuration/test_core.py::test_mujoco_logs_into_the_configured_file_never_the_working_directory`.
- **What would close it:** the owner's decision on the integrator, with a new baseline (a physics change).

## The MG06 test time is a placeholder

`Configuration/Cell.yaml` `Process.Test_Duration` is 2.0 s (owner, 2026-10-24); the real test time of the customer's PCB on
the MG06 is not known.

- **Impact:** the default program's cycle time (and F9's) includes this 2.0 s; a real test time changes it one for one.
- **What would close it:** the measured test time of the real board.

## PATH_FORK_OPEN starts with a velocity step

The recorded opening path is joined at a row where the flange already moves at 39.1 mm/s, after a MoveL that ends at rest
(design-decisions). The trajectory's joint speeds jump there.

- **Impact:** the servo's tracking error and the fork's force on the handle bar at the join are not known for the program yet
  (the reference plan has the same step and passed, SPEC §4.6).
- **Evidence:** `Program.Build_Program`'s note; `Data/Paths/fork_close_open_path.npz` rows 662-665.
- **What would close it:** step 5's run of the default program reports the tracking error and the bar force there.

## The recorded fork paths exceed the assumed acceleration limits

The fork paths, replayed at their recorded timing, reach 2.03x the assumed joint accelerations (`PATH_FORK_CLOSE`), and
`PATH_FORK_OPEN`'s join is a velocity step from rest to 39.1 mm/s (8.44x the assumed acceleration); the limits themselves are an
assumption (SPEC §4.4). The program check reports these peaks and does not limit them (owner, 2026-10-24).

- **Impact:** for the real-cell transfer (SPEC §17) the recorded timing is not executable as is: a real controller needs a
  ramped join into `PATH_FORK_OPEN` and the paths limited to the real joint accelerations. The simulation ran them (SPEC §4.6).
- **What would close it:** the real AUBO i5's acceleration limits, and the paths re-timed (ramped join, limited path) to them.

## The I/O ports of the program's signals are placeholders

`Configuration/Constants.yaml` declares `DO_VACUUM` and `DI_VACUUM_OK` with `Port: "placeholder, real-cell I/O map not
defined"` (F10). The simulation drives them through `Simulation.yaml` (`Actuators`, `Signal_Response`), which the ports do
not affect.

- **Impact:** the program cannot be transferred to the real cell's controller until every signal has its real port.
- **What would close it:** the real cell's I/O map; each `Port` then names its controller channel.

## MoveSpline's execution on the AUBO controller is not verified

F9's `Robot.MoveSpline(W_<SEGMENT>)` (only in `Data/Optimized/Program_Optimized.yaml`, owner, 2026-10-24) is a
time-parametrized sequence of joint waypoints: the simulation plays it as joint targets every 4 ms. How the AUBO i5
controller would execute it - streamed joint targets at a fixed rate, or MoveJ through the waypoints with blending - is not
known; the two differ in path, timing and the limits that apply.

- **Impact:** the optimized program cannot be transferred as is; its cycle time and clearances hold for the simulated
  execution only.
- **What would close it:** the controller's interface for time-parametrized joint motion verified before any transfer
  (SPEC §17), and the optimized program's check repeated with that execution model.

## No station user frames

Every `Pose` is in the robot base frame and every `Offset` in its Reference point's tool frame (F10). The real cell will
usually teach a station (a crate, the loader, MG06) in its own user frame, so a station that moves needs only its frame
re-taught.

- **Impact:** a moved station means re-resolving every point that depends on it (`Example/Keypoints/resolve_keypoint.py`
  per point).
- **What would close it:** a future step that adds user frames to `Constants.yaml` (a frame per station, points and offsets
  given in it), with the owner's decision on their names and how they are taught.

## Simplifications of the model

What the simulation does not model; whoever compares it with the real cell checks these first.

- The vacuum is a constant force per touching cup (7 N): no leaks, no build-up of the vacuum (valve,
  ejector).
- Robot joint torques and armature are estimates; the acceleration limits are assumed (SPEC §4.4, §4.10).
- Cables and the robot pedestal (except its top plate) are not in the collision model.
- The loader's masses are estimates (SPEC §4.10).
- The PCB's components are one box in collision.
- **Impact:** the comparison with reality is mainly the structure of the cycle; the absolute 65.8 s depends
  on the chosen 60 % speed and the assumed accelerations.
- **What would close it:** the real-cell validation procedure (item above), measured joint data.

## The JIC Smart Factory logo is a recoloured copy, not an official monochrome file

`Media/JIC_Smart_Factory_Black.svg` and `_White.svg` are recolourings of the official purple vector (path
and viewBox unchanged), a temporary project departure from jic-sf-ui-style ("never recoloured"), accepted
by the owner (2026-10-24, design-decisions: the brand). An official monochrome JIC file has been requested
from JIC.

- **Impact:** the JIC logo in the README header (and later the Phase 2 header) is not the brand's own file.
- **Pinned by:** `test/Example/Media/test_make_logo_variants.py` (one colour, original geometry).
- **What would close it:** the official monochrome file from JIC replaces the recoloured one (in
  `Media/Original/`, made into `_Black` / `_White` by the script, or used as it is).

## The PCB is one green mesh: board and components are not told apart

`pcb_vis` is a single mesh; components are drawn in the same green as the board.

- **Impact:** visual only; the collision geoms are separate boxes and unaffected.
- **What would close it:** splitting the mesh into the board and the parts above the board top, and
  colouring the components grey/black (brief §7 step 2, optional follow-up, `FEATURES.md`).
