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

Since 2026-10-24 (owner) the scripts and the tests show it as one line on stderr instead -
`[INFO] Known: robot MJCF integrator ignored, scene uses Euler (KNOWN_ISSUES)` - by `Scene.Compile_Spec`, which matches
exactly this message (at the attach and at the compile) and passes any other warning on; the model is unchanged.

- **Impact:** none: one INFO line per compiled model; not a fault.
- **Evidence:** `MjLogConfig` probe 2026-10-24: the message is logged at the attach in the scene build.
- **Pinned by:** `test/Configuration/test_core.py::test_mujoco_logs_into_the_configured_file_never_the_working_directory`,
  `test/Scene/test_core.py::test_the_known_attach_warning_becomes_one_info_line`, `::test_any_other_warning_is_still_shown`.
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

## The carried PCB's tilt limit is an assumption

`Optimization.yaml Carry_Max_Tilt` is 20 deg (owner, 2026-10-24: it costs no time - the cycle 57.108 s against 57.132 s at the
first, assumed 30 deg; To_Output then tilts the board 17.73 deg in 1.768 s instead of 28.98 deg in 1.792 s). No measurement of
what the four cups hold while the board tilts and accelerates exists.

F11's physics run (2026-10-25) confirms it in the simulation: the optimized carries slip at most 1.28 mm (To_Loader, tool tilt
up to 10.5 deg) and 1.14 mm (To_Output, up to 16.8 deg and 4.72 m/s^2) with 4 of 4 cups, below the 1.98 mm warning (owner: the
value stays).

- **Impact:** a planned transfer may tilt the board more than the vacuum holds on the real cell; the simulation's adhesion
  model (a constant force per cup while in contact) is not the real cup.
- **What would close it:** a real-cell value.

## No station user frames

Every `Pose` is in the robot base frame and every `Offset` in its Reference point's tool frame (F10). The real cell will
usually teach a station (a crate, the loader, MG06) in its own user frame, so a station that moves needs only its frame
re-taught.

- **Impact:** a moved station means re-resolving every point that depends on it (`Example/Keypoints/resolve_keypoint.py`
  per point).
- **What would close it:** a future step that adds user frames to `Constants.yaml` (a frame per station, points and offsets
  given in it), with the owner's decision on their names and how they are taught.

## The vacuum's adhesion holds only while a cup touches: the bellows' preload can throw the board off

MuJoCo's adhesion actuator pulls only through active contacts (the cups and the PCB have no margin, SPEC §4.7). At the take
from the loader the 1.5 mm press compresses the bellows by about 0.9 mm (3000 N/m: 2.7 N per cup, 10.8 N in all) while
the jaws hold the board; when the jaws open, that force throws the ~90 g board off the cups at about 120 m/s² and, once a
cup face has left it, nothing pulls it back (F11, 2026-10-24: the taught program lost the board at 58.375 s, the optimized
one in the next carry; the verified legacy run held it on 2 of 4 cups by chance - moving its start by -10 or +50 µm loses it).
A real vacuum cup holds through a small gap and would likely keep the board. The take therefore relieves the press first (back
to touch, `P_LOADER_TAKE_TOUCH`, SPEC §4.7): conservative real-cell practice as well.

- **Impact:** a program that opens a support under a pressed board loses it in the simulation; the real cell may forgive it.
- **What would close it:** a vacuum model with a holding range, or a real-cell observation of the take.

## The board sags in the jaws after the re-grip from MG06

Measured in physics at the take (F11, 2026-10-24, 10 ms before the cups touch): the board lies 0.7-0.8 deg about its X axis
and about 0.16 mm lower along its normal than the nominal hold pose (taught 0.74 deg / -0.153 mm, optimized 0.74 deg /
-0.159 mm, legacy 0.84 deg / -0.165 mm); the jaws touch it only along the front ~38 mm of its edges (board y -39.6 to -2 mm),
so the rear half sags. The 1.5 mm press then compresses the front cups by 0.93-1.15 mm and the rear ones by 0.49-0.72 mm.
The take is not re-taught to it (owner): with the press relieved before the jaws open the board stays on 4 of 4 cups.

- **Impact:** the kinematic model (the program check, the point table) poses the board at the nominal hold pose, 0.7 deg
  from where physics settles it; the press depths it reports at the take are nominal.
- **What would close it:** a real-cell measurement of the board in the jaws after the re-grip, or a jaw model that clamps
  the whole edge.

## The fork's force on the handle bar has two modes

The recorded closing path presses the fork into the handle bar (up to 0.54 mm, SPEC §4.7). At that stiff contact the fork
touches the bar at 4 or at 6 points, and sub-micrometre state differences decide which: the PCB's start moved +1 µm, or a wait
of 4 ms or more before the path, flips it (F11, 2026-10-25; not the robot's settle state - the optimized robot stands still
5 s before the path, the taught one still moves, and either mode occurs in both). Over the 14-run sweep (start -5 to +5 µm,
waits 0 / 4 ms / 0.1 s, both programs): 4 points (11 runs) - net force on the bar 229 N, handle hinge load 9.4 N m (summed
normal force 291 N); 6 points (3 runs) - 247 N, 11.5 N m (371 N). The summed force adds the squeeze between the slot's
opposing walls (37 N), so the bounds use the net force and the hinge load (SPEC §10.1). Retracting the recording off the bar
removes the second mode and lowers the force (0.55 mm: 187 N, 8.2 N m), but only 0.55-0.60 mm latch MG06; 0.65 mm already
stops the handle at 25.5 deg, short of the toggle's dead point (design-decisions). The recording stays. The real MG06 handle's
force limit is not known.

- **Impact:** the fork force is one of two values; the bounds cover the higher one; the real handle could be overloaded by a
  force the simulation accepts. The physics regression accepts either mode against that mode's own baseline, with a warning
  naming it, and fails a third (SPEC §10.2); the report names the mode (`(4 contact points)`).
- **What would close it:** the MG06 maker's handle force limit; a fork path re-recorded so that it does not press into the bar.

## The simulated fork-handle contact is stiff; the real robot is not

The fork closes MG06 position-controlled through a stiff contact without compliance: in the simulation the servos follow
the recorded path while it presses 0.54 mm into the bar (net force up to 247 N on the bar, a wrench of up to 239 N and
16.1 N m at the tool flange, joint torques up to 35 % of the model's actuator limits; F11, 2026-10-25). Passing the toggle's
dead point needs about 156 N on the bar in the simulated MG06 (at 27.5 deg of handle, the same with and without the
overlap). On the real cell the fork, the VGC10 and the robot would deflect, and the AUBO's collision detection may stop
the robot. The AUBO i5's joint torque limits and collision thresholds and the VGC10's force and moment ratings are not
known (no official figure found: the i5 manual v4.5.13 found online is a scanned copy without its tables, the VGC10
datasheet gives the payload only, 6 kg with default cups, 15 kg with custom attachments).

- **Impact:** the closing may not work, or may trip the collision detection, on the real cell, although it passes here.
- **What would close it:** the closing tried on the real cell at reduced speed with force limits (collision detection at a
  low level) active; the vendors' torque, force and moment ratings.

## The VGC10 interface carries about 1.6 x its payload's weight while the fork closes MG06

The main real-cell risk of the closing (owner, 2026-10-25). In the simulation the tool flange, and so the VGC10's
interface with the fork attachment, carries a wrench of up to 239 N and 16.1 N m while the fork presses the handle bar
(the taught and the optimized program, F11, 2026-10-25). 239 N is about 1.6 x the weight of the VGC10's largest payload
rating, 15 kg with custom attachments (147 N); the datasheet gives the payload only, no force or moment rating of the
interface, so the 16.1 N m has nothing to be compared with. The acceptance bounds on the fork (`Fork_Net_Force_Max`,
`Handle_Load_Max`) are regression bounds derived from the simulation, not this rating (SPEC §10.1).

- **Impact:** the VGC10, its attachment or the fork could be overloaded on the real cell although the run passes here; the
  simulated values are of a stiff contact (the entry above) and may differ on the real cell either way.
- **Evidence:** the flange wrench measured in the F11 fork-force investigation (2026-10-25; the entry above; the
  baselines pin the net force and the hinge load, not the wrench); the VGC10 datasheet (payload 6 kg with default cups,
  15 kg with custom attachments). The owner will ask OnRobot and AUBO for the ratings.
- **What would close it:** OnRobot's force and moment ratings of the VGC10 interface (and AUBO's of the i5's flange and
  joints), the bounds replaced by them or by a margin under them; or the closing measured on the real cell with a force
  sensor or the controller's force estimate, at reduced speed.

## Real-cell transfer: a controller waits for "in position" before a LOCKED operation

The optimized program's splines end where a LOCKED operation begins (the crate pick, the handover, the place) with up to
1 mm of residual tool error (0.91 / 0.41 / 1.00 mm; the taught MoveJs 0.19 / 0.13 / 0.25 mm), measured in physics F11,
2026-10-25. In the simulation it has no effect: each LOCKED step starts with a 15 mm approach, and the contact forces in those
steps are the same in both programs (within 0.3 N). A real controller would wait for "in position" (every joint's error and
speed below its threshold) before the next operation; the program does not (owner: not now).

- **Impact:** on the real cell the optimized cycle may be longer by the in-position waits (about 0.1 s per join, estimated).
- **What would close it:** the controller's in-position thresholds, and the rule in the optimizer if they cost time.

## The loader's actuator model is one load case

`Program.Core.Loader_Actual` (the kinematic checks' loader, F11) takes each axis' mass and gravity load at the trajectory's
start; in physics the z load changes with the PCB in the jaws and the jaws' contacts, which leaves up to 1.04 mm (y 0.17 mm),
measured on the optimized program 2026-10-25. `Clearance.Tracking_Margin` covers it.

- **Impact:** a free transfer measured within about 1 mm of the rule may be closer in physics.
- **What would close it:** the load per sample (the PCB's location), if a margin of 1 mm ever matters.

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
