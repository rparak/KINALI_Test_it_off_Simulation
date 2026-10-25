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
state. Not fixed on purpose: changing either file changes the physics or the vendored model. The scripts and the tests
show it as one line on stderr, `[INFO] Known: robot MJCF integrator ignored, scene uses Euler (KNOWN_ISSUES)`
(`Scene.Compile_Spec` matches exactly this message, at the attach and at the compile, and passes any other warning on;
owner, 2026-10-24); MuJoCo's log file goes to `logger/` (design-decisions: logs).

- **Impact:** none: one INFO line per compiled model; not a fault.
- **Evidence:** `MjLogConfig` probe 2026-10-24: the message is logged at the attach in the scene build.
- **Pinned by:** `test/Configuration/test_core.py::test_mujoco_logs_into_the_configured_file_never_the_working_directory`,
  `test/Scene/test_core.py::test_the_known_attach_warning_becomes_one_info_line`, `::test_any_other_warning_is_still_shown`.
- **What would close it:** the owner's decision on the integrator, with a new baseline (a physics change).

## The MG06 test time is a placeholder

`Configuration/Cell.yaml` `Process.Test_Duration` is 2.0 s (owner, 2026-10-24); the real test time of the customer's PCB on
the MG06 is not known.

- **Impact:** both programs' cycle times include this 2.0 s; a real test time changes them one for one.
- **What would close it:** the measured test time of the real board.

## The recorded fork paths exceed the assumed acceleration limits

The fork paths, replayed at their recorded timing, reach 2.03x the assumed joint accelerations (`PATH_FORK_CLOSE`), and
`PATH_FORK_OPEN`'s join is a velocity step from rest to 39.1 mm/s (8.44x the assumed acceleration); the limits themselves are an
assumption (SPEC §4.4). The program check reports these peaks and does not limit them (owner, 2026-10-24).

- **Impact:** for the real-cell transfer (SPEC §17) the recorded timing is not executable as is: a real controller needs a
  ramped join into `PATH_FORK_OPEN` and the paths limited to the real joint accelerations. The simulation runs them as
  recorded (design-decisions: PATH_FORK_OPEN keeps the recorded velocity step).
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

`Optimization.yaml Carry_Max_Tilt` is 20 deg (owner, 2026-10-24; design-decisions). No measurement of what the four cups hold
while the board tilts and accelerates exists. The physics confirms it in the simulation: the optimized carries slip at most
1.28 mm (To_Loader, tool tilt up to 10.5 deg) and 1.31 mm (To_Output, up to 16.8 deg and 4.71 m/s^2) with 4 of 4 cups, below the
1.98 mm warning (the physics baseline; owner: the value stays). Not done (owner, 2026-10-25): an acceleration limit for the
carrying transfers.

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
cup face has left it, nothing pulls it back (measured 2026-10-24: both programs lost the board this way). A real vacuum cup
holds through a small gap and would likely keep the board. The take therefore relieves the press first (back
to touch, `P_LOADER_TAKE_TOUCH`, SPEC §4.7): conservative real-cell practice as well.

- **Impact:** a program that opens a support under a pressed board loses it in the simulation; the real cell may forgive it.
- **What would close it:** a vacuum model with a holding range, or a real-cell observation of the take.

## The real loader gripper's behaviour is not verified

The simulated SCHUNK EGI 040-EI centres the PCB with stiffly synchronized jaws at 70 N (the data sheet's maximum) and 100 mm/s
at most; the loader releases the PCB 0.15 mm above its MG06 seat (L_PLACE) and re-grips it 0.3 mm above the finger ledges
(L_FETCH) (B work, 2026-10-25; design-decisions).

- **Impact:** the real gripper's centring, its force and speed settings and the real release and re-grip heights may differ;
  the place, the seating and the re-grip pass here on the model's values.
- **What would close it:** on the real cell: the gripper's configured force and speed, its centring of a board placed off-centre,
  the PCB's height above its seat at the release and above the ledges at the re-grip.

## The handover relies on a simulated PCB position on the cups

The handover point is taught 0.4 mm along the jaw axis so that the PCB, where it sits on the cups in physics, is on the jaw
centreline (design-decisions: the handover). The simulated bellows have no
lateral compliance (the real ones do), so the cup loss on centring is partly a model limitation. The gripper centres only along
the jaw axis (tool x): moving the PCB on the cups after the pick by +-0.2 / +-0.5 mm there keeps 4/4 cups (centring up to 0.52
mm). Across it (tool y) nothing is centred: +0.5 mm gives a PCB-in-MG06 error of 0.64 / 0.76 mm (fails the 0.5 mm item) and
0.001 mm side-guide clearance; +0.2 mm passes (0.49 / 0.37 mm error); -0.5 mm passes (the programs' own -0.30 / -0.17 mm y error
reduces). Measured limit along tool y: about +0.2 / -0.5 mm with that offset, about +-0.3 mm without it.

- **Impact:** the real handover point and the PCB's position on the cups cannot be taken from the simulation.
- **What would close it:** the handover point taught on the real cell with real boards and the real centring tolerance verified;
  the pick in the input crate accurate to about +-0.3 mm along tool y, or a y-locating feature in MG06 or the jaws.

## The fork's force on the handle bar is chaotic

The recorded closing path presses the fork into the handle bar (up to 0.54 mm, SPEC §4.7). At that stiff contact the fork
touches the bar at 4 or at 6 points, and sub-micrometre state differences decide which: the PCB's start moved +1 µm, or a wait
of 4 ms or more before the path, flips it (2026-10-25; not the robot's settle state - the optimized robot stands still 5 s
before the path, the taught one still moves, and either outcome occurs in both). The envelope runs (SPEC §10.2): 4 points -
net force on the bar 229.0 N, handle hinge load 9.38 N m; 6 points - 247.3 N / 11.45 N m in the taught program, 222.2 N /
10.01 N m in the optimized one. The summed normal force adds the squeeze between the slot's opposing walls, so the bounds use
the net force and the hinge load (SPEC §10.1). Retracting the recording off the bar was tried and not adopted
(design-decisions: the recorded closing path stays). The real MG06 handle's force limit is not known.

- **Impact:** the outcomes are not well-defined modes; the bounds cover the highest one measured; the real handle could be
  overloaded by a force the simulation accepts. The physics regression compares the fork's metrics with an envelope over 10
  perturbation runs per program (± 10 %; SPEC §10.2); the report gives the contact count for information.
- **What would close it:** the MG06 maker's handle force limit; a fork path re-recorded so that it does not press into the bar.

## The simulated fork-handle contact is stiff; the real robot and the VGC10 interface are not checked

The fork closes MG06 position-controlled through a stiff contact without compliance: in the simulation the servos follow
the recorded path while it presses 0.54 mm into the bar (net force up to 247 N on the bar, a wrench of up to 239 N and
16.1 N m at the tool flange, joint torques up to 34 % of the model's actuator limits; measured 2026-10-25). Passing the
toggle's dead point needs about 156 N on the bar in the simulated MG06 (at 27.5 deg of handle, the same with and without the
overlap). On the real cell the fork, the VGC10 and the robot would deflect, and the AUBO's collision detection may stop the
robot. The main real-cell risk of the closing (owner, 2026-10-25): the flange wrench of 239 N is about 1.6 x the weight of the
VGC10's largest payload rating (15 kg with custom attachments, 147 N); the datasheet gives the payload only (6 kg with default
cups), no force or moment rating of the interface, so the 16.1 N m has nothing to be compared with. The AUBO i5's joint torque
limits and collision thresholds are not known either (the i5 manual v4.5.13 found online is a scanned copy without its tables).
The acceptance bounds on the fork are regression bounds derived from the simulation, not these ratings (SPEC §10.1).

- **Impact:** the closing may not work, may trip the collision detection, or may overload the VGC10, its attachment or the
  fork on the real cell although the run passes here; the simulated values are of a stiff contact and may differ either way.
- **Evidence:** the flange wrench measured in the fork-force investigation (2026-10-25; the baselines pin the net force and
  the hinge load, not the wrench); the VGC10 datasheet.
- **What would close it:** the closing tried on the real cell at reduced speed with force limits (collision detection at a
  low level) active, or measured with a force sensor or the controller's force estimate; OnRobot's force and moment ratings
  of the VGC10 interface and AUBO's of the i5's flange and joints, the bounds then replaced by them or by a margin under them.

## The optimized take relies on the loader's timing: Wait(1.648)

`S11_To_Take` of the optimized program starts `Loader.Start(J_RETURN)`, waits a fixed `Wait(1.648)` and moves the robot from
`P_LOADER_WAIT` to `P_LOADER_ABOVE`; the wait was planned against the loader's modelled timing, and the robot crosses the
loader's Y path just behind its Z carriage. Measured 2026-10-25 (`Example/Report/compare_report.py`, `Report.Core.Stretched_Distance`:
J_RETURN slowed, the robot's lines as programmed, I8's exact distance on the loader's actuator model): as programmed 19.39 mm;
`J_RETURN` +2.2 % slower falls below Loader_Free + Tracking_Margin (14 mm), +3.7 % below Loader_Free (10 mm), +7.2 % below the
acceptance floor (1 mm), +7.6 % the tool meets the Z carriage; +25 % or more -18.5 mm. J_INSERT, J_PARK and J_FETCH are
insensitive (the robot waits for them or moves away). No other part of either program relies on a time.

- **Impact:** the optimized program is not robust to a real loader a few percent slower than its model (or to a late start,
  latency, a stall); in the simulation it passes because the loader is the model.
- **What would close it:** start `W_TO_TAKE` on a condition - `Loader.WaitDone` of a split `J_RETURN_Y` (MoveZ L_TRAVEL, MoveY
  L_HOME) or a loader input "Y at home" - since with the loader's Y at home the whole spline keeps more than 50 mm at any Z
  (`Report.Core.Clear_Height`); expected cycle cost 0.000 s (the spline still ends before the loader's last move). Proposed in
  `Comparison.md` section 4, not implemented (owner's decision).

## Real-cell transfer: a controller waits for "in position" before a LOCKED operation

The optimized program's splines end where a LOCKED operation begins (the crate pick, the handover, the place) with up to
1 mm of residual tool error (0.91 / 0.41 / 1.00 mm; the taught MoveJs 0.19 / 0.13 / 0.25 mm), measured in physics on
2026-10-25 before the B work's re-teach (not re-measured since). In the simulation it has no effect: each LOCKED step starts with a 15 mm approach, and the contact forces in those
steps are the same in both programs (within 0.3 N). A real controller would wait for "in position" (every joint's error and
speed below its threshold) before the next operation; the program does not (owner: not now).

- **Impact:** on the real cell the optimized cycle may be longer by the in-position waits (about 0.1 s per join, estimated).
- **What would close it:** the controller's in-position thresholds, and the rule in the optimizer if they cost time.

## The loader's actuator model is one load case

`Program.Check.Loader_Actual` (the kinematic checks' loader) takes each axis' mass and gravity load at the trajectory's
start; in physics the z load changes with the PCB in the jaws and the jaws' contacts, which leaves up to 1.04 mm (y 0.17 mm),
measured on the optimized program 2026-10-25. `Clearance.Tracking_Margin` covers it.

- **Impact:** a free transfer measured within about 1 mm of the rule may be closer in physics.
- **What would close it:** the load per sample (the PCB's location), if a margin of 1 mm ever matters.

## Simplifications of the model

What the simulation does not model; whoever compares it with the real cell checks these first.

- The vacuum is a constant force per touching cup (7 N): no leaks, no build-up of the vacuum (valve,
  ejector).
- Robot joint torques and armature are estimates; the acceleration limits are assumed (SPEC §4.4, §4.10).
- Cables are not in the collision model; the table top is in the check model only (design-decisions: the table boxes), and the
  physics model has no table.
- The loader's masses are estimates (SPEC §4.10).
- The PCB's components are one box in collision.
- **Impact:** the comparison with reality is mainly the structure of the cycle; the absolute cycle times (70.4 s taught,
  58.7 s optimized) depend on the chosen 60 % speed and the assumed accelerations.
- **What would close it:** the real-cell validation procedure (item above), measured joint data.

## The JIC Smart Factory logo is a recoloured copy, not an official monochrome file

`Media/JIC_Smart_Factory_Black.svg` and `_White.svg` are recolourings of the official purple vector (path
and viewBox unchanged), a temporary project departure from jic-sf-ui-style ("never recoloured"), accepted
by the owner (2026-10-24, design-decisions: the brand). An official monochrome JIC file has been requested
from JIC.

- **Impact:** the JIC logo in the README header is not the brand's own file.
- **Pinned by:** `test/Example/Media/test_make_logo_variants.py` (one colour, original geometry).
- **What would close it:** the official monochrome file from JIC replaces the recoloured one (in
  `Media/Original/`, made into `_Black` / `_White` by the script, or used as it is).

## The PCB is one green mesh: board and components are not told apart

`pcb_vis` is a single mesh; components are drawn in the same green as the board.

- **Impact:** visual only; the collision geoms are separate boxes and unaffected.
- **What would close it:** splitting the mesh into the board and the parts above the board top, and
  colouring the components grey/black (brief §7 step 2, an optional follow-up; FEATURES.md > Open).
