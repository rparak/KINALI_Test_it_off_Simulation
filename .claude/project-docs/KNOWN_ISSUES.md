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

## The crate penetration is −0.14 mm in this environment, −0.08 mm in the brief's reference

The brief's reference result (produced on the original machine of `mg06_cell_mujoco`, environment not
recorded) gives the deepest PCB penetration into the crate as −0.08 mm; every run here gives −0.14 mm at
t = 62.46 s. Every other number of the report is identical.

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
file goes to `logger/MUJOCO_LOG.TXT` (`Configuration.Core.Configure_MuJoCo_Log`, called by every script,
the legacy shims and the test suite), never into the working directory.

- **Impact:** noise on stderr and in the test summary's warnings; not a fault.
- **Evidence:** `MjLogConfig` probe 2026-10-24: the message is logged at the attach in the scene build.
- **Pinned by:** `test/Configuration/test_core.py::test_mujoco_logs_into_the_configured_file_never_the_working_directory`.
- **What would close it:** the owner's decision on the integrator, with a new baseline (a physics change).

## The legacy cycle's --render video option does not work in the pinned environment

`legacy/test_cell_cycle.py --render FILE.mp4` imports `imageio` (and Pillow), which are not in
`requirements.txt` (owner's pin list, 2026-10-24). Pillow is installed as a dependency of matplotlib;
`imageio` is not, so the option fails at its import. The cycle run and `--record` are unaffected.

- **Impact:** no video of the legacy cycle from this environment.
- **Evidence:** `install.sh` output 2026-10-24 (no imageio installed).
- **What would close it:** a video option in the `Example/` run script written with a pinned package, or
  imageio added to the pins (owner's decision); otherwise it goes with `legacy/` in step 6.

## The previous Czech README has pre-restructure paths

`docs/README_cs_mg06_cell.md` (the README of `mg06_cell_mujoco`) still names `cell.py`,
`test_cell_cycle.py`, `tools/plan_cell.py`, `cell_scene.xml` at the root and the CAD-era file names.

- **Impact:** its commands do not run as written; its physics notes and CAD deviations are still right.
- **What would close it:** the full README (brief §7 step 7), after which the file is removed.

## The PCB is one green mesh: board and components are not told apart

`pcb_vis` is a single mesh; components are drawn in the same green as the board.

- **Impact:** visual only; the collision geoms are separate boxes and unaffected.
- **What would close it:** splitting the mesh into the board and the parts above the board top, and
  colouring the components grey/black (brief §7 step 2, optional follow-up, `FEATURES.md`).
