# Known issues

**Simulation passed ≠ safe on the real cell.** Every motion this project produces is verified only in
simulation; whether any output will drive the real robot or loader is still open (SPEC §14). Until that
is decided, nothing here is a statement about the real cell.

This file lists what the project does not model or has not verified. It contains only what has been
decided or measured.

## The baseline was measured on Python 3.14.7 / numpy 2.5.4; the re-baseline in the pinned Python 3.11 environment is pending

`docs/baseline_report.txt` and `docs/baseline_model_fingerprint.json` were produced on 2026-10-24 with
Python 3.14.7, numpy 2.5.4, MuJoCo 3.16.0, macOS 26.6.2, Apple M2 Pro. The pinned environment is
Python 3.11 with numpy 2.4.6 (design-decisions). The cycle has not yet been run there.

- **Impact:** until the re-baseline, an empty diff is proven only in the 3.14 environment.
- **Evidence:** SPEC §4.6; `.venv` of 2026-10-24.
- **What would close it:** `install.sh`, the cycle run and the fingerprint compare in the 3.11
  environment; an identical report is recorded as the baseline environment, a different one is stored
  as the new baseline in its own commit with the differences listed here (`FEATURES.md` F3, item D).

## The crate penetration is −0.14 mm in this environment, −0.08 mm in the brief's reference

The brief's reference result (produced on the original machine of `mg06_cell_mujoco`, environment not
recorded) gives the deepest PCB penetration into the crate as −0.08 mm; every run here gives −0.14 mm at
t = 62.46 s. Every other number of the report is identical.

- **Impact:** none on acceptance (bound 0.5 mm, SPEC §10.1); it shows that the report is reproducible
  only per environment, so the baseline is per environment.
- **Evidence:** `docs/baseline_report.txt` against git `427e076:docs/PROJECT_BRIEF.md` §3.
- **What would close it:** the original environment's versions; not expected to be available.

## MuJoCo warns about an integrator conflict when the robot is attached

Every build of the cell makes MuJoCo log (written to `MUJOCO_LOG.TXT` in the current working directory,
and printed):

```
WARNING: Attach conflict when attaching 'aubo_i5' to 'mg06_loader_scene', policy is 'warning'
integrator: parent has 0 (default), child has 3, keeping parent value
```

The cause is `<option integrator="implicitfast"/>` (line 4) of the vendored robot MJCF
`assets/robot/aubo_i5/aubo_i5.xml`, while `scene.xml` sets no integrator (Euler, the default). MuJoCo
keeps the parent's Euler; the verified physics was run that way, so the warning describes the verified
state. Not fixed on purpose: changing either file changes the physics or the vendored model.

- **Impact:** a log file appears in whatever directory a script is started from; the message is noise,
  not a fault.
- **Evidence:** `MjLogConfig` probe 2026-10-24: the file is written at the attach in the scene build.
- **What would close it:** routing MuJoCo's log into `logger/` (`MjLogConfig.logfile`, `FEATURES.md`
  F3); the warning itself only if the owner decides the integrator.

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
