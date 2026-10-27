# mujoco-robot-cell: project reference for KINALI_Test_it_off_Simulation

This project's part of the simulation method. The generic skill is personal (`~/.claude/skills/`); this file overrides it
where they differ. No `SKILL.md` is tracked here (a personal skill wins over a project skill of the same name). Every value
cites the record that holds it; the record wins where they differ.

## Scenario and pages

Scenario 3, a cell: the robot, the loader (two axes and a gripper) and the MG06 tester the robot opens and closes, with
shared zones and handovers. Pages: all core pages + cell, interlocks, cell-optimization.

## Moving objects

| Object | Kind | Model source | Joints and actuators | Commanded by |
|---|---|---|---|---|
| AUBO i5 | 6-axis arm | `models/aubo_i5/` (vendored, MIT), patched in code by `src/Scene/Core.py` | position servos with velocity feed-forward, gravity compensation; torque limits and armature are the vendored model's estimates (SPEC §4.10, §10.1 item 17) | the program's segments (`Program.yaml`) |
| VGC10 + fork | vacuum tool + fork | `models/robot_tool/vgc10_opener_free.xml` | 4 cups, each a bellows slide + adhesion (SPEC §4.10) | `SetDO(DO_VACUUM)` |
| Loader | 2 linear axes + EGI 040 gripper | `models/scene.xml` | Y / Z servos behind a 30 ms `filterexact` lag; synchronized jaws, 70 N (SPEC §4.3, design-decisions) | `Loader_Jobs` (`Loader.Start` / `Loader.WaitDone`) |
| MG06 | mechanism (lid, handle, press, lock) | `models/scene.xml` | passive, moved by the fork | `Tester_Jobs` (`Test`) |
| PCB | part | `models/scene.xml` (free body) | - | custody rules (SPEC §7.9) |

## Frames and units

The MG06 fixture frame: Z up, front -Y (SPEC §4.2); the robot base pose in `Cell.yaml` `Robot` (SPEC §4.2). Joint order
pan, lift, elbow, wrist1, wrist2, wrist3; TCPs `aubo/tool/suction_tcp` (cup face, 260.6 mm from the flange) and the flange
(SPEC §4.3, §4.8). Code in SI; the YAML files in deg and m (SPEC §8).

## Model

- Scene `models/scene.xml` (from Blender; never regenerated, SPEC §3). CAD deviations: design-decisions "The CAD deviations
  stay compensated in scene.xml". What is never changed: project-style-rpx's `mjcf.md`.
- MuJoCo 3.16.0 pinned (design-decisions). Known warning: KNOWN_ISSUES "MuJoCo warns about an integrator conflict when the
  robot is attached" (filtered by its exact message in `Scene.Core`).
- Fingerprint `test/Scene/data/baseline_model_fingerprint.json`, compared by `Example/Scene/model_fingerprint.py`; one
  accepted difference, `geoms/pcb_vis/geom_rgba`.
- Check model: the planning model with the table top as two boxes (design-decisions "The check model ...").

## Program and checks

- Files: `Configuration/Constants.yaml`, `Simulation.yaml`, `Program.yaml`, read by `src/Configuration/Program_Parser.py`;
  `Optimization.yaml` for the optimizer; trajectory grid 4 ms (SPEC §9).
- Contact groups: the names in `src/Configuration/Const.py`, their depths in `Cell.yaml` `Clearance`; clearance 3 mm, the
  sampling step, `Loader_Free` (10 mm) and `Tracking_Margin` also in `Cell.yaml` (SPEC §8).
- Handling rules in force: SPEC §4.7.

## Physics and acceptance

- Run: `Example/Simulation/run_program.py` (exit 0 / 1 / 2); instability fails the run (SPEC §4.7, `Simulation_Error_Cls`).
- Acceptance: SPEC §10.1 items 1-25, bounds in `Cell.yaml` `Acceptance`.
- Not safety limits: items 14-15 are regression bounds derived from the simulation, item 17 a model-based bound (SPEC §10.1).

## Regression

- Golden: `test/Program/data/` (`Example/Program/golden_program.py`). Physics baselines: `test/Simulation/data/`
  (`Example/Simulation/physics_baseline.py`: `CONST_TOLERANCES`, an envelope over `CONST_PERTURBATIONS`, 10 runs per program;
  SPEC §10.2).
- Check tiers: `CLAUDE.md` > Branches and checks.

## Cell

Jobs, `Start` / `WaitDone`, custody: SPEC §7.9, §8. Locked / free segments and the interlocks I1-I10: SPEC §7.11. Shared zones
(the jaw zone, the MG06 corridor): SPEC §7.12. The gating rule: `Program.Check.Gating_Faults`. Stretch factors: the loader
0.5-2 x, the robot 10-100 % (SPEC §7.12). Exit and waiting point `P_LOADER_WAIT` (`Optimization.yaml` `Exit_Points`,
`Waiting_Points`). The optimized program `Data/Optimized/Program_Optimized.yaml`, written by
`Example/Program/optimize_program.py`.

## Views and media

Named views `Report.Media.CONST_VIEWS` (8, SPEC §7.12); media budgets in readme-writer's `check_readme.json`; the videos are
v1.0 release assets (readme-writer's project reference).

## Real robot

Not validated: the sentence opens `KNOWN_ISSUES.md` (and feature-manager's project reference). Open transfer items: the
real-cell entries of KNOWN_ISSUES, SPEC §17; the first-run order: README > How the model was built.

## Departures from the generic skill

- Time-optimal timing is the project's own `Planner.Topp` (toppra has no macOS arm64 wheel), with no jerk limit (owner:
  bang-bang accelerations): design-decisions "TOPP is our own".
- `PATH_FORK_OPEN` keeps its recorded velocity step at its join (design-decisions).
