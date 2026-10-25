# project-style-rpx: project reference for KINALI_Test_it_off_Simulation

The conventions of this repository that the generic skill leaves to the project, as
`.claude/project-docs/SPEC.md` decides them. The generic skill is personal (`~/.claude/skills/`,
`rparak/rp-claude-skills`); its checker `check_style.py` is part of it. This file overrides the
generic skill where they differ. The repository holds no `SKILL.md`: a personal skill wins over a
project skill of the same name, so a copy would be ignored and drift (`.gitignore` excludes one). ✅ = enforced (checker, test); 📄 = documentation only.

Project-only page in this folder: **mjcf.md** (the scene, the vendored robot, what is never changed).

Brand skill: **jic-sf-ui-style** (JIC Smart Factory) for every visual of the project - the README header now,
the Phase 2 application later (toolkit open, design-decisions). Its logo rules apply with one departure (below).

**Status (2026-10-24):** in force; the code was migrated to it in `FEATURES.md` F3 (before: 13 files,
429 issues; after: 25 files, 0 issues).

## Header and checker

- Holder: **JIC Smart Factory** - `Copyright (c) 2026 JIC Smart Factory` in `LICENSE` and every header (as in
  the template UR_PathGen_SF_1); the `Author   :` line names the author, **Roman Parak**. The checker enforces
  the copyright line (`check_style.json` `holder`), not the author line. Vendored licences keep their holders
  (`models/aubo_i5/LICENSE_Manipulator-Mujoco`). The `#`-comment header form. `File Name:` examples:
  `../Scene/Core.py` for `src/Scene/Core.py`, `../Example/Simulation/run_program.py`, `../test/Scene/test_core.py`.
- `check_style.json` (this folder): class roots `src`, `Example`; test root `test`; check roots `src`,
  `Example`, `test`; no exclusions.
- Command: `.venv/bin/python ~/.claude/skills/project-style-rpx/scripts/check_style.py src Example test`.

## Layout and layers (SPEC §6)

```
src/<Domain>/Core.py   Configuration, Scene, Kinematics, Collision, Render, Simulation, Keypoints, Trajectory, Program, Planner,
                       Optimizer;
                       plus src/Configuration/Program_Parser.py (the cell program's reader, F10)
Example/<Domain>/<script>.py   Simulation/run_program.py, Scene/export_cell_scene.py, Scene/model_fingerprint.py,
                       Render/show_cell.py, Render/render_image.py, Media/make_logo_variants.py,
                       Keypoints/check_keypoints.py, Keypoints/resolve_keypoint.py, Keypoints/show_keypoint.py,
                       Keypoints/step_keypoints.py, Program/build_program.py, Program/show_program.py,
                       Program/optimize_program.py, Program/compare_programs.py
test/<Domain>/test_core.py   mirroring src/; test/Example/<Domain>/test_<script>.py; test/Install/test_install.py;
                       test/Docs/test_layout.py; reference data (the baselines) in test/<Domain>/data/
models/                the MJCFs and their meshes (mjcf.md)
Data/<Purpose>/        PCB/pcb_meta.json, Paths/fork_close_open_path.npz, Optimized/Program_Optimized.yaml
Configuration/         Cell.yaml; the cell program Constants.yaml, Simulation.yaml, Program.yaml (F10); Optimization.yaml (F9)
Media/                 the README images: <Logo>_Black.svg / <Logo>_White.svg (JIC_Smart_Factory, KINALI), written by Example/Media/make_logo_variants.py; Original/ the owner's files
logger/                git-ignored: logs; Data/Generated/<Kind>/ git-ignored: generated artefacts
```

The scripts folder is `Example/`, one subfolder per `src/` domain a script belongs to (`Keypoints`, `Media`,
`Program`); a script builds the repository root from `__file__` two folders up. Scripts may import
other scripts as modules (none does yet). `Data/` subfolders are named by purpose, Pascal case like the
domains.

Layer order 📄:

```
Configuration, Render     import nothing of the project
Scene                     → Configuration
Kinematics, Collision     → Scene
Simulation                → Configuration, Scene, Collision
Keypoints                 → Configuration, Scene, Kinematics, Collision
Trajectory                → Configuration (the file format only, no MuJoCo)
Planner                   imports nothing of the project (numpy, scipy)
Program                   → Configuration, Scene, Kinematics, Collision, Keypoints, Trajectory, Planner
Optimizer                 → Configuration, Kinematics, Collision, Keypoints, Planner, Program, Trajectory
Example/                  → any of src/
```

`Render` takes an `MjModel` / `MjData` and knows nothing of the cell; `Planner` takes a validity function and knows nothing
of the cell either. Trajectory (F8.3) and Program (F8.4) are the step 4a domains, Planner and Optimizer step 4b's (F9).

## Root ✅

The root holds only `README.md`, `LICENSE`, `CLAUDE.md`, `install.sh`, `requirements.txt`,
`pytest.ini`, `.gitignore`, `.claude`, `src`, `test`, `Example`, `models`, `Data`, `Media`, `Configuration`.
Exception, with its reason: **`pytest.ini`** (the test runner reads it at the root). `logger/` exists on disk
but is git-ignored, so it is not a root entry. Pinned by `test/Docs/test_layout.py`.

## Formats - one reader and writer each 📄 (SPEC §9)

| Format | Read and written by |
|---|---|
| `models/scene.xml`, `models/aubo_i5/aubo_i5.xml`, `models/robot_tool/vgc10_opener_free.xml` | `Scene.Core` through MjSpec; never written (mjcf.md) |
| `models/cell_scene.xml` | written by `Scene.Core.Export_Cell_Scene`; read only by MuJoCo's viewer |
| `Data/PCB/pcb_meta.json` | `Configuration.Core.Read_PCB_Meta`; written by hand (the PCB poses only) |
| `Configuration/Cell.yaml` | `Configuration.Core.Load_Cell_Configuration`; written by hand |
| `Configuration/Constants.yaml`, `Simulation.yaml`, `Program.yaml` | `Configuration.Program_Parser.Load_Constants`, `Load_Simulation`, `Load_Program`; written by hand (a script that finds a value to change prints the line: `resolve_keypoint.py`) |
| `Configuration/Optimization.yaml` | `Configuration.Program_Parser.Load_Optimization`; written by hand |
| `Data/Optimized/Program_Optimized.yaml` | written by `Optimizer.Core.Optimized_Text` (only through `Example/Program/optimize_program.py`); read by `Configuration.Program_Parser.Load_Program` |
| `Data/Generated/Trajectories/<program>.npz` (+ `.csv` export) | `Trajectory.Core.Write_Trajectory`, `Read_Trajectory`; filled by `Program.Core.Build_Program` |
| `Data/Paths/fork_close_open_path.npz` | `Program.Core` (`Robot.MovePath`); recorded in the MG06 project, never written |
| a program's physics report | `Simulation.Core.Format_Program_Report`; printed and written to `Data/Generated/Reports/<program>_physics.txt` by `Example/Simulation/run_program.py` |
| the physics baselines (`test/Simulation/data/baseline_*`) | `Example/Simulation/physics_baseline.py` (writes and compares) |
| the logo variants `Media/<Logo>_<Black|White>.svg` | `Example/Media/make_logo_variants.py` (`Recolour_Logo`, `Write_Logo`, `Check_Logo`); read by the README |
| the model fingerprint JSON | `Example/Scene/model_fingerprint.py` (`Model_Fingerprint`, `Compare_Fingerprints`) |
| `logger/*.log`, `logger/MUJOCO_LOG.TXT` | written by the scripts through `Configuration.Core` (`Script_Log_Cls`, `Configure_MuJoCo_Log`) and by MuJoCo; never read by code |

## Configuration

- Files (SPEC §8): `Configuration/Cell.yaml`, read only by `src/Configuration/Core.py` (`Load_Cell_Configuration` ->
  `Cell_Configuration_Str`, tested in `test/Configuration/test_configuration_files.py`); the cell program
  `Configuration/Constants.yaml`, `Simulation.yaml`, `Program.yaml`, read only by `src/Configuration/Program_Parser.py`
  (`Load_Constants`, `Load_Simulation`, `Load_Program` -> `Program_Str`, the typed IR; tested in
  `test/Configuration/test_program_parser.py`: each validation error named with its file and line, on mutated copies of
  the shipped files). Every YAML file goes through `Configuration.Core.Read_YAML` (duplicate keys rejected, only `true` /
  `false` booleans, each string with its line).
- Keys Pascal_Snake_Case. Constants are UPPERCASE with their section's prefix (`T_`, `P_`, `L_`, `G_`, `PATH_`, `DO_`,
  `DI_`), segments `S<n>_<Name>`, jobs `J_<NAME>`; the contact groups and PCB locations stay lowercase (code constants).
  No optional fields: `Reference: null`, `Offset: null`, `PCB_Hold: null` and `Contacts: []` are values, not omissions
  (owner, 2026-10-24). Every declared constant is used.
- Program lines are calls parsed with `ast`, never evaluated (design-decisions).
- Not configured: what the model holds (joint limits, loader ranges, the lid's open stop) - read from the
  compiled model. `ON` / `OFF` / `Open` are text (the loader reads only `true` / `false` as booleans).
- Single legal value, so a code constant: the number of suction cups (`Scene.CONST_CUP_N`), the PCB
  locations, the contact groups, the planning margin factor (2 x the clearance).
- Derived, not stored: the adhesion force (pressure x cup area), the cup pad size, the planning margin.

## Units

SI in code: [m], [rad], [s], [N], [kg]. The report prints mm and deg (`Simulation.Core.Format_Report`).

## Errors

`Configuration_Error_Cls` (a missing or malformed data file), `Scene_Error_Cls` (a model file that does
not build), `Simulation_Error_Cls` (MuJoCo reset the state: *"The simulation became unstable at
t = … s (phase …): MuJoCo reset the state. …"*). A failed acceptance check is not an exception: it is a
returned list, printed by the script, exit code 1.

## Installer

- Targets: macOS on Apple silicon (verified), Ubuntu Linux (unverified, stubs only).
- Interpreter: Python 3.11, `/opt/homebrew/bin/python3.11` on macOS, `python3.11` on Linux.
- Verification step: import mujoco, numpy, scipy, yaml; mujoco == 3.16.0. Creates `logger/`, `Data/Generated/`.
- `requirements.txt`: direct dependencies pinned `==` (the owner's list), grouped by user; no lock file.
- Test: `test/Install/test_install.py`.

## Tests

- Everyday `.venv/bin/python -m pytest -m "not slow"`; full `.venv/bin/python -m pytest` before a commit.
- Reference data: `test/Simulation/data/baseline_report_<program>.txt` and `baseline_metrics_<program>.json`,
  `test/Program/data/` (the golden), `test/Scene/data/baseline_model_fingerprint.json`
  (since F3).
- Equivalence outputs: the printed report, the fingerprint (project-optimize reference).

## Project rules

- Logs: scripts print `[INFO]` / `[WARNING]` / `[ERROR]` lines (no `logging` module) and write every line
  to `logger/<script>.log` through `Script_Log_Cls.Line` (`print(log.Line(text))`); MuJoCo's own log goes
  to `logger/MUJOCO_LOG.TXT` (`Configure_MuJoCo_Log`, called first by every script and by
  `test/conftest.py`). `Example/Simulation/run_program.py` prints the report on stdout and its own lines on stderr,
  so stdout is the report byte for byte.
- Module-level private helpers: `_Pascal_Snake_Case` (e.g. `Scene.Core._Apply_Look`), like test
  helpers (the generic skill leaves it *Unsettled*; decided 2026-10-24).
- Stream protocols are not implemented in `src/` (no `write`/`flush` methods, which the naming rule would
  refuse); a script prints what `Script_Log_Cls.Line` returns instead.
- The verified physics values are never changed in a style or structure change (SPEC §3, mjcf.md).

## Departures from the generic skill

- The logos are black and white recolourings of the originals (owner, 2026-10-24), although jic-sf-ui-style
  says its logos are "never recoloured" and the purple JIC logo is for white grounds: a temporary departure
  until the official monochrome JIC file (requested from JIC) replaces it (design-decisions: the brand).
- `.claude/project-docs/FEATURES.md`: a fourth internal document, the feature tracker the owner asked
  for; the generic skill defines three.
