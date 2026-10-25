# project-style-rpx: project reference for KINALI_Test_it_off_Simulation

The conventions of this repository that the generic skill leaves to the project, as
`.claude/project-docs/SPEC.md` decides them. The generic skill is personal (`~/.claude/skills/`,
`rparak/rp-claude-skills`); its checker `check_style.py` is part of it. This file overrides the
generic skill where they differ. The repository holds no `SKILL.md`: a personal skill wins over a
project skill of the same name, so a copy would be ignored and drift (`.gitignore` excludes one). ✅ = enforced (checker, test); 📄 = documentation only.

Project-only page in this folder: **mjcf.md** (the scene, the vendored robot, what is never changed).

Brand skill: **jic-sf-ui-style** (JIC Smart Factory) for the JIC Smart Factory logo in the README header - its source
file (`Media/Original/jic-smart-factory.svg`, byte-identical to the skill's asset) and its logo rules, with one departure
(below). Nothing else of it is used: there is no GUI (owner, 2026-10-25), and the figures use neither its palette nor its
fonts (`compare_programs.py` uses the dataviz palette).

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
src/<Domain>/Core.py   the domain's primary module: Configuration, Scene, Kinematics, Collision, Render, Simulation, Keypoints,
                       Trajectory, Program, Planner, Optimizer
src/<Domain>/<Topic>.py  a second concern of a domain: Configuration/Program_Parser.py (the cell program's reader),
                       Program/Check.py (the trajectory check), Simulation/Report.py (a run's acceptance and report),
                       Optimizer/Measure.py (what the optimized program is measured by)
src/<Domain>/Const.py  only where more than one module of the domain uses a constant: Configuration, Program, Simulation
Example/<Domain>/<script>.py   one subfolder per domain a script belongs to: Scene, Render, Media, Keypoints, Program, Simulation
test/<Domain>/test_<module>.py   mirroring src/ (a Const.py has none: it is tested through its readers);
                       test/Example/<Domain>/test_<script>.py; test/Install/test_install.py; test/Docs/test_layout.py;
                       reference data (the baselines) in test/<Domain>/data/
models/                the MJCFs and their meshes (mjcf.md)
Data/<Purpose>/        PCB/pcb_meta.json, Paths/fork_close_open_path.npz, Optimized/Program_Optimized.yaml,
                       Experiments/<name>/ (a not-adopted experiment: its .patch, script, README.md)
Configuration/         Cell.yaml; the cell program Constants.yaml, Simulation.yaml, Program.yaml; Optimization.yaml
Media/                 the README images: <Logo>_Black.svg / <Logo>_White.svg (JIC_Smart_Factory, KINALI), written by
                       Example/Media/make_logo_variants.py; Original/ the owner's files
logger/                git-ignored: logs; Data/Generated/<Kind>/ git-ignored: generated artefacts
```

**The module layout ✅** (owner, 2026-10-25; design-decisions: the module layout). `Core.py` is each domain's primary
module. A second concern gets its own topic module, named for it, that imports the domain's `Core.py` and never the
other way round. A constant one module uses is defined in that module; a constant more than one module of the domain
uses is defined in the domain's `Const.py`, which holds nothing else. Another domain imports a name from the module
that defines it. A `Const.py` that imports nothing carries its description as a `#` comment block (a docstring would
be its first statement and read as its header). Pinned by `test/Docs/test_layout.py`: the declared modules per domain,
each module's test file, each constant with its readers. The checker's configuration has no layout key.

The scripts folder is `Example/`; a script builds the repository root from `__file__` two folders up. A script may
import another as a module through `importlib` (`physics_baseline.py` uses `run_program.py`'s `Load_Run`;
`show_physics.py` too). `Data/` subfolders are named by purpose, Pascal case like the domains.

Layer order 📄 (by domain; within a domain: `Const` ← `Core` ← the topic modules):

```
Configuration, Render, Planner   import nothing of the project (Planner: numpy, scipy)
Scene                     → Configuration
Kinematics                → Scene
Collision                 → Configuration, Scene
Trajectory                → Configuration (the file format only, no MuJoCo)
Simulation                → Configuration, Scene, Collision, Trajectory
Keypoints                 → Configuration, Scene, Kinematics, Collision
Program                   → Configuration, Scene, Kinematics, Collision, Keypoints, Trajectory, Planner
Optimizer                 → Configuration, Kinematics, Collision, Keypoints, Planner, Program, Trajectory
Example/                  → any of src/
```

`Render` takes an `MjModel` / `MjData` and knows nothing of the cell; `Planner` takes a validity function and knows nothing
of the cell either.

## Root ✅

The root holds only `README.md`, `LICENSE`, `CLAUDE.md`, `install.sh`, `requirements.txt`,
`pytest.ini`, `.gitignore`, `.claude`, `src`, `test`, `Example`, `models`, `Data`, `Media`, `Configuration`.
Exception, with its reason: **`pytest.ini`** (the test runner reads it at the root). `logger/` exists on disk
but is git-ignored, so it is not a root entry. Pinned by `test/Docs/test_layout.py`, which also refuses two tracked paths
that differ only in letter case (macOS's file system is case-insensitive: a case-only rename goes through a temporary name).

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
| a program's physics report | `Simulation.Report.Format_Program_Report`; printed and written to `Data/Generated/Reports/<program>_physics.txt` by `Example/Simulation/run_program.py` |
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

SI in code: [m], [rad], [s], [N], [kg]. The report prints mm and deg (`Simulation.Report.Format_Report`).

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

- Per commit `.venv/bin/python -m pytest -n 8 -m "not slow"`; per accepted feature the full `.venv/bin/python -m pytest -n 8`
  (pytest-xdist; the suite also passes serially, `-n 0`). The two tiers of checks: `CLAUDE.md`.
- Shared fixtures in `test/conftest.py`: the cell, the program, one build per program and session (`built_trajectory`), the
  in-process build and check (`program_built`, `program_checked`, `optimized_built`), the probe helpers.
- Reference data: `test/Simulation/data/baseline_report_<program>.txt` and `baseline_metrics_<program>.json`,
  `test/Program/data/` (the golden), `test/Scene/data/baseline_model_fingerprint.json`.
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

- A topic module's own `*_Str` structures live in that module (`Program_Check_Str` in `Program/Check.py`), not in the
  domain's `Core.py`; a domain's shared constants live in its `Const.py` (design-decisions: the module layout).

- The logos are black and white recolourings of the originals (owner, 2026-10-24), although jic-sf-ui-style
  says its logos are "never recoloured" and the purple JIC logo is for white grounds: a temporary departure
  until the official monochrome JIC file (requested from JIC) replaces it (design-decisions: the brand).
- `.claude/project-docs/FEATURES.md`: a fourth internal document, the feature tracker the owner asked
  for; the generic skill defines three.
