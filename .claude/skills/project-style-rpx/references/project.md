# project-style-rpx: project reference for KINALI_Test_it_off_Simulation

The conventions of this repository that the generic skill leaves to the project, as
`.claude/project-docs/SPEC.md` decides them. The generic skill is personal (`~/.claude/skills/`,
`rparak/rp-claude-skills`); its checker `check_style.py` is part of it. This file overrides the
generic skill where they differ. The repository holds no `SKILL.md`: a personal skill wins over a
project skill of the same name, so a copy would be ignored and drift (`.gitignore` excludes one). ✅ = enforced (checker, test); 📄 = documentation only.

Project-only page in this folder: **mjcf.md** (the scene, the vendored robot, what is never changed).

Brand skill: **jic-sf-ui-style** (JIC Smart Factory) for the JIC Smart Factory logo in the README header - its source
file (`Media/Original/jic-smart-factory.svg`, byte-identical to the skill's asset) and its logo rules, with one departure
(below). Nothing else of it is used: there is no GUI (owner, 2026-10-25), and its figures follow the generic default look
(references/figures.md).

**Figure style 📄** (owner, 2026-10-25; design-decisions: the figure style): the generic default look (references/figures.md)
in `src/Report/Style.py`, used by the comparison's figures (`Example/Report/compare_report.py`); the name helper is
`Style.Label`. The project's choices: one colour per program in every figure (Default: sky blue / blue; Optimized: the JIC
purple's 40 % tint on dark, the purple on light - each at least 4.5:1 on its background, `test/Report/test_style.py`). The
programs are shown as "Default program" and "Optimized program" (`Style.CONST_PROGRAM_NAMES`); the code keeps taught /
optimized. `compare_programs.py`'s figure keeps the dataviz palette (a departure, below).

**Known gap 📄** (owner, 2026-10-27): no test regenerates the README charts and compares them with the committed files in
`Media/` (references/figures.md > Saving); `test/Example/Report/test_compare_report.py` compares `--media` with `--out` only.
The figures are checked at release. The SVGs are not byte-stable (matplotlib's random clip and marker ids; design-decisions:
the media are encoded with Pillow and imageio-ffmpeg), and the `svg.hashsalt` fix is not yet verified.

## Header and checker

- Holder: **JIC Smart Factory** - `Copyright (c) 2026 JIC Smart Factory` in `LICENSE` and every header (as in
  the template UR_PathGen_SF_1); the `Author   :` line names the author, **Roman Parak**. The checker enforces
  the copyright line (`check_style.json` `holder`). Vendored licences keep their holders
  (`models/aubo_i5/LICENSE_Manipulator-Mujoco`). The `#`-comment header form. `File Name:` examples:
  `../Scene/Core.py` for `src/Scene/Core.py`, `../Example/Simulation/run_program.py`, `../test/Scene/test_core.py`.
- `check_style.json` (this folder): class roots `src`, `Example`; test root `test`; check roots `src`,
  `Example`, `test`; no exclusions.
- Command: `.venv/bin/python ~/.claude/skills/project-style-rpx/scripts/check_style.py src Example test`.

## Layout and layers (SPEC §6)

```
src/<Domain>/Core.py   the domain's primary module: Configuration, Scene, Kinematics, Collision, Render, Simulation, Keypoints,
                       Trajectory, Program, Planner, Optimizer, Report
src/<Domain>/<Topic>.py  a second concern of a domain: Configuration/Program_Parser.py (the cell program's reader),
                       Program/Check.py (the trajectory check), Simulation/Report.py (a run's acceptance and report),
                       Optimizer/Measure.py (what the optimized program is measured by), Report/Media.py (the
                       comparison's frames, videos, GIFs), Report/Style.py (the figure style)
src/<Domain>/Const.py  only where more than one module of the domain uses a constant: Configuration, Program, Simulation
Example/<Domain>/<script>.py   one subfolder per domain a script belongs to: Scene, Render, Media, Keypoints, Program, Simulation,
                       Report
test/<Domain>/test_<module>.py   mirroring src/ (a Const.py has none: it is tested through its readers);
                       test/Example/<Domain>/test_<script>.py; test/Install/test_install.py; test/Docs/test_layout.py;
                       reference data (the baselines) in test/<Domain>/data/
models/                the MJCFs and their meshes (mjcf.md)
Data/<Purpose>/        PCB/pcb_meta.json, Paths/fork_close_open_path.npz, Optimized/Program_Optimized.yaml,
                       Experiments/<name>/ (a not-adopted experiment: its .patch, script, README.md)
Configuration/         Cell.yaml; the cell program Constants.yaml, Simulation.yaml, Program.yaml; Optimization.yaml
Media/                 the README images: <Logo>_Black.svg / <Logo>_White.svg (JIC_Smart_Factory, KINALI), written by
                       Example/Media/make_logo_variants.py; Cycle_<Side_by_Side|Optimized>.gif, View_*.png, Collision_*.png,
                       Key_Points.png, written by Example/Report/make_media.py; Default_vs_Optimized_* and Timeline_* (dark,
                       light; PNG, SVG), written by Example/Report/compare_report.py --media Media; Original/ the owner's files
logger/                git-ignored: logs; Data/Generated/<Kind>/ git-ignored: generated artefacts
```

**The module layout ✅** (owner, 2026-10-25; design-decisions: the module layout): the generic rule (references/structure.md
> Modules), with the modules listed above. A `Const.py` that imports nothing carries its description as a `#` comment block (a
docstring would be its first statement and read as its header). Pinned by `test/Docs/test_layout.py`: the declared modules per domain,
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
Report                    → Configuration, Kinematics, Collision, Simulation, Program, Optimizer, Trajectory, Render
Example/                  → any of src/
```

`Render` takes an `MjModel` / `MjData` and knows nothing of the cell; `Planner` takes a validity function and knows nothing
of the cell either.

## Root ✅

The root holds only `README.md`, `LICENSE`, `CITATION.cff`, `CLAUDE.md`, `install.sh`, `requirements.txt`,
`pytest.ini`, `.gitignore`, `.claude`, `src`, `test`, `Example`, `models`, `Data`, `Media`, `Configuration`.
Exceptions, with their reasons: **`pytest.ini`** (the test runner reads it at the root); **`CITATION.cff`** (GitHub reads it at
the root for *Cite this repository*; owner, 2026-10-26; checked by `test/Docs/test_citation.py`). `logger/` exists on disk
but is git-ignored, so it is not a root entry. Pinned by `test/Docs/test_layout.py`, which also refuses two tracked paths
that differ only in letter case.

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
| the comparison `Data/Generated/Reports/Comparison/` (Comparison.md, figures, videos, view, point and collision images) | `Example/Report/compare_report.py`, `make_media.py` (through `Report.Core`, `Report.Media`); never read |
| the README media `Media/Cycle_<Side_by_Side|Optimized>.gif`, `Media/View_*.png`, `Media/Collision_*.png`, `Media/Key_Points.png` | `Example/Report/make_media.py` (`Report.Media.Write_Gif`, `Write_Png`, `Contact_Sheet`); read by the README |
| the README charts `Media/Default_vs_Optimized_<dark|light>.<png|svg>`, `Media/Timeline_<dark|light>.<png|svg>` | `Example/Report/compare_report.py --media Media` (`Report.Style.Save_Figure`); read by the README |
| the model fingerprint JSON | `Example/Scene/model_fingerprint.py` (`Model_Fingerprint`, `Compare_Fingerprints`) |
| `logger/*.log`, `logger/MUJOCO_LOG.TXT` | written by the scripts through `Configuration.Core` (`Script_Log_Cls`, `Configure_MuJoCo_Log`) and by MuJoCo; never read by code |

## Configuration

- Files (SPEC §8): `Configuration/Cell.yaml`, read only by `src/Configuration/Core.py` (`Load_Cell_Configuration` ->
  `Cell_Configuration_Str`, tested in `test/Configuration/test_configuration_files.py`); the cell program
  `Configuration/Constants.yaml`, `Simulation.yaml`, `Program.yaml`, read only by `src/Configuration/Program_Parser.py`
  (`Load_Constants`, `Load_Simulation`, `Load_Program` -> `Program_Str`, the typed IR; tested in
  `test/Configuration/test_program_parser.py`: each validation error named with its file and line, on mutated copies of
  the shipped files). Every YAML file goes through `Configuration.Core.Read_YAML`, the strict reader (references/yaml.md).
- Keys Pascal_Snake_Case. Constants are UPPERCASE with their section's prefix (`T_`, `P_`, `L_`, `G_`, `PATH_`, `DO_`,
  `DI_`), segments `S<n>_<Name>`, jobs `J_<NAME>`; the contact groups and PCB locations stay lowercase (code constants).
  The fields that may be explicitly empty (references/yaml.md): `Reference: null`, `Offset: null`, `PCB_Hold: null`,
  `Contacts: []` (owner, 2026-10-24). Every declared constant is used.
- Program lines: the configuration mini-language (references/yaml.md), parsed with `ast` (design-decisions).
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
  `test/conftest.py`). `Example/Simulation/run_program.py` is a report script (references/python.md: report on stdout).
- Module-level private helpers: `_Pascal_Snake_Case` (e.g. `Scene.Core._Apply_Look`), like test
  helpers (the generic skill leaves it *Unsettled*; decided 2026-10-24).
- Stream protocols are not implemented in `src/` (no `write`/`flush` methods, which the naming rule would
  refuse); a script prints what `Script_Log_Cls.Line` returns instead.
- The verified physics values are never changed in a style or structure change (SPEC §3, mjcf.md).

## Departures from the generic skill

- The logos are black and white recolourings of the originals (owner, 2026-10-24), although jic-sf-ui-style
  says its logos are "never recoloured" and the purple JIC logo is for white grounds: a temporary departure
  until the official monochrome JIC file (requested from JIC) replaces it (design-decisions: the brand).
- `Example/Program/compare_programs.py`'s figure keeps its own palette (the dataviz one) and one light theme, and does not
  use `Report.Style` (owner: kept, 2026-10-27), although references/figures.md has every figure script use the one style
  module. Aligning it with `Report.Style` is a possible later task.
