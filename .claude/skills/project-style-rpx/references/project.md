# project-style-rpx: project reference for KINALI_Test_it_off_Simulation

The conventions of this repository that the generic skill leaves to the project, as
`.claude/project-docs/SPEC.md` decides them. The generic skill is personal (`~/.claude/skills/`,
`rparak/rp-claude-skills`); its checker `check_style.py` is part of it. This file overrides the
generic skill where they differ. The repository holds no `SKILL.md`: a personal skill wins over a
project skill of the same name, so a copy would be ignored and drift (`.gitignore` excludes one). ✅ = enforced (checker, test); 📄 = documentation only.

Project-only page in this folder: **mjcf.md** (the scene, the vendored robot, what is never changed).

**Status (2026-10-24):** in force; the code was migrated to it in `FEATURES.md` F3 (before: 13 files,
429 issues; after: 25 files, 0 issues).

## Header and checker

- Holder: **Roman Parak** - `Copyright (c) 2026 Roman Parak`, `Author   : Roman Parak`; licence MIT
  (`LICENSE`). The `#`-comment header form. `File Name:` examples: `../Scene/Core.py` for
  `src/Scene/Core.py`, `../Example/Simulation/run_cycle.py`, `../test/Scene/test_core.py`.
- `check_style.json` (this folder): class roots `src`, `Example`; test root `test`; check roots `src`,
  `Example`, `test`; no exclusions.
- Command: `.venv/bin/python ~/.claude/skills/project-style-rpx/scripts/check_style.py src Example test`.

## Layout and layers (SPEC §6)

```
src/<Domain>/Core.py   Configuration, Scene, Kinematics, Collision, Render, Simulation
Example/<Domain>/<script>.py   Simulation/run_cycle.py, Scene/export_cell_scene.py, Scene/model_fingerprint.py,
                       Render/show_cell.py, Render/render_image.py
test/<Domain>/test_core.py   mirroring src/; test/Example/<Domain>/test_<script>.py; test/Install/test_install.py;
                       test/Docs/test_layout.py; reference data (the baselines) in test/<Domain>/data/
models/                the MJCFs and their meshes (mjcf.md)
Data/<Purpose>/        PCB/pcb_meta.json, Paths/fork_close_open_path.npz, Reference/cell_plan.npz
Configuration/         (step 3) the cell, keypoint and program YAML; created with its first file
logger/                git-ignored: logs; Data/Generated/<Kind>/ git-ignored: generated artefacts
```

The scripts folder is `Example/`, one subfolder per `src/` domain a script belongs to (later `Keypoints`,
`Program`, `Media`); a script builds the repository root from `__file__` two folders up. Scripts may import
other scripts as modules (none does yet). `Data/` subfolders are named by purpose, Pascal case like the
domains.

Layer order 📄:

```
Configuration, Render     import nothing of the project
Scene                     → Configuration
Kinematics, Collision     → Scene
Simulation                → Configuration, Scene, Collision
Example/                  → any of src/
```

`Render` takes an `MjModel` / `MjData` and knows nothing of the cell. Keypoints, Program and
Trajectory are added in steps 3–4 as their own domains; no placeholder module before (skeleton rule).

## Root ✅

The root holds only `README.md`, `LICENSE`, `CLAUDE.md`, `install.sh`, `requirements.txt`,
`pytest.ini`, `.gitignore`, `.claude`, `src`, `test`, `Example`, `models`, `Data`.
Exception, with its reason: **`pytest.ini`** (the test runner reads it at the root). `logger/` exists on disk
but is git-ignored, so it is not a root entry. Pinned by `test/Docs/test_layout.py`.

## Formats - one reader and writer each 📄 (SPEC §9)

| Format | Read and written by |
|---|---|
| `models/scene.xml`, `models/aubo_i5/aubo_i5.xml`, `models/robot_tool/vgc10_opener_free.xml` | `Scene.Core` through MjSpec; never written (mjcf.md) |
| `models/cell_scene.xml` | written by `Scene.Core.Export_Cell_Scene`; read only by MuJoCo's viewer |
| `Data/PCB/pcb_meta.json` | `Configuration.Core.Read_PCB_Meta`; written by hand |
| `Data/Paths/fork_close_open_path.npz` | no reader now (the legacy planner, git tag `legacy-planner-reference`); step 4 adds one |
| `Data/Reference/cell_plan.npz` | `Simulation.Core.Read_Legacy_Plan`; written by the legacy planner (git tag `legacy-planner-reference`), never written now |
| the cycle report | `Simulation.Core.Format_Report`; printed and copied to `Data/Generated/Reports/report.txt` by `Example/Simulation/run_cycle.py` |
| the model fingerprint JSON | `Example/Scene/model_fingerprint.py` (`Model_Fingerprint`, `Compare_Fingerprints`) |
| `logger/*.log`, `logger/MUJOCO_LOG.TXT` | written by the scripts through `Configuration.Core` (`Script_Log_Cls`, `Configure_MuJoCo_Log`) and by MuJoCo; never read by code |

## Configuration

None yet (SPEC §8). The cell's values are `CONST_*` in `src/Scene/Core.py`, each with its source comment
kept from the verified code. Step 3 adds `Configuration/` and its loader in `src/Configuration/Core.py`.

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
- Reference data: `test/Simulation/data/baseline_report.txt`, `test/Scene/data/baseline_model_fingerprint.json`
  (since F3).
- Equivalence outputs: the printed report, the fingerprint (project-optimize reference).

## Project rules

- Logs: scripts print `[INFO]` / `[WARNING]` / `[ERROR]` lines (no `logging` module) and write every line
  to `logger/<script>.log` through `Script_Log_Cls.Line` (`print(log.Line(text))`); MuJoCo's own log goes
  to `logger/MUJOCO_LOG.TXT` (`Configure_MuJoCo_Log`, called first by every script and by
  `test/conftest.py`). `Example/Simulation/run_cycle.py` prints the report on stdout and its own lines on stderr, so
  stdout is the report byte for byte.
- Module-level private helpers: `_Pascal_Snake_Case` (e.g. `Scene.Core._Apply_Look`), like test
  helpers (the generic skill leaves it *Unsettled*; decided 2026-10-24).
- Stream protocols are not implemented in `src/` (no `write`/`flush` methods, which the naming rule would
  refuse); a script prints what `Script_Log_Cls.Line` returns instead.
- The verified physics values are never changed in a style or structure change (SPEC §3, mjcf.md).

## Departures from the generic skill

- `.claude/project-docs/FEATURES.md`: a fourth internal document, the feature tracker the owner asked
  for; the generic skill defines three.
