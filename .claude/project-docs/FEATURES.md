# Features

The feature tracker (the owner asked for one; feature-manager defines none, so this is a recorded
departure in the style project reference). Each feature: requirements restated, sub-tasks, status.
The specification is [SPEC.md](SPEC.md); the brief it was derived from is git `427e076:docs/PROJECT_BRIEF.md`.
Baseline: `docs/baseline_report.txt`, `docs/baseline_model_fingerprint.json`.

Status: `[ ]` open, `[x]` done and verified, `[~]` deferred.

## F1 - Restructure into the `kinali_sim` package (brief §7 step 1) - done, superseded by F3

Requirements:

1. Layout of brief §5.1: `src/kinali_sim/`, `models/`, `assets/`, `data/`, `config/`, the output folder, `legacy/`, `tests/`.
2. Every move is a `git mv` (history kept).
3. All file paths resolve through `kinali_sim.paths`, relative to the package, never to the working directory.
4. The compiled model is unchanged: fingerprint compare against the baseline reports 0 differences.
5. `kinali-sim run` reproduces `docs/baseline_report.txt` line for line, writes the report into the output folder, and exits non-zero
   on instability or a failed check.
6. `kinali-sim export-xml` writes `models/cell_scene.xml`, loadable from the repository root.
7. `legacy/test_cell_cycle.py` and `legacy/plan_cell.py` stay runnable; only imports and paths change.
8. Installable with `pip install -e .[dev]`, console script `kinali-sim`.

Sub-tasks:

- [x] A1 baseline report (`docs/baseline_report.txt`)
- [x] A2 model fingerprint tool + baseline (`tools/model_fingerprint.py`)
- [x] A3 brief to `docs/PROJECT_BRIEF.md`, `CLAUDE.md`
- [x] B1 `git mv` into the target layout
- [x] B2 `paths.py`, all asset / data paths updated
- [x] B3 `pyproject.toml`, editable install
- [x] B4 `cli.py`: `export-xml`, `run`
- [x] B5 tests: paths, model fingerprint, legacy regression (slow)
- [x] B6 verify: fingerprint 0 differences, report identical

## F2 - Green PCB (brief §7 step 2) - done

Requirements:

1. `pcb_vis` rgba `0.91 0.45 0.03 1` -> `0.05 0.42 0.18 1` in `models/scene.xml`; no other place sets the PCB colour.
2. PCB collision geoms untouched.
3. Fingerprint compare shows exactly one difference: `geoms/pcb_vis/geom_rgba`.
4. `kinali-sim run` still reproduces the baseline report.
5. Proof image `pcb_green.png` (then in the output folder; now `Data/Generated/Images/`) from camera `cam_pcb` at the initial state.
6. README stub: install, `kinali-sim run`, `kinali-sim export-xml`, viewer command. The previous Czech README is kept
   as `docs/README_cs_mg06_cell.md` until the full README (brief §7 step 7).

Sub-tasks:

- [x] C1 rgba change, grep for other PCB colour / material settings
- [x] C2 export, load and viewer check, proof image
- [x] C3 fingerprint: exactly one difference; report unchanged
- [x] C4 README stub
- [~] Follow-up: split the PCB mesh into board and components (components above the board top) and colour the
      components grey/black (brief §7 step 2, optional). Not done in F2.

## F3 - RPX migration (owner, 2026-10-24) - done

Restates the owner's request: the whole project follows project-style-rpx and the other personal
skills, with github.com/rparak/UR_PathGen_SF_1 as the structural template (structure and conventions
only, no code copied); this replaces brief §5.1 where they conflict. Physics, model and results do not
change. Includes the owner's addendum items B (requirements.txt), C (install.sh), D (re-baseline in
Python 3.11). Item A (`.claude/settings.json`, attribution off) is done (commit `1420e60`).

Requirements:

1. Layout of SPEC §6: `src/<Domain>/Core.py` without `__init__.py`, `Example/`, `test/` mirroring
   `src/`, `install.sh` + `requirements.txt` + `pytest.ini`, `models/`, `Data/`; every move a `git mv`.
2. RPX naming, headers (holder Roman Parak, MIT), docstrings, typing, `CONST_*`, `*_Error_Cls` in every
   file of `src`, `Example`, `test`; the style checker clean on them, or each finding justified.
3. The fingerprint compare against `docs/baseline_model_fingerprint.json` shows only
   `geoms/pcb_vis/geom_rgba`.
4. `Example/run_cycle.py` prints a report that diffs empty against the baseline report; exit 1 on a
   failed check, 2 on instability.
5. Fast tests pass; the slow regression passes SPEC §10.1 (criteria 1–7, penetration ≤ 0.5 mm included).
6. `./install.sh --recreate` works on this machine into a new `.venv` (Python 3.11); every script runs
   from any working directory; `python -m mujoco.viewer --mjcf=models/cell_scene.xml` loads.
7. Logs go to `logger/` (each script's printed lines, and MuJoCo's log), never to the root or the cwd.
8. Not changed: SPEC §3's list (physics), `legacy/` logic and numbers, `scene.xml` beyond a mesh path.
9. Not implemented: keypoints, program, trajectory (steps 3–4).

### Target `src/` parts and what goes into each

| Part | From | Holds |
|---|---|---|
| `src/Configuration/Core.py` | `src/kinali_sim/paths.py`; the JSON read in `model.py` | `CONST_ROOT`, `CONST_MODELS`, `CONST_DATA`, `CONST_OUTPUT`, `CONST_LOGGER`, `CONST_LEGACY` and the file constants; `PCB_Meta_Str`, `Read_PCB_Meta`; `Configure_MuJoCo_Log` (sets `MjLogConfig.logfile` to `logger/MUJOCO_LOG.TXT`, probe 2026-10-24: nothing is written to the cwd then); `Tee_Stream_Cls` (writes a script's lines to stdout and its log file; it is handed the streams, it prints nothing itself); `Configuration_Error_Cls`. The YAML loader comes in step 3 |
| `src/Scene/Core.py` | `model.py` | the cell constants (`CONST_ROBOT_BASE`, …, sources kept); `Crate_Center`, `PCB_In_Crate`, `Build_Cell_Spec`, `Build_Cell_Model`, `Export_Cell_Scene`; the look as private `__Apply_Look` (see decision 3) |
| `src/Kinematics/Core.py` | `kinematics.py` (IK half) | `Robot_Kinematics_Cls(model, data)`: `Set_Scene`, `Site_Pose`, `Solve_IK`; `Quaternion_To_Matrix`, `TCP_For_PCB` |
| `src/Collision/Core.py` | `kinematics.py` (contact half); `classify`, `cup_contacts` of the legacy run | `Build_Planning_Model` (the cell with the 6 mm geom margin on robot and PCB geoms), `Robot_Collision_Cls(model, data)`: `Contacts`, `Clearance`; `Contact_Classifier_Cls` (the five categories of SPEC §4.6, the fork exemption while closing / opening); `Cup_Contacts`; `CONST_COLLISION_CHECKS` |
| `src/Render/Core.py` | the render code of the green-PCB proof; the viewer command | `Render_Image(model, data, camera, width, height)`, `Launch_Viewer(model, data)`; no project imports |
| `src/Simulation/Core.py` | `legacy/test_cell_cycle.py` (its loop, ported, not moved) and `cli.acceptance` | `Legacy_Plan_Str`, `Read_Legacy_Plan`; `Cycle_Result_Str`; `Run_Cycle(model, plan)` with the instability guard (`Simulation_Error_Cls`); `Format_Report(result) -> list of lines`; `Check_Acceptance(result) -> list of Acceptance_Check_Str` with SPEC §10.1's limits as `CONST_*` |
| Keypoints / Program / Trajectory / Planner | - | not created now (skeleton rule: no placeholder module); steps 3–4 |

### Files: old → new

| Old | New |
|---|---|
| `src/kinali_sim/model.py` | `src/Scene/Core.py` (`git mv`) |
| `src/kinali_sim/kinematics.py` | `src/Kinematics/Core.py` (`git mv`); its contact half into `src/Collision/Core.py` |
| `src/kinali_sim/paths.py` | `src/Configuration/Core.py` (`git mv`) |
| `src/kinali_sim/cli.py` | split: `Example/run_cycle.py` (`git mv`), `Example/export_cell_scene.py`; `acceptance` → `Simulation.Core.Check_Acceptance` |
| `src/kinali_sim/__init__.py`, `pyproject.toml`, `config/.gitkeep`, the output folder's `.gitkeep`, `src/kinali_sim.egg-info` | removed |
| `tools/model_fingerprint.py` | `Example/model_fingerprint.py` (`git mv`) |
| (new) | `Example/show_cell.py`, `Example/render_image.py`, `src/Collision/Core.py`, `src/Render/Core.py`, `src/Simulation/Core.py` |
| `assets/mg06/`, `assets/machine/`, `assets/workstation/` | `models/mg06/`, `models/machine/`, `models/workstation/` (`scene.xml` meshdir `../assets` → `.`) |
| `assets/robot/aubo_i5/` | `models/aubo_i5/` |
| `assets/robot_tool/` | `models/robot_tool/` |
| `data/` | `Data/` |
| `tests/test_paths.py`, `test_model.py`, `test_cli.py`, `test_regression_legacy.py`, `conftest.py` | `test/Configuration/test_core.py`, `test/Scene/test_core.py`, `test/Example/test_run_cycle.py`, `test/Simulation/test_core.py`, `test/conftest.py` |
| `docs/baseline_report.txt` | `test/Simulation/data/baseline_report.txt` |
| `docs/baseline_model_fingerprint.json` | `test/Scene/data/baseline_model_fingerprint.json` |
| `docs/README_cs_mg06_cell.md` | stays (root exception until step 7) |
| `legacy/plan_cell.py`, `legacy/test_cell_cycle.py`, `legacy/cell_plan.npz` | stay; imports back to `import cell` / `import robot_kin as K` |
| (new) | `legacy/cell.py`, `legacy/robot_kin.py`: shims mapping the old names onto `src/` |

### Names: old → new

| Old | New |
|---|---|
| `paths.ROOT`, `MODELS`, `ASSETS`, `DATA`, `CONFIG`, `OUTPUT`, `LEGACY` | `Configuration.Core.CONST_ROOT`, `CONST_MODELS`, - (gone), `CONST_DATA`, - (step 3), `CONST_OUTPUT`, `CONST_LEGACY`; new `CONST_LOGGER` |
| `paths.SCENE_XML`, `CELL_SCENE_XML`, `ROBOT_XML`, `TOOL_XML`, `PCB_META`, `FORK_PATH`, `LEGACY_PLAN`, `REPORT` | `CONST_SCENE_XML`, `CONST_CELL_SCENE_XML`, `CONST_ROBOT_XML`, `CONST_TOOL_XML`, `CONST_PCB_META`, `CONST_FORK_PATH`, `CONST_LEGACY_PLAN`, `CONST_REPORT_FILE` |
| `model.P` (dict) | `Configuration.Core.Read_PCB_Meta() -> PCB_Meta_Str` (`Cad_Pos`, `Place_Pos`, `Hold_Pos`, `Quat`, `Y_Home`, `Y_Place`, `Z_Place`, `Jaw_Grip`, `Jaw_Open`, `Machine_Shift`) |
| `ROBOT_BASE`, `ROBOT_YAW`, `ROBOT_JOINTS`, `KP`, `KV`, `FMAX`, `ARMATURE`, `VMAX_DATASHEET` | `Scene.Core.CONST_ROBOT_BASE`, `CONST_ROBOT_YAW`, `CONST_ROBOT_JOINTS`, `CONST_KP`, `CONST_KV`, `CONST_FMAX`, `CONST_ARMATURE`, `CONST_VMAX_DATASHEET` |
| `CUP_R`, `CUP_N`, `VACUUM_PA`, `VACUUM_FORCE`, `CUP_XY`, `LOADER_TAU` | `CONST_CUP_R`, `CONST_CUP_N`, `CONST_VACUUM_PA`, `CONST_VACUUM_FORCE`, `CONST_CUP_XY`, `CONST_LOADER_TAU` |
| `CRATES`, `CRATE_FLOOR`, `CRATE_TOP`, `PCB_HALF_T`, `SUCTION_IN_PCB`, `R_TOOL_IN_PCB` | `CONST_CRATES`, `CONST_CRATE_FLOOR`, `CONST_CRATE_TOP`, `CONST_PCB_HALF_T`, `CONST_SUCTION_IN_PCB`, `CONST_R_TOOL_IN_PCB` |
| `crate_center`, `pcb_in_crate`, `build_spec`, `build_model`, `export_xml`, `style` | `Crate_Center`, `PCB_In_Crate`, `Build_Cell_Spec`, `Build_Cell_Model`, `Export_Cell_Scene`, `__Apply_Look` (private) |
| `kinematics.M`, `D`, `JID`, `QADR`, `DADR`, `LO`, `HI`, `SITE_TCP`, `SITE_FLANGE`, `ROBOT_BODIES`, `QA`…`QPCB` | properties of `Robot_Kinematics_Cls` / `Robot_Collision_Cls` (`Model`, `Data`, `Q_Address`, …); no module-level model any more (style: no hidden module state). The shim `legacy/robot_kin.py` rebuilds the old globals from one shared instance |
| `CLEAR` | `Collision.Core.CONST_PLANNING_CLEARANCE` |
| `set_scene`, `site_pose`, `ik`, `quat2mat`, `tcp_for_pcb` | `Robot_Kinematics_Cls.Set_Scene`, `.Site_Pose`, `.Solve_IK`; `Quaternion_To_Matrix`, `TCP_For_PCB` |
| `contacts`, `clearance` | `Robot_Collision_Cls.Contacts`, `.Clearance` |
| `cli.acceptance`, `PLACE_ERR_MM`, `PLACE_TILT_DEG`, `CRATE_DIST_MM`, `LID_OPEN_DEG`, `OPEN_TOL_DEG`, `N_PICKS`, `N_CUPS` | `Simulation.Core.Check_Acceptance`, `CONST_PLACE_ERROR`, `CONST_PLACE_TILT`, `CONST_CRATE_DISTANCE`, `CONST_LID_OPEN`, `CONST_OPEN_TOLERANCE`, `CONST_PICKS`, `CONST_CUPS`; new `CONST_PENETRATION` (0.5 mm) |
| `cli.cmd_run`, `cmd_export_xml`, `load_legacy_cycle`, `_Tee`, `main` | `Example/run_cycle.py: main`, `Example/export_cell_scene.py: main`, - (the run is in `src/`), `Configuration.Core.Tee_Stream_Cls`, - |
| legacy run: `run`, `mj`, `loader_target`, `robot_target`, `phase`, `classify`, `cup_contacts`, `load_plan`, `COLLISION_CHECKS` | `Simulation.Core.Run_Cycle`, `Min_Jerk`, `Loader_Target`, `Robot_Target`, `Phase_At`, `Collision.Core.Contact_Classifier_Cls.Classify`, `Cup_Contacts`, `Simulation.Core.Read_Legacy_Plan`, `Collision.Core.CONST_COLLISION_CHECKS`. The legacy file itself keeps its names (frozen) |
| fingerprint: `h`, `_names`, `_table`, `_ref`, `fingerprint`, `compare`, `build_model`, `main` | `Example/model_fingerprint.py`: `Hash_Value`, `Element_Names`, `Field_Table`, `Element_Reference`, `Model_Fingerprint`, `Compare_Fingerprints`, - (uses `Scene.Core`), `main` |

### Example/ scripts (each from `assets/script_template.py`, runnable from any directory)

| Script | Replaces | Does |
|---|---|---|
| `run_cycle.py [--plan FILE] [--out DIR]` | `kinali-sim run`, `python legacy/test_cell_cycle.py` | physics run of a plan; prints the report (stdout) and the acceptance (`[INFO]`/`[ERROR]` lines); writes `DIR/report.txt` (default then the output folder, now `Data/Generated/Reports/`); exit 0 / 1 / 2 |
| `export_cell_scene.py [--out DIR]` | `kinali-sim export-xml` | writes `cell_scene.xml` (default `models/`) |
| `show_cell.py [--headless]` | `python -m mujoco.viewer --mjcf=…` | the cell in the viewer (blocking `launch`, works under `python` on macOS); `--headless` builds and exits (for the test) |
| `render_image.py [--camera NAME] [--out DIR]` | the green-PCB proof | offscreen image, saved with matplotlib (`pcb_green.png` by default, decision 5) |
| `model_fingerprint.py [--compare FILE] [--out FILE]` | `tools/model_fingerprint.py` | writes or compares the fingerprint |

No `kinali-sim` console script: the skills use no package; the commands are the `Example/` scripts.

### Tests

`test/` per the skill, `pytest.ini` (`--import-mode=importlib`, `testpaths = test`, `slow` marker),
`test/conftest.py` puts `src/` on `sys.path` and routes MuJoCo's log to `logger/`. Run:
`.venv/bin/python -m pytest -m "not slow"` and `.venv/bin/python -m pytest -m slow`.

| File | Pins |
|---|---|
| `test/Configuration/test_core.py` | every constant path exists, from another cwd; `Read_PCB_Meta` values; a missing / malformed file is a `Configuration_Error_Cls` naming it; the MuJoCo log lands in `logger/` |
| `test/Scene/test_core.py` | the model sizes; the fingerprint equals the baseline except `pcb_vis` rgba; the PCB is green; the collision geoms' colours; every mesh of every MJCF exists; `cell_scene.xml` loads |
| `test/Kinematics/test_core.py` | IK round trip on brief §6 keypoints (TCP pose from FK, IK back, within 1e-5 m) |
| `test/Collision/test_core.py` | a known-colliding and a known-clear configuration; the classifier's five categories |
| `test/Render/test_core.py` | an image of the requested size whose `pcb_vis` pixels are green |
| `test/Simulation/test_core.py` | `Check_Acceptance` per criterion (synthetic results); `Format_Report` on a fixed result; *slow*: the full run, §10.1 1–7, and the report equal to `test/Simulation/data/baseline_report.txt` |
| `test/Example/test_<script>.py` | each script in a fresh interpreter from `tmp_path`: exit codes, outputs in `--out`, logs in `logger/`, nothing written to the cwd |
| `test/Install/test_install.py` | `bash -n`, the OS detection, the Python 3.11 search, `--detect-only`, the log format; sourced with stubs (template pattern) |
| `test/Docs/test_layout.py` | the root entries (written out), the three project docs tracked, `.claude/skills` tracks only `references/` files (through `git ls-files`, so the git-ignored local copies do not count) |

### logger/

Git-ignored, created by `install.sh` and by `Configuration.Core` when a script starts. Contents:
`logger/<script>.log` (the script's printed lines, overwritten per run) and `logger/MUJOCO_LOG.TXT`
(MuJoCo's own log, through `MjLogConfig.logfile`; the attach warning lands there). No `logging` module
(style rule). The tests route MuJoCo's log there too (conftest).

### Install (addendum B, C)

`requirements.txt` with exactly the owner's pins and header:

```
# Pinned for Python 3.11 (macOS Apple silicon, Ubuntu Linux); installed and verified <date verified>.
# Simulation, kinematics and collision queries (src/Scene, src/Kinematics, src/Collision, src/Simulation; scipy planned)
mujoco==3.16.0
numpy==2.4.6
scipy==1.17.1
# Cell configuration (planned: the YAML loader of step 3)
PyYAML==6.0.3
# Collision-set build only (planned: no collision-set build in this project yet); the tests never run CoACD
trimesh==5.1.1
rtree==1.4.1
coacd==1.0.14
# Offscreen images (Example/render_image.py) and README figures (planned)
matplotlib==3.11.2
# Tests
pytest==9.1.1
```

`install.sh`: `#!/usr/bin/env bash`, the licence header, `set -euo pipefail`, functions per
project-style-rpx references/shell.md and the template (`Log`, `Die`, `Detect_Platform`, `Find_Python`,
`Ensure_Venv`, `Install_Requirements`, `Verify_Install`, `Make_Folders`, `Print_Next_Steps`, `Main`),
log lines `YYYY-MM-DD HH:MM:SS [LEVEL  ] message`. Options `--detect-only`, `--recreate`. Steps: detect
Darwin / Linux; find Python 3.11 (decision 7); create or reuse `.venv`; `pip install --upgrade pip` and
`-r requirements.txt`; import mujoco, numpy, scipy, yaml and print the versions, fail unless mujoco is
3.16.0; create `logger/` and the output folder (now `Data/Generated/`); print the next commands (cycle run, tests, viewer). No sudo, no
global installs, re-runnable. Tested on this machine with `--recreate`.

### Re-baseline (addendum D)

After `install.sh`: the cycle and the fingerprint compare in the 3.11 `.venv`. Identical → the 3.11
environment is recorded as the baseline environment (SPEC §4.6, KNOWN_ISSUES item closed). Different →
the diff is shown, SPEC §10.1 checked, a KNOWN_ISSUES entry added, and the 3.11 report committed alone
as "Re-baseline for pinned Python 3.11 environment" (the old one stays in history). Fingerprint: only
`pcb_vis` rgba allowed.

### Header and licence text

Every `.py` (and `install.sh`) opens with `project-style-rpx/assets/header.txt`: `MIT License`,
`Copyright (c) 2026 JIC Smart Factory`, the MIT text, `Author   : Roman Parak`, `File Name: ../<path>`.

### legacy/

Stays until step 6, outside the checker's roots. `plan_cell.py` and `test_cell_cycle.py` go back to
their original imports (`import cell`, `import robot_kin as K`, the folder put on `sys.path` from
`__file__`); the shims `legacy/cell.py` and `legacy/robot_kin.py` (new, plain style, outside the
checker) expose the old names on top of `src/`. Kept: `load_plan`, the result dict and `--out` of F1.
`test_cell_cycle.py` still prints the identical report; `plan_cell.py` still regenerates the plan to
≤ 1e-12 rad, which proves the ported Kinematics and Collision.

### Conflicts and how they are resolved

| # | Between | Conflict | Resolution |
|---|---|---|---|
| 1 | brief §5.1 vs skills | package `kinali_sim`, `pyproject.toml`, `tests/`, `config/`, console script | skills (owner): see the tables above |
| 2 | skills vs template | the template has no `logger/`; the skills forbid `logging` | printed `[INFO]` lines, duplicated to `logger/<script>.log` by a stream tee; MuJoCo's log routed there |
| 3 | owner's expected parts vs template | Render with the look/colours | the look stays in `Scene` (decision 3) |
| 4 | skills (`project-bootstrap`) vs owner | the bootstrap commits the SPEC alone after approval | committed now as *Draft* with the references, as the owner asked; set to *Approved* on approval |
| 5 | skills (one reader per format) vs legacy | the frozen `test_cell_cycle.py` reads `cell_plan.npz` beside `Simulation` | recorded exception until step 6 |
| 6 | skills (no SKILL.md in the repo) vs owner ("copies stay git-ignored") | `.claude/skills/` was a nested git clone of `rp-claude-skills` (same commit `c5f16c7` as `~/.claude/skills`, clean): git cannot track files inside an embedded repository, so the references could not be committed beside it | the clone was moved out of the repository (2026-10-24, not deleted; restore: `git clone https://github.com/rparak/rp-claude-skills .claude/skills`); `.claude/skills/` holds only the references; `.gitignore` excludes any `SKILL.md`; the layout test checks git |
| 7 | shell.md (never delete `.venv`) vs addendum C | `--recreate` deletes `.venv` | only with the explicit flag; without it a broken `.venv` is reported, never deleted |
| 8 | shell.md (never a bare `python3`) vs addendum C | "then a python3 that is 3.11" | decision 7 |
| 9 | skills (no optional dependency, nothing the SPEC does not define) vs addendum B | scipy, trimesh, rtree, coacd have no user | kept pinned and marked *planned* (owner's list); decision 6 |
| 10 | addendum B vs F2 | Pillow saved `pcb_green.png` but is not in the pin list | matplotlib saves it (decision 5) |
| 11 | skills (root holds only declared entries) | `legacy/`, `docs/` | temporary root exceptions with reasons (style project reference) |
| 12 | skills (three project docs) vs owner | the tracker `FEATURES.md` | kept in `.claude/project-docs/` as a recorded departure |
| 13 | style (no hidden module state) vs `kinematics.py` | it builds a model at import | classes taking a model; the shim rebuilds the old globals for legacy |
| 14 | brief vs skills | `docs/PROJECT_BRIEF.md` would duplicate the SPEC | removed from the tree; SPEC cites git `427e076` |

### Decisions (answered by the owner, 2026-10-24)

1 Roman Parak (corrected 2026-10-24: holder JIC Smart Factory, author Roman Parak; design-decisions). 2 port into `src/Simulation` now; proof: empty report diff against the baseline and
against the frozen legacy run. 3 the look stays in `Scene`. 4 meshes into `models/<name>/`. 5 matplotlib;
the broken legacy `--render` goes to `KNOWN_ISSUES.md`. 6 keep as *planned*. 7 no bare `python3`: only
`python3.11` and explicit absolute candidates. 8 simulation now, real-cell transfer planned (SPEC §17).
Conditions: C1 (skills resolve from a fresh session) done; C2 (SPEC covers the brief) done. Part 2 rules:
case-only renames through a temporary name; after every commit the fingerprint, the fast tests and, for
Scene / Kinematics / Collision / Simulation / models, the cycle diff.

The questions as asked:

1. **Holder** in every header: `Roman Parak` (from `LICENSE`, recommended) or another name (the
   template uses JIC Smart Factory).
2. **Port the physics loop into `src/Simulation`** (recommended; proven by the empty report diff
   against both the baseline and the frozen legacy run), or keep `Example/run_cycle.py` a wrapper around
   `legacy/test_cell_cycle.py` until step 5.
3. **The look in `Scene`** (recommended: lights, materials and cameras are part of the compiled model and
   its fingerprint; `Render` stays model-agnostic), or in `Render` as `Apply_Look(spec)` with
   `Scene → Render`.
4. **Meshes into `models/<name>/`** (recommended: one root entry fewer, like the template's
   `models/`), or keep `assets/`.
5. **`pcb_green.png` with matplotlib** (`imsave`, recommended: in the pin list), or add `Pillow` with a pin.
   Note: the legacy `--render` video option needs `imageio` and Pillow and will not work without them.
6. **Unused pins** (scipy, trimesh, rtree, coacd; matplotlib if decision 5 is not matplotlib): keep as
   *planned* (recommended, the owner's list), or drop until used.
7. **Python lookup**: `/opt/homebrew/bin/python3.11`, then `python3.11` on `PATH`, then a bare `python3`
   only if it reports 3.11 (addendum) - the skills forbid calling a bare `python3` (on this machine it is
   miniforge's, version check only would be harmless but still a call). Recommended: drop the bare
   `python3` step.
8. **Safety sentence** (SPEC §14 question 1): does an output ever drive the real cell?

### Sub-tasks and commits (done 2026-10-24)

Checks after every commit: the fingerprint compare (only `geoms/pcb_vis/geom_rgba`), the fast tests, and
for Scene / Kinematics / Collision / Simulation / models the cycle report diff against the baseline.

- [x] F3.1 `9bcbd77` install: `requirements.txt`, `install.sh` (`--recreate` run: Python 3.11.2, numpy
      2.4.6), `test/Install/test_install.py` (16), `pytest.ini`, `pyproject.toml` removed. Re-baseline (D):
      the 3.11 report is byte-identical to the 3.14 one; no new baseline. Fingerprint 1 diff; fast 43
      passed; report diff empty.
- [x] F3.3 `c6282af` Configuration + Scene; meshes into `models/<name>/`; `data` -> `Data` through a
      temporary name; `legacy/cell.py`. Fingerprint 1 diff; fast 52 passed; report diff empty (legacy and
      CLI); plan regeneration 1.24e-14 rad.
- [x] F3.4 `0dc7dad` Kinematics + Collision; `legacy/robot_kin.py`. Fingerprint 1 diff; fast 79 passed;
      report diff empty; plan regeneration 1.24e-14 rad.
- [x] F3.5 `803d94d` Render + Simulation (the loop ported). Fingerprint 1 diff; fast 97 passed; report
      diff empty (legacy and port); every raw value the same float as the frozen legacy run.
- [x] F3.6 `9c21513` Example scripts, `test/` complete, layout test; `src/kinali_sim`, `tests/`,
      `tools/` removed. Fingerprint 1 diff; fast 99 passed (103 collected); report diff empty
      (`Example/run_cycle.py` stdout and `report.txt`, legacy); style checker 25 files, 0 issues.
- [x] F3.7 docs: SPEC, CLAUDE.md, README stub commands, the skill references, KNOWN_ISSUES.

Mutations that changed nothing (findings about the code, not the tests): the phase label sampled every 10
instead of 20 steps, and a 0.1 mm instead of 0.5 mm cup-touch threshold, change no output of the legacy
plan (every phase starts on a 20-step boundary; the cups sit deeper than 0.1 mm).

## F4 - Root clean-up: legacy, docs and the output folder removed; Data/ and Example/ in subfolders (owner, 2026-10-24) - done

Requirements:

1. Annotated tag `legacy-planner-reference` on the last commit with the legacy planner; recorded in
   design-decisions and in SPEC (fork path, keypoints).
2. `legacy/cell_plan.npz` kept as the regression input under `Data/`; the planner, the legacy cycle test and
   the shims deleted; the ported run still diffs empty against the baseline report.
3. `docs/README_cs_mg06_cell.md` compared with the project docs; what is still valid and missing moved there
   (or into a step 7 README note here); then deleted.
4. `Data/` and `Example/` in subfolders by domain or purpose (style skill), the step 3 YAML location decided
   (no files); `test/Example/` mirrors `Example/`.
5. The root output folder replaced by one rule for generated artefacts, recorded in design-decisions; none left.
6. Every move a `git mv`; after every commit: fingerprint (only pcb_vis rgba), fast tests, the cycle diff,
   the style checker 0 issues. Final check in a fresh clone.

Sub-tasks:

- [x] F4.1 Tag and remove legacy planner
- [x] F4.2 Remove docs/ after moving remaining content
- [x] F4.3 Structure Data/ and Example/ into subfolders
- [x] F4.4 Replace the output folder with the generated-artefacts rule (logger/ + Data/Generated/<Kind>/)
- [x] F4.5 Fresh-clone check (2026-10-24, at 5fb719b): ./install.sh exit 0 (Python 3.11.2, mujoco 3.16.0); full
      pytest 103 passed in 46 s; Example/Simulation/run_cycle.py exit 0, stdout and
      Data/Generated/Reports/report.txt diff empty against the baseline, nothing in the working directory;
      the viewer loaded models/cell_scene.xml (10 s, no error). git grep: 'output/' only in the brief's
      verbatim quotes in SPEC §2 and §16; no 'out/' path.

Per-commit checks (fingerprint / fast tests / cycle diff / style): bf210a3, c3ccae9, 50a9793, 5fb719b - each
1 difference (pcb_vis rgba) / 99 passed / empty / 0 issues.

## F5 - Logos and the JIC Smart Factory × KINALI brand (owner, 2026-10-24)

Requirements: black SVG versions of the JIC Smart Factory, KINALI and Test-it-off logos (white for dark
grounds), by official vector (a), recoloured vector (b) or, only after the owner's approval of a tool,
traced raster (c); never a PNG wrapped in SVG; clean SVGs (viewBox, no width/height, no metadata); originals
kept; a deterministic script in Example/Media/; a check image in Data/Generated/Images/; Media/ in the root
layout; the brand and the open Phase 2 toolkit recorded; a README logo header. No change in src/, the
physics or the model.

- [x] Inventory: JIC Smart Factory (SVG, vector, purple #640aba, identical to jic-sf-ui-style's asset) -> b;
      KINALI (SVG, vector, white #fff) -> b; Test-it-off (PNG 2608 x 1768, raster, colour on white) -> c,
      stopped for the owner's decision on a tracing tool.
- [x] Example/Media/make_logo_variants.py, Media/JIC_Smart_Factory_*.svg, Media/KINALI_*.svg, logo_check.png
- [x] Tests (15), each rule broken once; layout test with Media
- [x] SPEC §1, §2, §6, §9; design-decisions (brand; Phase 2 toolkit open); KNOWN_ISSUES (raster logo); style
      and readme-writer references; README header and context
- [x] Test-it-off: the logo is not used (owner, 2026-10-24); the PNG removed, the product named only.
- [x] The name "Test-it-off Simulation" and the spelling Test-it-off everywhere (design-decisions).

## F6 - Step 3: cell configuration and keypoints (owner, 2026-10-24) - done

Requirements (restated): the cell configuration and the named keypoints in YAML in the root `Configuration/`
(SPEC §6, §8); one loader with validation; IK resolution by family and seed; posing the whole cell at a
keypoint in the viewer; a headless keypoint check; the 22 keypoints of SPEC §4.8 at full precision; a
controller-friendly format (SPEC §17). No change to the physics, the model, the cycle run or the baseline.

### Measured for this design (2026-10-24, read-only)

- **Full precision.** Every §4.8 keypoint is a segment end of `Data/Reference/cell_plan.npz` where the plan
  holds still (joint speed exactly 0 over the neighbouring samples; `home` = the file's `q_home`). Those rows
  differ from the §4.8 table by at most 7.6e-5 rad, and their FK differs from the table's TCP / flange
  position by at most 0.081 mm (`loader_approach`) - within the table's rounding (1e-4 rad, 0.1 mm).
- **Relative definitions** reproduce the plan: `input_pcb_press` = the TCP holding the PCB in the input crate
  + 1.5 mm along tool +Z: 24 µm, 0.0000°; `loader_handover` = the TCP holding the PCB at the jaws' hold pose:
  19 µm; `output_place` = holding the PCB in the output crate - 1 mm along tool +Z: 19 µm. The residue is the
  legacy IK's tolerance, so a stale-resolution tolerance of 1 µm would be wrong: proposed 0.05 mm / 0.01°.
- **Contacts within 3 mm** (planning model, robot side): `home` none; `input_pcb_press` the four cups 1.5 mm
  into the PCB; `fork_closed_end` the fork 6.0 mm into the handle bar (the tight recorded path, SPEC §4.7);
  `output_place` the cups on the PCB (0.0 mm). The carried PCB is not a robot body: the check needs a second
  query, the PCB against the scene (the jaws at the handover, the crate floor at the place).
- **Viewer keys.** Every letter and `, / ; ' \` `` ` `` toggles a visual or render flag (`mjVISSTRING`,
  `mjRNDSTRING`); the viewer also binds Space, Backspace, the arrows, Tab, Esc, `[ ]`, `- =`, F1-F5, 0-5.
  Not in those: `.`, Page Up / Page Down, Home / End. A key callback needs `mujoco.viewer.launch_passive`,
  which on macOS runs only under `.venv/bin/mjpython` (present).
- **Model limits.** The robot joints are ±3.04 rad in the vendored `models/aubo_i5/aubo_i5.xml` (and
  `ctrlrange ±3.04` set in `Scene.Core`); the loader ranges, MG06 lid / handle ranges in `models/scene.xml`.

### Proposal

**1. `Configuration/Cell.yaml`** (the template's `Configuration/Cell_SF1.yaml` form; keys Pascal_Snake_Case,
every field `REQUIRED.` with unit and source; angles in deg, lengths in m, converted once by the loader):

| Section.Field | Unit | Source | Read by (now) |
|---|---|---|---|
| `Robot.Base_Position`, `Robot.Base_Yaw` | m, deg | CAD (top of `deska_robot`), SPEC §4.2 | `Scene` (today `CONST_ROBOT_BASE/YAW`) |
| `Robot.Joint_Limits` | deg | the vendored MJCF ±3.04 rad - **question 1** | the check (and `Scene`'s ctrlrange) |
| `Robot.Speed_Fraction`, `Robot.Datasheet_Speeds`, `Robot.Acceleration_Limits` (*assumed*) | -, deg/s, deg/s² | AUBO i5 datasheet; SPEC §4.4 | step 4 - **question 2** |
| `Vacuum.Pressure`, `Vacuum.Cup_Diameter` (force derived, not stored) | Pa, m | VGC10 datasheet, SPEC §4.3 | `Scene` (today constants) |
| `MG06.Open_Lid`, `MG06.Closed_Lid`, `MG06.Closed_Handle` | deg | scene stop; SPEC §4.7 (−0.5°, 28.65°) | posing, the check |
| `Clearance.Robot` (3 mm), `Clearance.Stale_Position` (0.05 mm), `Clearance.Stale_Angle` (0.01°), `Clearance.Check_Position` (0.1 mm) | m, deg | SPEC §2, measured above | the check, `Collision` (planning margin = 2 × clearance, derived) |
| `Acceptance.*` (0.5 mm, 0.2°, 2 mm, 2°, 0.5 mm penetration) | m, deg | SPEC §10.1 | `Simulation.Check_Acceptance` (today constants) |
| `Simulation.Phase_Steps` (20), `Simulation.Cup_Touch` (0.5 mm) | -, m | the legacy run; no effect on the reference plan (F3.5); sensitivity in step 6 | `Simulation`, `Collision` |

Values that move from code constants into the file are written so they convert to the same floats; the
fingerprint and the report diff prove it after the move (one commit).

**2. `Configuration/Keypoints.yaml`** - header: angles [deg], lengths [m], joint order shoulder_pan,
shoulder_lift, elbow, wrist_1, wrist_2, wrist_3 (the model's `aubo/<name>` joints), frames named
(`suction_tcp`, `tool_flange`, `fork_center`; the MG06 fixture frame for positions). Sections:

- `Robot.<keypoint>`: `Joints` (REQUIRED, the source of truth, deg with 6 decimals: the round trip moves the
  pose < 0.1 µm), `Family` (REQUIRED, `wrist_up` / `wrist_down`), `Context` (REQUIRED: the scene the keypoint is
  checked and shown in - `Loader`, `Gripper`, `PCB`, `MG06`), `Note` (REQUIRED); optional (a recorded project
  exception to "no optional fields"): `Check` (frame + position [+ rpy] as a check value), `Relative_To`
  (`Target` = another keypoint or `pcb_hold@<location>`, `Frame`, `Offset` [m] in that frame's axes),
  `Allow_Contact` (named groups: `cups_pcb`, `handle_bar`, `pcb_jaws`, `pcb_crate`).
- `Loader.<position>`: `Y`, `Z` [m] (home, travel_place, place, park). `Gripper.<state>`: [m] per jaw (open,
  grip, clamp). The PCB locations are code constants (a single legal set): `input_crate`, `cups`, `jaws`,
  `mg06`, `output_crate`.

```yaml
Robot:
  home:
    # The parking pose, tool down. Full precision from Data/Reference/cell_plan.npz (q_home); SPEC §4.8.
    Joints: [45.242793, 39.593430, 105.038309, -24.555084, 89.999924, 0.242831]   # [deg] REQUIRED.
    Family: wrist_up
    Check: {Frame: suction_tcp, Position: [0.3000, -0.5200, 0.4000]}               # [m] SPEC §4.8
    Context: {Loader: home, Gripper: open, PCB: input_crate, MG06: open}
    Note: Start and end of the cycle.
  input_pcb_press:
    Joints: [26.638999, -20.811883, 122.063611, 52.875494, 90.000000, -18.361001]
    Family: wrist_up
    Relative_To: {Target: pcb_hold@input_crate, Frame: suction_tcp, Offset: [0, 0, 0.0015]}   # [m] 1.5 mm press
    Allow_Contact: [cups_pcb]
    Context: {Loader: home, Gripper: open, PCB: input_crate, MG06: open}
    Note: Cups pressed 1.5 mm onto the PCB, then vacuum on (SPEC §4.7).
  loader_handover:
    Joints: [-24.615662, 4.968975, 123.008998, 24.903697, 81.624008, -69.385948]
    Family: wrist_up
    Relative_To: {Target: pcb_hold@jaws, Frame: suction_tcp, Offset: [0, 0, 0]}
    Allow_Contact: [cups_pcb, pcb_jaws]
    Context: {Loader: home, Gripper: open, PCB: cups, MG06: open}
    Note: PCB at the hold pose, no press - 0.3 mm above the finger ledges (SPEC §4.7).
  fork_closed_end:
    Joints: [-29.547477, 1.304953, 138.071309, -43.231989, -74.548206, 65.994483]
    Family: wrist_down
    Check: {Frame: tool_flange, Position: [0.0550, -0.2840, 0.3154]}
    Allow_Contact: [handle_bar]
    Context: {Loader: park, Gripper: open, PCB: mg06, MG06: closed}
    Note: End of the recorded closing path (path fork_close, SPEC §4.9); the bar on the slot bottom.
  output_place:
    Joints: [59.945628, -12.144714, 134.701735, 56.846449, 90.000000, 14.945628]
    Family: wrist_up
    Relative_To: {Target: pcb_hold@output_crate, Frame: suction_tcp, Offset: [0, 0, -0.001]}   # 1 mm above the floor
    Allow_Contact: [cups_pcb, pcb_crate]
    Context: {Loader: home, Gripper: open, PCB: cups, MG06: open}
    Note: PCB 1 mm above the output crate floor, then vacuum off.
Loader:
  home: {Y: -0.1000, Z: 0.0}            # [m] SPEC §4.4
  park: {Y: -0.1995, Z: 0.020}
Gripper:
  open: 0.035                           # [m] per jaw, SPEC §4.4
  clamp: 0.00575                        # jaw_grip - 12 mm: the force-limited clamp command
```

**3. Determinism.** `Joints` is REQUIRED for every robot keypoint and is the only thing posing and playback
read: no IK at show or play time. `Relative_To` and `Check` are check values. `check_keypoints.py` recomputes the
target and fails a keypoint whose `Joints` do not reach it within `Clearance.Stale_Position` /
`Stale_Angle` ("stale"), naming the difference and the command that prints the corrected line:
`Example/Keypoints/resolve_keypoint.py --name <k>` (IK by `Family`, seeded from the keypoint's current joints or
`--seed <keypoint>`, refused if the solution changes family). The script prints the `Joints:` line; no tool
writes the YAML (project-style-rpx references/yaml.md) - the owner pastes it, git records it.

**4. Precision.** The 22 `Joints` come from the plan's stationary segment-end rows (`q_home` for home),
rounded to 6 decimals of a degree. Tests: each within 1e-4 rad of the §4.8 table, FK within 0.1 mm of the
§4.8 position, and within 1 µm of the plan row's FK. The method is recorded in SPEC §4.8.

**5. API.** `Configuration.Core` stays the only YAML reader (the template's `Load_Configuration`,
`Load_Job`): `Load_Cell_Configuration(path) -> Cell_Configuration_Str`, `Load_Keypoints(path) -> Keypoints_Str`
(`Robot_Keypoint_Str`, `Loader_Position_Str`, ...), every problem collected and named by file and key path
(`Configuration/Keypoints.yaml: Robot.home.Joints: 5 values, 6 required`), unknown keys returned as WARNING
notes. A new domain `src/Keypoints/Core.py` (→ Configuration, Scene, Kinematics, Collision): `Target_Pose`,
`Resolve_Keypoint`, `Pose_Cell` (robot + loader + jaws + PCB at a location + MG06 open / closed into an
`MjData`), `Check_Keypoints -> List[Keypoint_Check_Str]`, `Format_Check_Table`. `Collision.Core` gains the
carried-PCB clearance query. `Scene`, `Simulation`, `Collision` read the moved values from
`Cell_Configuration_Str` (the scripts and tests load it once).

**6. Scripts (`Example/Keypoints/`)**: `show_keypoint.py --name <k> [--loader <pos>] [--gripper <state>]
[--pcb <location>] [--mg06 open|closed] [--headless] [--image FILE]` (blocking viewer under python;
`--image` writes an offscreen render into `Data/Generated/Images/`); `step_keypoints.py` (passive viewer
under `mjpython` on macOS; **Page Down** next, **Page Up** previous, `.` as an alternative - question 4);
`check_keypoints.py` (the table, exit 1 on a failure); `resolve_keypoint.py --name <k> [--seed <k>]`.

**7. Check rules**, one row per keypoint: joint limits (from the model); `Family` = sign of wrist_2 (a
WARNING within 1° of zero: `via_mg06_high` is at −0.94°); `Check` within 0.1 mm (and 0.1° if a rotation is
given); `Relative_To` within the stale tolerance; clearance ≥ 3 mm from the robot and tool - and the PCB when
`Context.PCB` is `cups` - to MG06, the workstation, the crates and the loader, except the `Allow_Contact`
groups (each allowed contact is listed with its depth, so an allowed contact that grows is visible). Table:
keypoint | family | limits | check | stale | min. clearance (pair) | allowed contacts | verdict; exit 1 on
any FAIL.

**8. KNOWN_ISSUES**: the model's joint zeros and directions against the real AUBO i5 controller are not
verified (recorded now).

### Approval (owner, 2026-10-24) and condition C1

Answers: 1 joint limits and loader ranges from the model; 2 speeds and accelerations in step 4; 3 the loader
positions move from `pcb_meta.json` to `Keypoints.yaml`, `machine_shift` removed (the loader shift is in SPEC
§4.7); 4 Page Down / Page Up and `.`, `mjpython` documented; 5 Pascal_Snake_Case keys; 6 no optional fields:
`Check` and `Fixture` required, `Allow_Contact` always present ([] when none), `Relative_To` always present
(null when unused).

C1 (the fork 6.0 mm into the bar at `fork_closed_end`): the probe posed MG06 at the settled closed state
(lid −0.5°, handle 28.65°, press 0) while `fork_closed_end` is the deepest point of the closing path, where
the recorded handle stands at 29.13° and the lid at 0.00° (row 286). Exact fork-bar distance
(`mj_geomDistance`, the physics model; the planning model's margin and the coupled press change nothing):

| Keypoint | MG06 state | Fork-bar |
|---|---|---|
| `fork_closed_end` | settled closed (the probe) | −5.95 mm |
| `fork_closed_end` | recorded row 286 (CLOSE): lid 0.00°, handle 29.13° | **−0.33 mm** |
| `fork_bar_centred` | settled closed (planned there; nearest recorded row 1.9 mm away) | **+3.93 mm** |
| `fork_open_join` | recorded row 663 (OPEN, the join: handle 28.32° ≤ 28.45°) | **+0.08 mm** |

Design change: `Fixture` (REQUIRED): `Open`, `Closed` (Cell.yaml `MG06`) or the recorded `{Lid, Handle,
Source}`; the press plate follows the handle through the model's equality (`press = 0.0104554 · handle³`,
read from the compiled model). `fork_start` row 23, `fork_closed_end` row 286, `fork_open_join` row 663;
`fork_bar_centred`, `fork_out_of_bar`, `fork_retreat_up` Closed.

### Implementation findings (2026-10-24)

- `loader_take_press` was posed with the gripper at `clamp` (the force-limited command, 12 mm behind the part):
  the jaw went 3.53 mm into the tool. The jaws stand at `grip` on the PCB: its Context is `Gripper: grip`.
- MuJoCo's box-box collider makes no contact for near boxes, whatever the margin: the check uses exact
  distances (design-decisions). They found the cup block 2.10 mm above a held PCB (0.62 mm while pressing):
  part of `cups_pcb`; and the robot's base on its plate (the mount, not checked).
- `pcb_jaws` was declared at the handover and the take but is never present: removed (allow only what is
  present, C1); a test holds every declared group to being observed. The group itself is removed too
  (owner, 2026-10-24: config, loader, check, tests; design-decisions); it comes back with a keypoint or step 4
  segment that observes a jaw contact.
- `fork_closed_end` with MG06 **Open**: the handle is rotated away, the fork is in free space (24.6 mm from
  MG06's base) - no collision, unlike the owner's expectation; with MG06 settled **Closed** the bar is 5.95 mm
  into the fork and the check fails. Both pinned; accepted by the owner (2026-10-24).

### Status (done 2026-10-24)

Checks after every commit: the fingerprint compare (only `geoms/pcb_vis/geom_rgba`), the cycle report diff
against the baseline, the fast tests, the style checker (0 issues).

- [x] F6.1 `e78aa8f`: `Configuration/Cell.yaml`, `Configuration/Keypoints.yaml` and their loader; the moved
      constants convert to the same floats (fingerprint and cycle report unchanged).
- [x] F6.2 `56ae14f`: `src/Keypoints/Core.py` (posing, check, resolution), exact clearance
      (`Collision.Close_Pairs`); the check: 0 FAIL, 1 WARN (`via_mg06_high`, the family switch), 21 PASS.
- [x] F6.3 `1c011b3`: `Example/Keypoints/` (check, resolve, show, step), `Render_Free_Image`, `Launch_Passive_Viewer`,
      the script tests. The viewer keys (Page Down / Page Up / `.`) tested by the owner: they work.
- [x] F6.4 `b15f6ec`: the unused contact group `pcb_jaws` removed (config, loader, check, tests, docs); the check
      table byte-identical (0 FAIL, 1 WARN, 21 PASS); 191 collected. Fingerprint 1 diff; report diff empty; fast
      187 passed; style checker 38 files, 0 issues.

Closing answers (owner, 2026-10-24) to the step 3 report:

1. The fixture-state test (`fork_closed_end`: settled Closed fails, the recorded state passes, Open passes):
   accepted.
2. The viewer keys Page Down / Page Up / `.`: tested by the owner, they work.
3. `pcb_jaws`: removed (F6.4); it comes back with a keypoint or step 4 segment that observes a jaw contact.
4. The resolver against the stored joints (up to 0.003°, within the stale tolerance): the stored values stay.
5. The Phase 2 toolkit stays open until step 8 (design-decisions).

### Questions (Part A)

1. Joint limits and loader ranges: keep the model (vendored MJCF, `scene.xml`) as their only source and let
   the check read them from the compiled model (recommended: one source of truth, nothing patched), or put
   them into `Cell.yaml` and patch the model from it (the template's way for its robot)?
2. Speeds and accelerations: add them in step 4 with their reader, the trajectory (recommended: the skills
   allow no field without a reader), or now as fields read by nothing yet?
3. `Data/PCB/pcb_meta.json` holds the loader positions (`y_home`, `y_place`, `z_place`, `jaw_grip`,
   `jaw_open`) and an unread `machine_shift`. Move the loader positions into `Keypoints.yaml` (one source;
   `Simulation` reads the home position from there; a format change) and drop `machine_shift` (recorded in SPEC
   §4.7 already)?
4. Viewer keys: Page Down / Page Up (plus `.`) - not verifiable from here; the owner presses them once.
5. YAML keys Pascal_Snake_Case (the skill) instead of the lowercase `joints`, `tcp`, `family` of the request;
   the keypoint names stay lowercase (they are names, SPEC §4.8).
6. The optional fields (`Check`, `Relative_To`, `Allow_Contact`) as a recorded exception to the skill's "no
   optional fields".

## F7 - Studio look: grey sky, dark 1 m grid floor, key light and fill (owner, 2026-10-24) - done

Requirements (restated): an Isaac Sim / Isaac Lab "studio" look for the viewer and the offscreen renders - a grey
gradient sky, haze and fog in the horizon colour, a dark 1 m grid floor with a subtle reflection reaching the fog,
one key spot light with shadows plus a weak directional fill, a tight `statistic`; visual only (no geom, body or
material other than the floor's changes; the cycle report and the keypoint table unchanged); the fingerprint
baseline updated for the visual differences only, in a separate commit.

Measured (Part A, 2026-10-24): the floor plane exists (`floor`, z −0.873 m = the workstation's lowest vertex,
visual-only); MuJoCo 3.16 maps one `texuniform` texture onto 2 m (→ `texrepeat` 2 2), puts the gradient's mean at
the horizon (→ nadir 0.64 0.68 0.73), fogs the whole skybox when fog is on, and scales fog by the extent (→ an
explicit statistic). Details: design-decisions (the studio look).

Owner's answers: corrections (a) texrepeat, (b) nadir, (d) statistic applied; (e) hard shadows accepted;
reflectance 0.05; `CONST_LOOK_*` in `Scene.Core`; `show_cell.py` stays on the blocking viewer (Rendering > Fog
documented); two commits (look, then baseline). Option A (fog on, uniform grey sky) or D (infinite plane, fog
off, gradient) chosen by a prototype: **A** - D has a hard horizon line (floor 51 / 255 against the sky's 115
within 5 rows; images `Data/Generated/Images/look_A_*.png`, `look_D_*.png`).

- [x] F7.1 `0f46736` the look (`Scene.Core`, `Render.Core`), `models/cell_scene.xml` regenerated, 11 tests (191 → 202
      collected; each broken once), SPEC §4.7 Look and §7.5, design-decisions. The fingerprint tests accept the
      6 visual differences until F7.2. Cycle report diff empty; keypoint table byte-identical; fast tests
      196 passed + the 2 fingerprint-script tests (24.4–25.7 s, before 22.9–26.0 s); style 0 issues.
- [x] F7.2 the fingerprint baseline: the 6 visual entries rewritten from `Model_Fingerprint` (`geoms/floor/geom_size`,
      `materials/grid/mat_reflectance`, `mat_shininess`, `mat_specular`, `mat_texrepeat`, `sizes/nlight` 3 → 2),
      `pcb_vis` kept as the one documented difference; the accepted lists back to `pcb_vis` only.

## F8 - Step 4a: the default program, its trajectory and checks (owner, 2026-10-24) - design, awaiting approval

Requirements (restated): a commented, sequential program in `Configuration/` that reads like a teach-pendant
program (named steps with a comment each; `Movej`, `Movel`, `Path`, `Loader`, `Gripper`, `Vacuum`, `Wait`, `Stop`);
not optimized (no blending, every motion ends at rest, robot and loader never move at the same time, offsets from
keypoints, vias where a person would put them); collision-free with every check passed; a parser that turns it
into a 4 ms trajectory of every element with the world state; a per-sample check (exact distances, ≥ 3 mm, the
table included in the check model only); build and show scripts. The MG06 test waits `Test_Duration` (2.0 s,
`Cell.yaml`). No change to the physics, the physics model, the cycle run (still `cell_plan.npz`) or the baselines.
Outputs are simulation results, not validated for the real cell.

### Measured for this design (2026-10-24, read-only)

- **Every legacy stop is already a keypoint.** The 22 keypoints of F6 are the reference plan's stationary segment
  ends, so they hold every approach, press, retreat and via the legacy sequence stops at (the three vias
  `via_input_to_loader`, `via_mg06_high`, `via_loader_to_output` were inserted by the legacy planner's collision
  fallback and are stationary). The default program below needs **no new point**.
- **The legacy sequence is already robot/loader-sequential**; the loader's own Y and Z overlap in it (e.g. Y
  starts 0.45 s into the 0.8 s Z lift). Serialising them (human style) adds about 1.35 s (0.25 + 0.7 + 0.1 + 0.3 s
  over the four loader moves), estimated from the plan's start times, not run.
- **The recorded fork path** (`Data/Paths/fork_close_open_path.npz`, 906 rows, rows mostly 20 ms apart):
  `CLOSE` rows 20-305 (0.40-6.09 s), `OPEN` rows 620-905 (12.38-18.07 s). `Fork_Close` (rows 1-305) starts and ends
  at rest. `Fork_Open`'s join (row 663, the first `OPEN` row with handle ≤ 28.45°: 28.32°) moves the flange at
  **37.6 mm/s**: joined from a MoveL that ends at rest it is a velocity step (the legacy plan has the same step;
  the verified physics ran with it).
- **The table has no collision geometry.** The workstation's collidable geoms are the pedestal and its plate, one
  frame post (`L_900`), the top beam (`L_340`), the camera holder, the cabinet and a box; the table top exists only
  as visual meshes. Connected components (scipy `connected_components`, coincident vertices welded) of the visual
  mesh `ws_vis_static_217_214_219` give two plates: x −0.211…−0.011 and 0.021…0.541 m, y −0.860…0.190 m, z
  −0.038…+0.002 m (the 32 mm gap between them holds two rails, top ≤ +0.002 m). Nothing else above the table is
  within the robot's reach without already being collidable (corner brackets up to z 0.164 m at x ≈ −0.2,
  y ≈ −0.8 m, 0.78 m from the base).
- **With these two plates as boxes**: the robot's minimum distance to them over the whole reference plan is
  **8.9 mm** (cup 1 at the input press, t = 3.26 s); the keypoint check table is **byte-identical** to today's;
  the carried PCB at `output_place` is **3.019 mm** from the table (19 µm above the 3 mm clearance), and the PCB
  lying in a crate is **2.000 mm** from it (the crate floor's thickness) - so a carried PCB on a crate floor (just
  after vacuum on at the pick) fails 3 mm against the table unless that pair belongs to `pcb_crate`.
- **Exact distance is slow**: `Collision.Close_Pairs` takes 13.8 ms per configuration (42 configurations of the
  plan, Apple M2 Pro); the margin-based `Contacts` on the planning model 0.64 ms. A 4 ms trajectory of about 17 000
  samples checked exactly at every sample would take about 4 min; the broad phase (a Python loop over about 35 ×
  500 geom pairs) is the cost.
- **`np.savez` is byte-stable** (no timestamp in the archive; two writes 2 s apart are identical).
- **toppra** (for F9): see F9.

### Proposal

**1. The program file** `Configuration/Program_Default.yaml`, read only by `Configuration.Core.Load_Program`
(one reader), written by hand. Keys Pascal_Snake_Case; step, keypoint and position names lowercase; no optional
fields; every problem collected and named by key path (`Steps[4].Commands[2].Movel.Speed: ...`). Sections:

- `Start`: the state before the first step - `Robot` (a keypoint), `Loader` (a position), `Gripper` (a state),
  `Vacuum` (`On`/`Off`), `PCB` (a location), `MG06` (`Open`/`Closed`).
- `Steps`: a list; each step has `Name` (unique, lowercase), `Comment` (what and why, shown by the viewer and the
  report), `Allow_Contact` (the contact groups any sample of the step may have within the clearance; `[]` when
  none; each declared group must be observed in the step, as for the keypoints) and `Commands`.
- Commands, each a one-key mapping with a fixed set of fields:

| Command | Form | Meaning |
|---|---|---|
| `Movej` | `Movej: <keypoint>` | min-jerk joint move to the keypoint's stored `Joints` |
| `Movel` | `Movel: {To: <keypoint>, Speed: <m/s>}` | straight TCP line + orientation slerp to the keypoint; `Speed` = peak TCP speed |
| `Path` | `Path: Fork_Close` / `Fork_Open` | the recorded flange path (SPEC §4.9) |
| `Loader` | `Loader: {Axis: Y/Z, To: <loader position>, Duration: <s>}` | one axis to that position's value, min-jerk |
| `Gripper` | `Gripper: {To: open/clamp, Duration: <s>}` | the jaw command, min-jerk |
| `Vacuum` | `Vacuum: On/Off` | an event (zero time) |
| `Wait` | `Wait: <s>` | everything holds |
| `Test` | `Test: {}` | the MG06 test: holds `Cell.yaml Process.Test_Duration`; requires MG06 closed and the PCB in MG06 |

Movej has no speed field: every Movej runs at `Cell.yaml Motion.Speed_Fraction` (a per-move speed would have one
legal value in this program). `Stop` is not in the table: see question 3.

**2. The steps of the whole cycle** (22 steps; the legacy sequence, the loader axes serialised, the 1 s legacy
wait replaced by `Test`). Lane: R = robot (and the gripper/vacuum it waits for), L = loader. Class: F9's
classification (below), not used by F8.

| # | Step | Lane | Class | Commands | Allow_Contact |
|---|---|---|---|---|---|
| 1 | `approach_input` | R | FREE | Movej above_input_pcb | [] |
| 2 | `pick_input` | R | LOCKED | Movel input_pcb_approach 0.20; Movel input_pcb_press 0.03; Vacuum On; Wait 0.3; Movel above_input_pcb 0.20 | cups_pcb, pcb_crate |
| 3 | `to_loader` | R | FREE | Movej via_input_to_loader; Movej above_loader | cups_pcb |
| 4 | `handover` | R | LOCKED | Movel loader_approach 0.20; Movel loader_handover 0.03; Gripper clamp 0.35; Wait 0.25; Vacuum Off; Wait 0.2; Movel above_loader 0.20 | cups_pcb, pcb_jaws |
| 5 | `home_after_handover` | R | FREE | Movej home | [] |
| 6 | `loader_insert` | L | LOCKED | Loader Z travel_place 0.8; Loader Y travel_place 2.0; Loader Z place 0.6; Wait 0.1; Gripper open 0.35; Wait 0.1 | [] |
| 7 | `loader_park` | L | LOCKED | Loader Z travel_place 0.5; Loader Y park 1.8; Loader Z park 0.8; Wait 0.15 | [] |
| 8 | `to_mg06` | R | FREE | Movej via_mg06_high; Movej above_handle; Movej fork_start | [] |
| 9 | `close_mg06` | R | LOCKED | Path Fork_Close; Movel fork_bar_centred 0.02 | handle_bar |
| 10 | `leave_mg06` | R | LOCKED | Movel fork_out_of_bar 0.12; Movel fork_retreat_up 0.25 | [] |
| 11 | `test` | R | LOCKED | Test | [] |
| 12 | `return_mg06` | R | LOCKED | Movel fork_out_of_bar 0.25; Movel fork_bar_centred 0.12 | [] |
| 13 | `open_mg06` | R | LOCKED | Movel fork_open_join 0.03; Path Fork_Open | handle_bar |
| 14 | `home_after_mg06` | R | FREE | Movej above_handle; Movej home | [] |
| 15 | `loader_fetch` | L | LOCKED | Loader Z travel_place 0.6; Loader Y travel_place 2.2; Loader Z place 0.6; Wait 0.1; Gripper clamp 0.35; Wait 0.2 | [] |
| 16 | `loader_return` | L | LOCKED | Loader Z travel_place 0.6; Loader Y home 2.0; Loader Z home 0.8; Wait 0.2 | [] |
| 17 | `to_take` | R | FREE | Movej above_loader | [] |
| 18 | `take_from_loader` | R | LOCKED | Movel loader_approach 0.20; Movel loader_take_press 0.03; Vacuum On; Wait 0.3; Gripper open 0.35; Wait 0.15; Movel loader_out_of_jaws 0.05; Movel above_loader 0.20 | cups_pcb, pcb_jaws |
| 19 | `to_output` | R | FREE | Movej via_loader_to_output; Movej above_output | cups_pcb |
| 20 | `place_output` | R | LOCKED | Movel output_approach 0.20; Movel output_place 0.03; Vacuum Off; Wait 0.3; Movel above_output 0.20 | cups_pcb, pcb_crate |
| 21 | `home_end` | R | FREE | Movej home | [] |
| 22 | `settle` | R | LOCKED | Wait 0.8 | [] |

Every speed, duration and wait is the legacy planner's (git tag `legacy-planner-reference`, `main`); the step
boundaries follow its phases, so step 5 can map the fork steps onto the physics classifier's closing / opening
exemption (`Collision.CONST_FORK_PHASES`): a step whose `Allow_Contact` holds `handle_bar` is a fork step. The
return from MG06 goes `above_handle` → `home` directly, as the legacy plan does (question 10).

**3. The YAML** (excerpts; the header follows `Configuration/Keypoints.yaml`'s form):

```yaml
## =========================================================================== ##
#   Description:
#     The default program of Test-it-off Simulation: the whole cycle as a human operator would teach it -
#     sequential steps, every motion ending at rest at its point, no blending, the robot and the loader never
#     moving at the same time (SPEC §2 item 4). Read only by src/Configuration/Core.py (Load_Program); written
#     by hand. Points are keypoints of Configuration/Keypoints.yaml (their stored Joints).
#
#   REQUIRED: every field is required; a missing or invalid field fails the load, naming it.
#   Units: Movel Speed [m/s] (the peak TCP speed), Duration and Wait [s]. Movej runs at Cell.yaml
#     Motion.Speed_Fraction of the AUBO i5 datasheet joint speeds.
#   Commands: Movej, Movel, Path, Loader, Gripper, Vacuum, Wait, Test (SPEC §8).
## =========================================================================== ##

# The state of the cell before the first step.
Start: {Robot: home, Loader: home, Gripper: open, Vacuum: Off, PCB: input_crate, MG06: Open}

Steps:
  - Name: approach_input
    Comment: "From home to 120 mm above the PCB in the input crate, through free space."
    Allow_Contact: []
    Commands:
      - Movej: above_input_pcb

  - Name: pick_input
    Comment: >-
      Straight down to 15 mm above the PCB, the last 15 mm at 30 mm/s until the cups are pressed 1.5 mm onto it
      (SPEC §4.7), vacuum on, let it build up, straight up out of the crate (its walls are 149 mm high).
    Allow_Contact: [cups_pcb, pcb_crate]
    Commands:
      - Movel: {To: input_pcb_approach, Speed: 0.20}
      - Movel: {To: input_pcb_press, Speed: 0.03}
      - Vacuum: On
      - Wait: 0.3
      - Movel: {To: above_input_pcb, Speed: 0.20}

  - Name: to_loader
    Comment: >-
      Carry the PCB over a free-space via to 160 mm above the loader jaws, along the plate normal, so the board
      does not swing over the crates or MG06's front.
    Allow_Contact: [cups_pcb]
    Commands:
      - Movej: via_input_to_loader
      - Movej: above_loader

  - Name: handover
    Comment: >-
      Down along the plate normal into the open jaws, the last 15 mm at 30 mm/s, to the hold pose with no press:
      the board stays 0.3 mm above the finger ledges (SPEC §4.7). The jaws clamp, vacuum off, straight back up.
    Allow_Contact: [cups_pcb, pcb_jaws]
    Commands:
      - Movel: {To: loader_approach, Speed: 0.20}
      - Movel: {To: loader_handover, Speed: 0.03}
      - Gripper: {To: clamp, Duration: 0.35}
      - Wait: 0.25
      - Vacuum: Off
      - Wait: 0.2
      - Movel: {To: above_loader, Speed: 0.20}

  # ... home_after_handover, loader_insert, loader_park ...

  - Name: to_mg06
    Comment: >-
      Over the free-space switch from the wrist-up to the wrist-down family (via_mg06_high, SPEC §4.7), above the
      handle, to the start of the recorded fork path. Joint moves only: a line never crosses wrist_2 = 0.
    Allow_Contact: []
    Commands:
      - Movej: via_mg06_high
      - Movej: above_handle
      - Movej: fork_start

  - Name: close_mg06
    Comment: >-
      The recorded closing path (lid down, handle to 29 deg; SPEC §4.9), then the bar to the centre of the fork
      slot, 2 mm off its bottom, at 20 mm/s (SPEC §4.10).
    Allow_Contact: [handle_bar]
    Commands:
      - Path: Fork_Close
      - Movel: {To: fork_bar_centred, Speed: 0.02}

  - Name: leave_mg06
    Comment: "Out of the bar 90 mm along the slot, then 120 mm up; MG06 stays closed (SPEC §4.10)."
    Allow_Contact: []
    Commands:
      - Movel: {To: fork_out_of_bar, Speed: 0.12}
      - Movel: {To: fork_retreat_up, Speed: 0.25}

  - Name: test
    Comment: "MG06 closed, the PCB inside, the robot away above the slot exit: the tester runs (Cell.yaml Process.Test_Duration)."
    Allow_Contact: []
    Commands:
      - Test: {}
```

**4. Points.** Auxiliary points (offsets, vias, retreats) are keypoints: they live in `Configuration/Keypoints.yaml`
`Robot:` with the same schema, so `check_keypoints.py`, `resolve_keypoint.py`, `show_keypoint.py` and
`step_keypoints.py` cover them unchanged. Named by role: `above_<place>` (the retreat height), `<place>_approach`
(15 mm before contact), `<place>_out_of_<thing>`, `via_<from>_to_<to>` (a free-space via, `Relative_To: null`, a
`Check` position), `fork_*` (the fork path's leave / return). An offset point has `Relative_To: {Target, Frame,
Offset}` and stored `Joints` (F6 determinism: the joints are the source of truth; the target is a check value,
stale beyond `Clearance.Stale_Position` / `Stale_Angle`; `resolve_keypoint.py` prints the corrected line). The
program refers to keypoints by name and reads only their `Joints`. Alternative rejected: a `Points:` section in
the program file - two places for robot poses, and the keypoint tools would not see them.

**5. `Cell.yaml` additions** (each REQUIRED, with its reader):

| Field | Value | Unit | Source | Read by |
|---|---|---|---|---|
| `Motion.Speed_Fraction` | 0.6 | - | the legacy plan (`SPEED`), SPEC §4.4 | MoveJ timing, MoveL limit check, F9's TOPP |
| `Motion.Datasheet_Speeds` | 220, 220, 220, 220, 230, 230 | deg/s | AUBO i5 datasheet, SPEC §4.4 | as above |
| `Motion.Acceleration_Limits` | 171.887339 ×3, 343.774677 ×3 (= 3, 6 rad/s²) | deg/s² | **assumption** (SPEC §4.4, KNOWN_ISSUES) | as above |
| `Motion.Movej_Min_Duration` | 0.5 | s | legacy `_jpath` | MoveJ |
| `Motion.Movel_Min_Duration` | 0.6 | s | legacy `linear_move` | MoveL |
| `Process.Test_Duration` | 2.0 | s | owner, 2026-10-24: a placeholder, the real test time is unknown (KNOWN_ISSUES) | `Test` |
| `MG06.Open_Join_Margin` | 0.2 | deg | legacy (`H_CLOSED - 0.2` = 28.45°, SPEC §4.9) | `Path: Fork_Open`'s join row |
| `Clearance.Check_Step` | 0.001 | m | proposed: the largest move of any robot or PCB point between two checked samples | the segment check |
| `Clearance.Allowed_Depth.pcb_jaws` | 0.0005 | m | as the other groups (owner, F6) | the check (the group comes back, question 6) |

**6. Motion laws** (every number's source in brackets):

- **MoveJ**: min-jerk `q0 + (q1 − q0)·(10s³ − 15s⁴ + 6s⁵)`, `T = max(Movej_Min_Duration, maxⱼ 1.875·|Δqⱼ|/v_maxⱼ,
  maxⱼ √(5.774·|Δqⱼ|/a_maxⱼ))`, `v_max = Speed_Fraction × Datasheet_Speeds`, `a_max = Acceleration_Limits`
  [SPEC §4.10, the legacy `_jpath`]; `n = round(T / 4 ms)` samples, `qd` analytic. Both ends are stored `Joints`.
- **MoveL**: TCP (`suction_tcp`; `tool_flange` when the target's `Check` frame is the flange - the fork moves) on
  a straight line, orientation slerp, min-jerk time law, `T = max(Movel_Min_Duration, 1.875·L/Speed)` [legacy
  `linear_move`]; IK per sample (`Kinematics.Solve_IK`) seeded from the previous sample with the null-space pull
  to it. Fails, naming the step, the command and the sample: IK not converged; a family flip (wrist_2's sign);
  |wrist_2| < `Family_Margin`. If a joint's finite-difference speed or acceleration exceeds its limit, `T` is
  stretched by the smallest factor that meets every limit and the report lists the stretch (a WARNING, not a
  silent clamp). The last sample is the target's stored `Joints`: the IK end must be within the stale tolerance
  of them, else the build fails ("stale keypoint").
- **Path**: `Fork_Close` = rows 1-305 (from `fork_start`, row 0 the start), `Fork_Open` = the `OPEN` rows after the
  first with handle ≤ `Closed_Handle − Open_Join_Margin` [SPEC §4.9, legacy `replay`]: flange IK per row seeded
  from the previous row, the recorded times, linearly interpolated onto the 4 ms grid. Its first and last poses
  must equal the start / end keypoints' (`fork_start` → `fork_closed_end`; `fork_open_join` → `fork_start`)
  within the stale tolerance.
- **Loader / Gripper**: one channel, min-jerk over `Duration` from its current value [legacy `loader_move`;
  `Simulation.Loader_Target`]; the trajectory carries the command, the 30 ms drive lag (SPEC §4.3) acts in step 5's
  physics only. The gripper column is the command (`clamp`); posing and the check use the jaws' real position
  (`grip` while the PCB is between them, the command otherwise).
- **Vacuum**: an event at the sample where it is commanded; `Wait`, `Test`: hold.
- **qd**: analytic for MoveJ; central differences for MoveL, Path, Loader, Gripper; zero at every MoveJ / MoveL
  end, so continuous at every join except `Fork_Open`'s first row (37.6 mm/s, measured above; question 9).

**7. World state per sample** (columns of the trajectory, explicit rules, a build error on a violated rule):

| Event | Requires | PCB becomes |
|---|---|---|
| `Vacuum: On` | PCB at `input_crate` or `jaws`, the TCP at that location's hold pose pressed (the step's `cups_pcb`) | `cups` - its pose relative to the TCP frozen as it is (the cups 1.5 mm compressed) |
| `Gripper: clamp` | loader at `home` with the PCB on the cups at the jaws' hold pose, or loader at `place` with the PCB in MG06 | `jaws` (at home: shared with the cups until `Vacuum: Off`) |
| `Vacuum: Off` at the loader | PCB clamped in the jaws | `jaws` (moves with the loader's Y/Z) |
| `Vacuum: Off` at the output crate | TCP at `output_place` | `output_crate` (its rest pose: dropped 1 mm) |
| `Gripper: open` at `place` | PCB in the jaws, loader at `place` | `mg06` (the place pose) |
| `Gripper: open` at `home` | PCB on the cups and in the jaws | `cups` |

MG06 (lid, handle; the press plate through the model's coupling): the recorded rows during a `Path`; otherwise the
`Fixture` of the motion's target keypoint (`Open`, `Closed` = `Cell.yaml MG06`, or a recorded row); held during
`Wait` / `Test` / loader steps; so settled `Closed` from `fork_bar_centred` to `fork_bar_centred` on the way back,
the recorded row 663 during the MoveL onto `fork_open_join` (as the legacy plan).

**8. The trajectory file** `Data/Generated/Trajectories/<program>.npz` (+ `<program>.csv`, a write-only export),
written and read only by `src/Trajectory/Core.py` (`Write_Trajectory`, `Read_Trajectory` → `Trajectory_Str`):

- per sample (4 ms, a code constant: the format's period): `t` [s], `robot_q` [N×6, rad], `robot_qd` [N×6, rad/s],
  `loader_y`, `loader_z` [m, commands], `gripper` [m per jaw, command], `vacuum` [0/1], `step_index`, `step_label`,
  `segment_id`; the world state: `pcb_location` (code + `pcb_locations` names), `pcb_pos` [m], `pcb_quat`,
  `mg06_lid`, `mg06_handle` [rad], `jaw` [m, the jaws' real position];
- per segment (one command): `segment_step`, `segment_command` (its text, e.g. `Movel input_pcb_press 0.03`),
  `segment_start`, `segment_end` (sample indices);
- `meta` (JSON): format version, program name, sha256 of the program, `Keypoints.yaml`, `Cell.yaml` and the fork
  path file, git commit (+ `dirty`), units (m, rad, s), the joint order and the TCP definition (SPEC §17).
- Deterministic: the same inputs and commit give a byte-identical npz and CSV (CSV numbers with fixed `%.9f`).
- Stays git-ignored (`Data/Generated/`); whether the trajectory becomes a committed deliverable is decided in
  step 6, when it replaces `cell_plan.npz`.

**9. The check model and the segment check** (A3):

- `Collision.Build_Planning_Model(cell)` gains the **table boxes** `table_left`, `table_right` (contype /
  conaffinity of the workstation, group 3) from `CONST_TABLE_BOXES` in `Collision.Core`: the measured plates above,
  bounding box with **no margin** (a plate is a box; a margin on the top face would put the PCB on the crate floor,
  2.0 mm away, and the output place, 3.019 mm, inside the clearance). A test re-derives both boxes from the mesh
  (connected components) and pins them to 0.5 mm; another pins that `Build_Cell_Model` (physics, fingerprint) has
  no table geom. `mj_geomDistance` ignores the 6 mm planning margin (to be confirmed by a test), so one model
  serves the exact check and F9's fast contact query.
- The keypoint check moves to `Build_Planning_Model` (question 8): one check model; the table stays
  byte-identical (measured above).
- `Close_Pairs` gets a vectorised broad phase (numpy bounding-sphere filter, then `mj_geomDistance` on the
  candidates only): a behaviour-preserving change, proven by the keypoint table byte-identical and the same pairs on
  every keypoint, timed before / after (project-optimize).
- Per sample, exact distances (`Close_Pairs`) from the robot's geoms and, while carried, the PCB's, to everything
  else in the sample's world state (MG06 state, loader Y/Z, jaws, table). Checked samples: every segment's ends,
  plus the next sample whenever any robot geom (bounding sphere, through the joint steps) or the PCB could have
  moved more than `Clearance.Check_Step` since the last checked one. A pair within `Clearance.Robot` fails unless its
  group is in the step's `Allow_Contact` and no deeper than its `Allowed_Depth`; every declared group must be
  observed in the step. Groups: `cups_pcb`, `handle_bar`, `pcb_crate` (the carried PCB on a crate - **and on the
  table under it**, question 7), `pcb_jaws` (back, question 6).
- Also per segment: joint limits (the model), the family (wrist_2's sign) and its margin, the peak joint speed and
  acceleration against the limits, MoveL stretches, the PCB state rules, `Test` preconditions.
- The report: one row per segment (step, command, duration, min clearance and pair, allowed contacts with depth,
  peaks, verdict), the cycle time, the events; `Data/Generated/Reports/<program>_check.txt`; exit 1 on a FAIL. It
  ends with the project's sentence: simulation results only, not validated for the real cell.

**10. Domains and layers** (style: one domain per concern; no placeholder module before its feature):

```
Configuration, Render     import nothing of the project (Configuration gains Load_Program)
Trajectory                → Configuration                   the file format only (no MuJoCo)
Scene                     → Configuration
Kinematics, Collision     → Scene
Simulation                → Configuration, Scene, Collision (step 5: → Trajectory)
Keypoints                 → Configuration, Scene, Kinematics, Collision
Program                   → Configuration, Scene, Kinematics, Collision, Keypoints, Trajectory
Planner (F9)              → Configuration, Kinematics, Collision, Program, Trajectory
```

`src/Program/Core.py`: `Build_Program(program, ...) -> Trajectory_Str` (the motion laws, the world state),
`Check_Trajectory(...) -> List[Segment_Check_Str]`, `Format_Check_Report`, `Pose_At(sample)` (for the viewer).

**11. Scripts** (`Example/Program/`): `build_program.py [--program FILE] [--out DIR] [--compare-legacy]` (the
trajectory, the CSV, the check report; `--compare-legacy` prints the diagnostic of item 12); `show_program.py
[--version default|optimized] [--step N] [--headless] [--image [FILE]]` (`mjpython`; kinematic playback in real time
from the trajectory's state columns, no physics; **Home** pause / resume, **Page Down** or `.` next step, **Page Up**
previous step, **End** to the end of the current step - the keys F6 found free; the step's name and comment shown
with the passive viewer's overlay text; `--step N` poses the cell at the end of step N). `--version optimized` is
refused until F9.

**12. Diagnostic against the legacy plan** (printed by `build_program.py --compare-legacy`, never an acceptance
bound): total time; per legacy phase vs the matching steps (duration, start); the four vacuum events and the
gripper / loader starts; joints at every keypoint (expected 0: stored joints, exact snap) and the largest joint
difference per step where the two are time-aligned (steps 1-4 should agree to the IK tolerance). Expected, not
run: steps 1-4 identical in time (vacuum on 3.276 s, off 9.524 s); then +0.25 s (loader insert), +0.7 s (park),
+1.0 s (test 2.0 s instead of the 1.0 s hold), +0.1 s and +0.3 s (fetch, return): cycle about 68.1 s against 65.77 s.

### Tests (F8; collected 202 now)

| File | Pins |
|---|---|
| `test/Configuration/test_configuration_files.py` (+) | `Load_Program`: every required field named literally, each command's fields, unknown command / keypoint / position / group named, the new `Cell.yaml` fields (units converted once) |
| `test/Trajectory/test_core.py` (new) | write → read round trip; byte-identical rewrite; the CSV columns; a wrong format version refused |
| `test/Program/test_core.py` (new) | MoveJ: T and samples equal the legacy rule (`T = max(0.5, 1.875Δq/v, √(5.774Δq/a))`) on `home → above_input_pcb`, and the reference plan's samples of that move to 1e-9 rad; MoveL: every TCP sample within 0.01 mm of the line, the last sample equal to the stored joints, a stale target refused, a family flip refused, a stretch reported; Path: the rows of each path and the join row (663, 28.32°), the ends equal the keypoints within the stale tolerance; the PCB state machine (each rule, each violation named); MG06 state per segment; determinism (build twice → identical bytes); a deliberately colliding step (a Movel through the crate wall) reported with its pair; an undeclared contact and a declared-but-absent group reported |
| `test/Collision/test_core.py` (+) | the table boxes re-derived from the mesh; no table geom in the physics model (the fingerprint unchanged); the vectorised broad phase returns the same pairs as the loop |
| `test/Example/Program/test_build_program.py`, `test_show_program.py` (new) | fresh interpreter, exit codes, outputs in `--out`, logs; `show_program.py --headless --step N --image` |

Each new test broken once; the collected count reported per commit.

### Commits (F8)

- F8.1 Collision: the vectorised broad phase (equivalence, timing), the table boxes in the planning model, the
  keypoint check on the planning model (table byte-identical), `pcb_crate` covering the table, `pcb_jaws` back.
- F8.2 Configuration: the `Cell.yaml` fields, `Configuration/Program_Default.yaml`, `Load_Program`, their tests.
- F8.3 Trajectory: the format, its tests.
- F8.4 Program: the motion laws and the world state (build), their tests.
- F8.5 Program: the segment check and report, the colliding-step test.
- F8.6 `Example/Program/build_program.py`, `show_program.py`, their tests; the legacy diagnostic in the report.
- F8.7 Docs: SPEC §6, §7 (Trajectory, Program), §8 (program, new fields), §9 (trajectory format; §16 superseded),
  design-decisions, KNOWN_ISSUES (the test duration is a placeholder; the `Fork_Open` velocity step), the README
  commands (readme-writer), the skill references (layout, layers, formats).

After every commit: the fingerprint compare (only `pcb_vis` rgba), the cycle report diff, the fast tests, the style
checker (0 issues), the keypoint check table byte-identical.

### Approval (owner, 2026-10-24)

Part A approved; F8 first, stop for acceptance before F9. Answers: 1 loader axes one after the other; 2 a `Duration` per
loader / gripper command with the legacy values, each with its source; 3 `Stop` dropped; 4 `Allow_Contact` per step, and
each group's deepest overlap in `Cell.yaml` (a deeper one fails even where allowed); the report shows per step and group
the minimum distance and where (sample, time) it occurs; 5 F9's multi-seed IK dropped (endpoints fixed by LOCKED steps;
design-decisions in F9); 6 `pcb_jaws` back, 0.5 mm; 7 the PCB-table pair exempt only while the PCB is in a crate - a
carried PCB or a PCB in the jaws keeps 3 mm to the table; 8 the keypoint check on the planning model, its table
byte-identical; 9 the `Fork_Open` velocity step kept, noted for step 5 (tracking error, force on the bar); 10 straight
home from MG06 (MoveJ, the family switch in joint space); 11 `Data/Optimized/` committed, trajectories git-ignored until
step 6; 12 loader steps LOCKED in F9; 13 TOPP without a jerk limit, step 5 reports tracking error and PCB slip for both
versions, acceptance as SPEC §10.1; 14 `Test: Process.Test_Duration` (an explicit reference).

Additions: B1 the no-press check at the handover (PCB-ledge gap 0.3 ± 0.1 mm at `loader_handover` and over the last
MoveL into it, the gap reported, a press fails); B2 the broad phase identical to the brute-force check on all keypoints
and ≥ 500 random samples, its speed-up reported; B3 the own TOPP validated analytically (F9); B4 the carried-PCB tilt
limit in FREE segments (F9); B5 byte-identical outputs on one machine and platform, the platform recorded; B6 new
baselines for both programs in step 6 (SPEC).

### Status

- [x] F8.1 Collision: `Close_Pairs` with a vectorised broad phase - identical pairs to the per-pair loop on the 22
      keypoints and 500 random plan samples (B2), 37.9x faster at 3 mm (15.98 -> 0.42 ms per sample); the table top as two
      boxes in the planning model only; the keypoint check on the planning model (table byte-identical); `pcb_jaws` back;
      the PCB-table pair `pcb_crate` only inside a crate (`Scene.Crate_Holding`). Found: generated contacts depend on
      unrelated geoms (one more workstation geom makes MuJoCo generate the PCB-crate contacts it missed) - the exact-
      distance decision's reason corrected. 202 -> 208 collected; each new test broken once (7 mutations).
- [x] F8.2 Configuration: `Cell.yaml` `Motion`, `Process.Test_Duration`, `Handover`, `MG06.Open_Join_Margin`,
      `Clearance.Check_Step`; `Configuration/Program_Default.yaml` (22 steps, 68 commands, every loader / gripper duration
      and wait with its legacy source); `Load_Program` (`Test: Process.Test_Duration` as a Cell.yaml reference;
      `Vacuum: On / Off` as YAML booleans). 208 -> 246 collected; 6 mutations, all failed.
- [x] F8.3 Trajectory: `src/Trajectory/Core.py` (`Trajectory_Str`, `Write_Trajectory` npz + CSV, `Read_Trajectory`);
      byte-identical rewrites. 246 -> 250 collected; 4 mutations, all failed.
- [x] F8.4 Program build: `src/Program/Core.py` (`Build_Program`, `Pose_Sample`; the motion laws, the world state).
      Default program: 17 067 samples, cycle 68.264 s (estimate 68.1 s; legacy 65.768 s); vacuum on / off at 3.276 / 9.524 s
      as the reference plan; no MoveL stretched; the `Fork_Open` step noted (39.1 mm/s). Build 1.2 s (33 s with the keypoint
      IK's null-space pull, dropped). Design corrected: the carried PCB relaxes to the cup face (the frozen pose put it 2.5 mm
      above the output floor); vacuum off in a crate needs the PCB within 2 mm of its floor pose. 250 -> 263 collected; 14
      mutations, all failed (two first survived and were sharpened: a bow along the line's own axis; a nondeterminism the
      speed post-processing absorbed, which exposed a sloppy condition, fixed).
- [x] F8.5 Program check: `Check_Trajectory`, `Format_Check_Report` (per step and group the closest pair with its sample and
      time; the handover gap; the limits; the keypoints used). Found and fixed: the carried PCB 1.64 mm from the input
      crate's wall (new auxiliary keypoint `input_pcb_clear`, the lift to 170 mm); the fork 8.41 mm through the handle bar on
      a direct return home (the legacy route over `via_mg06_high`, Part A corrected); the fork 5.95 mm in the bar when MG06
      switched to closed at the bar-centring MoveL (MG06 now moves along each motion). Open: `Fork_Close`'s recorded 0.54 mm
      overlap against the 0.5 mm `handle_bar` limit (owner). Default program: cycle 68.852 s, B1 gap 0.318 mm. Keypoint table:
      the 22 rows byte-identical, one row added. 263 -> 270 collected; 10 mutations, each test passing first and failing mutated (the harness now checks
      both; it had counted a test that failed unmutated as a catch).
- [x] F8.6 Scripts: `Example/Program/build_program.py` (trajectory npz + csv, check report, `--compare-legacy`; exit 0 / 1),
      `Example/Program/show_program.py` (mjpython playback with the step label; `--step` / `--time` / `--headless` / `--image`;
      refuses a trajectory whose inputs changed). Diagnostic: cycle 68.852 s against 65.768 s (+3.084 s: lift +0.472, loader
      axes in turn +1.104, test +1.000, fetch / return +0.600, the rest -0.092). 270 -> 276 collected (the 6 script tests
      slow: each runs the build and check, about 12 s); 8 mutations, all failed. Full suite 276 passed in 112 s, fast 266 in
      58 s. The viewer's live playback (keys, overlay) is for the owner to check under mjpython.

## F9 - Step 4b: the optimized program (owner, 2026-10-24) - design, awaiting approval; built after F8 is accepted

Requirements (restated): the same process as F8 with the process-critical segments unchanged (LOCKED) and the
free-space transfers between them planned (FREE): IK, RRT-Connect (own, fixed seed), shortcut, spline with a
collision recheck, time-optimal timing within the AUBO i5 limits; the robot and the loader in parallel under
interlocks, robot↔loader checked over time, redundant motions removed; deterministic; stored without a tool
writing configuration; the same checks as F8 and later steps 5/6; scripts to re-optimize, play and compare.

### LOCKED and FREE

The classification is explicit: `Configuration/Optimization.yaml` lists the FREE steps by name; every other step
is LOCKED. A LOCKED step's samples relative to its own start are byte-identical in both versions (its start time
may move); a FREE step's motion is replaced. Consecutive FREE robot steps with only loader steps between them are
one FREE segment from the end of the previous LOCKED robot step to the start of the next: their intermediate
points (`home`, the vias) are dropped - that is the removal of redundant motion.

| Segment | From → to | Default steps it replaces | Carries the PCB | Family |
|---|---|---|---|---|
| FREE 1 | `home` → `above_input_pcb` | 1 | no | up |
| LOCKED | crate pick: approach, press 1.5 mm, vacuum, lift (`pick_input`) | 2 | from vacuum on | up |
| FREE 2 | `above_input_pcb` → `above_loader` | 3 | yes | up |
| LOCKED | handover, no press; clamp; vacuum off; up (`handover`) | 4 | until vacuum off | up |
| FREE 3 | `above_loader` → `fork_start` (home dropped) | 5, 8 | no | up → down (switch in free space) |
| LOCKED | `close_mg06` (Fork_Close + bar centred), `leave_mg06`, `test`, `return_mg06`, `open_mg06` (open join + Fork_Open) | 9-13 | no | down |
| FREE 4 | `fork_start` → `above_loader` (home dropped) | 14, 17 | no | down → up |
| LOCKED | take: approach, press 1.5 mm, vacuum, gripper open, out of the jaws, up (`take_from_loader`) | 18 | from vacuum on | up |
| FREE 5 | `above_loader` → `above_output` | 19 | yes | up |
| LOCKED | place: approach, last 15 mm at 30 mm/s, vacuum off, up (`place_output`) | 20 | until vacuum off | up |
| FREE 6 | `above_output` → `home` | 21 | no | up |
| LOCKED | loader: `loader_insert`, `loader_park`, `loader_fetch`, `loader_return` (shape and durations; start times move) | 6, 7, 15, 16 | - | - |
| LOCKED | `settle` | 22 | - | - |

Every MoveL near contact (the last 15 mm at 30 mm/s), both fork paths and their joins, and the MG06 closed state
rules sit inside LOCKED steps. No MoveL is FREE, so no MoveL can cross the family switch.

### The pipeline per FREE segment (A → B; robot limits = `Cell.yaml Motion`)

1. **IK with several seeds - not needed (question 5).** A and B are the stored `Joints` of LOCKED steps' ends, and
   those steps are byte-identical, so B's configuration is fixed: there is nothing to choose. Proposal: no IK step.
2. **RRT-Connect** in the 6-D joint space within the model's limits, own implementation, `numpy.random.default_rng(
   Seed + segment index)`; the edge check interpolates at `Edge_Step` (joint space); the node / edge check is the
   fast margin-based contact query on the planning model (0.64 ms, the 6 mm margin = 2 × clearance), with the
   world state of the segment's time window (the loader at every position it takes in that window: conservative
   for parallel motion).
3. **Shortcut**: `Shortcut_Iterations` random pairs on the path (same generator), replaced by a straight joint
   segment when its edge check passes.
4. **Spline + recheck**: a cubic spline through the waypoints over the chord-length parameter (scipy
   `CubicSpline`, `bc_type='clamped'`), checked densely; where a span fails, the polyline's midpoint of that span
   is inserted and the spline refitted, up to `Max_Refits` (then the segment fails, naming it).
5. **Time-optimal timing**: own TOPP (see the feasibility finding) with per-joint `|q'ṡ| ≤ v_max`,
   `|q'u + q''ṡ²| ≤ a_max`, `ṡ = 0` at both ends (LOCKED neighbours end and start at rest), on `Topp_Grid_Points`;
   integrated to time, rounded up to whole 4 ms samples (a time stretch ≥ 1 keeps the limits), the last sample = B.
6. **The exact check** of F8 (per sample, `Close_Pairs`, the full world state over time). A failure → the segment is
   re-planned with the planner's margin raised one step (`Margin_Steps`); still failing → the build fails naming
   the segment and the pair.

### Synchronization

Lanes: robot (with the gripper and vacuum of its LOCKED steps) and loader. Each lane keeps its program order. A
step starts at the earliest time that meets its lane, the interlocks and the robot↔loader clearance over time
(exact, per sample of the overlap, `Clearance.Robot`, no contact group allowed between them); if the clearance
fails, the later-starting step is delayed sample by sample until it passes (deterministic). A FREE robot segment
that arrives before its next LOCKED step may start, waits at its end point (B).

| # | Interlock | Why |
|---|---|---|
| I1 | `handover` starts only when the loader is at `home` and the gripper open (the start state; no loader step before it) | the jaws must be ready, the PCB must enter open jaws |
| I2 | `loader_insert` starts only after `handover` ends (vacuum off, the robot 160 mm out along the plate normal) | the loader may move only after the robot left the jaw zone |
| I3 | `close_mg06` starts only after `loader_park` ends (gripper open, loader parked) | MG06 is closed only with the PCB released and the loader parked |
| I4 | `test` starts only after `leave_mg06` ends (MG06 closed, the fork out of the bar) | the tester runs closed, the robot away |
| I5 | `return_mg06` starts only after `test` ends | no motion at MG06 during the test |
| I6 | `loader_fetch` starts only after `open_mg06` ends (MG06 open, the fork off the handle) | the loader enters MG06 only when it is open and the fork is gone |
| I7 | `take_from_loader` starts only after `loader_return` ends (loader home, PCB clamped) | the robot enters the jaw zone only when the loader is home |
| I8 | the robot's and the loader's samples keep ≥ 3 mm between robot / tool / carried PCB and loader / jaws / PCB in the jaws, at every sample where both exist | parallel motion is safe only where checked |

Expected parallel overlaps (not measured): FREE 3 with `loader_insert` / `loader_park` (the robot waits at
`fork_start` for I3); FREE 4 with `loader_fetch` / `loader_return` (the robot waits at `above_loader` for I7, if I8
passes there; otherwise it waits earlier on its path). Gain estimate: not measured - the compare script reports it.

### toppra feasibility (PyPI, 2026-10-24)

`toppra` 0.6.10 (2026-08-28), **MIT**. Wheels: Linux x86_64 only (manylinux2014 / musllinux, CPython 3.9-3.14, cp311
included). **No macOS wheel in any release** (0.2.2a0 … 0.6.10), **no aarch64 wheel**, and **no sdist since 0.6.3**
(0.6.3 and earlier: sdist, a Cython build). So it cannot be installed from a wheel on the primary platform (macOS
arm64), and 0.6.10 cannot be installed there at all; a source build of 0.6.3 would need Cython and a compiler in
`install.sh`. It also pulls `matplotlib` (pinned already) and `scipy`. **Not recommended.**

**Proposal: an own TOPP** in `src/Planner/Core.py`, numpy only: the reachability formulation of TOPP-RA (Pham &
Pham 2018) for box velocity and acceleration limits on a grid `s₀…s_N`, with `x = ṡ²`, `u = s̈`,
`xᵢ₊₁ = xᵢ + 2Δsᵢ uᵢ`: the velocity limits bound `xᵢ` directly (`xᵢ ≤ minⱼ v_maxⱼ²/q'ⱼ(sᵢ)²`); the acceleration
limits are `2·6` half-planes in `(u, x)` per grid point, so each backward controllable-set step and each forward
greedy step is a 1-D interval intersection in closed form (no LP solver). Deterministic, tested against the
closed-form time of a straight joint line (trapezoid / triangle profile, max-speed and max-acceleration cases)
and against `ṡ = 0` at both ends. No new package. Not modelled: jerk (bang-bang accelerations; question 13).

### Where the optimized program lives

- `Configuration/Optimization.yaml` (hand-written, read by `Configuration.Core.Load_Optimization`): `Free_Steps`
  (names), `Seed`, `Rrt` (`Max_Iterations`, `Edge_Step` [deg], `Goal_Bias` [-]), `Shortcut_Iterations`, `Spline`
  (`Check_Step` [deg], `Max_Refits`), `Topp_Grid_Points`, `Margin_Steps` [m]. The robot limits stay in `Cell.yaml`
  (one source, so both versions run with the same limits and compare fairly).
- **`Data/Optimized/Program_Optimized.yaml`**, written only by `Example/Program/optimize_program.py` through
  `Planner.Core.Write_Optimized_Program`, read by `Configuration.Core.Load_Program` (the same reader as the default:
  one program format). Committed: it is generated data, like `Data/Reference/cell_plan.npz`, not configuration
  (no tool writes `Configuration/`). It is the program format with: `Kind: optimized`; the LOCKED steps copied
  verbatim from the default program; each FREE segment as `Spline: {Waypoints: [[deg × 6], ...]}` (the output of
  RRT + shortcut + reinserted points: the random part); `Schedule` (each step's start time [s] - recomputed and
  compared at build); `Inputs` (sha256 of `Program_Default.yaml`, `Keypoints.yaml`, `Cell.yaml`,
  `Optimization.yaml`, the fork path file, and of the compiled planning model). The build recomputes spline and
  TOPP (deterministic) and refuses a stale file, naming the changed input and the command to re-run.
  `Kind: default` files may not hold `Spline`, `Schedule` or `Inputs`.

### Scripts and tests (F9)

- `optimize_program.py [--out DIR]`: re-runs the pipeline, writes `Program_Optimized.yaml` and its report (per
  segment: RRT nodes, waypoints before / after shortcut, refits, duration, peaks; the schedule; the interlock times).
- `build_program.py --program Data/Optimized/Program_Optimized.yaml`, `show_program.py --version optimized`.
- `compare_programs.py [--out DIR]`: default vs optimized - cycle time; Σ|Δq| per joint and TCP path length; PCB
  path length; loader travel; the number of stops (robot at rest between motions); per segment and per step;
  a text table (`Data/Generated/Reports/compare_programs.txt`) and a matplotlib figure (lanes over time for both
  versions + the metric bars, `Data/Generated/Images/compare_programs.png`; dataviz skill).
- Tests: `test/Planner/test_core.py` - TOPP against the closed form; RRT / shortcut / spline deterministic with a
  seed (twice → identical); a spline recheck that inserts a point; the interlocks I1-I8 hold on the optimized
  timeline; LOCKED steps byte-identical between the versions (relative to their start); the optimized program passes
  every F8 check and is not slower than the default; a stale `Program_Optimized.yaml` refused;
  `test/Example/Program/test_optimize_program.py`, `test_compare_programs.py`.

### Commits (F9, after F8 is accepted)

F9.1 `Optimization.yaml`, `Load_Optimization`, the optimized program format in `Load_Program`; F9.2
`src/Planner/Core.py` RRT-Connect, shortcut, spline, TOPP; F9.3 synchronization and interlocks; F9.4
`optimize_program.py`, `Data/Optimized/Program_Optimized.yaml`, the optimized build and playback; F9.5
`compare_programs.py`; F9.6 docs (SPEC, design-decisions, KNOWN_ISSUES, README, references).

### Questions (Part A, F8 and F9)

1. Loader axes in the default program: serialised (Z, then Y, then Z; recommended, human style; about +1.35 s) or
   overlapped as in the legacy plan?
2. Loader and gripper timing: a `Duration` per command with the legacy values (recommended; no loader datasheet
   speed in the project) or axis speed / acceleration limits in `Cell.yaml` (no source yet)?
3. `Stop` has no effect in either version as designed: every F8 motion ends at rest, and F9's FREE segments end at
   rest at their LOCKED neighbours. Drop it (recommended), or keep it as an assertion (the build fails unless the
   robot and loader are at rest there)?
4. `Allow_Contact` per step (recommended, shown above) or per motion command (finer, more fields)?
5. F9 step 1 (multi-seed IK for B) cannot choose anything while LOCKED steps are byte-identical: drop it
   (recommended), or allow a LOCKED step to start from another IK solution of the same pose (breaks byte-identity)?
6. `pcb_jaws` comes back (the handover clamp and the take: the carried PCB between closing jaws), `Allowed_Depth`
   0.5 mm?
7. The carried PCB against the table counts as `pcb_crate` (the crate floor lies between; 2.0 mm when the PCB lies
   on it)?
8. The keypoint check moves to the planning model with the table (one check model; its table byte-identical,
   measured)?
9. `Fork_Open`'s velocity step (37.6 mm/s at the join): keep as recorded (recommended for step 6: the verified
   physics ran it) or ramp the first rows in with a time warp?
10. The return from MG06: `above_handle` → `home` directly as the legacy plan (recommended), or back through
   `via_mg06_high` (symmetric, one more stop)?
11. The optimized program committed in `Data/Optimized/` with its input hashes (recommended), and the trajectories
   kept git-ignored until step 6?
12. Loader steps LOCKED in F9 (shape and durations, start times move; recommended) or may F9 overlap the loader's
   own Y and Z?
13. TOPP has no jerk limit (bang-bang accelerations at the limits): accept for F9 and let step 5 show the tracking
   and the PCB slip (recommended), or add a jerk-limited smoothing first?
14. The test command as `Test: {}` (its duration from `Cell.yaml`) - or `Wait` with a named duration?

## For the step 7 README (from the previous Czech README, removed in F4.2)

- An overview paragraph: the cell continues `mg06_loader_mujoco`; the AUBO i5 on the pedestal plate with the
  same end effector as the MG06 project (VGC10 with the fork); the robot model from
  ian-chuang/Manipulator-Mujoco, MIT (`models/aubo_i5/LICENSE_Manipulator-Mujoco`).
- The cycle in six steps, as users read it (SPEC §4.5), and what the result means: the robot touches
  nothing but the handle bar while closing and opening; the PCB ends in the right crate within 1 mm of the
  centre, flat; an unstable run ends with an error instead of restarting silently.
- In the viewer, `[` / `]` switch between the cameras `cam_pcb` and `cam_tool`.
- The simplifications (KNOWN_ISSUES > Simplifications of the model) in user terms.

## Brief coverage (condition C2, 2026-10-24)

Every section and table of the owner's brief (git `427e076:docs/PROJECT_BRIEF.md`) and where it lives
now. Checked mechanically: all 314 numeric tokens of the brief occur in the project docs; all 22 keypoint
rows are byte-identical in SPEC §4.8; of the backticked names, only `cell.build_model()` and
`kinali-sim ...` are not repeated (old API, superseded).

| Brief | Now | Note |
|---|---|---|
| Header (language, owner's languages) | `CLAUDE.md`; feature-manager reference | |
| §1 Context, cell table | SPEC §1, §4.1 | table verbatim in content |
| §2 Goals, Phase 1 items 1–6 | SPEC §1, §2 items 1–7 | |
| §2 Phase 2 | SPEC §2 (Phase 2 paragraph + brief §8 verbatim) | |
| §3 file table | SPEC §4.6 (verbatim) | new places: F1 / F3 *Files: old → new* |
| §3 run commands | SPEC §4.6 (verbatim) | superseded by `Example/` scripts (F3) |
| §3 reference result | SPEC §4.6 (verbatim, beside this environment's baseline) | crate −0.08 vs −0.14 mm: `KNOWN_ISSUES.md` |
| §3 Process (65.8 s), steps 1–6 | SPEC §4.5 | same steps and values (y −199.5 mm, z +20 mm, ~29°) |
| §4 Frames | SPEC §4.2 | base (0.42072, −0.33497, 0.162), yaw −135°, tilt 8.94° |
| §4 qpos layout (all rows) | SPEC §4.3 | identical values |
| §4 Actuators (all rows), sites, cameras, timestep / cones / impratio / noslip | SPEC §4.3 | identical values |
| §4 Key numbers: loader home / place / travel / park, jaws open / grip / clamp | SPEC §4.4 | identical |
| §4 Key numbers: PCB in the jaws, placed in MG06 | SPEC §4.4 | identical |
| §4 Key numbers: PCB in a crate, crate centres | SPEC §4.4 | identical |
| §4 Key numbers: suction TCP in the PCB frame, tool axes | SPEC §4.4 | identical |
| §4 Key numbers: speeds 220 / 230°/s, 60 %, accel 3 / 6 rad/s² (assumed), MoveL 200–250 / 30 mm/s | SPEC §4.4 | identical; assumption marked |
| §4 Lessons: vacuum 1.5 mm press, no press at the handover (0.3 mm above the ledges) | SPEC §4.7 | |
| §4 Lessons: IK families wrist up / down, switch via `via_mg06_high` | SPEC §4.7 | |
| §4 Lessons: fork path tight, bar centred 5 mm play, 28.65°, −0.5°, `H_CLOSED`, `LID_CLOSED`, never push the handle down | SPEC §4.7 | |
| §4 Lessons: MuJoCo 3.16 `gap` / `includemargin`, solref 0.002, solimp 0.99 0.999, solmix 100 | SPEC §4.7; design-decisions (MuJoCo 3.16.0) | |
| §4 Lessons: instability, `d.time` backwards | SPEC §4.7, §13 risk 3 | |
| §4 Lessons: CAD deviations (+100 mm Y, +0.95 mm X, −0.8 mm holder, 200 mm cabinet) | SPEC §4.7; design-decisions (CAD deviations) | |
| §5.1 Proposed structure | superseded: SPEC §6 (RPX layout); design-decisions (RPX over brief §5.1) | |
| §5.2 Keypoint file (proposal) | SPEC §16.1 (verbatim) | |
| §5.3 Program file (proposal) | SPEC §16.2 (verbatim); commands also SPEC §2 item 4 | |
| §5.4 Trajectory output | SPEC §16.3 (verbatim), §9 | |
| §5.5 CLI (proposal) | SPEC §16.4 (verbatim, superseded) | mapping to `Example/` scripts there |
| §5.6 Acceptance criteria | SPEC §10.1 | plus penetration ≤ 0.5 mm (owner, F3) |
| §6 Initial keypoints, all 22 rows | SPEC §4.8 (verbatim) | |
| §6 fork path definition (`path: fork_close`, `path: fork_open`, ≤ 28.45°, row fields) | SPEC §4.9 (verbatim) | |
| §7 Work plan steps 1–7 and the work rule | SPEC §2 (steps restated) + verbatim block | |
| §8 Phase 2 outline | SPEC §2 (verbatim block) | |
