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

## F6 - Step 3: cell configuration and keypoints (owner, 2026-10-24) - approved, in progress

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
  present, C1); a test holds every declared group to being observed.
- `fork_closed_end` with MG06 **Open**: the handle is rotated away, the fork is in free space (24.6 mm from
  MG06's base) - no collision, unlike the owner's expectation; with MG06 settled **Closed** the bar is 5.95 mm
  into the fork and the check fails. Both pinned; open question for the owner.

### Status (2026-10-24)

- F6.1 done (e78aa8f): `Configuration/Cell.yaml`, `Configuration/Keypoints.yaml` and their loader; the moved
  constants convert to the same floats (fingerprint and cycle report unchanged).
- F6.2 done (56ae14f): `src/Keypoints/Core.py` (posing, check, resolution), exact clearance
  (`Collision.Close_Pairs`); the check: 0 FAIL, 1 WARN (`via_mg06_high`, the family switch), 21 PASS.
- F6.3 done: `Example/Keypoints/` (check, resolve, show, step), `Render_Free_Image`, `Launch_Passive_Viewer`,
  the script tests. Not verified here: the viewer keys (Page Down / Page Up / `.`) - the owner presses them.

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
