# Design decisions

Deliberate choices whose reason is not visible in the code. Each one says what would reopen it.

## The project follows project-style-rpx and the UR_PathGen_SF_1 layout, not the brief's package layout

**Decision.** The code, tests, scripts, installer and records follow the personal skills
(project-style-rpx first) with github.com/rparak/UR_PathGen_SF_1 as the structural template:
`src/<Domain>/Core.py` without `__init__.py`, `Example/` scripts, `test/` mirroring `src/`,
`install.sh` + `requirements.txt`, `.claude/project-docs/`. This replaces the layout of the brief §5.1
(package `kinali_sim`, `pyproject.toml`, console script `kinali-sim`, `tests/`, `config/`) wherever they
conflict (owner, 2026-10-24). Where the skills and the template disagree, the skills win and the
difference is recorded (`FEATURES.md` F3).

**Why.** One convention across the owner's projects; the skills carry the checker, the test discipline
and the records that keep a verified simulation verified.

**Alternatives.** The brief's package layout (step 1 was first done that way, commit `42a6121`):
rejected by the owner for consistency. Copying the template's code: rejected - only its structure and
conventions are taken.

**Revisit if** the owner's skills change their layout rules.

Pinned by `test/Docs/test_layout.py` (root entries, project docs, skill references).

## Simulation now, real-cell transfer planned

**Decision.** The project is a simulation today; keypoints and programs are intended to be transferred to
the real AUBO i5 later (owner, 2026-10-24). So: the formats stay controller-friendly (joint values in a
defined unit and order, the TCP definition, speeds as % of the datasheet, SPEC §17); the simulation-only
deviations (the CAD shifts in `scene.xml`, the assumed acceleration limits, the vacuum's adhesion model)
are checked against the real cell before a transfer; every output is marked as not validated for the
real cell (`KNOWN_ISSUES.md` opening, the README alert) until the real-cell validation procedure exists.

**Why.** A test on the real cell is planned; a format that cannot be loaded into the controller, or a
model deviation nobody re-checks, would make the transfer unsafe or redo the work.

**Alternatives.** Simulation only, formats free: rejected, the transfer is planned. Validating now:
not possible, the procedure and the real cell's measurements do not exist yet.

**Revisit if** the real-cell validation procedure is defined, or the transfer is cancelled.

Pinned by: nothing yet (the formats come in steps 3–4; their tests will pin unit and order).

## MuJoCo is pinned to 3.16.0

**Decision.** `mujoco==3.16.0` in `requirements.txt`; the installer checks the version.

**Why.** The physics was tuned and verified on 3.16 (SPEC §4.7): in 3.16 `gap` is not applied to
`includemargin`, which is why the PCB and the cups carry no `margin`/`gap`; another version may treat
contacts differently. A pinned version also keeps the baseline deterministic: on one machine and one
version the report is reproduced byte for byte (measured 2026-10-24, three runs, empty diffs).

**Alternatives.** An unpinned or `>=` dependency: rejected, a silent MuJoCo update could change every
contact number and the baseline would no longer be comparable.

**Revisit if** a MuJoCo release is needed for a feature; the upgrade is its own change with a new
baseline, the fingerprint re-measured and the margin behaviour re-checked.

Pinned by `test/Scene/test_core.py` (fingerprint), `test/Simulation/test_core.py` (slow regression) and `test/Install/test_install.py` (the version check).

## Python 3.11, pinned requirements, installed by install.sh

**Decision.** The interpreter is Python 3.11 (`/opt/homebrew/bin/python3.11` on macOS, `python3.11` on
Linux); every direct dependency is pinned `==` in `requirements.txt` with the owner's pin list; the
environment is created by `install.sh` into `.venv`. `pyproject.toml` is removed (owner, 2026-10-24).

**Why.** The same versions as the owner's other MuJoCo project, so both share one verified toolchain;
project-style-rpx prescribes the installer and pinned direct dependencies. The interpreter is named
explicitly because a bare `python3` on this machine is miniforge's.

**Alternatives.** Keeping the Python 3.14.7 / numpy 2.5.4 environment the baseline was first measured
in: rejected by the owner. The baseline was re-measured in the pinned environment (2026-10-24, Python
3.11.2, numpy 2.4.6, MuJoCo 3.16.0, `install.sh --recreate`): the cycle report is byte-identical to the
3.14 one and the fingerprint compare shows only `geoms/pcb_vis/geom_rgba`, so no new baseline was
needed and the 3.11 environment is the baseline environment (SPEC §4.6). `pyproject.toml` with an editable install: rejected, the skills use no package and
no `__init__.py`.

**Revisit if** a pinned package has no wheel for a target, or the owner moves both projects to a newer
Python.

Pinned by `test/Install/test_install.py` (16 tests, sourced with stubs).

## The legacy plan stays the regression input until step 6; the legacy planner is kept as a git tag

**Decision.** `Data/Reference/cell_plan.npz` (the verified IK plan) stays the regression input of the cycle run
(`Example/Simulation/run_cycle.py`, `test/Simulation`) until the keypoint program reproduces it (brief §7 step 6).
The legacy planner `legacy/plan_cell.py`, the legacy cycle test `legacy/test_cell_cycle.py` and their shims were
removed (owner, 2026-10-24, F4) after the annotated tag **`legacy-planner-reference`** (commit `f754e2d`) was set
on the last commit holding them. *Corrected 2026-10-24: until F4 the planner and the cycle test stayed in
`legacy/`, frozen.*

**Why.** The plan is the only verified end-to-end cycle, so every change before step 6 is measured against
it. The planner is not product code; step 4 needs it only as a reference for the fork-path joins, the
leave / return path around the closed handle and the keypoints, which `git show
legacy-planner-reference:legacy/plan_cell.py` gives. The port of the loop into `src/Simulation` was proven
before the removal: the same report and every measured value the same float as the legacy run (F3.5); the
planner regenerated the plan through the ported Kinematics and Collision code to 1.24e-14 rad (F3.4).

**Alternatives.** Keeping `legacy/` until step 6: rejected by the owner (root layout). Deleting the plan
too: rejected, nothing else would show that the physics is unchanged.

**Revisit if** step 6 passes: the keypoint program's trajectory replaces the plan as the regression input.

Pinned by `test/Simulation/test_core.py` (slow: the report equals the baseline; raw-value pins).

## The MG06 lock has 1.5 mm radial play

**Decision.** The lock's hook and pin in `models/scene.xml` have 1.5 mm radial play instead of the 0.5 mm of
`mg06_loader_mujoco` (the `GAP` of the MG06 build there).

**Why.** The robot pushes the lid down harder than the free-guided fork of the earlier version: with 0.5 mm
the lid sank 0.6 mm into the base, the hook hit the pin already at 27° and the handle never reached 29°.
With 1.5 mm the handle closes to 28.7° (SPEC §4.6) and the lid rises 0.5° in the play.

**Alternatives.** A softer lid contact: rejected, it would change the verified contact parameters.

**Revisit if** the real lock is measured.

Pinned by the fingerprint (`test/Scene/data/baseline_model_fingerprint.json`) and the report's
"MG06 closed" line (`test/Simulation/test_core.py`).

## The PCB is drawn in solder-mask green

**Decision.** The visual geom `pcb_vis` has rgba `0.05 0.42 0.18 1` in `models/scene.xml`; no material
(the look code skips `pcb*` geoms). The PCB's collision geoms keep their colours (group 3, hidden by
default).

**Why.** Brief goal 5: a realistic solder-mask green instead of orange. Measured on a `cam_pcb` render:
mean RGB 18/122/55 over the PCB's pixels (2026-10-24). The fingerprint shows exactly this one difference
from the baseline.

**Alternatives.** A material with a texture: not needed for the goal. Splitting the mesh into board and
components with their own colours: an open follow-up (`KNOWN_ISSUES.md`).

**Revisit if** the PCB mesh is split.

Pinned by `test/Scene/test_core.py::test_the_pcb_is_green_without_a_material` and `test/Render/test_core.py::test_the_pcb_camera_shows_a_green_pcb`.

## The CAD deviations stay compensated in scene.xml

**Decision.** `models/scene.xml` keeps three deliberate deviations from the CAD: the loader shifted
+100 mm in Y and +0.95 mm in X; the rear-right holder support (`holder_10`) lowered by 0.8 mm; the
electrical cabinet 200 mm deep instead of 300 mm. The scene is never regenerated from Blender.

**Why.** Without them the cycle does not work in the model (SPEC §4.10; the previous Czech README): the loader
lacks about 75 mm of stroke without the +100 mm; the jaw centre sits 0.95 mm left of the nest centre and
the nest has no width play, so the PCB lands on a pin edge and stays tilted 1.4°; `holder_10` is 0.8 mm
higher than the other three supports and the PCB's centre of mass lies outside the other three, so the
board always tilts onto it; with the 300 mm cabinet the Z-axis motor runs about 65 mm into it at
insertion.

**Alternatives.** Correcting the CAD: the owner's to decide (the holder may be a model error or a cut-out
in the PCB, recommended to check). Regenerating the scene: rejected, it would drop the corrections.

**Revisit if** the CAD is corrected or the real cell is measured.

Pinned by the fingerprint (`test/Scene/data/baseline_model_fingerprint.json`).
