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

Pinned by (after F3) `test/Docs/test_layout.py` (root entries, skill folders).

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

Pinned by `test/Scene/test_core.py` (fingerprint) and the slow regression (after F3).

## Python 3.11, pinned requirements, installed by install.sh

**Decision.** The interpreter is Python 3.11 (`/opt/homebrew/bin/python3.11` on macOS, `python3.11` on
Linux); every direct dependency is pinned `==` in `requirements.txt` with the owner's pin list; the
environment is created by `install.sh` into `.venv`. `pyproject.toml` is removed (owner, 2026-10-24).

**Why.** The same versions as the owner's other MuJoCo project, so both share one verified toolchain;
project-style-rpx prescribes the installer and pinned direct dependencies. The interpreter is named
explicitly because a bare `python3` on this machine is miniforge's.

**Alternatives.** Keeping the Python 3.14.7 / numpy 2.5.4 environment the baseline was first measured
in: rejected by the owner; the baseline is re-measured in the pinned environment instead
(`KNOWN_ISSUES.md`). `pyproject.toml` with an editable install: rejected, the skills use no package and
no `__init__.py`.

**Revisit if** a pinned package has no wheel for a target, or the owner moves both projects to a newer
Python.

Pinned by `test/Install/test_install.py` (after F3).

## The legacy plan stays the regression reference until step 6

**Decision.** `legacy/cell_plan.npz` (the verified IK plan), `legacy/plan_cell.py` (its planner) and
`legacy/test_cell_cycle.py` (its physics run) stay in the repository, frozen: only imports and paths
change. The cycle run of `Example/run_cycle.py` is proven against them; they are removed when the
keypoint program reproduces the reference (brief §7 step 6).

**Why.** It is the only verified end-to-end cycle; every refactor before step 6 is measured against it
(SPEC §10.2). The planner reproduces its plan to 1.2e-14 rad, so it also proves the kinematics and
collision code it uses.

**Alternatives.** Removing it with the restructure: rejected, nothing else would show that the physics
is unchanged. Porting it into `src/`: rejected, it is the reference, not the product.

**Revisit if** step 6 passes.

Pinned by `test/Simulation/test_core.py` (slow regression; after F3).

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

Pinned by `tests/test_model.py::test_pcb_is_green` (to move to `test/Scene/test_core.py`).

## The CAD deviations stay compensated in scene.xml

**Decision.** `models/scene.xml` keeps three deliberate deviations from the CAD: the loader shifted
+100 mm in Y and +0.95 mm in X; the rear-right holder support (`holder_10`) lowered by 0.8 mm; the
electrical cabinet 200 mm deep instead of 300 mm. The scene is never regenerated from Blender.

**Why.** Without them the cycle does not work in the model (`docs/README_cs_mg06_cell.md`): the loader
lacks about 75 mm of stroke without the +100 mm; the jaw centre sits 0.95 mm left of the nest centre and
the nest has no width play, so the PCB lands on a pin edge and stays tilted 1.4°; `holder_10` is 0.8 mm
higher than the other three supports and the PCB's centre of mass lies outside the other three, so the
board always tilts onto it; with the 300 mm cabinet the Z-axis motor runs about 65 mm into it at
insertion.

**Alternatives.** Correcting the CAD: the owner's to decide (the holder may be a model error or a cut-out
in the PCB, recommended to check). Regenerating the scene: rejected, it would drop the corrections.

**Revisit if** the CAD is corrected or the real cell is measured.

Pinned by the fingerprint (`docs/baseline_model_fingerprint.json`).
