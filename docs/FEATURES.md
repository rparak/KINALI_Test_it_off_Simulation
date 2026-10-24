# Features

Tracked with the feature-manager procedure (requirements restated, premise checked, measured before/after).
Brief: [PROJECT_BRIEF.md](PROJECT_BRIEF.md). Baseline: [baseline_report.txt](baseline_report.txt),
[baseline_model_fingerprint.json](baseline_model_fingerprint.json).

Status: `[ ]` open, `[x]` done and verified, `[~]` deferred.

## F1 - Restructure into the `kinali_sim` package (brief §7 step 1)

Requirements:

1. Layout of brief §5.1: `src/kinali_sim/`, `models/`, `assets/`, `data/`, `config/`, `output/`, `legacy/`, `tests/`.
2. Every move is a `git mv` (history kept).
3. All file paths resolve through `kinali_sim.paths`, relative to the package, never to the working directory.
4. The compiled model is unchanged: fingerprint compare against the baseline reports 0 differences.
5. `kinali-sim run` reproduces `docs/baseline_report.txt` line for line, writes `output/report.txt`, and exits non-zero
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

## F2 - Green PCB (brief §7 step 2)

Requirements:

1. `pcb_vis` rgba `0.91 0.45 0.03 1` -> `0.05 0.42 0.18 1` in `models/scene.xml`; no other place sets the PCB colour.
2. PCB collision geoms untouched.
3. Fingerprint compare shows exactly one difference: `geoms/pcb_vis/geom_rgba`.
4. `kinali-sim run` still reproduces the baseline report.
5. Proof image `output/pcb_green.png` from camera `cam_pcb` at the initial state.
6. README stub: install, `kinali-sim run`, `kinali-sim export-xml`, viewer command. The previous Czech README is kept
   as `docs/README_cs_mg06_cell.md` until the full README (brief §7 step 7).

Sub-tasks:

- [x] C1 rgba change, grep for other PCB colour / material settings
- [x] C2 export, load and viewer check, proof image
- [x] C3 fingerprint: exactly one difference; report unchanged
- [x] C4 README stub
- [~] Follow-up: split the PCB mesh into board and components (components above the board top) and colour the
      components grey/black (brief §7 step 2, optional). Not done in F2.
