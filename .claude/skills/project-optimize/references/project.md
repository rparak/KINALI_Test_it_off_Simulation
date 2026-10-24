# project-optimize: project reference for KINALI_Test_it_off_Simulation

This project's part of optimization work. The generic skill is personal (`~/.claude/skills/`); this
file overrides it where they differ. No `SKILL.md` is tracked here.

## Outputs compared byte for byte

| Output | Produced by (into a temporary directory) | Deterministic inputs | Identical on two runs (date) |
|---|---|---|---|
| The cycle report (stdout) | until F3: `.venv/bin/kinali-sim run --report "$SCRATCH/report.txt"`; after: `.venv/bin/python Example/run_cycle.py --out "$SCRATCH"` | `legacy/cell_plan.npz`, the scene files, `Data/pcb_meta.json`; no randomness | yes, 2026-10-24 (four runs in two layouts, empty diffs against `docs/baseline_report.txt`) |
| The model fingerprint | `Example/model_fingerprint.py --compare <baseline>` (until F3 `tools/model_fingerprint.py`) | the scene files | yes, 2026-10-24 |
| `models/cell_scene.xml` | `Example/export_cell_scene.py --out "$SCRATCH"` (until F3 `kinali-sim export-xml --out`) | the scene files | not yet checked twice |
| `legacy/cell_plan.npz` | `legacy/plan_cell.py --out "$SCRATCH/plan.npz"` | the scene files, `Data/fork_close_open_path.npz` | arrays equal to 1.2e-14 rad, not bytes (2026-10-24) |

The report is the contract: a refactor of the physics path is proven by an empty diff against the
baseline report, plus the acceptance checks.

## Normalisation filter for printed output

Nothing varies in the report body. The acceptance block and the `report written to` line (stderr) name
an output path; compare stdout only. MuJoCo's attach warning goes to stderr.

## Never touched

- `models/scene.xml` (except decided changes), `models/aubo_i5/` (vendored), `models/robot_tool/`.
- `legacy/` logic and numbers (design-decisions), until step 6.
- The baseline files in `docs/` (to move to `test/<Domain>/data/`).

## Reached without a grep hit

- Entry points: `Example/*.py`, `install.sh` (names `requirements.txt`), `legacy/*.py` through the shims.
- MuJoCo names by string: bodies, geoms, sites, actuators, cameras (`pcb_vis`, `aubo/tool/suction_tcp`,
  `vacuum0`, `cam_pcb`, ...) in code and tests.
- Files by path string: `models/`, `Data/`, `legacy/cell_plan.npz`, the baselines,
  `.claude/skills/project-style-rpx/references/check_style.json`.
- Docs: SPEC, design-decisions, KNOWN_ISSUES, FEATURES, the README.

## Slow paths and timings

| Path | Time | Machine, date |
|---|---|---|
| cycle run (65.8 s simulated) | 15–16 s | Apple M2 Pro, 2026-10-24 |
| `legacy/plan_cell.py` | 6.9 s | Apple M2 Pro, 2026-10-24 |
| test suite, fast / slow | 1.2 s / 15.1 s | Apple M2 Pro, 2026-10-24 (brief layout) |

## Interpreter and tools

- Project python: `.venv/bin/python` (Python 3.11 after F3); never a bare `python3` (miniforge on this
  machine). Explicit python for throwaway environments: `/opt/homebrew/bin/python3.11`.
- Code folders for radon: `src Example`.
