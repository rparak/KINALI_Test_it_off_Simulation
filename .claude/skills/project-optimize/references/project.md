# project-optimize: project reference for KINALI_Test_it_off_Simulation

This project's part of optimization work. The generic skill is personal (`~/.claude/skills/`); this
file overrides it where they differ. No `SKILL.md` is tracked here.

## Outputs compared byte for byte

| Output | Produced by (into a temporary directory) | Deterministic inputs | Identical on two runs (date) |
|---|---|---|---|
| The cycle report (stdout) | `.venv/bin/python Example/run_cycle.py --out "$SCRATCH"` | `Data/Reference/cell_plan.npz`, the scene files, `Data/pcb_meta.json`; no randomness | yes, 2026-10-24 (every run of F1-F3, Python 3.14 and 3.11, empty diffs against `test/Simulation/data/baseline_report.txt`) |
| The model fingerprint | `.venv/bin/python Example/model_fingerprint.py` (compares with the baseline) | the scene files | yes, 2026-10-24 |
| `models/cell_scene.xml` | `.venv/bin/python Example/export_cell_scene.py --out "$SCRATCH"` | the scene files | identical to the committed file apart from `meshdir` (`test/Scene/test_core.py`) |

The report is the contract: a refactor of the physics path is proven by an empty diff against the
baseline report, plus the acceptance checks.

## Normalisation filter for printed output

Nothing varies in the report body. The acceptance block and the `report written to` line (stderr) name
an output path; compare stdout only. MuJoCo's attach warning goes to stderr.

## Never touched

- `models/scene.xml` (except decided changes), `models/aubo_i5/` (vendored), `models/robot_tool/`.
- `Data/Reference/cell_plan.npz` (the regression input, design-decisions), until step 6.
- The baseline files in `test/Simulation/data/` and `test/Scene/data/`.

## Reached without a grep hit

- Entry points: `Example/*.py`, `install.sh` (names `requirements.txt`); `test/conftest.py` imports
  `Example/model_fingerprint.py`.
- MuJoCo names by string: bodies, geoms, sites, actuators, cameras (`pcb_vis`, `aubo/tool/suction_tcp`,
  `vacuum0`, `cam_pcb`, ...) in code and tests.
- Files by path string: `models/`, `Data/`, the baselines,
  `.claude/skills/project-style-rpx/references/check_style.json`.
- Docs: SPEC, design-decisions, KNOWN_ISSUES, FEATURES, the README.

## Slow paths and timings

| Path | Time | Machine, date |
|---|---|---|
| cycle run (65.8 s simulated) | 15–16 s | Apple M2 Pro, 2026-10-24 |
| test suite, `-m "not slow"` / full | about 15 s / about 45 s (99 / 103 tests) | Apple M2 Pro, 2026-10-24 |

## Interpreter and tools

- Project python: `.venv/bin/python` (Python 3.11); never a bare `python3` (miniforge on this
  machine). Explicit python for throwaway environments: `/opt/homebrew/bin/python3.11`.
- Code folders for radon: `src Example`.
