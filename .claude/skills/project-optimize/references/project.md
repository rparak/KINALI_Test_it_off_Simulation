# project-optimize: project reference for KINALI_Test_it_off_Simulation

This project's part of optimization work. The generic skill is personal (`~/.claude/skills/`); this
file overrides it where they differ. No `SKILL.md` is tracked here.

## Outputs compared byte for byte

| Output | Produced by (into a temporary directory) | Deterministic inputs | Identical on two runs (date) |
|---|---|---|---|
| The cycle report (stdout) | `.venv/bin/python Example/Simulation/run_cycle.py --out "$SCRATCH"` | `Data/Reference/cell_plan.npz`, the scene files, `Data/PCB/pcb_meta.json`; no randomness | yes, 2026-10-24 (every run of F1-F3, Python 3.14 and 3.11, empty diffs against `test/Simulation/data/baseline_report.txt`) |
| The model fingerprint | `.venv/bin/python Example/Scene/model_fingerprint.py` (compares with the baseline) | the scene files | yes, 2026-10-24 |
| The program trajectory and check report | `.venv/bin/python Example/Program/build_program.py --out "$SCRATCH"` | the configuration files (`Cell.yaml`, `Constants.yaml`, `Simulation.yaml`, `Program.yaml`), the fork path, the scene files; no randomness | yes, 2026-10-24 (`test/Program/test_core.py::test_the_build_is_deterministic`; byte-identical on one machine and platform; `test/Program/test_golden.py` pins it to the F8 build) |
| The optimized program and its report | `.venv/bin/python Example/Program/optimize_program.py --out "$SCRATCH"` | the configuration files with `Optimization.yaml` (its seed), the fork path, the scene files | yes, 2026-10-24 (byte-identical to the committed `Data/Optimized/Program_Optimized.yaml`: `test/Example/Program/test_optimize_program.py`; one machine and platform) |
| `models/cell_scene.xml` | `.venv/bin/python Example/Scene/export_cell_scene.py --out "$SCRATCH"` | the scene files | identical to the committed file apart from `meshdir` (`test/Scene/test_core.py`) |

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
  `Example/Scene/model_fingerprint.py`.
- MuJoCo names by string: bodies, geoms, sites, actuators, cameras (`pcb_vis`, `aubo/tool/suction_tcp`,
  `vacuum0`, `cam_pcb`, ...) in code and tests.
- Files by path string: `models/`, `Data/`, the baselines,
  `.claude/skills/project-style-rpx/references/check_style.json`.
- Docs: SPEC, design-decisions, KNOWN_ISSUES, FEATURES, the README.

## Slow paths and timings

| Path | Time | Machine, date |
|---|---|---|
| cycle run (65.8 s simulated) | 15–16 s | Apple M2 Pro, 2026-10-24 |
| test suite, `-m "not slow"` / full | about 80 s / about 204 s (355 / 371 tests) | Apple M2 Pro, 2026-10-24 (after F9) |
| optimizer (Example/Program/optimize_program.py) | 34 s | Apple M2 Pro, 2026-10-24 |
| program build / check (default program) | 1.2 s / 8 s | Apple M2 Pro, 2026-10-24 |

## Interpreter and tools

- Project python: `.venv/bin/python` (Python 3.11); never a bare `python3` (miniforge on this
  machine). Explicit python for throwaway environments: `/opt/homebrew/bin/python3.11`.
- Code folders for radon: `src Example`.
