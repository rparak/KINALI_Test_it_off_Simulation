# project-optimize: project reference for KINALI_Test_it_off_Simulation

This project's part of optimization work. The generic skill is personal (`~/.claude/skills/`); this
file overrides it where they differ. No `SKILL.md` is tracked here.

## Outputs compared byte for byte

| Output | Produced by (into a temporary directory) | Deterministic inputs | Identical on two runs (date) |
|---|---|---|---|
| The programs' physics reports | `.venv/bin/python Example/Simulation/physics_baseline.py --out "$SCRATCH"` (after `build_program.py` for both programs) | the trajectories, the scene files, `Data/PCB/pcb_meta.json`; no randomness | yes, 2026-10-25 (byte-identical reports on one machine; `test/Example/Simulation/test_physics_baseline.py`); compared with tolerances in the regression (SPEC §10.2) |
| The model fingerprint | `.venv/bin/python Example/Scene/model_fingerprint.py` (compares with the baseline) | the scene files | yes, 2026-10-24 |
| The program trajectory and check report | `.venv/bin/python Example/Program/build_program.py --out "$SCRATCH"` | the configuration files (`Cell.yaml`, `Constants.yaml`, `Simulation.yaml`, `Program.yaml`), the fork path, the scene files; no randomness | yes, 2026-10-24 (`test/Program/test_core.py::test_the_build_is_deterministic`; byte-identical on one machine and platform; `test/Program/test_golden.py` pins it to the golden data, `Example/Program/golden_program.py`) |
| The optimized program and its report | `.venv/bin/python Example/Program/optimize_program.py --out "$SCRATCH"` | the configuration files with `Optimization.yaml` (its seed), the fork path, the scene files | yes, 2026-10-24 (byte-identical to the committed `Data/Optimized/Program_Optimized.yaml`: `test/Example/Program/test_optimize_program.py`; one machine and platform) |
| `models/cell_scene.xml` | `.venv/bin/python Example/Scene/export_cell_scene.py --out "$SCRATCH"` | the scene files | identical to the committed file apart from `meshdir` (`test/Scene/test_core.py`) |

The physics reports are the contract: a refactor of the physics path is proven by an empty diff against both programs'
baseline reports on the same machine (stricter than the regression's drift tolerances), plus the acceptance checks.

## Normalisation filter for printed output

Nothing varies in the physics report body. The acceptance block and the `report written to` line (stderr) name an output
path; compare stdout only, the output folder's path replaced by a token. MuJoCo's attach warning goes to stderr. Two more
fields vary by design, and only these are filtered:

- a trajectory's `meta` records `git_commit` and `git_dirty` (provenance): compare every array of the `.npz` byte for byte
  and `meta` with those two keys removed;
- the optimizer's report and stdout print its wall-clock time (`made in N s`): replace the number by a token.

## Never touched

- `models/scene.xml` (except decided changes), `models/aubo_i5/` (vendored), `models/robot_tool/`.
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

Apple M2 Pro, 2026-10-25, three runs each (spread under 3 %):

| Path | Time |
|---|---|
| one program's physics run (`run_program.py`, 70.4 s simulated) | 19.4 s |
| both physics baselines, compare (`physics_baseline.py`, the two runs in parallel) | 20.8 s (35.8 s sequential) |
| the baselines' capture, `--out` (20 runs in parallel) | 56.3 s (369.5 s sequential) |
| one build with its check (`build_program.py`), taught / optimized | 10.8 s / 10.2 s |
| the kinematic golden (`golden_program.py`) | 10.8 s |
| the optimizer (`optimize_program.py`) | 59.8 s |
| fingerprint / point table / style checker | 1.2 s / 0.4 s / 0.7 s |
| test suite `-m "not slow"`, `-n 8` / serial | about 30 s / about 2 min (426 tests) |
| test suite, full, `-n 8` / serial | about 2.8 min / about 7.4 min (446 tests) |

## Interpreter and tools

- Project python: `.venv/bin/python` (Python 3.11); never a bare `python3` (miniforge on this
  machine). Explicit python for throwaway environments: `/opt/homebrew/bin/python3.11`.
- Code folders for radon: `src Example`.
