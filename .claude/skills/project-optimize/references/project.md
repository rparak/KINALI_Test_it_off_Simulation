# project-optimize: project reference for KINALI_Test_it_off_Simulation

This project's part of optimization work (`SKILL.md` holds the procedure, the interpreter and the radon command).

## Cleanup criteria and the review of what can go

Anything the no-clutter rule excludes is a cleanup finding (style project reference > Layout and layers > No clutter). For the
review of what can go (`SKILL.md` > Cleanup > A review of what can go), in this project:

- *Used by*: the workflow is `Example/cell.py`'s subcommands; the checks are the two tiers in `CLAUDE.md` > Branches and
  checks; the tests are `pytest -n 8`.
- The restore point is written in the change's design-decisions entry and in `CLAUDE.md` > Branches and checks (F20:
  `c43f07961adbabf0edbffa891355afda323c29f7`).
- Kept history, where `git grep` may still find a removed name: `design-decisions.md` (superseded entries and the change's
  own entry), `SPEC.md`'s history notes, `FEATURES.md`'s done lines, the skills' project references where they say what is in
  git at a commit. Committed artefacts: the SVGs in `Media/` (they name matplotlib, which wrote them). A case-insensitive hit
  is checked by hand (`Compare_Programs` in `physics_baseline.py` is not the removed `compare_programs.py`).
- What stays (step 7): byte for byte the kinematic golden (`golden_program.py`, 0 differences), the point table and the
  check output (`cell.py check`), the build's check report and the optimized program, and the printed reports after the
  normalisation filter (below); the physics runs within the baselines' drift tolerances and the fork's envelope
  (`physics_baseline.py`, 0 failures; SPEC §10.2, `physics_baseline.CONST_TOLERANCES`, each program's `envelope` in
  `test/Simulation/data/baseline_metrics_<program>.json`). A change to the physics path itself is held to the empty diff of
  the reports (Outputs compared byte for byte).
- Install measurement, three runs with a warm pip cache, with `python3.11`, the interpreter `./install.sh --detect-only` names
  (`Using Python 3.11 at ...`): `python3.11 -m venv "$SCRATCH/v" && time "$SCRATCH/v/bin/pip" install -q -r requirements.txt`,
  then `du -sh "$SCRATCH/v"` and `"$SCRATCH/v/bin/pip" list | tail -n +3 | wc -l` (the packages, without the two header
  lines). 2026-10-29, Apple M2 Pro: 7.5-8.1 s, 289 MB, 21 packages (F20's numbers: design-decisions, one user entry point >
  Measured).
- The fresh clone: `git clone -q . "$SCRATCH/clone"`; in it, with `HOME="$SCRATCH/home" TMPDIR="$SCRATCH/tmp"` (both empty):
  `./install.sh`, `.venv/bin/python Example/cell.py all`, both tiers, both checkers. 2026-10-29: install 21.4 s (no pip
  cache under the empty `HOME`), `cell.py all` 52.5 s, the full suite 552 passed in 289.4 s.

## Outputs compared byte for byte

An internal module of `Example/Cell/` has no command line of its own (`cell.py` calls its `main(argv, log_name)`); to produce
its output with options such as `--out`, run it through that seam with the runner of `test/conftest.py`:
`RUN='import importlib.util, sys; spec = importlib.util.spec_from_file_location("module", sys.argv[1]); module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module); sys.exit(module.main(sys.argv[2:]))'`,
then `.venv/bin/python -c "$RUN" Example/Cell/<module>.py <options>`.

| Output | Produced by (into a temporary directory) | Deterministic inputs | Identical on two runs (date) |
|---|---|---|---|
| The programs' physics reports | `.venv/bin/python Example/Maintenance/physics_baseline.py --out "$SCRATCH"` (after `Cell/build_program.py` for both programs) | the trajectories, the scene files, `Data/PCB/pcb_meta.json`; no randomness | yes, 2026-10-25 (byte-identical reports on one machine; `test/Example/Maintenance/test_physics_baseline.py`); compared with tolerances in the regression (SPEC §10.2) |
| The model fingerprint | `.venv/bin/python Example/Maintenance/model_fingerprint.py` (compares with the baseline) | the scene files | yes, 2026-10-24 |
| The program trajectory and check report | `.venv/bin/python -c "$RUN" Example/Cell/build_program.py --out "$SCRATCH"` | the configuration files (`Cell.yaml`, `Constants.yaml`, `Simulation.yaml`, `Program.yaml`), the fork path, the scene files; no randomness | yes, 2026-10-24 (`test/Program/test_core.py::test_the_build_is_deterministic`; byte-identical on one machine and platform; `test/Program/test_golden.py` pins it to the golden data, `Example/Maintenance/golden_program.py`) |
| The optimized program and its report | `.venv/bin/python -c "$RUN" Example/Cell/optimize_program.py --out "$SCRATCH"` | the configuration files with `Optimization.yaml` (its seed), the fork path, the scene files | yes, 2026-10-24 (byte-identical to the committed `Data/Optimized/Program_Optimized.yaml`: `test/Example/Cell/test_optimize_program.py`; one machine and platform) |
| `models/cell_scene.xml` | `.venv/bin/python Example/Maintenance/export_cell_scene.py --out "$SCRATCH"` | the scene files | identical to the committed file apart from `meshdir` (`test/Scene/test_core.py`) |

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

- Entry points: `Example/cell.py`, `Example/Maintenance/*.py`, `install.sh` (names `requirements.txt`); `cell.py` imports
  `Example/Cell/*.py` and `physics_baseline.py` by name; `test/conftest.py` imports `Example/Maintenance/model_fingerprint.py`
  and runs `Example/Cell/` modules through their `main`.
- MuJoCo names by string: bodies, geoms, sites, actuators, cameras (`pcb_vis`, `aubo/tool/suction_tcp`,
  `vacuum0`, `cam_pcb`, ...) in code and tests.
- Files by path string: `models/`, `Data/`, the baselines, the skills' configurations
  (`.claude/skills/project-style-rpx/references/check_style.json`, `.claude/skills/readme-writer/references/check_readme.json`),
  every file under `.claude/skills/` (pinned by `test/Docs/test_layout.py`; the skills' tests through `pytest.ini` `testpaths`).
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
| test suite `-m "not slow"`, `-n 8` / serial | 43.8 s (one run, 2026-10-26) / about 2 min (503 tests; serial not re-measured) |
| test suite, full, `-n 8` / serial | 5.0 min (301.7-303.2 s, three runs, 2026-10-26) / about 7.4 min (526 tests; serial not re-measured) |
| after F18 (2026-10-28, one run each, not re-measured over three): 595 fast / 618 full tests, the skills' 88 under a second | fast 48.0-52.1 s; full 336.3 s (338.1 s in a fresh clone) |
| after F20 (2026-10-29, one run each): 532 fast / 552 full tests; `cell.py all` (the optimizer skipped); `install.sh` in a fresh clone (warm pip cache) | fast 36.2-70.1 s over the series' runs; full 286.0 s (290.9 s in a fresh clone); `all` 41.3 s (52.9 s in a fresh clone); install 22.6 s |
| after F22 and the `cell.py all` test's split (2026-10-29, alone; 556 full / 534 fast): the full suite, default `load` scheduling / `--dist worksteal` (one run); the split's three tests; peak memory of all suite processes (sampled each second) | full 309.2 s and 310.2 s (290.1 s before the split, 554 tests) / 253.7 s; B 154.2 s, C 52.6 s, A 45.4 s (A, B, C and the `run` test on one worker); 6.4 GB of 16 GB; the largest single process 2.1 GB (`/usr/bin/time -l`) |

Open (optional, a later measurement; owner, 2026-10-27): the 15 slowest tests (`pytest --durations=15`) and the serial
suites over three runs.

## Outputs with a tolerance

None: every output above is compared byte for byte. (The regenerated videos' PSNR tolerance went with the media generator
in F20; it is in git at `c43f079`.)
