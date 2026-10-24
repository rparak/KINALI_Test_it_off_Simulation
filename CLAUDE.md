# CLAUDE.md

MuJoCo simulation of the KINALI "Test it off" PCBA test cell. The full brief (goals, model reference, lessons learned,
target design, work plan) is in [docs/PROJECT_BRIEF.md](docs/PROJECT_BRIEF.md) - read it before changing anything.
Feature tracking: [docs/FEATURES.md](docs/FEATURES.md). Old Czech README: [docs/README_cs_mg06_cell.md](docs/README_cs_mg06_cell.md) (paths there are pre-restructure).

- Language of code, comments, commits and docs: English (the owner writes in Czech/Slovak).
- The physics is verified: never change timestep, solver options, solref/solimp/solmix/margin/gap, friction, masses,
  geometry, actuator gains, the vacuum/adhesion setup or the CAD deviations in the scene without an explicit request.
- Regression baseline: [docs/baseline_report.txt](docs/baseline_report.txt) (printed report) and
  [docs/baseline_model_fingerprint.json](docs/baseline_model_fingerprint.json) (compiled model).

## Commands

Interpreter: `.venv/bin/python` (MuJoCo 3.16.0). Setup: `python3 -m venv .venv && .venv/bin/pip install -e ".[dev]"`.

```bash
.venv/bin/kinali-sim run                     # legacy plan, physics + report (output/report.txt) + acceptance; ~16 s
.venv/bin/kinali-sim export-xml              # models/cell_scene.xml
.venv/bin/python -m mujoco.viewer --mjcf=models/cell_scene.xml
.venv/bin/python -m pytest -m "not slow"     # fast tests
.venv/bin/python -m pytest -m slow           # full-cycle regression
.venv/bin/python tools/model_fingerprint.py --compare docs/baseline_model_fingerprint.json
```

## Layout

`src/kinali_sim/` package (`paths.py` resolves every folder from the package, never from the working directory;
`model.py` builds the cell; `kinematics.py` IK + collision queries; `cli.py`), `models/` scene MJCFs, `assets/` meshes,
`data/` PCB poses + recorded fork path, `legacy/` the verified IK planner, plan and cycle run (reference until brief
§7 step 6), `config/` (empty until step 3), `output/` (git-ignored), `tests/`, `tools/`.
