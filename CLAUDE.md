# CLAUDE.md

MuJoCo simulation of the KINALI "Test it off" PCBA test cell. The full brief (goals, model reference, lessons learned,
target design, work plan) is in [docs/PROJECT_BRIEF.md](docs/PROJECT_BRIEF.md) - read it before changing anything.
Feature tracking: [docs/FEATURES.md](docs/FEATURES.md).

- Language of code, comments, commits and docs: English (the owner writes in Czech/Slovak).
- The physics is verified: never change timestep, solver options, solref/solimp/solmix/margin/gap, friction, masses,
  geometry, actuator gains, the vacuum/adhesion setup or the CAD deviations in the scene without an explicit request.
- Regression baseline: [docs/baseline_report.txt](docs/baseline_report.txt) (printed report) and
  [docs/baseline_model_fingerprint.json](docs/baseline_model_fingerprint.json) (compiled model).

## Commands

Interpreter: `.venv/bin/python` (MuJoCo 3.16.0).

```bash
.venv/bin/python test_cell_cycle.py                                               # full cycle + report (~16 s)
.venv/bin/python cell.py && .venv/bin/python -m mujoco.viewer --mjcf=cell_scene.xml
.venv/bin/python tools/model_fingerprint.py --compare docs/baseline_model_fingerprint.json   # 0 differences expected
```
