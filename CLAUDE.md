# KINALI_Test_it_off_Simulation

MuJoCo simulation of the KINALI "Test it off" PCBA test cell (AUBO i5 + VGC10/fork, MG06 tester, two-axis
loader, crates). The internal docs are in `.claude/project-docs/`: the specification `SPEC.md`, the open
limitations `KNOWN_ISSUES.md`, the reasons behind choices `design-decisions.md`, and the feature tracker
`FEATURES.md`. `README.md` is the public page; it does not mention the internal docs or the skills.

Communicate with the owner in English only: reports, questions, summaries, commit messages and every
file. The owner's notes may be in Czech or Slovak; answer in English anyway. Commits carry no attribution
trailer (`.claude/settings.json`).

The verified physics is never changed without an explicit request (SPEC §3, the style project page
`mjcf.md`). Regression baseline: `docs/baseline_report.txt`, `docs/baseline_model_fingerprint.json`.

## Skills

The skills are personal (`~/.claude/skills/`, `rparak/rp-claude-skills`); this repository tracks only
their project layer, `.claude/skills/<skill>/references/` (never a `SKILL.md`; `.gitignore` excludes one).

- `project-style-rpx` — code, shell, YAML and Markdown conventions; `references/project.md`, `mjcf.md`,
  `check_style.json`. Use it for every change.
- `feature-manager` — how a feature, a behaviour change or a bug fix is made; `references/project.md`.
- `project-optimize` — behaviour-preserving work; the outputs compared in `references/project.md`.
- `readme-writer` — every README change; `references/project.md`.

## Commands

Never use a bare `python3` on this machine (miniforge's). Use `.venv/bin/python` (Python 3.11).
The layout is being migrated to the RPX layout (`FEATURES.md` F3); until then:

```bash
./install.sh                                  # .venv (Python 3.11) + requirements.txt; --recreate, --detect-only
.venv/bin/python legacy/test_cell_cycle.py    # legacy plan, physics + report; ~15 s
PYTHONPATH=src .venv/bin/python -m kinali_sim.cli run          # the same + acceptance + output/report.txt
PYTHONPATH=src .venv/bin/python -m kinali_sim.cli export-xml   # models/cell_scene.xml
.venv/bin/python -m mujoco.viewer --mjcf=models/cell_scene.xml
.venv/bin/python -m pytest -m "not slow"     # fast tests
.venv/bin/python -m pytest -m slow           # full-cycle regression
.venv/bin/python tools/model_fingerprint.py --compare docs/baseline_model_fingerprint.json
.venv/bin/python ~/.claude/skills/project-style-rpx/scripts/check_style.py src Example test   # after F3
```
