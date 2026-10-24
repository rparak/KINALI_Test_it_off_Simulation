# KINALI_Test_it_off_Simulation

MuJoCo simulation of the KINALI "Test it off" PCBA test cell (AUBO i5 + VGC10/fork, MG06 tester, two-axis
loader, crates). The internal docs are in `.claude/project-docs/`: the specification `SPEC.md`, the open
limitations `KNOWN_ISSUES.md`, the reasons behind choices `design-decisions.md`, and the feature tracker
`FEATURES.md`. `README.md` is the public page; it does not mention the internal docs or the skills.

Communicate with the owner in English only: reports, questions, summaries, commit messages and every
file. The owner's notes may be in Czech or Slovak; answer in English anyway. Commits carry no attribution
trailer (`.claude/settings.json`).

The verified physics is never changed without an explicit request (SPEC §3, the style project page
`mjcf.md`). Regression baseline: `test/Simulation/data/baseline_report.txt`,
`test/Scene/data/baseline_model_fingerprint.json`.

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

```bash
./install.sh                                     # .venv (Python 3.11) + requirements.txt; --recreate, --detect-only
.venv/bin/python -m pytest -m "not slow"         # everyday, about 15 s
.venv/bin/python -m pytest                       # before a commit, about 45 s
.venv/bin/python ~/.claude/skills/project-style-rpx/scripts/check_style.py   # src Example test, 0 issues expected
.venv/bin/python Example/run_cycle.py [--out DIR] # legacy plan: report (stdout + DIR/report.txt) + acceptance; exit 0/1/2; ~15 s
.venv/bin/python Example/model_fingerprint.py    # compare with the baseline: 1 difference expected (pcb_vis rgba)
.venv/bin/python Example/export_cell_scene.py    # models/cell_scene.xml
.venv/bin/python Example/show_cell.py            # MuJoCo viewer on the cell
.venv/bin/python Example/render_image.py         # output/pcb_green.png
.venv/bin/python legacy/test_cell_cycle.py       # the frozen reference run (until step 6)
```

Logs: `logger/<script>.log` and `logger/MUJOCO_LOG.TXT` (git-ignored).
