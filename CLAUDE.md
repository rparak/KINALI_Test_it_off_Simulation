# Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo

Repository `KINALI_Test_it_off_Simulation`; project name **Test-it-off Simulation** (product spelling: Test-it-off).

MuJoCo simulation of KINALI's Test-it-off PCBA test cell (AUBO i5 + VGC10/fork, MG06 tester, two-axis
loader, crates). The internal docs are in `.claude/project-docs/`: the specification `SPEC.md`, the open
limitations `KNOWN_ISSUES.md`, the reasons behind choices `design-decisions.md`, and the feature tracker
`FEATURES.md`. `README.md` is the public page; it does not mention the internal docs or the skills.

Communicate with the owner in English only: reports, questions, summaries, commit messages and every
file. The owner's notes may be in Czech or Slovak; answer in English anyway. Commits carry no attribution
trailer (`.claude/settings.json`).

The verified physics is never changed without an explicit request (SPEC §3, the style project page
`mjcf.md`). Regression baselines: the programs' physics, `test/Simulation/data/baseline_report_<taught|optimized>.txt`
with their metrics (bounds and drift tolerances, SPEC §10.2); the kinematic golden, `test/Program/data/`; the model,
`test/Scene/data/baseline_model_fingerprint.json`. The legacy plan's run and baseline are retired (commit
`7a8560c8c16806739a02da8e989c2a2f65d1e41d`).

## Branches

Work directly on main. Commit only after the usual checks pass: the fingerprint (`model_fingerprint.py`, 1 difference),
the kinematic golden (`golden_program.py`, 0 differences), both physics baselines (`physics_baseline.py`, 0 failures), the
point table, the fast tests, the style checker. Push after each accepted feature. A failed experiment is undone with git
revert, never with force-push or history rewrite. Restore points are referenced by commit hash: `f754e2d0379b06ffb43286f4fb7aabf88e25ca5f` (the last commit with
the legacy planner), `4110010eeec0377c4e1801f7a2311994f79ab038` (the state before F9), `7a8560c8c16806739a02da8e989c2a2f65d1e41d` (the last commit with the legacy cycle
regression). Tags are used only for releases (`vMAJOR.MINOR`, annotated).

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
.venv/bin/python -m pytest -m "not slow"         # everyday, about 2 min
.venv/bin/python -m pytest                       # before a commit, about 7.5 min
.venv/bin/python ~/.claude/skills/project-style-rpx/scripts/check_style.py   # src Example test, 0 issues expected
.venv/bin/python Example/Simulation/run_program.py [--program taught|optimized] [--trajectory F] [--out DIR]   # physics of a program; exit 0/1/2; ~17 s
.venv/bin/python Example/Simulation/physics_baseline.py [--program P] [--trajectories DIR] [--out DIR]   # both programs vs their physics baselines; exit 0/1/2; ~35 s
.venv/bin/mjpython Example/Simulation/show_physics.py [--program taught|optimized] [--speed F] [--time T] [--watchdog S]   # physics in the viewer, real time
.venv/bin/python Example/Simulation/show_physics.py --headless --time T [--program ...] [--image [FILE]]   # physics to T; offscreen image
.venv/bin/python Example/Scene/model_fingerprint.py    # compare with the baseline: 1 difference expected (pcb_vis rgba)
.venv/bin/python Example/Scene/export_cell_scene.py    # models/cell_scene.xml
.venv/bin/python Example/Render/show_cell.py            # MuJoCo viewer on the cell
.venv/bin/python Example/Media/make_logo_variants.py # Media/<Logo>_Black|White.svg + Data/Generated/Images/logo_check.png
.venv/bin/python Example/Render/render_image.py         # Data/Generated/Images/pcb_green.png
.venv/bin/python Example/Keypoints/check_keypoints.py [--name K]   # the keypoint check table; exit 1 on a FAIL
.venv/bin/python Example/Keypoints/resolve_keypoint.py --name K [--seed K]   # prints the corrected Joints line
.venv/bin/python Example/Keypoints/show_keypoint.py --name K --headless [--image [FILE]]   # pose; offscreen image
.venv/bin/mjpython Example/Keypoints/show_keypoint.py --name K [--loader P --gripper S --pcb L --mg06 Open|Closed]
.venv/bin/mjpython Example/Keypoints/step_keypoints.py [--start K]   # Page Down / '.' next, Page Up previous
.venv/bin/python Example/Program/build_program.py [--program F] [--out DIR]   # trajectory + check report; exit 0/1
.venv/bin/python Example/Program/show_program.py (--step N | --time T) --headless [--image [FILE]]   # pose a moment; offscreen image
.venv/bin/mjpython Example/Program/show_program.py [--version default|optimized] [--watchdog S]   # playback: Home pause, Page Down/'.' next segment, Page Up, End
.venv/bin/python Example/Program/optimize_program.py [--out DIR]   # Data/Optimized/Program_Optimized.yaml + report; exit 0/1; ~55 s
.venv/bin/python Example/Program/compare_programs.py [--out DIR]   # legacy / default / optimized: table + figure
.venv/bin/python Example/Program/golden_program.py [--out DIR --source TEXT]   # taught program vs test/Program/data golden; exit 0/1
```

The viewer scripts (`show_keypoint.py`, `step_keypoints.py`, `show_program.py`, `show_physics.py`) use
`mujoco.viewer.launch_passive`, which macOS runs only under `.venv/bin/mjpython`; under `.venv/bin/python` they exit 2
naming the mjpython command. The cell configuration (`Cell.yaml`) and the cell
program (`Constants.yaml` named values, `Simulation.yaml` simulation-only data, `Program.yaml` the program; read by
`src/Configuration/Program_Parser.py`) and the optimizer's settings (`Optimization.yaml`) are edited by hand in `Configuration/` (no
tool writes them). `Data/Optimized/Program_Optimized.yaml` is written only by `Example/Program/optimize_program.py` (committed;
re-run it when an input changes - a build refuses it stale).

History across the RPX rewrite (commit `c6282af`, files rewritten below git's 50 % rename threshold):
`git log --follow -M20% <file>`. The legacy planner and cycle test: commit `f754e2d0379b06ffb43286f4fb7aabf88e25ca5f`.

Logs: `logger/<script>.log` and `logger/MUJOCO_LOG.TXT`; generated artefacts: `Data/Generated/<Kind>/` (both git-ignored).
