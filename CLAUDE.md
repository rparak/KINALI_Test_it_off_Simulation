# Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo

Repository `KINALI_Test_it_off_Simulation`; project name **Test-it-off Simulation** (product spelling: Test-it-off).

MuJoCo simulation of KINALI's Test-it-off PCBA test cell (AUBO i5 + VGC10/fork, MG06 tester, two-axis
loader, crates). The internal docs are in `.claude/project-docs/`: the specification `SPEC.md`, the open
limitations `KNOWN_ISSUES.md`, the reasons behind choices `design-decisions.md`, and the feature tracker
`FEATURES.md`. `README.md` is the public page; it does not mention the internal docs (it names the shipped skills).

Communicate with the owner in English only: reports, questions, summaries, commit messages and every
file. The owner's notes may be in Czech or Slovak; answer in English anyway. Commits carry no attribution
trailer (`.claude/settings.json`).

The verified physics is never changed without an explicit request (SPEC §3, the style project page
`mjcf.md`). Regression baselines: the programs' physics, `test/Simulation/data/baseline_report_<taught|optimized>.txt`
with their metrics (bounds and drift tolerances, SPEC §10.2); the kinematic golden, `test/Program/data/`; the model,
`test/Scene/data/baseline_model_fingerprint.json`.

## Branches and checks

Work directly on main. Push after each accepted feature. A failed experiment is undone with git revert, never with
force-push or history rewrite. Tags are used only for releases (`vMAJOR.MINOR`, annotated). Restore points by commit
hash: `f754e2d0379b06ffb43286f4fb7aabf88e25ca5f` (the legacy planner), `4110010eeec0377c4e1801f7a2311994f79ab038` (before F9),
`7a8560c8c16806739a02da8e989c2a2f65d1e41d` (the legacy cycle regression), `c43f07961adbabf0edbffa891355afda323c29f7` (before F20:
the removed generators and scripts).

The checks, in two tiers (Apple M2 Pro times):

- **Per commit** (about 1 min): the fingerprint (`Maintenance/model_fingerprint.py`, 1 difference: `pcb_vis` rgba), the kinematic
  golden (`Maintenance/golden_program.py`, 0 differences), the point table (`cell.py check`, 0 FAIL), the fast tests
  (`pytest -n 8 -m "not slow"`), the style checker (0 issues).
- **Per accepted feature, before the push** (about 5.5 min): both physics baselines with their envelopes
  (`Maintenance/physics_baseline.py`, 0 failures) and the full suite (`pytest -n 8`).

A change on the physics or build path is also proven by its outputs, byte for byte (the project-optimize reference).

## Skills

The repository ships its five Claude Code skills in `.claude/skills/<skill>/` (`SKILL.md`, `references/`, and
`assets/` or `scripts/` where a skill has them; owner, 2026-10-28). Each is self-contained: the generic rules in
`SKILL.md`, this project's decisions in `references/project.md`. Licence: `.claude/skills/LICENSE` (MIT, Roman Parak).

- `project-style-rpx` — code, shell, YAML and Markdown conventions; `references/project.md`, `mjcf.md`,
  `check_style.json`; the style checker. Use it for every change.
- `feature-manager` — how a feature, a behaviour change or a bug fix is made; `references/project.md`.
- `project-optimize` — behaviour-preserving work; the outputs compared in `references/project.md`.
- `readme-writer` — every README change; `references/project.md`, `check_readme.json`; the README checker and renderer.
- `support-brief` — creates or refreshes `.claude/project-docs/SUPPORT_CHAT.md`, the brief the owner pastes into a new
  Claude chat that acts as the project's senior technical support; `references/project.md`. Its `SKILL.md` is generic
  (no project content). Invoke `/support-brief`: CREATE when the file does not exist, REFRESH when it does (the skill
  states which); add "no review" to write, check and commit locally without stopping for the report (never a push before
  the owner's acceptance). When the brief is updated:
  feature-manager's `references/project.md` > The support-chat brief.

A personal skill of the same name (`~/.claude/skills/<skill>/`) takes precedence over the project's copy, so none of the
five is installed personally on this machine (`KNOWN_ISSUES.md`).

## Development setup

Everything the agent conventions need is in the repository: the skills, their checkers and their tests
(`.claude/skills/`, Skills above). No personal skill is needed; nothing in `src/`, `test/`, `Example/` or `.claude/skills/`
reads `~/.claude/` (verified 2026-10-28: a fresh clone with `HOME` set to an empty directory installs and passes the
per-commit tier, the full suite and both checkers). The skills came from
`rparak/rp-claude-skills` at `ec7af02` and are maintained here from then on (design-decisions: the skills in the repository).

## Commands

Never use a bare `python3` on this machine (miniforge's). Use `.venv/bin/python` (Python 3.11).

```bash
./install.sh                                     # .venv (Python 3.11) + requirements.txt; --recreate, --detect-only
.venv/bin/python -m pytest -n 8 -m "not slow"    # per commit, about 45 s (serially, -n 0: about 2 min)
.venv/bin/python -m pytest -n 8                  # per accepted feature, about 5 min (serially: about 7.5 min)
.venv/bin/python .claude/skills/project-style-rpx/scripts/check_style.py   # src Example test, 0 issues expected
.venv/bin/python .claude/skills/readme-writer/scripts/check_readme.py README.md   # 0 errors expected (1 warning: the length)
/opt/homebrew/bin/python3.11 -m venv "$SCRATCH/gfm" && "$SCRATCH/gfm/bin/pip" install -q cmarkgfm==2025.10.22   # throwaway env, never .venv
"$SCRATCH/gfm/bin/python" .claude/skills/readme-writer/scripts/render_readme.py README.md --out "$SCRATCH/readme.html"   # 0 leftovers
.venv/bin/python Example/cell.py --help                        # the user entry point; <subcommand> --help for each
.venv/bin/python Example/cell.py check [--name K]              # the point table + the configuration and programs check; exit 0/1; ~3 s
.venv/bin/python Example/cell.py point K [--seed K] [--show | --headless] [--loader L --gripper G --pcb LOC --mg06 S]   # a point's row, its corrected Joints/Pose lines
.venv/bin/python Example/cell.py build                         # the Default program: trajectory + check report; exit 0/1; ~11 s
.venv/bin/python Example/cell.py optimize                      # Data/Optimized/Program_Optimized.yaml + its trajectory; exit 0/1; ~70 s
.venv/bin/python Example/cell.py run [--program default|optimized]   # both programs vs the physics baselines + the table; exit 0/1/2; ~21 s
.venv/bin/python Example/cell.py show [--program P] [--physics] [--step N | --time T] [--speed F] [--headless] [--watchdog S]   # the viewer (re-launches under mjpython on macOS)
# Maintenance tools (Example/Maintenance/, documented here only):
.venv/bin/python Example/Maintenance/model_fingerprint.py      # compare with the baseline: 1 difference expected (pcb_vis rgba); --out FILE writes
.venv/bin/python Example/Maintenance/export_cell_scene.py      # models/cell_scene.xml
.venv/bin/python Example/Maintenance/golden_program.py [--out DIR --source TEXT]   # taught program vs test/Program/data golden; exit 0/1; ~11 s
.venv/bin/python Example/Maintenance/physics_baseline.py [--program P] [--trajectories DIR] [--out DIR]   # both programs vs their physics baselines (in parallel); exit 0/1/2; ~21 s; --out ~1 min
```

The viewers (`cell.py show`, `cell.py point --show`) use `mujoco.viewer.launch_passive`, which macOS runs only under
`.venv/bin/mjpython`: `cell.py` re-launches itself under it with the same arguments, or exits 2 with the install hint when it
is missing. The cell configuration (`Cell.yaml`) and the cell
program (`Constants.yaml` named values, `Simulation.yaml` simulation-only data, `Program.yaml` the program; read by
`src/Configuration/Program_Parser.py`) and the optimizer's settings (`Optimization.yaml`) are edited by hand in `Configuration/` (no
tool writes them). `Data/Optimized/Program_Optimized.yaml` is written only by `cell.py optimize` (`Example/Cell/optimize_program.py`;
committed; re-run it when an input changes - a build refuses it stale).

History across the RPX rewrite (commit `c6282af`, files rewritten below git's 50 % rename threshold):
`git log --follow -M20% <file>`. The full feature records: `git show e7739f4:.claude/project-docs/FEATURES.md`.

Logs: `logger/cell_<subcommand>.log`, `logger/<tool>.log` (the maintenance tools) and `logger/MUJOCO_LOG.TXT`; generated artefacts: `Data/Generated/<Kind>/` (both git-ignored).
