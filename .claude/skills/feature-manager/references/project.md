# feature-manager: project reference for KINALI_Test_it_off_Simulation

This project's part of the change procedure. The generic skill is personal (`~/.claude/skills/`);
this file overrides it where they differ. No `SKILL.md` is tracked here (a personal skill wins over a
project skill of the same name). Style rules come from project-style-rpx and its project reference
(`.claude/skills/project-style-rpx/references/project.md`, `mjcf.md`).

## What the output is

Test-it-off Simulation, a MuJoCo simulation of KINALI's Test-it-off cell (SPEC §1): the compiled cell model, the cell
program (`Constants.yaml`, `Simulation.yaml`, `Program.yaml`) and the optimized program, their trajectory files and check
reports, and the programs' physics runs with their reports and acceptance verdicts. The owner and whoever writes the robot program act on it.

## Physical system and safety

Today the output drives nothing real; a transfer to the real AUBO i5 is planned (SPEC §17,
design-decisions). Every result carries: **"Outputs (keypoints, programs, trajectories) are simulation
results and are NOT validated for driving the real cell."** Before any use on real hardware they go
through the real-cell validation procedure (to be defined, `KNOWN_ISSUES.md`), at reduced speed with the
robot controller's own safety limits active. Formats stay controller-friendly (SPEC §17).

Language: English in every file, report and commit; the owner's notes may be in Czech or Slovak.

## Always "large" (ask before building)

- The verified physics (SPEC §3, mjcf.md): timestep, solver options, contact parameters, friction,
  masses, geometry, actuator gains and ranges, vacuum, bellows, the CAD deviations.
- Formats: `models/scene.xml`, `Data/PCB/pcb_meta.json`, the trajectory file and
  the cell program's three files and their grammar (SPEC §8, F10), the report's lines.
- The acceptance criteria (SPEC §10.1) and the 3 mm keypoint / segment clearance.
- `requirements.txt` pins and the interpreter.

## Real data for reproducing a premise

The two programs and their physics runs (`Example/Simulation/run_program.py`; their baselines and the kinematic golden in
`test/<Domain>/data/`; the legacy planner in commit `f754e2d0379b06ffb43286f4fb7aabf88e25ca5f`, the legacy cycle run in
`7a8560c8c16806739a02da8e989c2a2f65d1e41d`), the fingerprint, the scene files. Walk the chain: the
plan's joint trajectory and events, the servo set-points, the contacts per category, the PCB pose at
each mark, the report line.

## Premise example

(Hypothetical) "the PCB lands crooked in MG06" could be a robot handover pressing the board onto the
jaw ledges (SPEC §4.7), the loader's X offset, the raised `holder_10`, slip in the cups during the
transfer, or a changed contact parameter. Each needs a different fix; widening the tilt bound would
hide all of them.

## Typical better levers

Fixing the one keypoint or approach speed instead of a contact parameter; reporting a near contact
instead of excluding the pair; failing a run explicitly instead of clamping a value.

## Domain checklist

**Physics and model**

- [ ] The compiled model's fingerprint is unchanged, or the change says which fields and why (SPEC §10.2).
- [ ] The printed report diffs empty against the baseline, or every changed line is explained (SPEC §10.2).
- [ ] No `margin`/`gap` on PCB or cup geoms (MuJoCo 3.16, SPEC §4.7).
- [ ] Instability fails the run (`d.time` backwards → exit 2), never silently (SPEC §4.7).
- [ ] The CAD deviations stay (design-decisions).

**Robot motion**

- [ ] Cups pressed 1.5 mm before vacuum on; no press at the loader handover (SPEC §4.7).
- [ ] IK family: wrist up for the PCB, wrist down for the fork; switch only in free space (SPEC §4.7).
- [ ] Fork path: never push the handle down before opening; bar centred in the slot when leaving (SPEC §4.7).
- [ ] Speeds and accelerations within the datasheet fraction and the assumed limits (SPEC §4.4).
- [ ] Clearance ≥ 3 mm for keypoints and segments, carried PCB included (SPEC §2 items 3–4).

**Acceptance**

- [ ] The §10.1 criteria are checked from the run's returned values, not from printed text.
- [ ] A bound is never widened to let a change pass; a pin reports its numbers.

**Simulation vs reality**

- [ ] Not modelled: cables, real vacuum dynamics beyond the adhesion force, the real loader controller,
      tolerances of the real cell. A change that relies on any of them says so.
- [ ] The baseline is per environment (`KNOWN_ISSUES.md`).

## Budgets

The 3 mm planning clearance (6 mm geom margin in the planning model, SPEC §7.4); the acceptance bounds
of SPEC §10.1. Before and after numbers for any change to physics, kinematics or checking: the full
report and the fingerprint.

## Visual changes

Open the cell in the viewer (`Example/Render/show_cell.py`) or render an image (`Example/Render/render_image.py`) and
look at what changed.

## Commits

- No attribution trailer (`.claude/settings.json`, owner 2026-10-24).
- Work directly on main, no feature branches (owner, 2026-10-24; `CLAUDE.md` > Branches): commit only after the usual
  checks pass, push after each accepted feature, undo a failed experiment with `git revert` (never force-push or rewrite).
- Never committed: `logger/`, `Data/Generated/` (the trajectories included; owner 2026-10-24), `.venv/`.
  The optimized program (`Data/Optimized/Program_Optimized.yaml`) is committed, with its input hashes: written only by
  `Example/Program/optimize_program.py`; when an input changes, run it again and commit the result as its own change.
- Only as its own deliberate change, with its report: a regenerated `models/cell_scene.xml` whose
  source changed, a re-captured physics baseline, golden data or fingerprint.

## Docs

- A change to the cell, a criterion or a format updates the SPEC section that states it.
- The tracker is `.claude/project-docs/FEATURES.md`.
- README: the triggers are in `.claude/skills/readme-writer/references/project.md` > Triggers.
