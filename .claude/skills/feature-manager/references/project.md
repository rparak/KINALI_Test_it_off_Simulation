# feature-manager: project reference for KINALI_Test_it_off_Simulation

This project's part of the change procedure (`SKILL.md` holds the procedure, the safety sentence and the commit rules).
Style rules come from project-style-rpx (`.claude/skills/project-style-rpx/`, its `references/project.md` and `mjcf.md`).

## What the output is

Test-it-off Simulation, a MuJoCo simulation of KINALI's Test-it-off cell (SPEC §1): the compiled cell model, the cell
program (`Constants.yaml`, `Simulation.yaml`, `Program.yaml`) and the optimized program, their trajectory files and check
reports, and the programs' physics runs with their reports and acceptance verdicts. The owner and whoever writes the robot program act on it.

## Always "large" (ask before building)

- The verified physics (SPEC §3, mjcf.md): timestep, solver options, contact parameters, friction,
  masses, geometry, actuator gains and ranges, vacuum, bellows, the CAD deviations.
- Formats: `models/scene.xml`, `Data/PCB/pcb_meta.json`, the trajectory file and
  the cell program's three files and their grammar (SPEC §8, F10), the report's lines.
- The acceptance criteria (SPEC §10.1) and the 3 mm keypoint / segment clearance.
- `requirements.txt` pins and the interpreter.

## Real data for reproducing a premise

The two programs and their physics runs (`Example/Cell/run_program.py`; their baselines and the kinematic golden in
`test/<Domain>/data/`; the legacy planner and the legacy cycle run at their restore points, `CLAUDE.md` > Branches and
checks), the fingerprint, the scene files. Walk the chain: the
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

**Structure**

- [ ] Does this need a new script, a `cell.py` subcommand, or nothing? (the no-clutter rule: style project reference >
      Layout and layers > No clutter)

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

**Simulation vs reality**

- [ ] Not modelled: cables, real vacuum dynamics beyond the adhesion force, the real loader controller,
      tolerances of the real cell. A change that relies on any of them says so.
- [ ] The baseline is per environment (`KNOWN_ISSUES.md`).

## Budgets

The 3 mm planning clearance (6 mm geom margin in the planning model, SPEC §7.4); the acceptance bounds
of SPEC §10.1. Before and after numbers for any change to physics, kinematics or checking: the full
report and the fingerprint.

## Visual changes

Open the cell in the viewer (`Example/cell.py show`, `Example/cell.py point <name> --show`) and look at what changed.

## Accepting an intended change

The pins an intended change moves are re-captured with `Example/cell.py all --recapture` (SPEC §10.2 > Re-capture after an
intended change): the kinematic golden, both physics baselines and the README results block. It prints what differs and
refuses when an acceptance check fails. The model fingerprint is not part of it (`Example/Maintenance/model_fingerprint.py
--out`). The commits follow `SKILL.md` §8: the change first, its pins failing in the expected ways named in the message; then
the re-captured files on their own, straight after it and in the same push, listing what changed and why. Then, when the
brief states a moved number, `/support-brief` REFRESH (below).

## The support-chat brief

`.claude/project-docs/SUPPORT_CHAT.md` is the brief the owner pastes into a new Claude chat that acts as the project's
senior technical support (owner, 2026-10-28; created by the support-brief skill's first run). Every accepted change that
alters anything stated in it updates it in the same commit, with the date and the commit in its section 9 (the commit it
was checked against, the support-brief skill > section 2). How: the support-brief skill, REFRESH.
