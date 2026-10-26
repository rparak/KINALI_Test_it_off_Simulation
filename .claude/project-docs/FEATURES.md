# Features

The feature tracker (the owner asked for one; a recorded departure in the style project reference): what is open, and one
line per done feature with its commits. Each feature's full record - requirements, design, measurements, sub-items - is
in git: `git show e7739f4:.claude/project-docs/FEATURES.md`. The sub-items that code and configuration cite (F3.4, F3.5,
F6, F8, F8.5, F9, F10, F11) are there. The specification is [SPEC.md](SPEC.md).

## Open

- [~] Step 7: the full README (owner, 2026-10-26: the outline given in full; written, awaiting the owner's acceptance), its tests
      `test/Docs/test_readme.py`, the timeline in `Media/`.
- [ ] Candidate experiment (owner, 2026-10-24: after step 6, now possible): start `J_INSERT` during the handover's last MoveL
      (at most -1.5 s). Risk: the loader lifts the PCB towards the cups while the robot is still leaving; it needs a
      time-dependent check (I2 relaxed, the LOCKED handover split) and the physics proof (design-decisions: the optimized
      take waits at P_LOADER_WAIT).
- [~] The PCB mesh split into board and components (F2's optional follow-up; KNOWN_ISSUES.md).

## Done

| F | Feature | Commits |
|---|---|---|
| F1 | The brief's `kinali_sim` package layout; superseded by F3 | `42a6121` |
| F2 | The PCB in solder-mask green (`pcb_vis` rgba 0.05 0.42 0.18 1, the fingerprint's one difference) | `8bb88ae` |
| F3 | The RPX migration: `src/<Domain>/Core.py`, `Example/`, mirrored tests, `install.sh` with pinned Python 3.11, `logger/`; outputs byte-identical | `3a4d604`…`f754e2d` |
| F4 | The root clean-up: the legacy planner and `docs/` removed, `Data/` and `Example/` in subfolders, `Data/Generated/<Kind>/` | `bf210a3`…`f93fc62` |
| F5 | The black and white SVG logos (JIC Smart Factory, KINALI), written by a script; the project name; the licence holder | `fa7376d`…`2e742d8` |
| F6 | Step 3: `Cell.yaml`, the keypoints, IK resolution, the cell posing, the check table, the viewer stepping | `0ccce43`…`e90c5a7` |
| F7 | The studio look: grey sky with fog, a dark 1 m grid floor, a key spot and a fill light | `0f46736`, `84e8011` |
| F8 | Step 4a: the taught program, the 4 ms trajectory format, the motion laws, the per-sample exact clearance check, the build and playback scripts | `563de49`…`e5ec42c` |
| F9 | Step 4b: the optimized program - own RRT-Connect, shortcut, spline and TOPP, the loader in parallel under I1-I8 (SPEC §7.11), the committed `Program_Optimized.yaml` | `2fdc71a`…`0ed2e58` |
| F10 | The cell program in three YAML files (Constants, Simulation, Program) with a strict line parser; proved against a golden build | `e77c004`…`92d3d03` |
| F11 | Step 5: the physics run of either program's trajectory (the drive, the lost-PCB stop, the vacuum switch, `run_program.py`, `show_physics.py`); accepted by the owner 2026-10-25 | `a6f1b5c`…`4a80db7` |
| F12 | Step 6: the acceptance items 12-17, the programs' physics baselines with drift tolerances, the legacy cycle run retired (its last commit `7a8560c`); accepted by the owner 2026-10-25 | `f6c5e47`…`a8080cc` |
| F13 | The loader gripper and the place into MG06: synchronized 70 N jaws, 0.6 s ramps, `L_PLACE` / `L_FETCH`, items 18-25, the fork envelope, the regenerated baselines; accepted by the owner 2026-10-25 | `ea41e5c`…`ccb78b8` |
| F14 | The comparison of the taught and the optimized program: `src/Report/`, `compare_report.py`, `make_media.py`, the timed-wait finding; accepted by the owner 2026-10-25 | `75ea8bc`…`bd09244` |
| F15 | The optimized take gated by `Loader.WaitDone(J_RETURN_Y)` (`J_RETURN` split), I9 and the gating rule; the F14 follow-ups (the MG06 swept zone, the named views and media, the figure style, the folders); accepted by the owner 2026-10-25 | `3d4418c`…`05556c4` |
| F16 | The optimized insert gated by the robot's order: To_MG06's leg to the exit point `P_LOADER_WAIT` before `Loader.Start(J_INSERT)` (`Optimization.yaml Exit_Points`), I10, the gating rule symmetric, the robot speed-override table, the shared zones split into the jaw zone and the MG06 corridor; accepted by the owner 2026-10-26 | `3eed5ae`…`2d7cccd` |
