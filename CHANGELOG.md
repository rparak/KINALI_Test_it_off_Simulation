# Changelog

Every output of this project is a simulation result; nothing is run on, or validated for driving, the real cell.

## [1.1] - 2026-11-25

One command for every action, the fork tool, the project video and its media, and the public documentation. The numbers
are in [docs/SPEC.md](docs/SPEC.md), the open limitations in [docs/KNOWN_ISSUES.md](docs/KNOWN_ISSUES.md). The cycle times
and the 28 acceptance checks are unchanged: 70.404 s and 59.148 s, 28 / 28 for both programs.

### Added

- **One command, `Example/cell.py`**, with the subcommands `check` (every key point and the programs), `point` (one point's
  check row and its corrected `Joints` and `Pose` lines), `build`, `optimize`, `run` (both programs against their physics
  baselines), `show` (the viewer, `--physics` for a physics run) and `all` (check, build, optimize when stale, run);
  `all --recapture` re-captures the kinematic golden and the physics baselines after an intended change.
- **The README results block** written from the physics baselines by `cell.py all --recapture`.
- **The fork tool `T_FORK`**: the wrist-down points defined on the fork slot's centre; their joints, the trajectories and
  the physics reports are unchanged.
- **The project video** (YouTube) and the README's GIFs, diagrams and stills.
- **`docs/`**: the specification and the known issues; **`CHANGELOG.md`**.

### Changed

- The README is ordered along the user's path: quick start, results, how it works, changing the key points.
- The per-step scripts of v1.0 (`Example/<Domain>/...`) became `cell.py` subcommands.
- MuJoCo's known attach warning passes through an exact-match filter at every compile; any other warning still shows.
- `CITATION.cff`: both authors with their affiliation, JIC Smart Factory.

### Removed

- The comparison and media generators of v1.0 (`compare_report.py`, `make_media.py`); the v1.0 figures and the release
  videos stay as published.
- Six dependencies nothing uses any more: trimesh, rtree, coacd, matplotlib, pillow, imageio-ffmpeg.
- The two v1.0 cycle GIFs of the README (the video replaces them).
- The development tooling: the Claude Code skills and the internal records.

## [1.0] - 2026-10-26

The first release.

### Added

- **The cell in MuJoCo**: the AUBO i5 with the OnRobot VGC10 vacuum gripper and the fork, the MG06 test fixture, the two-axis
  loader with the SCHUNK EGI gripper, the input and output crates; the PCB in solder-mask green; the studio look.
- **The cell program** in three hand-edited YAML files (`Constants.yaml`, `Simulation.yaml`, `Program.yaml`), read by a
  strict line parser: 25 named robot points with IK resolution and the point check (IK, family, clearance).
- **Two programs for the same process**: the *Default program*, taught as an operator would teach it (sequential, the robot
  and the loader never moving at the same time), and the *Optimized program* (RRT-Connect, shortcut, cubic spline and TOPP
  for the free transfers; the loader in parallel under ten interlocks and a gating rule that forbids any parallel part
  resting on timing alone).
- **Checks at two levels**: every program built into a 4 ms trajectory with an exact per-sample clearance check, then run
  with full physics against 28 acceptance checks.
- **Regression**: the physics baselines of both programs (drift tolerances, and envelopes over 10 perturbed runs for the
  fork's contact), the kinematic golden of the Default program, the compiled model's fingerprint.
- **A measured comparison** of the two programs, with figures and videos (the release's assets).
- `install.sh` with Python 3.11 and pinned requirements; the README; `CITATION.cff`.
