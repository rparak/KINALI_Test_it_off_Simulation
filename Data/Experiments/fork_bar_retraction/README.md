# The fork-bar retraction experiment (F11 option 2, not adopted)

What it does: the recorded closing path `PATH_FORK_CLOSE` presses the fork up to 0.54 mm into the MG06 handle bar. The
experiment moves each row of the path off the bar by a retraction `Bar_Retraction` [m] along every nearby fork face's normal,
fading out where the recorded gap reaches 0.2 mm, and runs the physics with it. It was run on 2026-10-25 and not adopted
(design-decisions: "The recorded closing path stays, and the fork is bounded on its net force and hinge load").

## Files

| File | What |
|---|---|
| `bar_retraction.patch` | `git diff 92d1943 cb69907`: `Bar_Retraction` in `Configuration/Constants.yaml` `Paths`, its parsing in `src/Configuration/Program_Parser.py`, the retraction in `src/Program/Core.py`, and the sweep script |
| `fork_retraction_sweep.py` | the sweep script on its own (the patch adds it as `Example/Simulation/fork_retraction_sweep.py`) |

Base commit: `92d1943` (the state the experiment ran on). The experiment's own commit was `cb69907` (not on main; this
folder replaces it).

## Result (2026-10-25, 14 runs per retraction: start offsets -5 / -1 / 0 / +1 / +5 µm, waits 0 / 4 ms / 0.1 s, both programs)

| Retraction | Overlap left | Latch (handle / lid) | Contact outcomes | Net force | Hinge load | Acceptance |
|---|---|---|---|---|---|---|
| 0 (as recorded) | 0.539 mm | 28.65 / -0.48 deg | 2 | 229.0-247.3 N | 9.38-11.45 N m | 14/14 |
| 0.55 mm | none | 28.53 / -0.49 deg | 1 | 186.8 N | 8.23 N m | 14/14 |
| 0.60 mm | none | 28.51 / -0.49 deg | 1 | 201.2 N | 9.23 N m | 14/14 |
| 0.65 mm | none | 25.51 / -0.64 deg | 1 | 406.7 N | 9.14 N m | 13/14 (no latch) |
| 0.70 mm | none | 25.51 / -0.64 deg | 1 | 406.7 N | 8.66 N m | 13/14 (no latch) |

Only 0.55-0.60 mm latch MG06 and pass; the owner adopts a retraction only when 0.55-0.65 mm all pass. Simulation results
only: not validated for the real cell.

## How to re-apply

```bash
# example: in a worktree at the base commit, from the repository root
git worktree add --detach ../fork_retraction 92d1943
cd ../fork_retraction
git apply <repository>/Data/Experiments/fork_bar_retraction/bar_retraction.patch
<repository>/.venv/bin/python Example/Simulation/fork_retraction_sweep.py 0.55 taught 0 0
```

The script's arguments: the retraction [mm], `taught` or `optimized`, the PCB's start offset along x [µm], a settle wait before
the closing path [s]; it prints one line per run (re-run 2026-10-25: 0.55 mm, taught, 0, 0 -> net 186.8 N, hinge 8.23 N m).

What the patch changes: `Bar_Retraction` [m] in `Constants.yaml` `Paths` (range 0-5 mm, 0: as recorded) retracts
`PATH_FORK_CLOSE` (`PATH_FORK_OPEN` stays 0), and `P_MG06_CLOSE_END` follows the retracted end. The wait goes after
`MoveJ(P_MG06_FORK_START)` in the taught program and after `Loader.WaitDone(J_PARK)` in the optimized one. The table's runs:
the retractions 0 / 0.55 / 0.60 / 0.65 / 0.70 mm x the offsets -5 / -1 / 0 / +1 / +5 µm (no wait) and the waits 0.004 / 0.1 s
(offset 0), both programs; MuJoCo 3.16.0, the environment of `test/Simulation/data/baseline_environment.json`.
