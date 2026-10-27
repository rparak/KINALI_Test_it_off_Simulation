# readme-writer: project reference for KINALI_Test_it_off_Simulation

The README decisions of this repository. The generic skill is personal (`~/.claude/skills/`); this file overrides it where
they differ. No `SKILL.md` is tracked here.

**Status:** the full README (brief §7 step 7), written 2026-10-26 from the owner's outline (the owner gave the sections, their
content and the Authors HTML in full: that outline stood in for the bootstrap proposal); accepted by the owner 2026-10-26.
Maintenance mode from here.

## Readers

- Main reader: an engineer who wants to run the cell simulation, compare the two programs and later edit the robot program.
- Others: a maintainer (tests, baselines, what to regenerate), a reader evaluating the project (the comparison, the media).

## Sources of numbers

| What the README shows | Source | Written by | Pinned by |
|---|---|---|---|
| Cycle times 70.404 / 59.148 s, 28/28 checks | `test/Simulation/data/baseline_report_<taught\|optimized>.txt` (first and last line) | `physics_baseline.py --out` | `test/Docs/test_readme.py` |
| Throughput, TCP / joint-space / PCB arc, clearances, tilt, slip, the critical path (-6.940 / -4.316 s), the stretch tables | `Data/Generated/Reports/Comparison/Comparison.md` (sections 3-7, 9) | `Example/Report/compare_report.py` | throughput: `test/Docs/test_readme.py`; the report: `test/Example/Report/test_compare_report.py` |
| The gate's cost (+0.432 s, 58.716 -> 59.148 s) | design-decisions (F16, Cost) | the owner's F16 record | `test/Optimizer/test_core.py` (58.716 s before) |
| The collision legend's counts | `make_media.py`'s output line (`Collision_Only_Overview_Right.png: ... categories`) and the image legend | `Example/Report/make_media.py` | the image |
| Datasheet values, estimates, CAD deviations, model numbers | SPEC §4.3, §4.4, §4.7, §4.10, `Configuration/Cell.yaml` comments | the owner | the fingerprint, the baselines |
| MuJoCo 3.16.0, Python 3.11, the pins | `requirements.txt`, `install.sh` | the owner | `test/Install/test_install.py` |
| Command outputs and times (install about 20 s, run about 20 s) | a fresh clone, 2026-10-26, Apple M2 Pro | - | - |

## Triggers

| A change to | Updates | Then run |
|---|---|---|
| A program, a point, `Cell.yaml`, `Optimization.yaml` or the model | the comparison table, the findings, the stretch tables, Figures 1, 2, 6-13 | `compare_report.py --media Media`, `make_media.py`, `test/Docs/test_readme.py` |
| A physics baseline | the cycle times, 28/28 | `test/Docs/test_readme.py` |
| A command or option of an `Example/` script or `install.sh` | Quick start, Usage | the command as documented, from a fresh clone |
| An acceptance check | Verification (the 28 checks) | - |
| A KNOWN_ISSUES item (the real-cell items above all) | How the model was built > Main limitations, the first-run order | - |
| The release (version, date) | `CITATION.cff` `version`, `date-released`; the README's release link | `test/Docs/test_citation.py` |
| The interpreter or a pin | badges, Requirements and platform | - |

## Statuses

The programs: "Default program" and "Optimized program" in every text, table, figure and caption (owner, 2026-10-26); the
script options keep `taught` / `default` / `optimized`, explained once in Usage. Every number is labelled a simulation result
([K] kinematic, [P] physics).

## Figures

All generated; none edited by hand.

| Figure | File | Written by |
|---|---|---|
| 1, 8 | `Media/Cycle_Side_by_Side.gif`, `Media/Cycle_Optimized.gif` | `Example/Report/make_media.py` |
| 2, 9-13 | `Media/View_<view>.png` | `make_media.py` |
| 3 | `Media/Key_Points.png` | `make_media.py` |
| 4, 5 | `Media/Collision_Overview_Right.png`, `Media/Collision_Loader_Handover.png` | `make_media.py` |
| 6, 7 | `Media/Default_vs_Optimized_<dark\|light>.png`, `Media/Timeline_<dark\|light>.png` (the SVGs beside them) | `compare_report.py --media Media` |

The renders (GIFs, views, collision images, key points) exist in one version: they are images of the scene, legible on both
themes. The charts are a `<picture>` (dark for the dark theme, light for the light one).

**Size budgets** (deliberate, SPEC §7.12): GIF previews at most 10 MB, the PNG renders (views, collision images, key points)
at most 1.5 MB, 30 MB in all (owner, 2026-10-27), set with their reasons in `check_readme.json` (`image_budgets`,
`media_budget`); the charts keep the 400 KB default. The full MP4 videos are v1.0 release
assets (owner, 2026-10-26), each linked from the README by its asset URL (`releases/download/v1.0/<file>`) beside the release
page's link; the side by side is `Default_vs_Optimized_Side_by_Side.mp4` (2560 x 720, no resolution suffix; owner, 2026-10-26).

The summary chart carries no title inside the image; Figure 6's caption does (owner, 2026-10-26; pinned by
`test/Example/Report/test_compare_report.py`).

The header's logos and the Authors table's avatars carry no "Figure N:" caption (owner, 2026-10-24 and 2026-10-26), configured in
`check_readme.json` beside this file. The avatars' alt text is a sentence of at least 12 words (owner, 2026-10-26: only the two
alt attributes of the owner's Authors HTML lengthened).

## Rules

- Root layout: `.claude/skills/project-style-rpx/references/project.md` > Root.
- Never mentioned in the README: `.claude/project-docs/`, the skills, `CLAUDE.md`'s content, a GUI, a "Phase 2"
  (`test/Docs/test_readme.py`). The limitations are summarized in the README itself.
- The title: "Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo" (design-decisions: the name).
- The logo header at the top: JIC Smart Factory and KINALI (Test-it-off by name only, no logo), each a `<picture>` with
  `prefers-color-scheme: dark` -> `_White.svg`, else `_Black.svg`. No funding band.
- The project context in the introduction uses only the owner's sentence (SPEC §1).
- The safety alert at the top, a `> [!WARNING]` with the sentence of `KNOWN_ISSUES.md`'s opening.
- The Authors section is the owner's HTML, unchanged but for the two avatars' alt text (owner, 2026-10-26). No contact section.
- A Citation section points at `CITATION.cff` (CFF 1.2.0, at the root; owner, 2026-10-26).
- The third-party notes name the AUBO i5 model (MIT), imageio-ffmpeg and its GPL ffmpeg build (design-decisions: the media are
  encoded with Pillow and imageio-ffmpeg), the datasheets cited.

## README tests

`test/Docs/test_readme.py` (fast): the cycle times and throughputs against the physics baselines, 28/28 in both, every image
shown exists, every README medium in `Media/` is shown, the rules above. `test/Docs/test_citation.py`: `CITATION.cff` against the
CFF 1.2.0 schema and the repository. Also `check_readme.py README.md` (0 errors) and `render_readme.py` (0 leftover Markdown).

## Pointers

`CLAUDE.md` > Skills; feature-manager's project reference > Docs.
