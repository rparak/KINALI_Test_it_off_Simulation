# readme-writer: project reference for KINALI_Test_it_off_Simulation

The README decisions of this repository (`SKILL.md` holds the procedure and the commands).

**Status:** the full README (brief §7 step 7), written 2026-10-26 from the owner's outline (the owner gave the sections, their
content and the Authors HTML in full); accepted by the owner 2026-10-26. Restructured 2026-10-29 (F20) from the owner's
outline (Rules > The order is the user's path). Rewritten by the owner on GitHub 2026-11-03 (`a5d3161`..`06594f1`) and kept
as it is: that structure is now the rule (Rules > The outline).

## Readers

- Main reader: an engineer who wants to run the cell simulation, change its key points and programs, and see whether the
  result still passes (owner, 2026-10-29).
- Others: a maintainer (tests, baselines, what to regenerate), a reader evaluating the project (the comparison, the media).

## Sources of numbers

| What the README shows | Source | Written by | Pinned by |
|---|---|---|---|
| The results block (between `<!-- results:start -->` and `<!-- results:end -->`): cycle times 70.404 / 59.148 s, throughputs, changes, 28/28 | `test/Simulation/data/baseline_report_<taught\|optimized>.txt` (first and last line) | `Example/Cell/results.py` (`Readme_Block`), through `cell.py all --recapture` only; nothing else of the README is written by code | `test/Docs/test_readme.py` |
| The v1.0 comparison: TCP / joint-space / PCB arc, clearances, tilt, slip, the critical path (-6.940 / -4.316 s), the stretch result (29.68 mm) | the v1.0 comparison report, `Comparison.md` (sections 3-7, 9), at v1.0 (`d8e3e2a`) | `Example/Report/compare_report.py`, in git at `c43f079` (removed in F20) | frozen: the README labels them v1.0 |
| The gate's cost (+0.432 s, 58.716 -> 59.148 s) | design-decisions (F16, Cost) | the owner's F16 record | `test/Optimizer/test_core.py` (58.716 s before) |
| The collision legend's counts | the v1.0 media run's output line (`Collision_Only_Overview_Right.png: ... categories`) and the image legend | `make_media.py`, in git at `c43f079` | the image |
| Datasheet values, estimates, CAD deviations, model numbers | SPEC §4.3, §4.4, §4.7, §4.10, `Configuration/Cell.yaml` comments | the owner | the fingerprint, the baselines |
| MuJoCo 3.16.0, Python 3.11, the pins | `requirements.txt`, `install.sh` | the owner | `test/Install/test_install.py` |
| Command outputs and times (`cell.py all` about 45 s and its last lines) | `cell.py all`, 2026-10-29, Apple M2 Pro (the fresh-clone check of F20) | - | - |
| Usage's times (build 11 s, optimize 70 s, run 21 s, all 45 s, all --recapture 2.5 min) | `CLAUDE.md` > Commands, the one source (Apple M2 Pro) | the owner | - |

## Triggers

| A change to | Updates | Then run |
|---|---|---|
| A program, a point, `Cell.yaml`, `Optimization.yaml` or the model | nothing generated in the README but the cycle times and 28/28 (below); the v1.0 numbers and media stay, labelled v1.0 | `test/Docs/test_readme.py` |
| A physics baseline | the results block: `cell.py all --recapture` regenerates it with the baselines | `test/Docs/test_readme.py` |
| A `cell.py` subcommand or option, or `install.sh` | Quick start, Change the key points, Usage; the architecture figure by hand when a subcommand or step changes (Figures) | the command as documented, from a fresh clone; `test/Docs/test_readme.py` |
| An acceptance check | the count 28 where the README states it (About, Results, Architecture, Usage, Change the key points) | `test/Docs/test_readme.py` |
| A KNOWN_ISSUES item | the safety alert when the opening sentence changes; About when a CAD deviation changes | `test/Docs/test_readme.py` |
| The release (version, date) | `CITATION.cff` `version`, `date-released`; the README's release link | `test/Docs/test_citation.py` |
| The interpreter or a pin | badges, Requirements and platform | - |
| A shipped skill added, removed or renamed, or a checker's command | Claude Code skills, Repository layout | `test/Docs/test_readme.py`, the commands as documented |

## Statuses

The programs: "Default program" and "Optimized program" in every text, table, figure and caption (owner, 2026-10-26); the
script options keep `taught` / `default` / `optimized`, explained once in Usage. Every number is labelled a simulation result
([K] kinematic, [P] physics).

## Figures

All generated at v1.0 (`d8e3e2a`) and committed; none edited by hand. Their generators are in git at `c43f079` (removed in
F20; design-decisions: one user entry point); a new generator needs the owner's approval. The one exception is Figure 6,
below.

| Figure | File | Written by (at v1.0) |
|---|---|---|
| 1, 9 | `Media/Cycle_Side_by_Side.gif`, `Media/Cycle_Optimized.gif` | `Example/Report/make_media.py` |
| 2, 10-14 | `Media/View_<view>.png` | `make_media.py` |
| 3 | `Media/Key_Points.png` | `make_media.py` |
| 4, 5 | `Media/Collision_Overview_Right.png`, `Media/Collision_Loader_Handover.png` | `make_media.py` |
| 6 | `Media/Architecture_<light\|dark>.svg` | by hand (owner, 2026-11-03), see below |
| 7, 8 | `Media/Default_vs_Optimized_<dark\|light>.png`, `Media/Timeline_<dark\|light>.png` (the SVGs beside them) | `compare_report.py --media Media` |

The renders (GIFs, views, collision images, key points) exist in one version: they are images of the scene, legible on both
themes. The charts are a `<picture>` (dark for the dark theme, light for the light one).

**Size budgets** (deliberate, SPEC §7.12): GIF previews at most 10 MB, the PNG renders (views, collision images, key points)
at most 1.5 MB, 30 MB in all (owner, 2026-10-27), set with their reasons in `check_readme.json` (`image_budgets`,
`media_budget`); the charts keep the 400 KB default. The full MP4 videos are v1.0 release
assets (owner, 2026-10-26), each linked from the README by its asset URL (`releases/download/v1.0/<file>`) beside the release
page's link; the side by side is `Default_vs_Optimized_Side_by_Side.mp4` (2560 x 720, no resolution suffix; owner, 2026-10-26).

The summary chart carries no title inside the image; Figure 7's caption does (owner, 2026-10-26).

**The architecture figure** (Figure 6, at the top of Usage; owner, 2026-11-03) is a hand-made SVG committed as is, with no
generator in the repository (the no-clutter rule): one row of six steps (Configure, Check, Build, Optimize, Run, Compare)
with their commands, the bracket `cell.py all runs 02 -> 05`, and the Show block linked to Build and Run. It carries no
number from the results or the configuration. Monochrome and flat, the owner's light and dark palettes, fonts by name with
fallbacks only, a `<picture>` like the header logos. It is updated by hand when the `cell.py` subcommands or the steps
change, rendered to PNG in both variants and looked at, then re-pinned (`test/Docs/test_readme.py`, `CONST_ARCHITECTURE`).

The header's logos and the Authors table's avatars carry no "Figure N:" caption (owner, 2026-10-24 and 2026-10-26), configured in
`check_readme.json` beside this file. The avatars' alt text is a sentence of at least 12 words (owner, 2026-10-26: only the two
alt attributes of the owner's Authors HTML lengthened).

## Rules

- **The outline** (owner, 2026-11-03: the README as rewritten on GitHub, kept as it is; Usage made visible the same
  day). Contents, then the user's path (owner, 2026-10-29, F20), visible: Quick start (`./install.sh` ->
  `cell.py all` -> `cell.py show`, with the last lines of `all`); Results (the results block, then the sentence that the
  media show the v1.0 programs, also in Figure 1's caption); Change the key points (edit -> `cell.py point <P>` ->
  `cell.py check` -> `cell.py all`, the table *When this fails | It means | Do*, and when to use `--recapture`); Files
  (*You edit | Generated by `cell.py` | Re-captured by `cell.py all --recapture` | Never edit by hand*). Then, visible: About
  (the owner's text, edited only lightly, owner 2026-11-03: the project context of SPEC §1 word for word, the cell, the two
  programs, the checks, the v1.0 comparison, then From CAD to MuJoCo, Collision geometry, Documented CAD deviations, Physics
  and dynamics from the datasheets, the solver); What the cell does (the six steps, two figures). Collapsed under their
  headings: Collision structure, Architecture. Usage is visible: every
  subcommand with one real example under the architecture figure (Figures), three workflow lists that link to Quick start and Change the key points instead of
  repeating them, the viewer keys with the Mac keyboard, the exit codes and outputs. The v1.0 comparison is visible (the
  table, the key findings, the charts, the GIF, the videos), with The stations close up collapsed. Repository layout and
  Requirements and platform collapsed; Claude Code skills as below; Citation, License and Authors visible. No
  Verification or Main limitations section (the limitations live in `KNOWN_ISSUES.md`; the safety alert stays). A new user
  action adds a row or a step in the user's path or in Usage, not a new section.
- **The visible-line budget** is 360 (owner, 2026-11-03: 349 measured with the owner's About and the visible Usage, plus a
  small margin), passed as `check_readme.py README.md --max-visible 360` (`CLAUDE.md` > Commands); a README that outgrows it is
  shortened or the budget is raised by the owner, never ignored.
- Root layout: `.claude/skills/project-style-rpx/references/project.md` > Root.
- Never mentioned in the README: `.claude/project-docs/`, `CLAUDE.md`'s content, a GUI, a "Phase 2"
  (`test/Docs/test_readme.py`).
- The README names the shipped skills (owner, 2026-10-28): the section *Claude Code skills* after Repository layout, one
  visible sentence, the table of the shipped skills with their tools and the usage collapsed; `.claude/skills/` is the only
  `.claude` path it names; the skills' licence in License (`test/Docs/test_readme.py`).
- The title: "Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo" (design-decisions: the name).
- The logo header at the top: JIC Smart Factory and KINALI (Test-it-off by name only, no logo), each a `<picture>` with
  `prefers-color-scheme: dark` -> `_White.svg`, else `_Black.svg`. No funding band.
- The project context in the introduction uses only the owner's sentence (SPEC §1).
- The safety alert at the top, a `> [!WARNING]` with the sentence of `KNOWN_ISSUES.md`'s opening.
- The Authors section is the owner's HTML, unchanged but for the two avatars' alt text (owner, 2026-10-26). No contact section.
- A Citation section points at `CITATION.cff` (CFF 1.2.0, at the root; owner, 2026-10-26).
- The third-party notes name the AUBO i5 model (MIT) and the datasheets cited (imageio-ffmpeg and its GPL ffmpeg build left
  with the media generator in F20).

## README tests

`test/Docs/test_readme.py` (fast): the cycle times and throughputs against the physics baselines, 28/28 in both, every image
shown exists, every README medium in `Media/` is shown, the logo and architecture figure bytes pinned, the rules above.
`test/Docs/test_citation.py`: `CITATION.cff` against the CFF 1.2.0 schema and the repository. Also
`check_readme.py README.md --max-visible 360` (0 errors, 0 warnings) and `render_readme.py` (0 leftover Markdown).

## Pointers

`CLAUDE.md` > Skills; feature-manager's project reference > Docs.
