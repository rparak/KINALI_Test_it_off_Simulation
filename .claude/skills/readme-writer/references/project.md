# readme-writer: project reference for KINALI_Test_it_off_Simulation

The README decisions of this repository. The generic skill is personal (`~/.claude/skills/`); this
file overrides it where they differ. No `SKILL.md` is tracked here.

**Status (2026-10-24):** the README is a stub (install, the cycle run, the export, the viewer). The full
README is brief §7 step 7, written in this skill's bootstrap procedure (survey, proposal, approval);
this file then gains its readers, figures and tests.

## Readers

- Main reader: an engineer who wants to run the cell simulation and later edit the robot program.
- Others: a maintainer (tests, layout, baselines).

## Sources of numbers

| What the README shows | Source | Written by | Pinned by |
|---|---|---|---|
| (stub) MuJoCo 3.16.0, Python 3.11 | `requirements.txt`, `install.sh` | the owner | `test/Install/test_install.py` |

## Triggers

| A change to | Updates | Then run |
|---|---|---|
| A command or option of an `Example/` script or `install.sh` | the commands in the README | the command as documented |
| The baseline report | any number the README quotes from it | - |
| The interpreter or a pin | install text, badges | - |

## Statuses

None yet.

## Figures

None yet. `Data/Generated/Images/` holds run outputs (proof images), not README figures. The README's images are
the logos in `Media/`, written by `Example/Media/make_logo_variants.py` (light: `_Black.svg`, dark: `_White.svg`).

## Rules

- Root layout: `.claude/skills/project-style-rpx/references/project.md` > Root.
- Never mentioned in the README: `.claude/project-docs/`, the skills, `CLAUDE.md`'s content.
- The title: "Test-it-off Simulation – Automated PCBA Testing Cell in MuJoCo" (design-decisions: the name).
- The logo header at the top: JIC Smart Factory and KINALI (Test-it-off by name only, no logo), each a
  `<picture>` with `prefers-color-scheme: dark` -> `_White.svg`, else `_Black.svg`. No funding band.
- The project context in the introduction uses only the owner's sentence (SPEC §1); nothing about funding,
  partners or claims beyond it.
- The safety alert at the top, a `> [!WARNING]` with the sentence of `KNOWN_ISSUES.md`'s opening
  (outputs are simulation results, not validated for the real cell; the validation procedure, reduced
  speed, the controller's safety limits).

The header's logos are a brand mark, not figures: they carry no "Figure N:" caption (owner, 2026-10-24). The
exception is configured in `check_readme.json` beside this file (`uncaptioned_figures`: each logo's light file and the
reason); the checker reports them as INFO and exits 0. Every other figure added later is captioned and numbered.

## README tests

None yet (step 7).

## Pointers

`CLAUDE.md` > Skills; feature-manager's project reference > Docs.
