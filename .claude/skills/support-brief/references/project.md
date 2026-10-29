# support-brief: project reference for KINALI_Test_it_off_Simulation

This project's part of the support-chat brief (`SKILL.md` holds the generic procedure, unchanged from the copy any
other project would use; owner, 2026-10-28).

## Location

`.claude/project-docs/SUPPORT_CHAT.md`, the skill's default, beside the internal docs. The README never names it
(readme-writer's project reference > Rules: no `.claude/project-docs/` in the README).

## When it is updated

The maintenance rule is in feature-manager's project reference (`.claude/skills/feature-manager/references/project.md`
> The support-chat brief), not here.

## What the brief carries from this project

- The safety sentence of feature-manager's `SKILL.md` (Physical systems), word for word, in section 1: the outputs are
  simulation results and are not validated for driving the real cell.
- The real-world impact for the survey: today the output drives nothing real; a transfer to the real AUBO i5 is planned
  (SPEC §17). The first run states which domain modules apply and why.
- Exact commands stay in `CLAUDE.md` > Commands; the brief points there.

## Length

At most 3,000 words by `wc -w .claude/project-docs/SUPPORT_CHAT.md`, in place of `SKILL.md` §4's 2,500 (owner, 2026-10-29:
2,694 at `e80320e`, with room for the next features). When a REFRESH would pass 2,750, tighten the wording first; raising the
limit is the owner's decision. Pinned by `test/Docs/test_support_chat.py`, which reads the limit from this section.

## Checks after writing

The per-commit tier (`CLAUDE.md` > Branches and checks). The light test is proposed at the first run.
