---
name: project-style-rpx
description: >-
  The code and repository conventions of Test-it-off Simulation (the RP style): Python names
  (_Cls/_Str/_Error_Cls/CONST_*, Pascal_Snake_Case functions), import typing as tp, numbered
  Description/Args/Returns docstrings, the #-comment licence header, pytest layout and test
  discipline, the structure rules (a Core.py per src domain with topic modules, mirrored tests, a
  layer order, one reader and writer per file format, no field without effect, a pinned root
  layout), the YAML configuration and installer rules, the Markdown records (SPEC, decisions, known
  issues), data figures, and this project's layout, formats, configuration and MJCF rules. Use it for
  any change in this repository - Python, a script, a configuration field, the installer, a record,
  a figure, a review - even when style is not mentioned. It runs scripts/check_style.py.
---

# project-style-rpx

The conventions of this repository. This file holds the rules that apply to any project in the RP
style; [references/project.md](references/project.md) holds this project's decisions (its domains
and layer order, its file formats, its configuration, its platforms) and
[references/mjcf.md](references/mjcf.md) the scene and what is never changed in it. Every rule says
whether something **enforces** it (✅ the checker, a test, the configuration loader) or whether it is
**documentation only** (📄). A practice followed by too little code to call a rule is marked
*Unsettled*.

**Assumes** (deliberate): a **Python 3.11** project with **pytest**; the **RP coding style** below
(names, docstrings, header) rather than PEP 8 naming or Google/NumPy docstrings; a **bash** installer
`install.sh` with a `.venv` and `requirements.txt`; **git**; YAML for configuration; the internal
records in `.claude/project-docs/`. The folders are `src/`, `test/`, `Example/` (the scripts) and
`Media/`. The project has no GUI (owner, 2026-10-25).

**Visual style.** Data figures follow [references/figures.md](references/figures.md) with the
project's choices in references/project.md (*Figure style*). The only brand asset used is the JIC
Smart Factory logo in the README header (references/project.md).

The naming and docstring style descends from rparak/RoLE (the "RP" style). Where RoLE differs (it
raises no exceptions, has no tests, writes its header as a docstring) these conventions do not
follow it.

## 0. Read the project's decisions first

`.claude/skills/project-style-rpx/references/`:

- `project.md` - the holder, domains and layer order, formats and their readers, configuration,
  installer targets, root entries and exceptions, project-only rules, the departures. Always read it.
- `mjcf.md` - the scene, the vendored robot and what is never changed; read it for anything under
  `models/` or `src/Scene/`.
- `check_style.json` - the checker's configuration (holder, licence, class, test and check roots,
  reasoned exclusions).

A rule of this file that a project decision replaces is written there, not here; a difference that
looks like a mistake is raised with the owner, never changed on your own. Run every command with
`.venv/bin/python` (`CLAUDE.md`), never a bare `python3`.

## 1. Run the checker before every commit

```bash
.venv/bin/python .claude/skills/project-style-rpx/scripts/check_style.py            # src Example test (check_roots)
.venv/bin/python .claude/skills/project-style-rpx/scripts/check_style.py <paths>    # or named files and folders
```

The checker finds the project root from the working directory upwards (or `--root DIR`) and refuses
to run without `check_style.json` (exit 2, never "clean"). It must report 0 issues. The commit that
adds a folder the checker should read also adds it to `check_roots`. The `exclude` list holds
reasoned exclusions; an entry without a reason is refused and the summary line names every
exclusion that applied. A path that does not exist, a non-`.py` file or an unreadable file is
refused (exit 2, named in the output); a directory walk skips hidden directories.

Its own tests, `scripts/test_check_style.py`, run in the project's suite (`pytest.ini` `testpaths`).
The checker and its tests are a stdlib tool in plain PEP 8 style and are not held to the rules they
check; they are kept byte-identical to their source (references/project.md > Header and checker).

## 2. What is enforced, and by what

| Rule | Enforced by |
|---|---|
| Licence header naming the project's holder (`Copyright (c) <year> <holder>`) with a `File Name:` line matching the file (a template field `{{...}}` or `<...>` is not compared) | ✅ checker |
| Class name Pascal_Snake_Case ending in `_Cls`, dataclass in `_Str`, an exception (directly or through a project class) in `_Error_Cls`; functions Pascal_Snake_Case, parameters snake_case or math symbols, module-level names `CONST_*` or a `*_Str` preset | ✅ checker (naming in pytest modules too, pytest's own names excepted) |
| Every parameter and return annotated | ✅ checker - not in pytest modules (`test_*.py`, `conftest.py`, anything under a test root), helpers included |
| A function named `main` (a script's entry point, `def main():` in assets/script_template.py) is exempt from the naming, docstring and typing checks | ✅ checker (the exemption) |
| Docstring with `Description:`, numbered `Args:` when there are arguments, `Returns:` when a value is returned | ✅ checker - not in pytest modules |
| `Initialization of the Class:` in a `_Cls` docstring | ✅ checker - except exception classes and Qt classes, directly or through a class of the same file or of the class roots |
| A Qt-dispatched camelCase method is exempt from naming **only when marked** `# << QT OVERRIDE >> #` on the line above | ✅ checker |
| Lines ≤ 150, ≤ 2 statements per `;` line, no mutable defaults, no `== True/False`, no invalid escapes, no `logging`, no relative imports, `import typing as tp`, no `from __future__ import annotations`, no dead `__name` override | ✅ checker |
| A comment above every import; `# Custom Lib.:` group with `#   ../path` lines | 📄 |
| `# << PRIVATE >> #` marker, private state as `self.__name` | 📄 |
| `*_Error_Cls` messages are full operator sentences | 📄 |
| Required configuration fields present, values valid, all problems listed at once, unknown key → WARNING; duplicate keys refused, only `true` / `false` booleans | ✅ the project's loader and its test (references/yaml.md) |
| A configuration mini-language is parsed, never evaluated | ✅ the loader's test (references/yaml.md) |
| A configuration field is read by something that can change behaviour | 📄 - a loader cannot see it |
| The root holds only the declared entries; no case-only duplicate paths | ✅ `test/Docs/test_layout.py` (references/structure.md) |
| Core.py + a topic module per second concern + Const.py for shared constants | ✅ `test/Docs/test_layout.py` (references/structure.md) |
| Committed generated data records its inputs' hashes (YAML by content) and is refused stale | ✅ the project's test (references/structure.md) |
| A known library warning is filtered by its exact message only | ✅ the project's test (references/python.md) |
| Figures: one style module, two themes, measured contrasts, PNG + SVG, generated | ✅ the project's figure tests (references/figures.md) |
| Layer order; one reader and writer per file format | 📄 (references/project.md) |
| Installer: OS detection, explicit interpreter, log format | ✅ `test/Install/test_install.py` (references/shell.md) |
| A comment is removed only when stale, false or restating the code | 📄 (references/python.md > Docstrings) |
| Test discipline: a test calls the code it names, never takes its expectation from it, is broken once, a rewrite matches its collected count, a behaviour-preserving change is proven equivalent | 📄 (references/python.md > Tests) |
| A script test runs the script in a fresh interpreter from an unrelated working directory | ✅ the project's script tests |
| No parameter or field with a single legal value | 📄 |
| design-decisions entries carry **Why**, **Alternatives** and **Revisit if** | 📄 (references/markdown.md) |

references/project.md and references/mjcf.md add the project's own rows (the fingerprint, the
formats' readers).

## 3. Python - the essentials

Full rules: [references/python.md](references/python.md).

1. Every `.py` file opens with the `#`-comment licence header. Copy [assets/header.txt](assets/header.txt):
   `Copyright (c) 2026 JIC Smart Factory`, `Author   : Roman Parak`, and `File Name:` set to the real
   path (`../Scene/Core.py` for `src/Scene/Core.py`, `../Example/Cell/run_program.py`,
   `../test/Scene/test_core.py`). Vendored licences keep their holders.
2. Names: `Pascal_Snake_Case_Cls`, `Pascal_Snake_Case_Str` dataclasses, `Pascal_Snake_Case`
   functions and methods (verb-first), `snake_case` parameters and locals (or math symbols such as
   `x`, `n`, `A_b`), `CONST_*` module constants. A parameter carrying a `*_Str` is named after it.
   Module-level private helpers and test helpers are `_Pascal_Snake_Case` (e.g.
   `Scene.Core._Apply_Look`; decided 2026-10-24).
3. `import typing as tp` and annotate everything; dataclass fields carry a comment ending in a
   `#   Unit [type]` line.
4. Docstrings: `Description:`, numbered `Args:` `(1) name [type]: ...`, `Returns:`
   `(1) parameter [type]: ...` - the RP format, not Google/NumPy/Sphinx. Units in brackets.
5. Errors: a module that can fail in a way the operator must see defines a `*_Error_Cls` and
   raises it with a full, actionable sentence.
6. Reporting: no `logging` module. Scripts print `[INFO]` / `[WARNING]` / `[ERROR]` lines. `src/`
   modules report through return values, exceptions and injected callbacks; they never print.
7. Tests: pytest, `test/<Domain>/test_<module>.py` mirroring `src/<Domain>/<Module>.py`, one test
   file per script, a `Description:` docstring per test saying what behaviour it pins.
8. No `__init__.py`. `src/` reaches `sys.path` through `test/conftest.py` in tests and, in scripts,
   through a path built from `__file__` - never a working-directory-relative path.
9. Line length: aim ≤ 120, the checker errors above 150.

## 4. Reference map

| Read | When |
|---|---|
| [references/project.md](references/project.md) | always, first |
| [references/mjcf.md](references/mjcf.md) | `models/`, `src/Scene/`, anything near the verified physics |
| [references/python.md](references/python.md) | any Python: header, names, imports, constants, docstrings, types, errors, tests, scripts |
| [references/structure.md](references/structure.md) | a new module, folder, file format, dependency or root entry; `.claude/project-docs/`; the skills layout |
| [references/yaml.md](references/yaml.md) | a configuration file or field |
| [references/shell.md](references/shell.md) | the installer |
| [references/markdown.md](references/markdown.md) | SPEC, design-decisions, KNOWN_ISSUES, FEATURES |
| [references/figures.md](references/figures.md) | a data figure: style module, themes, palette, saving, the default look |
| [assets/module_template.py](assets/module_template.py), [assets/script_template.py](assets/script_template.py), [assets/config_template.yaml](assets/config_template.yaml) | a new module, script or configuration file |

## Unsettled - not rules yet

Ask before writing new code either way, unless references/project.md settles it.

- **Import group order** among standard library, third-party and `typing`. Only the comment per
  import and the trailing `# Custom Lib.:` group are rules.
- **A shared structures package.** A `*_Str` lives with the module that defines its behaviour
  (references/structure.md); whether a shared `Parameters` package is ever needed is open.
