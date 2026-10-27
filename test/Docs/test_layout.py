## =========================================================================== ##
# MIT License
# Copyright (c) 2026 JIC Smart Factory
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
## =========================================================================== ##
# Author   : Roman Parak
# File Name: ../test/Docs/test_layout.py
## =========================================================================== ##

# System (Default)
import os
# AST (Abstract syntax trees)
import ast
# Subprocess (Subprocess management)
import subprocess

"""
Description:
    Initialization of constants.

    The root holds only the declared entries (style project reference > Root, SPEC §6), and each src/ domain only its
    declared modules, its constants with their readers (style project reference > Layout; design-decisions: the module
    layout); written out here, never read from the README or the filesystem.
"""
# The entries git tracks at the root. Exceptions with their reasons: pytest.ini (the runner reads it at the root); CITATION.cff
# (GitHub reads it at the root; owner, 2026-10-26).
#   Unit [Set<string>]
CONST_ROOT_ENTRIES = {'README.md', 'LICENSE', 'CITATION.cff', 'CLAUDE.md', '.gitignore', '.claude', 'install.sh', 'requirements.txt', 'pytest.ini',
                      'src', 'test', 'Example', 'models', 'Data', 'Media', 'Configuration'}
# The internal documents, in .claude/project-docs/.
#   Unit [Tuple<string>]
CONST_PROJECT_DOCS = ('SPEC.md', 'KNOWN_ISSUES.md', 'design-decisions.md', 'FEATURES.md')
# The project layer of the personal skills: the only files tracked under .claude/skills/.
#   Unit [Set<string>]
CONST_SKILL_FILES = {'feature-manager/references/project.md', 'project-optimize/references/project.md',
                     'project-style-rpx/references/project.md', 'project-style-rpx/references/check_style.json',
                     'project-style-rpx/references/mjcf.md', 'readme-writer/references/project.md',
                     'readme-writer/references/check_readme.json', 'mujoco-robot-cell/references/project.md'}
# The modules of each src/ domain (owner, 2026-10-25): Core.py, the domain's primary module; a topic module where the domain
# has a second concern (Program_Parser.py, Check.py, Report.py, Measure.py, Media.py, Style.py); Const.py only where more than one of its
# modules uses a constant.
#   Unit [Dict<string, Set<string>>]
CONST_DOMAIN_MODULES = {'Configuration': {'Core.py', 'Const.py', 'Program_Parser.py'}, 'Scene': {'Core.py'}, 'Kinematics': {'Core.py'},
                        'Collision': {'Core.py'}, 'Render': {'Core.py'}, 'Simulation': {'Core.py', 'Const.py', 'Report.py'},
                        'Keypoints': {'Core.py'}, 'Trajectory': {'Core.py'}, 'Program': {'Core.py', 'Const.py', 'Check.py'},
                        'Planner': {'Core.py'}, 'Optimizer': {'Core.py', 'Measure.py'}, 'Report': {'Core.py', 'Media.py', 'Style.py'}}


def _Tracked(root):
    result = subprocess.run(['git', 'ls-files'], cwd=root, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    return result.stdout.splitlines()


def test_the_root_holds_only_the_declared_entries(root):
    """
    Description:
        The files git tracks at the root are the declared entries and nothing else.
    """

    assert {path.split('/', 1)[0] for path in _Tracked(root)} == CONST_ROOT_ENTRIES


def test_the_project_docs_are_tracked(root):
    """
    Description:
        The internal documents are tracked in .claude/project-docs/ (a .gitignore rule would leave them out
        of every clone).
    """

    tracked = set(_Tracked(root))
    for name in CONST_PROJECT_DOCS:
        assert '.claude/project-docs/' + name in tracked, name


def test_the_skill_folders_track_only_the_project_references(root):
    """
    Description:
        Under .claude/skills/ git tracks exactly the project references: a SKILL.md there would be ignored
        in favour of the personal skill of the same name and drift unseen.
    """

    prefix = '.claude/skills/'
    assert {p[len(prefix):] for p in _Tracked(root) if p.startswith(prefix)} == CONST_SKILL_FILES


def _Constants(tree):
    """
    Description:
        The CONST_ names a module defines at its top level.
    """

    return {n.id for s in tree.body if isinstance(s, ast.Assign) for t in s.targets for n in ast.walk(t)
            if isinstance(n, ast.Name) and n.id.startswith('CONST_')}


def _Domain_Trees(root, domain):
    """
    Description:
        The parsed modules of a src/ domain that git tracks, by file name.
    """

    prefix = 'src/%s/' % domain
    files = [p for p in _Tracked(root) if p.startswith(prefix) and p.endswith('.py')]
    return {p[len(prefix):]: ast.parse(open(os.path.join(root, p), encoding='utf-8').read()) for p in files}


def test_each_domain_holds_its_declared_modules(root):
    """
    Description:
        The Python files git tracks under src/ are the declared modules of the declared domains and nothing else, and each
        module but a Const.py has its test file test/<Domain>/test_<module>.py (a Const.py is tested through its readers).
    """

    tracked = _Tracked(root)
    found = {}
    for path in tracked:
        if path.startswith('src/') and path.endswith('.py'):
            domain, name = path.split('/')[1:3]
            found.setdefault(domain, set()).add(name)
    assert found == CONST_DOMAIN_MODULES
    missing = ['test/%s/test_%s' % (d, m.lower()) for d, names in CONST_DOMAIN_MODULES.items() for m in names
               if m != 'Const.py' and 'test/%s/test_%s' % (d, m.lower()) not in tracked]
    assert missing == []


def test_a_constant_lives_with_its_only_reader_or_in_the_domains_const(root):
    """
    Description:
        Within a domain, a constant that more than one module uses is defined in its Const.py, and a constant one module
        uses is defined in that module: no module uses a constant another non-Const module of its domain defines, every
        constant of a Const.py is used by at least two other modules of its domain, and a Const.py holds only its imports,
        its description and CONST_ assignments.
    """

    wrong = []
    for domain in CONST_DOMAIN_MODULES:
        trees = _Domain_Trees(root, domain)
        defined = {name: _Constants(tree) for name, tree in trees.items()}
        used = {name: {n.id for n in ast.walk(tree) if isinstance(n, ast.Name) and isinstance(n.ctx, ast.Load)} for name, tree in trees.items()}
        for name, constants in defined.items():
            readers = {c: sorted(m for m in trees if m != name and c in used[m]) for c in constants}
            if name == 'Const.py':
                wrong += ['%s/Const.py %s: used by %s' % (domain, c, r) for c, r in readers.items() if len(r) < 2]
                wrong += ['%s/Const.py holds a %s' % (domain, type(s).__name__) for s in trees[name].body
                          if not isinstance(s, (ast.Import, ast.ImportFrom, ast.Assign, ast.Expr))]
            else:
                wrong += ['%s/%s %s: used by %s' % (domain, name, c, r) for c, r in readers.items() if r]
    assert wrong == []


def test_no_path_differs_from_another_only_in_case(root):
    """
    Description:
        No two tracked paths differ only in letter case (macOS file systems are case-insensitive; the
        data -> Data rename went through a temporary name).
    """

    paths = _Tracked(root)
    assert len({p.lower() for p in paths}) == len(paths)
