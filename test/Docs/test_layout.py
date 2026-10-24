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

# Subprocess (Subprocess management)
import subprocess

"""
Description:
    Initialization of constants.

    The root holds only the declared entries (style project reference > Root, SPEC §6); written out here,
    never read from the README or the filesystem.
"""
# The entries git tracks at the root. Exception with its reason: pytest.ini (the runner reads it at the root).
#   Unit [Set<string>]
CONST_ROOT_ENTRIES = {'README.md', 'LICENSE', 'CLAUDE.md', '.gitignore', '.claude', 'install.sh', 'requirements.txt', 'pytest.ini',
                      'src', 'test', 'Example', 'models', 'Data', 'Media'}
# The internal documents, in .claude/project-docs/.
#   Unit [Tuple<string>]
CONST_PROJECT_DOCS = ('SPEC.md', 'KNOWN_ISSUES.md', 'design-decisions.md', 'FEATURES.md')
# The project layer of the personal skills: the only files tracked under .claude/skills/.
#   Unit [Set<string>]
CONST_SKILL_FILES = {'feature-manager/references/project.md', 'project-optimize/references/project.md',
                     'project-style-rpx/references/project.md', 'project-style-rpx/references/check_style.json',
                     'project-style-rpx/references/mjcf.md', 'readme-writer/references/project.md'}


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


def test_no_path_differs_from_another_only_in_case(root):
    """
    Description:
        No two tracked paths differ only in letter case (macOS file systems are case-insensitive; the
        data -> Data rename went through a temporary name).
    """

    paths = _Tracked(root)
    assert len({p.lower() for p in paths}) == len(paths)
