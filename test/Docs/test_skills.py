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
# File Name: ../test/Docs/test_skills.py
## =========================================================================== ##

# System (Default)
import os
# Regular expressions (Regular expression operations)
import re
# Subprocess (Subprocess management)
import subprocess
# YAML (The skills' frontmatter)
import yaml

"""
Description:
    Initialization of constants.

    The skills the repository ships are self-contained (owner, 2026-10-28; design-decisions: the skills in the
    repository): each SKILL.md names its own folder, every relative link resolves inside the repository, and no file
    points into a personal install or at the bootstrap files left out.
"""
# The shipped skills, each a folder .claude/skills/<name>/ with a SKILL.md.
#   Unit [Tuple<string>]
CONST_SKILLS = ('project-style-rpx', 'feature-manager', 'project-optimize', 'readme-writer', 'support-brief')
# A description longer than this is cut by Claude Code's skill list.
#   Unit [int]
CONST_DESCRIPTION_MAX = 1024
# Never in a skill file: a path to a file in a personal install, the bootstrap files left out.
#   Unit [List<string>]
CONST_FORBIDDEN = [r'~/\.claude/skills/[\w-]+/\w', r'bootstrap\.md', r'project_reference_template', r'README_template',
                   r'check_style_template']
# The generic skill: its SKILL.md is copied into another project unchanged (owner, 2026-10-28).
#   Unit [string]
CONST_GENERIC_SKILL = 'support-brief'
# Never in the generic SKILL.md: this project's names (its decisions are in its references/project.md).
#   Unit [List<string>]
CONST_PROJECT_TERMS = [r'Test-it-off', r'KINALI', r'MuJoCo', r'AUBO', r'MG06', r'\bJIC\b', r'Roman Parak', r'SPEC §']
# The only .claude/ paths the generic SKILL.md names: the folder and the brief's default location.
#   Unit [Set<string>]
CONST_GENERIC_PATHS = {'.claude/', '.claude/project-docs/SUPPORT_CHAT.md'}


def _Skill_Files(root):
    result = subprocess.run(['git', 'ls-files', '.claude/skills'], cwd=root, capture_output=True, text=True, timeout=60)
    assert result.returncode == 0, result.stderr
    return [p for p in result.stdout.splitlines() if p.endswith(('.md', '.json', '.py', '.txt', '.yaml'))]


def test_each_skill_names_its_folder(root):
    """
    Description:
        Each shipped SKILL.md opens with a frontmatter whose name is its folder and whose description is set, short
        enough for the skill list and free of angle brackets; Claude Code finds a skill by it.
    """

    for name in CONST_SKILLS:
        with open(os.path.join(root, '.claude', 'skills', name, 'SKILL.md'), encoding='utf-8') as handle:
            text = handle.read()
        assert text.startswith('---\n'), name
        meta = yaml.safe_load(text.split('---\n')[1])
        assert meta['name'] == name
        assert 0 < len(meta['description']) <= CONST_DESCRIPTION_MAX, name
        assert not re.search(r'[<>]', meta['description']), name


def test_every_relative_link_in_the_skills_resolves(root):
    """
    Description:
        Every relative Markdown link in a skill's files points at a file that exists, so no link leads to a file left
        out of the repository. Code (a fenced block, an inline span) holds examples, not links.
    """

    for path in _Skill_Files(root):
        if not path.endswith('.md'):
            continue
        with open(os.path.join(root, path), encoding='utf-8') as handle:
            text = re.sub(r'`[^`\n]*`', '', re.sub(r'```.*?```', '', handle.read(), flags=re.S))
        for target in re.findall(r'\]\(([^)\s#]+)', text):
            if re.match(r'[a-z]+:', target):
                continue
            assert os.path.exists(os.path.join(root, os.path.dirname(path), target)), (path, target)


def test_no_skill_file_points_outside_the_repository(root):
    """
    Description:
        No skill file names a file in a personal install (~/.claude/skills/<skill>/...) or one of the bootstrap files
        the repository does not ship. The scripts are kept byte-identical to their source and are not read (their
        accepted upstream text: the style project reference > Header and checker).
    """

    for path in [p for p in _Skill_Files(root) if '/scripts/' not in p]:
        with open(os.path.join(root, path), encoding='utf-8') as handle:
            text = handle.read()
        for pattern in CONST_FORBIDDEN:
            assert not re.search(pattern, text), (path, pattern)


def test_the_support_brief_skill_is_generic(root):
    """
    Description:
        The support-brief SKILL.md names no project (its decisions are in its references/project.md) and no .claude/
        path but the brief's default location, so it can be copied into another project unchanged.
    """

    with open(os.path.join(root, '.claude', 'skills', CONST_GENERIC_SKILL, 'SKILL.md'), encoding='utf-8') as handle:
        text = handle.read()
    for pattern in CONST_PROJECT_TERMS:
        assert not re.search(pattern, text, flags=re.I), pattern
    assert set(re.findall(r'\.claude/[\w./-]*', text)) <= CONST_GENERIC_PATHS


def test_the_brief_maintenance_rule_is_in_feature_manager_only(root):
    """
    Description:
        The rule that an accepted change updates SUPPORT_CHAT.md in the same commit is written once, in feature-manager's
        project reference, which points to support-brief for how; the support-brief SKILL.md refers to the rule and does
        not restate it.
    """

    with open(os.path.join(root, '.claude', 'skills', 'feature-manager', 'references', 'project.md'), encoding='utf-8') as handle:
        rule = handle.read().split('## The support-chat brief', 1)[1].split('\n## ', 1)[0]
    assert 'SUPPORT_CHAT.md' in rule and 'same commit' in rule and 'support-brief' in rule
    with open(os.path.join(root, '.claude', 'skills', CONST_GENERIC_SKILL, 'SKILL.md'), encoding='utf-8') as handle:
        assert 'same commit' not in handle.read()
