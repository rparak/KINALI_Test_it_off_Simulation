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
# File Name: ../test/Docs/test_support_chat.py
## =========================================================================== ##

# System (Default)
import os
# Datetime (Basic date and time types)
import datetime
# Math (Mathematical functions)
import math
# Regular expressions (Regular expression operations)
import re
# Subprocess (Subprocess management)
import subprocess
# Importlib (The implementation of import)
import importlib.util
# PyTest (Framework for testing) [pip3 install pytest]
import pytest

"""
Description:
    The support-chat brief, .claude/project-docs/SUPPORT_CHAT.md (the support-brief skill; owner, 2026-10-28): its sections
    0-9 in order, the commit and date it was checked against (sections 0 and 9 the same, an ancestor of HEAD), the
    feature-manager safety sentence word for word, and its key numbers against their sources: the physics baselines, Cell.yaml
    and Optimization.yaml.
"""
# The brief, the safety sentence's source and the baseline tool, relative to the repository root.
#   Unit [string]
CONST_BRIEF = os.path.join('.claude', 'project-docs', 'SUPPORT_CHAT.md')
CONST_SAFETY_SOURCE = os.path.join('.claude', 'skills', 'feature-manager', 'SKILL.md')
CONST_TOOL = os.path.join('Example', 'Simulation', 'physics_baseline.py')
# The brief's sections, in order (the support-brief skill > section 2).
#   Unit [Tuple<string>]
CONST_SECTIONS = ('0. How to use this brief', '1. The project in brief', '2. Current state', '3. Architecture',
                  '4. Lessons and rules that must not break', '5. How work is done', '6. Open items and risks',
                  "7. The assistant's role", '8. How to answer', '9. Updates')


def _Brief(root):
    with open(os.path.join(root, CONST_BRIEF), encoding='utf-8') as handle:
        return handle.read()


def _Row(brief, label):
    rows = [line for line in brief.splitlines() if line.startswith('| %s' % label)]
    assert len(rows) == 1, label
    return rows[0]


@pytest.fixture(scope='module')
def baselines(root):
    spec = importlib.util.spec_from_file_location('physics_baseline', os.path.join(root, CONST_TOOL))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return {program: module.Read_Baseline(program) for program in ('taught', 'optimized')}


def test_the_brief_has_its_sections_in_order(root):
    """
    Description:
        Sections 0-9 as level-2 headings, in the skill's order.
    """

    assert tuple(re.findall(r'^## (.+)$', _Brief(root), re.M)) == CONST_SECTIONS


def test_the_brief_names_the_commit_it_was_checked_against(root):
    """
    Description:
        Section 0 and section 9 name the same commit and date; the commit is an ancestor of HEAD (a commit cannot name
        itself, so it is the last commit before the brief's), and the date is the commit's date or later.
    """

    brief = _Brief(root)
    state = re.findall(r'^State as of commit `([0-9a-f]{7,40})`, (\d{4}-\d{2}-\d{2})\.$', brief, re.M)
    update = re.findall(r'^Last updated (\d{4}-\d{2}-\d{2}), checked against commit `([0-9a-f]{7,40})`\.$', brief, re.M)
    assert len(state) == 1 and len(update) == 1
    (commit, date), (date_9, commit_9) = state[0], update[0]
    assert (commit, date) == (commit_9, date_9)
    result = subprocess.run(['git', 'merge-base', '--is-ancestor', commit, 'HEAD'], cwd=root, capture_output=True, timeout=60)
    assert result.returncode == 0, commit
    result = subprocess.run(['git', 'log', '-1', '--format=%ad', '--date=short', commit], cwd=root, capture_output=True,
                            text=True, timeout=60)
    assert result.returncode == 0 and datetime.date.fromisoformat(date) >= datetime.date.fromisoformat(result.stdout.strip())


def test_the_brief_carries_the_safety_sentence_word_for_word(root):
    """
    Description:
        The safety sentence of feature-manager's SKILL.md (Physical systems), bold and quoted, in section 1.
    """

    with open(os.path.join(root, CONST_SAFETY_SOURCE), encoding='utf-8') as handle:
        sentence = re.search(r'\*\*("Outputs .+?")\*\*', ' '.join(handle.read().split()), re.S).group(1)
    brief = _Brief(root)
    section_1 = ' '.join(brief[brief.index('## 1.'):brief.index('## 2.')].split())
    assert '**%s**' % sentence in section_1


def test_the_brief_numbers_match_their_sources(root, baselines, cell_configuration, program):
    """
    Description:
        The cycle times, throughputs, checks, slip, tracking, free-transfer distance and the fork's envelopes against the
        physics baselines; the bounds against Cell.yaml Acceptance; the tilt limit against Optimization.yaml; the test time
        and the speed fraction against Cell.yaml.
    """

    from Configuration.Program_Parser import Load_Optimization
    brief = _Brief(root)
    m = {name: baseline['metrics'] for name, baseline in baselines.items()}
    for baseline in baselines.values():
        assert any('(28 of 28)' in line for line in baseline['report'])
    pairs = [('Cycle time [s]', '%.3f', lambda x: x['cycle_time']),
             ('Throughput [PCB/h]', '%.1f', lambda x: 3600.0 / x['cycle_time']),
             ('Closest robot - loader, free transfer [mm]', '%.1f', lambda x: 1000.0 * x['free_loader_distance']),
             ('PCB slip in the cups [mm]', '%.2f', lambda x: 1000.0 * x['slip']),
             ('Joint tracking error [rad]', '%.4f', lambda x: x['tracking']),
             ('Fork net force on the bar [N]', '%.1f-%.1f', lambda x: tuple(x['envelope']['fork_net_force'])),
             ('Handle hinge load [N m]', '%.2f-%.2f', lambda x: tuple(x['envelope']['handle_load']))]
    for label, form, value in pairs:
        cells = [c.strip() for c in _Row(brief, label).split('|')[2:4]]
        assert [c.split(' (')[0] for c in cells] == [form % value(m['taught']), form % value(m['optimized'])], label
    assert [c.strip() for c in _Row(brief, 'Acceptance checks').split('|')[2:4]] == ['28/28', '28/28']
    assert '(bound %.2f)' % (1000.0 * cell_configuration.Slip_Max) in _Row(brief, 'PCB slip in the cups')
    assert '(bound %g)' % cell_configuration.Tracking_Max in _Row(brief, 'Joint tracking error')
    assert '(bound %g)' % cell_configuration.Fork_Net_Force_Max in _Row(brief, 'Fork net force')
    assert '(bound %g)' % cell_configuration.Handle_Load_Max in _Row(brief, 'Handle hinge load')
    assert '(limit %g, assumed)' % round(math.degrees(Load_Optimization(program).Carry_Max_Tilt), 6) in _Row(brief, 'Carried PCB tilt')
    assert 'test (%.1f s placeholder)' % cell_configuration.Test_Duration in brief
    assert 'the %g %% speed' % (100.0 * cell_configuration.Speed_Fraction) in brief
