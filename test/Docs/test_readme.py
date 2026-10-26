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
# File Name: ../test/Docs/test_readme.py
## =========================================================================== ##

# System (Default)
import os
# Regular expressions (Regular expression operations)
import re
# Subprocess (Subprocess management)
import subprocess

"""
Description:
    The README against its sources (readme-writer project reference > README tests): the cycle times and the acceptance
    count against the programs' physics baselines, every image it shows present, every README medium in Media/ shown, and
    the project's README rules.
"""
# The physics baselines' reports, by the README's program name.
#   Unit [Dict<string, string>]
CONST_BASELINES = {'Default program': 'test/Simulation/data/baseline_report_taught.txt',
                   'Optimized program': 'test/Simulation/data/baseline_report_optimized.txt'}
# Never in the README (CLAUDE.md, the readme-writer project reference > Rules; owner, 2026-10-26).
#   Unit [List<string>]
CONST_FORBIDDEN = ['.claude', 'project-docs', 'SKILL.md', 'PyQt', 'Phase 2']


def _Readme(root):
    with open(os.path.join(root, 'README.md'), encoding='utf-8') as handle:
        return handle.read()


def _Baseline_Lines(root, name):
    with open(os.path.join(root, CONST_BASELINES[name]), encoding='utf-8') as handle:
        return handle.read().splitlines()


def _Cycle(root, name):
    match = re.fullmatch(r'program \S+: physics run of its trajectory, cycle (\d+\.\d{3}) s', _Baseline_Lines(root, name)[0])
    assert match, CONST_BASELINES[name]
    return match.group(1)


def test_the_cycle_times_and_throughputs_are_the_physics_baselines(root):
    """
    Description:
        The README's cycle times (the comparison table and Figure 1's caption) are the physics baselines' (70.404 s,
        59.148 s), and its throughputs are 3600 s divided by them, to 0.1 PCB/h.
    """

    text = _Readme(root)
    default, optimized = _Cycle(root, 'Default program'), _Cycle(root, 'Optimized program')
    assert '| Cycle time [s] [K] | %s | %s |' % (default, optimized) in text
    assert 'finishes after %s s against %s s' % (optimized, default) in text
    assert '| Throughput [PCB/h] [K] | %.1f | %.1f |' % (3600 / float(default), 3600 / float(optimized)) in text


def test_both_programs_pass_the_28_checks_as_the_readme_says(root):
    """
    Description:
        Each physics baseline ends with 28 of 28 checks met, and the README says 28/28 for both programs.
    """

    for name in CONST_BASELINES:
        assert '(28 of 28)' in _Baseline_Lines(root, name)[-1], name
    text = _Readme(root)
    assert '| Acceptance checks [P] | 28/28 pass | 28/28 pass |' in text
    assert 'Both programs pass 28 of 28' in text


def _Shown(text):
    return set(re.findall(r'(?:src|srcset)="(Media/[^"]+)"', text))


def test_every_image_the_readme_shows_exists(root):
    """
    Description:
        Every Media/ file the README shows (an <img> or a <picture> source) is tracked by git.
    """

    tracked = set(subprocess.run(['git', 'ls-files', 'Media'], cwd=root, capture_output=True, text=True, timeout=60,
                                 check=True).stdout.splitlines())
    shown = _Shown(_Readme(root))
    assert shown and shown <= tracked, sorted(shown - tracked)


def test_every_readme_medium_is_shown(root):
    """
    Description:
        Every image in Media/ (not the owner's originals) is shown in the README; a chart's SVG beside its shown PNG is the
        same figure in another format and is not shown.
    """

    tracked = subprocess.run(['git', 'ls-files', 'Media'], cwd=root, capture_output=True, text=True, timeout=60,
                             check=True).stdout.splitlines()
    media = {p for p in tracked if not p.startswith('Media/Original/')}
    expected = {p for p in media if not (p.endswith('.svg') and p[:-4] + '.png' in media)}
    assert expected == _Shown(_Readme(root))


def test_the_readme_keeps_the_project_rules(root):
    """
    Description:
        The README names neither the internal documents nor the skills, mentions no GUI or phase, calls the programs the
        Default program and the Optimized program, and carries the safety alert near the top.
    """

    text = _Readme(root)
    for word in CONST_FORBIDDEN:
        assert word not in text, word
    assert 'taught program' not in text.lower() and 'optimised' not in text.lower()
    assert text.count('Default program') >= 5 and text.count('Optimized program') >= 5
    alert = text.index('> [!WARNING]')
    assert alert < text.index('## Contents')
    assert 'are simulation results and are NOT validated for driving the real cell.' in text[alert:alert + 400]
