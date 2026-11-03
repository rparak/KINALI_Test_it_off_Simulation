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
# Importlib (The implementation of import)
import importlib.util
# Hashlib (Secure hashes)
import hashlib

"""
Description:
    The README against its sources (readme-writer project reference > README tests): the results block against the
    programs' physics baselines (Example/Cell/results.py writes it), the acceptance count, every image it shows present,
    every README medium in Media/ shown, the logo variants' bytes, and the project's README rules.
"""
# The physics baselines' reports, by the README's program name.
#   Unit [Dict<string, string>]
CONST_BASELINES = {'Default program': 'test/Simulation/data/baseline_report_taught.txt',
                   'Optimized program': 'test/Simulation/data/baseline_report_optimized.txt'}
# The logo variants' sha256 (written once from the owner's originals, F5; their generator is in git at c43f079; design-decisions:
# the brand, KNOWN_ISSUES: the JIC logo is a recoloured copy).
#   Unit [Dict<string, string>]
CONST_LOGOS = {'Media/JIC_Smart_Factory_Black.svg': 'd3df6501fe45fa0f75321e219d0abda0d71022f95279b1d176dca86a6e2ab59b',
               'Media/JIC_Smart_Factory_White.svg': '8dbd848c53caca842e8007234de7e89c3266f6827cdaa5d4e4b1f1212ede77bb',
               'Media/KINALI_Black.svg': '17a10180531339376e99b6077126b943919b4eebfe18daf87619de626f0a9305',
               'Media/KINALI_White.svg': 'cdbf9ae4fd375192ac496f549746d246460cb4af62f01b392a2591b506399ba6'}
# Never in the README (CLAUDE.md, the readme-writer project reference > Rules; owner, 2026-10-26; the skills named since
# owner, 2026-10-28).
#   Unit [List<string>]
CONST_FORBIDDEN = ['project-docs', 'PyQt', 'Phase 2']


def _Readme(root):
    with open(os.path.join(root, 'README.md'), encoding='utf-8') as handle:
        return handle.read()


def _Baseline_Lines(root, name):
    with open(os.path.join(root, CONST_BASELINES[name]), encoding='utf-8') as handle:
        return handle.read().splitlines()


def _Results(root):
    """
    Description:
        Example/Cell/results.py, the one writer of the README's results block.
    """

    spec = importlib.util.spec_from_file_location('results', os.path.join(root, 'Example', 'Cell', 'results.py'))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_the_results_block_is_the_one_the_physics_baselines_give(root):
    """
    Description:
        The README's results block (between its markers, once) is what cell.py all --recapture writes from the physics
        baselines: the cycle times (70.404 s, 59.148 s), the throughputs (3600 s / cycle, to 0.1 PCB/h), the changes and 28/28;
        Figure 1's caption carries no cycle time (the media show the v1.0 programs).
    """

    results, text = _Results(root), _Readme(root)
    block = results.Readme_Block(*results.Baseline_Results(os.path.join(root, 'test', 'Simulation', 'data')))
    assert text.count(results.CONST_BLOCK_START) == 1 and block in text
    assert '| Cycle time [s] [P] | 70.404 | 59.148 | \u221216.0 % |' in block
    caption = text.split('**Figure 1:**', 1)[1].split('\n\n', 1)[0]
    assert not re.search(r'\d+\.\d{3} s', caption) and 'The media show the v1.0 programs.' in caption


def test_both_programs_pass_the_28_checks_as_the_readme_says(root):
    """
    Description:
        Each physics baseline ends with 28 of 28 checks met, and the README says 28/28 for both programs.
    """

    for name in CONST_BASELINES:
        assert '(28 of 28)' in _Baseline_Lines(root, name)[-1], name
    text = _Readme(root)
    assert '| Acceptance checks [P] | 28/28 pass | 28/28 pass |' in text


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
        Every image in Media/ is shown in the README; a chart's SVG beside its shown PNG is the
        same figure in another format and is not shown.
    """

    tracked = subprocess.run(['git', 'ls-files', 'Media'], cwd=root, capture_output=True, text=True, timeout=60,
                             check=True).stdout.splitlines()
    media = set(tracked)
    expected = {p for p in media if not (p.endswith('.svg') and p[:-4] + '.png' in media)}
    assert expected == _Shown(_Readme(root))


def test_the_readme_keeps_the_project_rules(root):
    """
    Description:
        The README names no internal document (of .claude/ only the skills), mentions no GUI or phase, calls the programs the
        Default program and the Optimized program, and carries the safety alert near the top.
    """

    text = _Readme(root)
    for word in CONST_FORBIDDEN:
        assert word not in text, word
    assert set(re.findall(r'\.claude/([\w-]+)', text)) == {'skills'}
    assert 'taught program' not in text.lower() and 'optimised' not in text.lower()
    assert text.count('Default program') >= 5 and text.count('Optimized program') >= 5
    alert = text.index('> [!WARNING]')
    assert alert < text.index('## Contents')
    assert 'are simulation results and are NOT validated for driving the real cell.' in text[alert:alert + 400]


def test_the_readme_names_the_shipped_skills(root):
    """
    Description:
        The README's skills table names exactly the skills git tracks in .claude/skills/ (one SKILL.md each), so a skill
        added, removed or renamed shows in the README.
    """

    tracked = subprocess.run(['git', 'ls-files', '.claude/skills'], cwd=root, capture_output=True, text=True, timeout=60)
    assert tracked.returncode == 0, tracked.stderr
    shipped = {p.split('/')[2] for p in tracked.stdout.splitlines() if p.endswith('/SKILL.md')}
    section = _Readme(root).split('## Claude Code skills', 1)[1].split('\n## ', 1)[0]
    assert set(re.findall(r'(?m)^\| `([\w-]+)` \|', section)) == shipped


def test_the_logo_variants_are_pinned(root):
    """
    Description:
        The four logo variants the README header shows are the committed ones, byte for byte (their sha256).
    """

    for path, digest in CONST_LOGOS.items():
        with open(os.path.join(root, path), 'rb') as handle:
            assert hashlib.sha256(handle.read()).hexdigest() == digest, path
