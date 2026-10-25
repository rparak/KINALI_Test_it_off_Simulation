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
# File Name: ../test/Example/Report/test_compare_report.py
## =========================================================================== ##

# OS (Operating system interfaces)
import os
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Example/Report/compare_report.py run as a person runs it (fresh interpreter, unrelated working directory), on both
    trajectories built into a temporary folder.
"""


@pytest.mark.slow
def test_the_comparison_is_written_with_its_figures(tmp_path, run_script, built_trajectory):
    """
    Description:
        F14: exit 0; DIR holds Comparison.md, its five figures and both physics reports, nothing in the working directory. The
        report gives the cycles (70.404 s, 58.716 s), the identical task, 28 of 28 checks in both, no loader slow-down up to 2 x
        breaking I8 (F15: the take gated by Loader.WaitDone(J_RETURN_Y)), and the not-validated sentence; the optimized
        program's physics report is byte for byte the one run_program.py writes (the recording does not change the run).
    """

    folder = os.path.dirname(str(built_trajectory('taught')))
    assert os.path.dirname(str(built_trajectory('optimized'))) == folder
    out, work = tmp_path / 'out', tmp_path / 'work'
    work.mkdir()
    result = run_script('Report/compare_report.py', ['--out', str(out), '--trajectories', folder], work)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-3000:]
    assert sorted(os.listdir(out)) == ['Comparison.md', 'Program_Optimized_physics.txt', 'Program_physics.txt', 'clearance.png',
                                       'comparison.png', 'joints.png', 'paths.png', 'timeline.png'] and os.listdir(work) == []
    text = (out / 'Comparison.md').read_text(encoding='utf-8')
    assert '- Cycle: taught 70.404 s, optimized 58.716 s [K] (16.6 % shorter).' in text
    assert 'taught 29, optimized 29 lines, identical in text and order' in text and 'their lines identical [C]' in text
    assert '- Acceptance: taught 28/28, optimized 28/28 checks pass [P].' in text
    assert '| Loader_Free + Tracking_Margin (I8) | 14.0 mm | none |' in text and '| contact | 0.0 mm | none |' in text
    assert '- A slower loader: no failure of I8 for any job up to 2 x as long' in text
    assert 'are simulation results and are NOT validated for driving the real cell.' in text
    check = tmp_path / 'check'
    run = run_script('Simulation/run_program.py', ['--program', 'optimized', '--trajectory', str(built_trajectory('optimized')), '--out',
                                                   str(check)], work)
    assert run.returncode == 0, run.stderr[-2000:]
    assert (out / 'Program_Optimized_physics.txt').read_bytes() == (check / 'Program_Optimized_physics.txt').read_bytes()
