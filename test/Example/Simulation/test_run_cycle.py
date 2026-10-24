## =========================================================================== ##
# MIT License
# Copyright (c) 2026 Roman Parak
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
# File Name: ../test/Example/Simulation/test_run_cycle.py
## =========================================================================== ##

# System (Default)
import os
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    Example/Simulation/run_cycle.py run as a person runs it (fresh interpreter, unrelated working directory).
"""
# The exit codes the script documents: all criteria met, one failed, cannot run / unstable.
#   Unit [int]
CONST_PASSED, CONST_FAILED, CONST_ERROR = 0, 1, 2


def _Log(root):
    with open(os.path.join(root, 'logger', 'run_cycle.log'), encoding='utf-8') as handle:
        return handle.read()


@pytest.mark.slow
def test_the_cycle_prints_the_baseline_report_and_passes(root, tmp_path, run_script):
    """
    Description:
        The default run exits 0; stdout is test/Simulation/data/baseline_report.txt byte for byte, and so is
        report.txt in the --out folder; stderr has ten PASS lines and the not-validated sentence; the log in logger/ holds
        the report; nothing is written to the working directory.
    """

    work = tmp_path / 'work'
    work.mkdir()
    result = run_script('Simulation/run_cycle.py', ['--out', str(tmp_path / 'out')], work)
    with open(os.path.join(root, 'test', 'Simulation', 'data', 'baseline_report.txt'), encoding='utf-8') as handle:
        baseline = handle.read()
    assert result.returncode == CONST_PASSED, result.stderr
    assert result.stdout == baseline
    assert (tmp_path / 'out' / 'report.txt').read_text() == baseline
    assert result.stderr.count('] PASS ') == 10 and 'FAIL' not in result.stderr
    assert 'not validated for the real cell' in result.stderr
    assert baseline.splitlines()[0] in _Log(root)
    assert os.listdir(work) == []


def test_a_plan_that_cannot_be_read_exits_2_with_an_error_line(root, tmp_path, run_script):
    """
    Description:
        A missing plan is an [ERROR] line naming it, exit 2, no report.
    """

    result = run_script('Simulation/run_cycle.py', ['--plan', str(tmp_path / 'absent.npz'), '--out', str(tmp_path / 'out')], tmp_path)
    assert result.returncode == CONST_ERROR
    assert '[ERROR] The plan %s cannot be read' % (tmp_path / 'absent.npz') in result.stderr
    assert not (tmp_path / 'out').exists() and '[ERROR]' in _Log(root)


def test_a_plan_that_misses_the_criteria_exits_1_naming_them(tmp_path, run_script):
    """
    Description:
        A short plan that never picks the PCB runs, writes its report, and fails the picks and the placement:
        FAIL lines, the count of failed checks, exit 1.
    """

    q = np.tile([0.7896, 0.6910, 1.8333, -0.4286, 1.5708, 0.0042], (300, 1))
    plan = tmp_path / 'short.npz'
    np.savez(plan, dt=0.004, q=q, q_home=q[0], loader='{"y": [], "z": [], "jaw": []}', events='[]',
             phases='[[0.0, "loader: up, to park"]]', J_OPEN=0.035, T=1.0)
    result = run_script('Simulation/run_cycle.py', ['--plan', str(plan), '--out', str(tmp_path / 'out')], tmp_path)
    assert result.returncode == CONST_FAILED, result.stderr
    assert '[ERROR] FAIL vacuum ON 2x with 4/4 cups' in result.stderr
    assert 'acceptance checks failed' in result.stderr and (tmp_path / 'out' / 'report.txt').exists()
