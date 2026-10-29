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
# File Name: ../test/Example/Cell/test_run_program.py
## =========================================================================== ##

# System (Default)
import os
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    Example/Cell/run_program.py run as a person runs it (fresh interpreter, unrelated working directory), on a trajectory
    built into a temporary folder.
"""
# The exit codes the script documents: all criteria met, one failed, cannot run / unstable.
#   Unit [int]
CONST_PASSED, CONST_FAILED, CONST_ERROR = 0, 1, 2


@pytest.fixture(scope='module')
def taught_trajectory(built_trajectory):
    return built_trajectory('taught')


@pytest.mark.slow
def test_the_taught_program_runs_and_meets_the_acceptance(tmp_path, run_script, taught_trajectory):
    """
    Description:
        F11: the taught program's physics run exits 0; stdout and DIR/Program_physics.txt hold the report: the program line, the
        holds of 4 of 4 cups, a row per step, the closing sentence with 28 of 28 checks; stderr the not-validated sentence;
        nothing is written to the working directory.
    """

    work = tmp_path / 'work'
    work.mkdir()
    result = run_script('Cell/run_program.py', ['--program', 'taught', '--trajectory', str(taught_trajectory), '--out', str(tmp_path / 'out')],
                        work)
    assert result.returncode == CONST_PASSED, result.stdout[-2000:] + result.stderr
    lines = result.stdout.splitlines()
    assert lines[0] == 'program Program: physics run of its trajectory, cycle 70.404 s'
    assert 'vacuum holds: 3.276-10.164 s min 4/4 cups; 58.608-67.080 s min 4/4 cups' in lines
    assert lines[-1] == 'Program: the physics run meets SPEC §10.1 (28 of 28). Simulation results only: not validated for the real cell.'
    assert (tmp_path / 'out' / 'Program_physics.txt').read_text().splitlines() == lines
    assert 'not validated for the real cell' in result.stderr and os.listdir(work) == []


def test_a_missing_or_stale_trajectory_exits_2(tmp_path, run_script, taught_trajectory):
    """
    Description:
        A missing trajectory and one built from another program (the optimized program's run given the taught one's file) are
        [ERROR] lines naming what to do, exit 2, no report.
    """

    result = run_script('Cell/run_program.py', ['--trajectory', str(tmp_path / 'absent.npz'), '--out', str(tmp_path / 'out')], tmp_path)
    assert result.returncode == CONST_ERROR and '[ERROR] ' in result.stderr and 'absent.npz' in result.stderr, result.stderr
    result = run_script('Cell/run_program.py', ['--program', 'optimized', '--trajectory', str(taught_trajectory), '--out',
                                                      str(tmp_path / 'out')], tmp_path)
    assert result.returncode == CONST_ERROR and 'was built from other inputs (program changed since); rebuild it: .venv/bin/python ' \
        'Example/Program/build_program.py --program Data/Optimized/Program_Optimized.yaml' in result.stderr, result.stderr
    assert not (tmp_path / 'out').exists()
