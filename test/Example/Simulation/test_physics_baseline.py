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
# File Name: ../test/Example/Simulation/test_physics_baseline.py
## =========================================================================== ##

# System (Default)
import os
# Pytest (Testing framework) [pip3 install pytest]
import pytest


@pytest.mark.slow
def test_the_optimized_program_compares_clean_and_out_writes_its_baseline(root, tmp_path, run_script):
    """
    Description:
        Step 6: from an unrelated directory, on a trajectory built into a temporary folder, the optimized program's physics run
        compares with its baseline without a failure (exit 0); --out writes its report and metrics (the report identical to the
        committed one on this machine) and the environment record; nothing in the working directory.
    """

    built, work = tmp_path / 'built', tmp_path / 'work'
    work.mkdir()
    program = os.path.join(root, 'Data', 'Optimized', 'Program_Optimized.yaml')
    assert run_script('Program/build_program.py', ['--program', program, '--out', str(built)], work).returncode == 0
    result = run_script('Simulation/physics_baseline.py', ['--program', 'optimized', '--trajectories', str(built)], work)
    assert result.returncode == 0 and '[INFO] optimized: 0 failure(s) (bounds and drift)' in result.stdout, result.stdout[-2000:]
    out = tmp_path / 'out'
    result = run_script('Simulation/physics_baseline.py', ['--program', 'optimized', '--trajectories', str(built), '--out', str(out)], work)
    assert result.returncode == 0 and sorted(os.listdir(out)) == ['baseline_environment.json', 'baseline_metrics_optimized.json',
                                                                  'baseline_report_optimized.txt'], result.stdout
    committed = open(os.path.join(root, 'test', 'Simulation', 'data', 'baseline_report_optimized.txt'), encoding='utf-8').read()
    assert (out / 'baseline_report_optimized.txt').read_text() == committed and os.listdir(work) == []
