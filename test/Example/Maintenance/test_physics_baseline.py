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
# File Name: ../test/Example/Maintenance/test_physics_baseline.py
## =========================================================================== ##

# System (Default)
import os
# JSON (JSON encoder and decoder)
import json
# Subprocess (Subprocess management)
import subprocess
# Pytest (Testing framework) [pip3 install pytest]
import pytest


def _Run_In(command, tree, script, arguments, cwd, temporary):
    """
    Description:
        Run tree/Example/<script> (command: the script_command fixture) from cwd with its own temporary folder (TMPDIR), so what
        the script leaves there can be seen.
    """

    return subprocess.run(command(tree, script) + list(arguments), cwd=str(cwd),
                          capture_output=True, text=True, timeout=900, env=dict(os.environ, TMPDIR=str(temporary)))


@pytest.mark.slow
def test_the_optimized_program_compares_clean_and_out_writes_its_baseline(root, tmp_path, script_command, fresh_tree):
    """
    Description:
        Step 6: in a fresh tree (no Data/Generated/, as in a fresh clone) and from an unrelated directory, on a trajectory built
        into a temporary folder, the optimized program's physics run compares with its baseline without a failure (exit 0);
        --out writes its report and metrics (the report identical to the committed one on this machine; the metrics with the
        envelope over the 10 perturbation runs, the baseline run first) and the environment record; every run, the settle-wait
        variants included, reads the trajectories from --trajectories; nothing is left in the working directory or the
        temporary folder.
    """

    tree, built, work, temporary = fresh_tree(tmp_path / 'tree'), tmp_path / 'built', tmp_path / 'work', tmp_path / 'tmp'
    work.mkdir()
    temporary.mkdir()
    program = os.path.join(str(tree), 'Data', 'Optimized', 'Program_Optimized.yaml')
    assert _Run_In(script_command, tree, 'Cell/build_program.py', ['--program', program, '--out', str(built)], work, temporary).returncode == 0
    arguments = ['--program', 'optimized', '--trajectories', str(built)]
    result = _Run_In(script_command, tree, 'Maintenance/physics_baseline.py', arguments, work, temporary)
    assert result.returncode == 0 and '[INFO] optimized: 0 failure(s) (bounds and drift)' in result.stdout, result.stdout[-2000:]
    out = tmp_path / 'out'
    result = _Run_In(script_command, tree, 'Maintenance/physics_baseline.py', arguments + ['--out', str(out)], work, temporary)
    assert result.returncode == 0 and sorted(os.listdir(out)) == ['baseline_environment.json', 'baseline_metrics_optimized.json',
                                                                  'baseline_report_optimized.txt'], result.stdout[-2000:]
    committed = open(os.path.join(root, 'test', 'Simulation', 'data', 'baseline_report_optimized.txt'), encoding='utf-8').read()
    assert (out / 'baseline_report_optimized.txt').read_text() == committed and os.listdir(work) == []
    assert os.listdir(temporary) == [] and not os.path.exists(tree / 'Data' / 'Generated' / 'Trajectories')
    metrics = json.loads((out / 'baseline_metrics_optimized.json').read_text())
    assert [r[:3] for r in metrics['envelope_runs']] == [list(p) for p in [(0.0, 0.0, 0.0), (-5e-6, 0.0, 0.0), (-1e-6, 0.0, 0.0), (1e-6, 0.0, 0.0),
                                                                         (5e-6, 0.0, 0.0), (-5e-6, 0.1, 0.0), (0.0, 0.1, 0.0), (5e-6, 0.1, 0.0),
                                                                         (0.0, 0.0, 1e-6), (0.0, 0.0, -1e-6)]]
    low, high = metrics['envelope']['fork_net_force']
    assert low <= metrics['fork_net_force'] <= high and sorted(metrics['envelope']) == ['fork_net_force', 'handle_load', 'joint_torque']
