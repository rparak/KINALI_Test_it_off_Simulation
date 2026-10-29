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
import sys
# JSON (JSON encoder and decoder)
import json
# Shutil (High-level file operations)
import shutil
# Subprocess (Subprocess management)
import subprocess
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.
"""
# The tracked folders a fresh clone holds that the physics scripts read (models/ is linked, not copied: 59 MB).
#   Unit [Tuple<string>]
CONST_FRESH_FOLDERS = ('src', 'Example', 'Configuration', 'Data', os.path.join('test', 'Simulation', 'data'))


def _Fresh_Tree(root, tree):
    """
    Description:
        A copy of the repository as a fresh clone has it: the tracked folders the physics scripts read, models/ linked, and no
        Data/Generated/ (git-ignored, so never in a clone).
    """

    for folder in CONST_FRESH_FOLDERS:
        shutil.copytree(os.path.join(root, folder), tree / folder, ignore=shutil.ignore_patterns('Generated', '__pycache__'))
    os.symlink(os.path.join(root, 'models'), tree / 'models')
    return tree


def _Run_In(tree, script, arguments, cwd, temporary):
    """
    Description:
        Run tree/Example/<script> from cwd with its own temporary folder (TMPDIR), so what the script leaves there can be seen.
    """

    return subprocess.run([sys.executable, os.path.join(str(tree), 'Example', script)] + list(arguments), cwd=str(cwd),
                          capture_output=True, text=True, timeout=900, env=dict(os.environ, TMPDIR=str(temporary)))


@pytest.mark.slow
def test_the_optimized_program_compares_clean_and_out_writes_its_baseline(root, tmp_path):
    """
    Description:
        Step 6: in a fresh tree (no Data/Generated/, as in a fresh clone) and from an unrelated directory, on a trajectory built
        into a temporary folder, the optimized program's physics run compares with its baseline without a failure (exit 0);
        --out writes its report and metrics (the report identical to the committed one on this machine; the metrics with the
        envelope over the 10 perturbation runs, the baseline run first) and the environment record; every run, the settle-wait
        variants included, reads the trajectories from --trajectories; nothing is left in the working directory or the
        temporary folder.
    """

    tree, built, work, temporary = _Fresh_Tree(root, tmp_path / 'tree'), tmp_path / 'built', tmp_path / 'work', tmp_path / 'tmp'
    work.mkdir()
    temporary.mkdir()
    program = os.path.join(str(tree), 'Data', 'Optimized', 'Program_Optimized.yaml')
    assert _Run_In(tree, 'Program/build_program.py', ['--program', program, '--out', str(built)], work, temporary).returncode == 0
    result = _Run_In(tree, 'Maintenance/physics_baseline.py', ['--program', 'optimized', '--trajectories', str(built)], work, temporary)
    assert result.returncode == 0 and '[INFO] optimized: 0 failure(s) (bounds and drift)' in result.stdout, result.stdout[-2000:]
    out = tmp_path / 'out'
    result = _Run_In(tree, 'Maintenance/physics_baseline.py', ['--program', 'optimized', '--trajectories', str(built), '--out', str(out)],
                     work, temporary)
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
