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
# File Name: ../test/Simulation/test_baselines.py
## =========================================================================== ##

# System (Default)
import os
import re
import copy
import subprocess
import sys
# Importlib (Import machinery)
import importlib.util
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    The physics regression of the two programs (step 6, SPEC §10.2): each program built into a temporary folder, run with
    physics and compared with its baseline in test/Simulation/data/ by Example/Simulation/physics_baseline.py: SPEC §10.1 and
    the metrics' drift tolerances fail it; the report's diff is printed, never failing.
"""
# The baseline tool and the repository root.
#   Unit [string]
CONST_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
CONST_TOOL = os.path.join(CONST_ROOT, 'Example', 'Simulation', 'physics_baseline.py')


@pytest.fixture(scope='module')
def tool():
    spec = importlib.util.spec_from_file_location('physics_baseline', CONST_TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def built(tmp_path_factory):
    out = tmp_path_factory.mktemp('trajectories')
    for arguments in ([], ['--program', os.path.join(CONST_ROOT, 'Data', 'Optimized', 'Program_Optimized.yaml')]):
        result = subprocess.run([sys.executable, os.path.join(CONST_ROOT, 'Example', 'Program', 'build_program.py'), '--out', str(out)] + arguments,
                                cwd=str(out), capture_output=True, text=True, timeout=600)
        assert result.returncode == 0, result.stdout[-2000:] + result.stderr
    return str(out)


@pytest.mark.slow
@pytest.mark.parametrize('program', ['taught', 'optimized'])
def test_the_program_meets_its_bounds_and_stays_within_its_drift_tolerances(tool, built, program):
    """
    Description:
        The program's physics run meets every SPEC §10.1 criterion (20) and no metric drifts from
        test/Simulation/data/baseline_metrics_<program>.json beyond physics_baseline.CONST_TOLERANCES; the report diff against
        baseline_report_<program>.txt is shown when it is not empty, never failing. The LOCKED handover and take report the
        robot's distance to the loader's carriages and jaws, a positive one: the cups touching the PCB in the jaws there is the
        allowed cups_pcb contact, not a robot - loader one (step 6 follow-up A4).
    """

    now = tool.Capture(program, built)
    rows = [line.split() for line in now['report'] if re.fullmatch(r'S\d+_(Handover|Take_From_Loader)', (line.split() or [''])[0])]
    assert len(rows) == 2 and all(row[3] == 'LOCKED' and float(row[6]) > 0.0 for row in rows), rows
    failures, diff = tool.Compare(now, tool.Read_Baseline(program))
    if diff:
        print('\n'.join(['report diff (information):'] + diff))
    assert failures == [], failures


def test_a_drift_beyond_its_tolerance_fails_and_a_report_diff_does_not(tool):
    """
    Description:
        Each tolerance kind: the cycle time exact (+1 ms fails), a vacuum event within one sample (4 ms passes, 4.1 ms fails), the
        slip within 0.2 mm, the fork's net force within 10 % (+9 % passes, +11 % fails), a changed number of values; a failed
        criterion fails; a report line that differs is only in the diff.
    """

    baseline = tool.Read_Baseline('taught')
    now = {'report': list(baseline['report']), 'metrics': copy.deepcopy(baseline['metrics']),
           'acceptance': [('a criterion', True, '')]}
    assert tool.Compare(now, baseline) == ([], [])
    now['report'][0] += ' (changed)'
    failures, diff = tool.Compare(now, baseline)
    assert failures == [] and any(line.startswith('+') and '(changed)' in line for line in diff)
    m, b = now['metrics'], baseline['metrics']
    m['vacuum_times'][0] += 0.004
    m['slip'] += 0.0002
    m['fork_net_force'] *= 1.09
    assert tool.Drift(m, b) == []
    m['cycle_time'] += 0.001
    m['vacuum_times'][0] += 0.0001
    m['fork_net_force'] = b['fork_net_force'] * 1.11
    m['hold_min_cups'] = m['hold_min_cups'][:1]
    assert [line.split(':')[0] for line in tool.Drift(m, b)] == ['cycle_time', 'vacuum_times', 'hold_min_cups', 'fork_net_force']
    now['acceptance'] = [('PCB slip <= 1.98 mm', False, '2.10 mm')]
    assert tool.Compare(now, baseline)[0][0] == 'acceptance: PCB slip <= 1.98 mm (2.10 mm)'
