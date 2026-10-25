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
def built(built_trajectory):
    built_trajectory('taught')
    return str(built_trajectory('optimized').parent)


@pytest.mark.slow
@pytest.mark.parametrize('program', ['taught', 'optimized'])
def test_the_program_meets_its_bounds_and_stays_within_its_drift_tolerances(tool, built, program):
    """
    Description:
        The program's physics run meets every SPEC §10.1 criterion (28) and no metric drifts from
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
        slip within 0.2 mm, the fork's net force within its envelope +- 10 % (the run +9 % passes, the envelope's max +11 %
        fails), a changed number of values; a failed
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
    m['fork_net_force'] = b['envelope']['fork_net_force'][1] * 1.11
    m['hold_min_cups'] = m['hold_min_cups'][:1]
    assert [line.split(':')[0] for line in tool.Drift(m, b)] == ['cycle_time', 'vacuum_times', 'hold_min_cups', 'fork_net_force']
    now['acceptance'] = [('PCB slip <= 1.98 mm', False, '2.10 mm')]
    assert tool.Compare(now, baseline)[0][0] == 'acceptance: PCB slip <= 1.98 mm (2.10 mm)'


@pytest.mark.parametrize('key, index, factor, passes', [
    ('fork_net_force', None, 1.0, True),
    ('fork_net_force', None, 'high', True),
    ('fork_net_force', None, 'over', False),
    ('fork_net_force', None, 'under', False),
    ('handle_load', None, 'over', False),
    ('joint_torque', 1, 'over', False),
    ('joint_torque', 2, 'under', False),
    ('joint_torque', 0, 1.09, True),
    ('joint_torque', 0, 1.11, False),
])
def test_the_fork_metrics_are_compared_with_the_envelope(tool, key, index, factor, passes):
    """
    Description:
        Item 1 (owner, 2026-10-25): the fork's net force, the hinge load and the J2 / J3 torques pass inside their program's
        envelope +- 10 % ('high': at its max + 9.9 %) and fail outside it ('over': max + 10.1 %, 'under': min - 10.1 %); the other
        torques keep +- 10 % of the baseline run (J1 +9 % passes, +11 % fails).
    """

    baseline = tool.Read_Baseline('taught')
    m = copy.deepcopy(baseline['metrics'])
    low, high = baseline['metrics']['envelope'][key]
    pick = (lambda v: v) if index is None else (lambda v: v[index])
    value = {'high': pick(high) * 1.099, 'over': pick(high) * 1.101, 'under': pick(low) * 0.899}.get(factor)
    if value is None:
        value = pick(m[key]) * factor
    if index is None:
        m[key] = value
    else:
        m[key][index] = value
    drift = tool.Drift(m, baseline['metrics'])
    assert (drift == []) == passes, drift


def test_a_failed_bound_fails_inside_the_envelope(tool):
    """
    Description:
        Item 1: a run whose fork net force is inside the envelope but fails SPEC §10.1 (the 290 N bound, here as a failed
        criterion) fails the regression: the bounds apply whatever the envelope.
    """

    baseline = tool.Read_Baseline('taught')
    now = {'report': list(baseline['report']), 'metrics': copy.deepcopy(baseline['metrics']),
           'acceptance': [('fork net force <= 290 N (sim. regression bound)', False, '291.0 N')]}
    failures, _ = tool.Compare(now, baseline)
    assert failures == ['acceptance: fork net force <= 290 N (sim. regression bound) (291.0 N)']


def test_a_metric_missing_from_the_baseline_is_named(tool):
    """
    Description:
        A metric the baseline does not have (one added since its capture, e.g. the place into MG06's, B work) is a drift naming
        it and the re-capture, not a crash.
    """

    baseline = tool.Read_Baseline('taught')
    now = copy.deepcopy(baseline['metrics'])
    old = {k: v for k, v in copy.deepcopy(baseline['metrics']).items() if k != 'place_drop'}
    assert 'place_drop: not in the baseline (re-capture it)' in tool.Drift(now, old)
