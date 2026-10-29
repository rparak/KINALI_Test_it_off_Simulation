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
# File Name: ../test/Program/test_golden.py
## =========================================================================== ##

# System (Default)
import os
import copy
# Importlib (Import machinery)
import importlib.util
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    The golden comparison of the taught program (SPEC §10.2): the program built now against test/Program/data/ (the record of
    the numeric arrays' sha256, the cycle time, the events, the steps, the segments and every command's values as float hex;
    the program check's report; the point table), read and captured by Example/Maintenance/golden_program.py. A change to the
    program or its build re-captures the data in its own commit, listing what changed.
"""
# The golden tool (a script, not part of src/).
#   Unit [string]
CONST_TOOL = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'Example', 'Maintenance', 'golden_program.py')


@pytest.fixture(scope='module')
def tool():
    spec = importlib.util.spec_from_file_location('golden_program', CONST_TOOL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.fixture(scope='module')
def now(tool):
    return tool.Capture()


@pytest.fixture(scope='module')
def golden(tool):
    return tool.Read_Golden()


def test_the_trajectory_arrays_are_byte_identical(now, golden):
    """
    Description:
        Every numeric array of the trajectory - t, robot_q, robot_qd, loader_y, loader_z, gripper, vacuum and the world state,
        steps and segments - has the golden sha256, dtype and shape: the same bytes.
    """

    assert sorted(now['program']['arrays']) == sorted(golden['program']['arrays'])
    for name, value in now['program']['arrays'].items():
        assert value == golden['program']['arrays'][name], name


def test_the_cycle_time_events_and_segments_are_the_same(now, golden):
    """
    Description:
        The cycle time, the sample count, the events (times bit-exact, names), the steps and the executed segments' texts in
        order are the golden ones.
    """

    for key in ('cycle_time', 'samples', 'events', 'step_names', 'segment_commands'):
        assert now['program'][key] == golden['program'][key], key


def test_every_command_has_the_golden_values(now, golden):
    """
    Description:
        The executed commands in order against the golden ones: the kind, the target, the joint targets (float hex, rad), the
        speeds, the durations, the timeouts and the loader and jaw values bit for bit.
    """

    assert len(now['program']['commands']) == len(golden['program']['commands'])
    for a, b in zip(now['program']['commands'], golden['program']['commands']):
        assert a == b, (a['text'], b['text'])


def test_the_check_report_is_the_golden_one(now, golden):
    """
    Description:
        The program check's report equals golden_check_report.txt line for line: every distance, sample, time, peak, gap,
        event and verdict the same.
    """

    assert now['report'] == golden['report']


def test_the_point_table_is_the_golden_one(now, golden):
    """
    Description:
        The point check table equals golden_keypoint_table.txt row for row.
    """

    assert now['table'] == golden['table']


def test_a_difference_is_named(tool, golden):
    """
    Description:
        Compare_Golden names each difference (an array, the cycle time, a command field, a report line) and nothing when the
        captures are the same ('source' is not compared).
    """

    changed = copy.deepcopy(golden)
    assert tool.Compare_Golden(changed, golden) == []
    changed['program']['source'] = 'elsewhere'
    changed['program']['arrays']['robot_q']['sha256'] = '0'
    changed['program']['cycle_time'] += 0.004
    changed['program']['commands'][1]['speed'] = '0x1p-1'
    changed['report'][2] = 'x'
    lines = tool.Compare_Golden(changed, golden)
    speed = 'command 1 (%s) speed' % golden['program']['commands'][1]['text']
    assert [line.split(':')[0] for line in lines] == ['array robot_q', 'cycle_time', speed, 'report line 3'], lines
