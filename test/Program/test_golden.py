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
import json
import hashlib
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the build and the check under test
#   ../Program/Core, ../Keypoints/Core
from Program.Core import Build_Program, Check_Trajectory, Format_Check_Report, Executed_Commands
from Keypoints.Core import Check_Keypoints, Format_Check_Table
#   ../Configuration/Core, ../Collision/Core, ../Kinematics/Core
from Configuration.Core import Read_PCB_Meta
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
from Kinematics.Core import Robot_Kinematics_Cls

"""
Description:
    Initialization of constants.

    The golden comparison of the program format change (FEATURES.md F10): the default program of git e5ec42c (F8 closed),
    captured before the change into test/Program/data/ (golden_program.json: the numeric arrays' sha256, the events, the
    commands' values as float hex; golden_check_report.txt; golden_keypoint_table.txt), against the program built from the
    new files. Only names may differ: name_map.json maps them (points, steps, the 69 executed command texts in order).
"""
# The test data folder.
#   Unit [string]
CONST_DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
# The old command kinds and the new ones each may become.
#   Unit [Dict<string, Tuple<string>>]
CONST_KINDS = {'Movej': ('MoveJ',), 'Movel': ('MoveL',), 'Path': ('MovePath',), 'Loader': ('MoveY', 'MoveZ'), 'Gripper': ('Gripper',),
               'Vacuum': ('SetDO',), 'Wait': ('Wait', 'WaitDI'), 'Test': ('Test',)}


def _Json(name):
    with open(os.path.join(CONST_DATA, name), encoding='utf-8') as handle:
        return json.load(handle)


@pytest.fixture(scope='module')
def golden():
    return _Json('golden_program.json')


@pytest.fixture(scope='module')
def names():
    return _Json('name_map.json')


@pytest.fixture(scope='module')
def tools(cell_configuration):
    model = Build_Planning_Model(cell_configuration)
    collision = Robot_Collision_Cls(model)
    return Robot_Kinematics_Cls(model, collision.Data), Read_PCB_Meta(), collision


@pytest.fixture(scope='module')
def built(program, cell_configuration, tools):
    kinematics, meta, _ = tools
    return Build_Program(program, cell_configuration, meta, kinematics, {})


def test_the_trajectory_arrays_are_byte_identical(built, golden):
    """
    Description:
        Every numeric array of the trajectory - t, robot_q, robot_qd, loader_y, loader_z, gripper, vacuum and the world state,
        steps and segments - has the golden sha256, dtype and shape: the same bytes.
    """

    tr, _ = built
    arrays = {'t': tr.T, 'robot_q': tr.Robot_Q, 'robot_qd': tr.Robot_QD, 'loader_y': tr.Loader_Y, 'loader_z': tr.Loader_Z, 'gripper': tr.Gripper,
              'vacuum': tr.Vacuum, 'pcb_location': tr.PCB_Location, 'pcb_pose': tr.PCB_Pose, 'mg06_lid': tr.MG06_Lid, 'mg06_handle': tr.MG06_Handle,
              'jaw': tr.Jaw, 'step_index': tr.Step_Index, 'segment_id': tr.Segment_Id}
    assert set(arrays) == set(golden['arrays'])
    for name, value in arrays.items():
        g = golden['arrays'][name]
        assert (hashlib.sha256(np.ascontiguousarray(value).tobytes()).hexdigest(), str(value.dtype), list(value.shape)) == (g['sha256'], g['dtype'],
                                                                                                                         g['shape']), name


def test_the_cycle_time_events_and_segments_are_the_same(built, golden, names):
    """
    Description:
        The cycle time (68.852 s), the sample count and the events (times bit-exact, names) are the golden ones; the 69 executed
        segments' texts are the golden ones through the name map, in order, and the steps the golden steps renamed.
    """

    tr, _ = built
    assert tr.T[-1] == golden['cycle_time'] == 68.852 and len(tr.T) == golden['samples']
    assert [[float(t).hex(), e] for t, e in tr.Events] == golden['events']
    assert [names['commands'][i] for i in range(len(golden['segment_commands']))] == [[a, b] for a, b in zip(golden['segment_commands'],
                                                                                                             tr.Segment_Command)]
    assert [names['steps'][s] for s in golden['step_names']] == tr.Step_Names


def test_every_command_has_the_golden_values(program, golden, constants, simulation):
    """
    Description:
        The executed commands in order against the golden ones: the same kind (renamed), the joint targets bit for bit (float
        hex, rad), the MoveL speeds, the loader / gripper / wait / test durations and the loader and jaw values bit for bit; a
        vacuum on's Wait 0.3 is the WaitDI's simulated response.
    """

    executed = Executed_Commands(program)
    assert len(executed) == len(golden['commands']) == 69
    for (_, _, command, text), old in zip(executed, golden['commands']):
        assert command.Kind in CONST_KINDS[old['kind']], (old['text'], text)
        if old['kind'] in ('Movej', 'Movel'):
            assert [float(q).hex() for q in command.Joints] == old['joints_rad_hex'], text
        if old['kind'] == 'Movel':
            assert command.Speed.hex() == old['speed'], text
        if old['kind'] in ('Loader', 'Gripper', 'Wait', 'Test'):
            assert command.Duration.hex() == old['duration'], text
        if old['kind'] == 'Loader':
            assert (command.Axis, command.Position.hex()) == (old['axis'], old['value_hex']), text
        if old['kind'] == 'Gripper':
            assert command.Jaw.hex() == old['value_hex'], text


def _Mapped_Report(text, names):
    """
    Description:
        The golden report with the new names: the program, the step rows' first column (a row: a name, then its start time),
        the note's step and command, the
        handover point; whitespace squeezed (the columns' widths follow the names).
    """

    commands = dict(names['commands'])
    lines = []
    for line in text.splitlines():
        tokens = line.split()
        if len(tokens) > 3 and tokens[0] in names['steps'] and tokens[1].replace('.', '', 1).isdigit():
            tokens[0] = names['steps'][tokens[0]]
        line = ' '.join(tokens)
        if line.startswith('note: '):
            step, rest = line[len('note: '):].split(', ', 1)
            command, rest = rest.split(': ', 1)
            line = 'note: %s, %s: %s' % (names['steps'][step], commands[command], rest)
        for old, new in list(names['program'].items()) + [('loader_handover', names['points']['loader_handover'])]:
            line = line.replace(old, new)
        lines.append(line)
    return lines


def test_the_check_report_is_the_golden_one_renamed(built, program, cell_configuration, tools, names):
    """
    Description:
        The program check's report equals golden_check_report.txt line for line once the names are mapped (and whitespace
        squeezed): every distance, sample, time, peak, gap, event and verdict the same.
    """

    kinematics, meta, collision = tools
    tr, notes = built
    lines = [' '.join(line.split()) for line in Format_Check_Report(Check_Trajectory(tr, program, notes, kinematics, collision,
                                                                                     cell_configuration, meta),
                                                                    tr, cell_configuration)]
    with open(os.path.join(CONST_DATA, 'golden_check_report.txt'), encoding='utf-8') as handle:
        assert lines == _Mapped_Report(handle.read(), names)


def test_the_point_table_is_the_golden_keypoint_table_renamed(constants, simulation, cell_configuration, tools, names):
    """
    Description:
        The point check table equals golden_keypoint_table.txt row for row with the points renamed, except its 'check' and
        'stale' columns, which now measure something else by design (the Pose against FK; the Reference / PCB-hold consistency):
        family, limits, the closest pair, the allowed contacts and the verdict are the same, and the summary lines too.
    """

    kinematics, meta, collision = tools
    new = Format_Check_Table(Check_Keypoints(kinematics, collision, constants, simulation, cell_configuration, meta))
    with open(os.path.join(CONST_DATA, 'golden_keypoint_table.txt'), encoding='utf-8') as handle:
        old = handle.read().splitlines()
    assert len(new) == len(old)

    def Columns(line, rename):
        """The row without its check and stale columns (fixed widths), the name renamed."""
        name = line[:21].strip()
        return (rename.get(name, name), line[22:39], line[60:])

    points = names['points']
    for a, b in zip(old[1:24], new[1:24]):
        assert Columns(a, points) == Columns(b, {}), (a, b)
    assert [line.replace('via_mg06_high', points['via_mg06_high']) for line in old[24:]] == new[24:]
    assert old[0] == new[0]
