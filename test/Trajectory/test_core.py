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
# File Name: ../test/Trajectory/test_core.py
## =========================================================================== ##

# System (Default)
import json
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test
#   ../Trajectory/Core
from Trajectory.Core import Trajectory_Str, Write_Trajectory, Read_Trajectory, Trajectory_Error_Cls, CONST_DT

"""
Description:
    Initialization of constants.

    The trajectory file format (SPEC §9, FEATURES.md F8) on a small synthetic trajectory: five samples, two steps, two
    segments, the values written out here.
"""


def _Small():
    """
    Description:
        A five-sample trajectory: a joint ramp in step 'a' (segment 0), a hold in step 'b' (segment 1), vacuum on at 8 ms.
    """

    n = 5
    q = np.outer(np.arange(n), np.arange(1, 7)) * 0.001
    return Trajectory_Str(T=np.arange(n) * CONST_DT, Robot_Q=q, Robot_QD=np.gradient(q, CONST_DT, axis=0), Loader_Y=np.full(n, -0.1),
                          Loader_Z=np.zeros(n), Gripper=np.full(n, 0.035), Vacuum=np.array([0, 0, 1, 1, 1], np.int8),
                          Step_Index=np.array([0, 0, 0, 1, 1], np.int32), Segment_Id=np.array([0, 0, 0, 1, 1], np.int32),
                          PCB_Location=np.array([0, 0, 1, 1, 1], np.int8), PCB_Pose=np.tile([0.1, -0.7, 0.005, 1, 0, 0, 0], (n, 1)),
                          MG06_Lid=np.full(n, -0.6974), MG06_Handle=np.zeros(n), Jaw=np.full(n, 0.035), Step_Names=['a', 'b'],
                          Segment_Step=np.array([0, 1], np.int32), Segment_Command=['Movej home', 'Wait 0.008'],
                          Segment_Start=np.array([0, 3], np.int32), Segment_End=np.array([3, 5], np.int32), Events=[(0.008, 'vacuum_on')],
                          Meta={'program': 'small', 'units': {'length': 'm', 'angle': 'rad', 'time': 's'}})


def test_a_trajectory_round_trips(tmp_path):
    """
    Description:
        Written and read back, every array, the names, the events and the metadata (plus the format version and the 4 ms
        period) are the same.
    """

    small = _Small()
    npz, csv = Write_Trajectory(small, str(tmp_path / 'small.npz'))
    back = Read_Trajectory(npz)
    for name in ('T', 'Robot_Q', 'Robot_QD', 'Loader_Y', 'Loader_Z', 'Gripper', 'Vacuum', 'Step_Index', 'Segment_Id', 'PCB_Location', 'PCB_Pose',
                 'MG06_Lid', 'MG06_Handle', 'Jaw', 'Segment_Step', 'Segment_Start', 'Segment_End'):
        assert np.array_equal(getattr(back, name), getattr(small, name)), name
    assert (back.Step_Names, back.Segment_Command, back.Events) == (['a', 'b'], ['Movej home', 'Wait 0.008'], [(0.008, 'vacuum_on')])
    assert back.Meta == {'program': 'small', 'units': {'length': 'm', 'angle': 'rad', 'time': 's'}, 'format_version': 1, 'dt': 0.004}
    assert csv == str(tmp_path / 'small.csv')


def test_the_same_trajectory_gives_the_same_bytes(tmp_path):
    """
    Description:
        Writing the same trajectory twice, at different times, gives byte-identical .npz and .csv files.
    """

    first = [open(p, 'rb').read() for p in Write_Trajectory(_Small(), str(tmp_path / 'one' / 'x.npz'))]
    second = [open(p, 'rb').read() for p in Write_Trajectory(_Small(), str(tmp_path / 'two' / 'x.npz'))]
    assert first == second


def test_the_csv_has_the_per_sample_columns(tmp_path):
    """
    Description:
        The CSV export: a header of t, q1..q6, qd1..qd6, the loader and gripper commands, vacuum, the step (index and
        label), the segment, the PCB location by name, MG06's lid and handle, the jaws; one row per sample, fixed 9 decimals.
    """

    _, csv = Write_Trajectory(_Small(), str(tmp_path / 'small.npz'))
    lines = open(csv, encoding='utf-8').read().splitlines()
    assert lines[0] == ('t,q1,q2,q3,q4,q5,q6,qd1,qd2,qd3,qd4,qd5,qd6,loader_y,loader_z,gripper,vacuum,step_index,step_label,segment_id,'
                        'pcb_location,mg06_lid,mg06_handle,jaw')
    assert len(lines) == 6
    row = lines[3].split(',')
    assert row[0] == '0.008000000' and row[1] == '0.002000000'
    assert row[16:22] == ['1', '0', 'a', '0', 'cups', '-0.697400000'] and lines[4].split(',')[18] == 'b'


def test_a_wrong_or_broken_file_is_refused(tmp_path):
    """
    Description:
        A missing file, another format version, a missing array and an array of the wrong length are refused with a
        Trajectory_Error_Cls naming the file and the rebuild command.
    """

    with pytest.raises(Trajectory_Error_Cls, match='cannot be read'):
        Read_Trajectory(str(tmp_path / 'none.npz'))
    npz, _ = Write_Trajectory(_Small(), str(tmp_path / 'small.npz'))
    data = dict(np.load(npz))
    for change, text in ((lambda d: d.update(meta=np.array(json.dumps({'format_version': 0}))), 'format version 0'),
                         (lambda d: d.pop('robot_qd'), 'lacks robot_qd'),
                         (lambda d: d.update(jaw=np.zeros(4)), 'wrong shape (jaw)')):
        broken = dict(data)
        change(broken)
        path = str(tmp_path / 'broken.npz')
        np.savez(path, **broken)
        with pytest.raises(Trajectory_Error_Cls, match=r'build_program\.py|Rebuild') as error:
            Read_Trajectory(path)
        assert text in str(error.value)
