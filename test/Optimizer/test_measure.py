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
# File Name: ../test/Optimizer/test_measure.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test and what it works on
#   ../Optimizer/Measure, ../Optimizer/Core
from Optimizer.Core import Free_Transfers
from Optimizer.Measure import Compare_Locked, Interlocks, Carried_Tilt, Step_Bounds

"""
Description:
    What the optimized program is measured by (SPEC §7.11), on the shipped files: its interlocks, its LOCKED parts against the
    taught program, the carried PCB's tilt (B4), the stops of the comparison.
"""


def test_the_interlocks_hold(optimized_built, cell_configuration):
    """
    Description:
        I1-I10 hold on the optimized trajectory, each with its times (I8: the robot at least the clearance from the loader
        wherever both may move; I9: the take's spline after J_RETURN_Y is done; I10: J_INSERT after the robot's motion out of
        the handover).
    """

    found = Interlocks(optimized_built['trajectory'], optimized_built['kinematics'], optimized_built['collision'], cell_configuration,
                       optimized_built['optimized'])
    assert [name for name, _, _ in found] == ['I%d' % k for k in range(1, 11)]
    assert all(ok for _, ok, _ in found), [text for _, ok, text in found if not ok]


def test_i9_fails_for_a_take_gated_by_a_timer(timed_take, optimized_built, cell_configuration, program):
    """
    Description:
        The forced failure of F15: with the take's spline after Loader.Start(J_RETURN_Y) and Wait(1.648), I9 fails - the motion
        to the take comes before the job's WaitDone and starts before its last loader move - while I8 still holds at the
        nominal timing (19.39 mm): the timer passes the distance check, only I9 and the gating rule see that it rests on the
        loader's timing. The taught program keeps I9 by its Loader.Run lines.
    """

    found = {name: (ok, text) for name, ok, text in Interlocks(timed_take[0], optimized_built['kinematics'], optimized_built['collision'],
                                                               cell_configuration, timed_take[1])}
    assert not found['I9'][0] and '`Robot.MoveSpline(W_TO_TAKE, v=0.6)`' in found['I9'][1]
    assert found['I8'][0] and 'closest 19.39 mm' in found['I8'][1]
    taught = {name: ok for name, ok, _ in Interlocks(optimized_built['default'], optimized_built['kinematics'], optimized_built['collision'],
                                                     cell_configuration, program)}
    assert taught['I9']


def test_i10_fails_for_an_insert_started_before_the_robot_leaves(prompt_insert, optimized_built, cell_configuration, program):
    """
    Description:
        The forced failure of F16: with Loader.Start(J_INSERT) before the robot's leg to its exit point, I10 fails - the job
        starts before the robot's motion out of the handover and moves before that motion ends - while I8 still holds at the
        nominal timing: only I10 and the gating rule see that it rests on the robot's timing. The taught program keeps I10 by
        its order (the robot home, then Loader.Run(J_INSERT)).
    """

    found = {name: (ok, text) for name, ok, text in Interlocks(prompt_insert[0], optimized_built['kinematics'], optimized_built['collision'],
                                                               cell_configuration, prompt_insert[1])}
    assert not found['I10'][0] and '`Robot.MoveSpline(W_TO_MG06_EXIT, v=0.6)`' in found['I10'][1]
    assert found['I8'][0]
    taught = {name: (ok, text) for name, ok, text in Interlocks(optimized_built['default'], optimized_built['kinematics'],
                                                                optimized_built['collision'], cell_configuration, program)}
    assert taught['I10'][0] and '`Loader.Run(J_INSERT)' in taught['I10'][1]


def _Locked(optimized_built, program):
    return {c.Name: c for c in Compare_Locked(optimized_built['default'], optimized_built['trajectory'], program,
                                              Free_Transfers(program, optimized_built['optimization'], optimized_built['default']))}


@pytest.mark.parametrize('part', ['S2_Pick_Input', 'S4_Handover', 'J_INSERT', 'J_PARK', 'S9_Close_MG06', 'S10_Leave_MG06', 'S11_Test',
                                  'S12_Return_MG06', 'S13_Open_MG06', 'J_FETCH', 'J_RETURN_Y', 'J_RETURN_Z', 'S18_Take_From_Loader',
                                  'S20_Place_Output', 'End_Settle'])
def test_a_locked_part_is_byte_identical_from_its_start(optimized_built, program, part):
    """
    Description:
        Every LOCKED part of the taught program, from its own start: a robot segment on every channel, a parallel loader job
        on the loader's channels - the same bytes in the optimized program (its start time moves). The parts after a taught
        MoveJ included: a MoveJ ends exactly on its stored joints (owner's decision, F9.6).
    """

    found = _Locked(optimized_built, program)
    assert found[part].Identical


def test_the_carried_pcb_keeps_its_tilt_limit(optimized_built):
    """
    Description:
        B4: in the FREE transfers that carry the PCB its tilt from horizontal stays within Carry_Max_Tilt (20 deg): 10.44 deg
        (To_Loader, routed farther from the loader, F11) and 16.70 deg (To_Output; Carry_Max_Tilt 20 deg), measured 2026-10-25
        (after the handover re-teach, B work item 2).
    """

    limit = optimized_built['optimization'].Carry_Max_Tilt
    tilts = {name: Carried_Tilt(optimized_built['trajectory'], *Step_Bounds(optimized_built['trajectory'], name))
             for name in ('To_Loader', 'To_Output')}
    assert all(t <= limit for t in tilts.values())
    assert round(math.degrees(tilts['To_Loader']), 2) == 10.44 and round(math.degrees(tilts['To_Output']), 2) == 16.70
