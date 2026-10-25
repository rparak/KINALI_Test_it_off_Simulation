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
        I1-I8 hold on the optimized trajectory, each with its times (I8: the robot at least the clearance from the loader
        wherever both may move).
    """

    found = Interlocks(optimized_built['trajectory'], optimized_built['kinematics'], optimized_built['collision'], cell_configuration,
                       optimized_built['optimized'])
    assert [name for name, _, _ in found] == ['I%d' % k for k in range(1, 9)]
    assert all(ok for _, ok, _ in found), [text for _, ok, text in found if not ok]


def _Locked(optimized_built, program):
    return {c.Name: c for c in Compare_Locked(optimized_built['default'], optimized_built['trajectory'], program,
                                              Free_Transfers(program, optimized_built['optimization'], optimized_built['default']))}


@pytest.mark.parametrize('part', ['S2_Pick_Input', 'S4_Handover', 'J_INSERT', 'J_PARK', 'S9_Close_MG06', 'S10_Leave_MG06', 'S11_Test',
                                  'S12_Return_MG06', 'S13_Open_MG06', 'J_FETCH', 'J_RETURN', 'S18_Take_From_Loader', 'S20_Place_Output',
                                  'End_Settle'])
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


def test_a_stop_is_a_tenth_of_a_second_at_rest_after_a_motion(optimized_built):
    """
    Description:
        E: a stop is every joint speed below 1e-3 rad/s (the sample's change over dt) for at least 0.1 s, after a motion: on a
        made-up motion - at rest 0.2 s first (not a stop), a move, 0.096 s at rest (too short), a move, 0.1 s at rest, a move
        at 0.5e-3 rad/s for 0.2 s (slower than the threshold: at rest), a move, 0.2 s at 5e-3 rad/s (a motion), 0.3 s at rest to
        the end - there are three, 0.6 s in all.
    """

    from Optimizer.Measure import Measure
    q = [np.zeros(6)]

    def Hold(seconds, speed=0.0):
        for _ in range(int(round(seconds / 0.004))):
            q.append(q[-1] + np.r_[speed * 0.004, np.zeros(5)])

    Hold(0.2), Hold(0.04, 0.5), Hold(0.096), Hold(0.04, 0.5), Hold(0.1), Hold(0.04, 0.5), Hold(0.2, 0.5e-3), Hold(0.04, 0.5), Hold(0.2, 5e-3)
    Hold(0.3)
    zeros = np.zeros(len(q))
    metrics = Measure(np.array(q), 0.004, optimized_built['kinematics'], zeros, zeros)
    assert metrics.Stops == 3 and metrics.Rest_Time == pytest.approx(0.6, abs=1e-9)
