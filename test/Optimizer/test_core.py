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
# File Name: ../test/Optimizer/test_core.py
## =========================================================================== ##

# Copy (Shallow and deep copy operations)
import copy
# Math (Mathematical functions)
import math
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test and what it works on
#   ../Optimizer/Core
from Optimizer.Core import (Free_Transfers, Compare_Locked, Interlocks, Carried_Tilt, Step_Bounds, Stale_Inputs, Schedule_Differences, Free_Space_Cls,
                            CONST_INPUTS)
from Configuration.Program_Parser import Load_Program, Load_Optimization, CONST_OPTIMIZED_PROGRAM_YAML
from Configuration.Core import Read_PCB_Meta
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
from Kinematics.Core import Robot_Kinematics_Cls
from Program.Core import Build_Program, Check_Trajectory

"""
Description:
    The optimized program (FEATURES.md F9) on the shipped files: the FREE transfers, the committed
    Data/Optimized/Program_Optimized.yaml current and passing every check, faster than the taught program, its interlocks,
    its LOCKED parts against the taught program, the carried PCB's tilt (B4), the stale and schedule refusals.
"""
@pytest.fixture(scope='module')
def built(cell_configuration, program, constants, simulation):
    model = Build_Planning_Model(cell_configuration)
    collision = Robot_Collision_Cls(model)
    kinematics = Robot_Kinematics_Cls(model, collision.Data)
    meta = Read_PCB_Meta()
    default, _ = Build_Program(program, cell_configuration, meta, kinematics, {})
    optimized = Load_Program(constants, simulation, cell_configuration, CONST_OPTIMIZED_PROGRAM_YAML)
    trajectory, notes = Build_Program(optimized, cell_configuration, meta, kinematics, {})
    return {'model': model, 'collision': collision, 'kinematics': kinematics, 'meta': meta, 'default': default, 'optimized': optimized,
            'trajectory': trajectory, 'notes': notes, 'optimization': Load_Optimization(program)}


def test_the_free_transfers_merge_across_the_loader_jobs(built, program):
    """
    Description:
        The eight FREE segments make six transfers; S5 + S8 (the loader's insert and park between) and S14 + S17 (fetch and
        return between) are one each, with those jobs in parallel; the cups carry the PCB in To_Loader and To_Output.
    """

    transfers = Free_Transfers(program, built['optimization'], built['default'])
    assert [(t.Name, t.Start, t.End, t.Jobs, t.Carrying) for t in transfers] == [
        ('Approach_Input', 'P_HOME', 'P_INPUT_ABOVE', [], False), ('To_Loader', 'P_INPUT_CLEAR', 'P_LOADER_ABOVE', [], True),
        ('To_MG06', 'P_LOADER_ABOVE', 'P_MG06_FORK_START', ['J_INSERT', 'J_PARK'], False),
        ('To_Take', 'P_MG06_FORK_START', 'P_LOADER_ABOVE', ['J_FETCH', 'J_RETURN'], False),
        ('To_Output', 'P_LOADER_ABOVE', 'P_OUTPUT_ABOVE', [], True), ('Home_End', 'P_OUTPUT_ABOVE', 'P_HOME', [], False)]
    assert transfers[2].Keys == ['S5_Home_After_Handover', 'S8_To_MG06'] and transfers[2].Job_Keys == ['S6_Loader_Insert', 'S7_Loader_Park']
    assert [t.Wait for t in transfers] == [None, None, None, 'P_LOADER_WAIT', None, None]


def test_the_shipped_optimized_program_is_current_and_matches_its_schedule(built):
    """
    Description:
        B5: the committed optimized program was made from the current inputs (every hash of Optimized.Inputs, the planning
        model's included) and its build starts every segment at its recorded Schedule time.
    """

    assert set(built['optimized'].Optimized.Inputs) == set(CONST_INPUTS) | {'planning_model'}
    assert Stale_Inputs(built['optimized'], built['model']) == []
    assert Schedule_Differences(built['optimized'], built['trajectory']) == []


def test_a_changed_input_or_schedule_is_named(built):
    """
    Description:
        An optimized program whose recorded input hash differs is stale, naming the input; a recorded start that the build
        does not reproduce is named with both times.
    """

    stale = copy.deepcopy(built['optimized'])
    stale.Optimized.Inputs['cell'] = '0' * 64
    assert Stale_Inputs(stale, built['model']) == ['cell']
    stale.Optimized.Schedule['S6_Close_MG06'] += 0.5
    assert Schedule_Differences(stale, built['trajectory']) == ['S6_Close_MG06: recorded 18.448 s, built 17.948 s']


@pytest.mark.slow
def test_the_optimized_program_passes_every_check_and_is_faster(built, cell_configuration):
    """
    Description:
        The optimized program passes the F8 check (every sample exact on the planning model, the loader moving in parallel;
        the limits; the handover's no press; the points used) and its cycle is shorter than the taught one (57.708 s against
        69.396 s, measured 2026-10-25 after F11's take fix and loader clearance).
    """

    check = Check_Trajectory(built['trajectory'], built['optimized'], built['notes'], built['kinematics'], built['collision'], cell_configuration,
                             built['meta'])
    assert check.Verdict == 'PASS', [f for s in check.Steps for f in s.Failures]
    assert built['trajectory'].T[-1] < built['default'].T[-1]
    assert round(float(built['trajectory'].T[-1]), 3) == 57.708


def test_the_interlocks_hold(built, cell_configuration):
    """
    Description:
        I1-I8 hold on the optimized trajectory, each with its times (I8: the robot at least the clearance from the loader
        wherever both may move).
    """

    found = Interlocks(built['trajectory'], built['kinematics'], built['collision'], cell_configuration, built['optimized'])
    assert [name for name, _, _ in found] == ['I%d' % k for k in range(1, 9)]
    assert all(ok for _, ok, _ in found), [text for _, ok, text in found if not ok]


def test_the_loader_moves_while_the_robot_moves(built):
    """
    Description:
        Parallel motion is real: samples where the robot's joints and the loader's axes both change exist in To_MG06 and
        To_Take, and none in the taught program.
    """

    for trajectory, expected in ((built['trajectory'], True), (built['default'], False)):
        robot = np.max(np.abs(np.diff(trajectory.Robot_Q, axis=0)), axis=1) > 1e-9
        loader = (np.abs(np.diff(trajectory.Loader_Y)) + np.abs(np.diff(trajectory.Loader_Z))) > 1e-12
        assert bool(np.any(robot & loader)) == expected


def _Locked(built, program):
    return {c.Name: c for c in Compare_Locked(built['default'], built['trajectory'], program,
                                              Free_Transfers(program, built['optimization'], built['default']))}


@pytest.mark.parametrize('part', ['S2_Pick_Input', 'S4_Handover', 'J_INSERT', 'J_PARK', 'S9_Close_MG06', 'S10_Leave_MG06', 'S11_Test',
                                  'S12_Return_MG06', 'S13_Open_MG06', 'J_FETCH', 'J_RETURN', 'S18_Take_From_Loader', 'S20_Place_Output',
                                  'End_Settle'])
def test_a_locked_part_is_byte_identical_from_its_start(built, program, part):
    """
    Description:
        Every LOCKED part of the taught program, from its own start: a robot segment on every channel, a parallel loader job
        on the loader's channels - the same bytes in the optimized program (its start time moves). The parts after a taught
        MoveJ included: a MoveJ ends exactly on its stored joints (owner's decision, F9.6).
    """

    found = _Locked(built, program)
    assert found[part].Identical


def test_the_carried_pcb_keeps_its_tilt_limit(built):
    """
    Description:
        B4: in the FREE transfers that carry the PCB its tilt from horizontal stays within Carry_Max_Tilt (20 deg): 10.44 deg
        (To_Loader, routed farther from the loader, F11) and 16.71 deg (To_Output; Carry_Max_Tilt 20 deg), measured 2026-10-25.
    """

    limit = built['optimization'].Carry_Max_Tilt
    tilts = {name: Carried_Tilt(built['trajectory'], *Step_Bounds(built['trajectory'], name)) for name in ('To_Loader', 'To_Output')}
    assert all(t <= limit for t in tilts.values())
    assert round(math.degrees(tilts['To_Loader']), 2) == 10.44 and round(math.degrees(tilts['To_Output']), 2) == 16.71


def test_the_free_space_keeps_the_clearance_and_the_tilt(built, cell_configuration, program):
    """
    Description:
        A transfer's validity: its end points valid at the first planner margin; the start of the carrying transfer tilted
        beyond a stricter limit is invalid; a configuration inside the input crate is invalid; the end point (the PCB 160 mm
        above the jaws) is valid with the loader clearance of Cell.yaml and invalid with 200 mm (F11).
    """

    transfers = Free_Transfers(program, built['optimization'], built['default'])
    carry = transfers[1]
    space = Free_Space_Cls(built['kinematics'], built['collision'], built['default'], carry, cell_configuration.Clearance,
                           built['optimization'].Carry_Max_Tilt, cell_configuration.Loader_Free + cell_configuration.Tracking_Margin)
    end = np.array(built['optimized'].Constants.Points[carry.End].Joints)
    assert space.Valid(end, built['optimization'].Margins[0])
    wide = Free_Space_Cls(built['kinematics'], built['collision'], built['default'], carry, cell_configuration.Clearance,
                          built['optimization'].Carry_Max_Tilt, 0.2)
    assert not wide.Valid(end, 0.0)
    strict = Free_Space_Cls(built['kinematics'], built['collision'], built['default'], carry, cell_configuration.Clearance, math.radians(5.0))
    assert not strict.Valid(end, 0.0)
    pick = np.array(built['optimized'].Constants.Points['P_INPUT_PICK'].Joints)
    assert not space.Valid(pick, 0.0)


def test_the_optimized_build_is_deterministic(built, cell_configuration):
    """
    Description:
        B5: the optimized program built twice gives the same bytes on every array (its splines timed again from the written
        waypoints).
    """

    again, _ = Build_Program(built['optimized'], cell_configuration, built['meta'], built['kinematics'], {})
    for channel in ('T', 'Robot_Q', 'Robot_QD', 'Loader_Y', 'Loader_Z', 'Gripper', 'Vacuum', 'PCB_Pose', 'MG06_Lid', 'MG06_Handle', 'Jaw',
                    'Segment_Id'):
        assert np.asarray(getattr(again, channel)).tobytes() == np.asarray(getattr(built['trajectory'], channel)).tobytes(), channel


def test_the_robot_leaves_mg06_at_once_and_waits_at_p_loader_wait(built, cell_configuration):
    """
    Description:
        G (F9.8): To_Take's first leg starts in the sample after Open_MG06 ends, together with J_FETCH (I6 still holds), so the
        robot never waits above MG06; it waits at P_LOADER_WAIT, which keeps more than Clearance.Loader_Free + Tracking_Margin
        (14 mm) from the loader and the PCB in its jaws - posed where its actuators put them (F11) - at every sample of J_FETCH and
        J_RETURN with the robot standing there (29.64 mm, measured 2026-10-25);
        its last leg ends at P_LOADER_ABOVE before the loader is home, so Take_From_Loader starts when J_RETURN ends.
    """

    from Optimizer.Core import Robot_Loader_Distance, Job_Bounds, Loader_Actual_Trajectory
    import mujoco
    tr, kinematics, collision = built['trajectory'], built['kinematics'], built['collision']
    texts = tr.Segment_Command
    leg = texts.index('Robot.MoveSpline(W_TO_TAKE_WAIT, v=0.6)')
    _, opened = Step_Bounds(tr, 'Open_MG06')
    assert int(tr.Segment_Start[leg]) == opened + 1 == Job_Bounds(tr, 'J_FETCH')[0]
    wait = np.array(built['optimized'].Constants.Points['P_LOADER_WAIT'].Joints)
    first, last = Job_Bounds(tr, 'J_FETCH')[0], Job_Bounds(tr, 'J_RETURN')[1]
    standing = Loader_Actual_Trajectory(tr, kinematics.Model)
    standing.Robot_Q = np.array(tr.Robot_Q)
    standing.Robot_Q[first:last + 1] = wait
    distance, _, _ = Robot_Loader_Distance(standing, kinematics, collision, range(first, last + 1))
    assert distance >= cell_configuration.Loader_Free + cell_configuration.Tracking_Margin and round(1000 * distance, 2) == 29.64
    final = texts.index('Robot.MoveSpline(W_TO_TAKE, v=0.6)')
    take, _ = Step_Bounds(tr, 'Take_From_Loader')
    assert int(tr.Segment_End[final]) <= Job_Bounds(tr, 'J_RETURN')[1] and take == int(tr.Segment_End[texts.index('Loader.WaitDone(J_RETURN)')])


def test_a_stop_is_a_tenth_of_a_second_at_rest_after_a_motion(built):
    """
    Description:
        E: a stop is every joint speed below 1e-3 rad/s (the sample's change over dt) for at least 0.1 s, after a motion: on a
        made-up motion - at rest 0.2 s first (not a stop), a move, 0.096 s at rest (too short), a move, 0.1 s at rest, a move
        at 0.5e-3 rad/s for 0.2 s (slower than the threshold: at rest), a move, 0.2 s at 5e-3 rad/s (a motion), 0.3 s at rest to
        the end - there are three, 0.6 s in all.
    """

    from Optimizer.Core import Measure
    q = [np.zeros(6)]

    def Hold(seconds, speed=0.0):
        for _ in range(int(round(seconds / 0.004))):
            q.append(q[-1] + np.r_[speed * 0.004, np.zeros(5)])

    Hold(0.2), Hold(0.04, 0.5), Hold(0.096), Hold(0.04, 0.5), Hold(0.1), Hold(0.04, 0.5), Hold(0.2, 0.5e-3), Hold(0.04, 0.5), Hold(0.2, 5e-3)
    Hold(0.3)
    zeros = np.zeros(len(q))
    metrics = Measure(np.array(q), 0.004, built['kinematics'], zeros, zeros)
    assert metrics.Stops == 3 and metrics.Rest_Time == pytest.approx(0.6, abs=1e-9)
