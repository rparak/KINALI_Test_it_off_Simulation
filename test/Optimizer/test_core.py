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
#   ../Optimizer/Core, ../Optimizer/Measure
from Optimizer.Core import Free_Transfers, Stale_Inputs, Free_Space_Cls, CONST_INPUTS
from Optimizer.Measure import Step_Bounds, Schedule_Differences
from Program.Core import Build_Program
from Program.Check import Check_Trajectory

"""
Description:
    The optimized program (FEATURES.md F9) on the shipped files: the FREE transfers, the committed
    Data/Optimized/Program_Optimized.yaml current and passing every check, faster than the taught program, the loader in
    parallel, the free space, the stale and schedule refusals, the waiting point.
"""


def test_the_free_transfers_merge_across_the_loader_jobs(optimized_built, program):
    """
    Description:
        The eight FREE segments make six transfers; S5 + S8 (the loader's insert and park between) and S14 + S17 (fetch and
        the return's two parts between, F15) are one each, with those jobs in parallel; the cups carry the PCB in To_Loader and To_Output.
    """

    transfers = Free_Transfers(program, optimized_built['optimization'], optimized_built['default'])
    assert [(t.Name, t.Start, t.End, t.Jobs, t.Carrying) for t in transfers] == [
        ('Approach_Input', 'P_HOME', 'P_INPUT_ABOVE', [], False), ('To_Loader', 'P_INPUT_CLEAR', 'P_LOADER_ABOVE', [], True),
        ('To_MG06', 'P_LOADER_ABOVE', 'P_MG06_FORK_START', ['J_INSERT', 'J_PARK'], False),
        ('To_Take', 'P_MG06_FORK_START', 'P_LOADER_ABOVE', ['J_FETCH', 'J_RETURN_Y', 'J_RETURN_Z'], False),
        ('To_Output', 'P_LOADER_ABOVE', 'P_OUTPUT_ABOVE', [], True), ('Home_End', 'P_OUTPUT_ABOVE', 'P_HOME', [], False)]
    assert transfers[2].Keys == ['S5_Home_After_Handover', 'S8_To_MG06'] and transfers[2].Job_Keys == ['S6_Loader_Insert', 'S7_Loader_Park']
    assert [t.Wait for t in transfers] == [None, None, None, 'P_LOADER_WAIT', None, None]
    assert [t.Exit for t in transfers] == [None, None, 'P_LOADER_WAIT', None, None, None]


def test_the_shipped_optimized_program_is_current_and_matches_its_schedule(optimized_built):
    """
    Description:
        B5: the committed optimized program was made from the current inputs (every hash of Optimized.Inputs, the planning
        model's included) and its build starts every segment at its recorded Schedule time.
    """

    assert set(optimized_built['optimized'].Optimized.Inputs) == set(CONST_INPUTS) | {'planning_model'}
    assert Stale_Inputs(optimized_built['optimized'], optimized_built['model']) == []
    assert Schedule_Differences(optimized_built['optimized'], optimized_built['trajectory']) == []


def test_a_changed_input_or_schedule_is_named(optimized_built):
    """
    Description:
        An optimized program whose recorded input hash differs is stale, naming the input; a recorded start that the build
        does not reproduce is named with both times.
    """

    stale = copy.deepcopy(optimized_built['optimized'])
    stale.Optimized.Inputs['cell'] = '0' * 64
    assert Stale_Inputs(stale, optimized_built['model']) == ['cell']
    stale.Optimized.Schedule['S6_Close_MG06'] += 0.5
    assert Schedule_Differences(stale, optimized_built['trajectory']) == ['S6_Close_MG06: recorded 19.384 s, built 18.884 s']


@pytest.mark.slow
def test_the_optimized_program_passes_every_check_and_is_faster(optimized_built, cell_configuration):
    """
    Description:
        The optimized program passes the F8 check (every sample exact on the planning model, the loader moving in parallel;
        the limits; the handover's no press; the points used) and its cycle is shorter than the taught one (59.148 s against
        70.404 s, measured 2026-10-25 after the insert's exit point, F16: 58.716 s before).
    """

    check = Check_Trajectory(optimized_built['trajectory'], optimized_built['optimized'], optimized_built['notes'], optimized_built['kinematics'],
                             optimized_built['collision'], cell_configuration, optimized_built['meta'])
    assert check.Verdict == 'PASS', [f for s in check.Steps for f in s.Failures]
    assert optimized_built['trajectory'].T[-1] < optimized_built['default'].T[-1]
    assert round(float(optimized_built['trajectory'].T[-1]), 3) == 59.148


def test_the_loader_moves_while_the_robot_moves(optimized_built):
    """
    Description:
        Parallel motion is real: samples where the robot's joints and the loader's axes both change exist in To_MG06 and
        To_Take, and none in the taught program.
    """

    for trajectory, expected in ((optimized_built['trajectory'], True), (optimized_built['default'], False)):
        robot = np.max(np.abs(np.diff(trajectory.Robot_Q, axis=0)), axis=1) > 1e-9
        loader = (np.abs(np.diff(trajectory.Loader_Y)) + np.abs(np.diff(trajectory.Loader_Z))) > 1e-12
        assert bool(np.any(robot & loader)) == expected


def test_the_free_space_keeps_the_clearance_and_the_tilt(optimized_built, cell_configuration, program):
    """
    Description:
        A transfer's validity: its end points valid at the first planner margin; the start of the carrying transfer tilted
        beyond a stricter limit is invalid; a configuration inside the input crate is invalid; the end point (the PCB 160 mm
        above the jaws) is valid with the loader clearance of Cell.yaml and invalid with 200 mm (F11).
    """

    transfers = Free_Transfers(program, optimized_built['optimization'], optimized_built['default'])
    carry = transfers[1]
    space = Free_Space_Cls(optimized_built['kinematics'], optimized_built['collision'], optimized_built['default'], carry,
                           cell_configuration.Clearance, optimized_built['optimization'].Carry_Max_Tilt,
                           cell_configuration.Loader_Free + cell_configuration.Tracking_Margin)
    end = np.array(optimized_built['optimized'].Constants.Points[carry.End].Joints)
    assert space.Valid(end, optimized_built['optimization'].Margins[0])
    wide = Free_Space_Cls(optimized_built['kinematics'], optimized_built['collision'], optimized_built['default'], carry,
                          cell_configuration.Clearance, optimized_built['optimization'].Carry_Max_Tilt, 0.2)
    assert not wide.Valid(end, 0.0)
    strict = Free_Space_Cls(optimized_built['kinematics'], optimized_built['collision'], optimized_built['default'], carry,
                            cell_configuration.Clearance, math.radians(5.0))
    assert not strict.Valid(end, 0.0)
    pick = np.array(optimized_built['optimized'].Constants.Points['P_INPUT_PICK'].Joints)
    assert not space.Valid(pick, 0.0)


def test_the_optimized_build_is_deterministic(optimized_built, cell_configuration):
    """
    Description:
        B5: the optimized program built twice gives the same bytes on every array (its splines timed again from the written
        waypoints).
    """

    again, _ = Build_Program(optimized_built['optimized'], cell_configuration, optimized_built['meta'], optimized_built['kinematics'], {})
    for channel in ('T', 'Robot_Q', 'Robot_QD', 'Loader_Y', 'Loader_Z', 'Gripper', 'Vacuum', 'PCB_Pose', 'MG06_Lid', 'MG06_Handle', 'Jaw',
                    'Segment_Id'):
        assert np.asarray(getattr(again, channel)).tobytes() == np.asarray(getattr(optimized_built['trajectory'], channel)).tobytes(), channel


def test_the_robot_leaves_mg06_at_once_and_waits_at_p_loader_wait(optimized_built, cell_configuration):
    """
    Description:
        G (F9.8): To_Take's first leg starts in the sample after Open_MG06 ends, together with J_FETCH (I6 still holds), so the
        robot never waits above MG06; it waits at P_LOADER_WAIT, which keeps more than Clearance.Loader_Free + Tracking_Margin
        (14 mm) from the loader and the PCB in its jaws - posed where its actuators put them (F11) - at every sample of J_FETCH and
        J_RETURN_Y and J_RETURN_Z with the robot standing there (29.68 mm, measured 2026-10-25); its last leg starts once
        J_RETURN_Y is done (F15: after its Loader.WaitDone, the loader's Y at home) and ends at P_LOADER_ABOVE before the loader
        is home, so Take_From_Loader starts when J_RETURN_Z ends.
    """

    from Optimizer.Measure import Job_Bounds
    from Program.Check import Robot_Loader_Distance, Loader_Actual_Trajectory
    tr, kinematics, collision = optimized_built['trajectory'], optimized_built['kinematics'], optimized_built['collision']
    texts = tr.Segment_Command
    leg = texts.index('Robot.MoveSpline(W_TO_TAKE_WAIT, v=0.6)')
    _, opened = Step_Bounds(tr, 'Open_MG06')
    assert int(tr.Segment_Start[leg]) == opened + 1 == Job_Bounds(tr, 'J_FETCH')[0]
    wait = np.array(optimized_built['optimized'].Constants.Points['P_LOADER_WAIT'].Joints)
    first, last = Job_Bounds(tr, 'J_FETCH')[0], Job_Bounds(tr, 'J_RETURN_Z')[1]
    standing = Loader_Actual_Trajectory(tr, kinematics.Model)
    standing.Robot_Q = np.array(tr.Robot_Q)
    standing.Robot_Q[first:last + 1] = wait
    distance, _, _ = Robot_Loader_Distance(standing, kinematics, collision, range(first, last + 1))
    assert distance >= cell_configuration.Loader_Free + cell_configuration.Tracking_Margin and round(1000 * distance, 2) == 29.68
    final = texts.index('Robot.MoveSpline(W_TO_TAKE, v=0.6)')
    take, _ = Step_Bounds(tr, 'Take_From_Loader')
    assert int(tr.Segment_Start[final]) == int(tr.Segment_End[texts.index('Loader.WaitDone(J_RETURN_Y)')])
    assert texts[final - 1] == 'Loader.Start(J_RETURN_Z)' and 'Wait(' not in ' '.join(texts[leg:final])
    assert int(tr.Segment_End[final]) <= Job_Bounds(tr, 'J_RETURN_Z')[1] and take == int(tr.Segment_End[texts.index('Loader.WaitDone(J_RETURN_Z)')])


def test_a_spline_that_comes_into_a_running_job_s_volume_waits_for_the_job(timed_take, optimized_built, cell_configuration, program):
    """
    Description:
        F15's optimizer rule: a spline that comes into the volume of a job it does not wait for runs after the next job's start
        - here W_TO_TAKE from slot 2 (after Loader.Start(J_RETURN_Y), the timer's) to slot 3 (after Loader.Start(J_RETURN_Z),
        so after Loader.WaitDone(J_RETURN_Y)), never after a Wait; past the last job's start it fails, naming the remedies.
    """

    from Optimizer.Core import Gate, Optimizer_Error_Cls
    from Program.Check import Gating_Faults
    tr = timed_take[0]
    fault, = Gating_Faults(tr, optimized_built['kinematics'], optimized_built['collision'], cell_configuration)
    assert fault.Kind == 'robot enters'
    take = next(t for t in Free_Transfers(program, optimized_built['optimization'], optimized_built['default']) if t.Name == 'To_Take')
    assert take.Jobs == ['J_FETCH', 'J_RETURN_Y', 'J_RETURN_Z']
    step = fault.Step
    assert Gate(tr, optimized_built['kinematics'], optimized_built['collision'], cell_configuration, step, take, 2, fault) == 3
    with pytest.raises(Optimizer_Error_Cls, match='split the job .* or gate it on an input from the loader'):
        Gate(tr, optimized_built['kinematics'], optimized_built['collision'], cell_configuration, step, take, 3, fault)



def test_the_insert_starts_after_the_robot_leaves_for_its_exit_point(optimized_built, prompt_insert, cell_configuration, program):
    """
    Description:
        F16: To_MG06 runs its leg to the exit point P_LOADER_WAIT (Optimization.yaml Exit_Points) before Loader.Start(J_INSERT),
        then its last leg while the job runs; the job's first loader move is the sample after the leg ends (the cycle 0.432 s
        longer than with the job started at once, 58.716 s). The mirror fault - the job started first, coming where the robot
        has been - cannot be gated by a later slot: Gate fails naming Exit_Points.
    """

    from Optimizer.Core import Gate, Optimizer_Error_Cls
    from Optimizer.Measure import Job_Bounds
    from Program.Check import Gating_Faults
    tr = optimized_built['trajectory']
    texts = tr.Segment_Command
    leg = texts.index('Robot.MoveSpline(W_TO_MG06_EXIT, v=0.6)')
    assert texts[leg + 1:leg + 4] == ['Loader.Start(J_INSERT)', 'Robot.MoveSpline(W_TO_MG06, v=0.6)', 'Loader.WaitDone(J_INSERT)']
    moved = next(i for i in range(Job_Bounds(tr, 'J_INSERT')[0], len(tr.T)) if tr.Loader_Z[i] != tr.Loader_Z[i - 1])
    assert moved == int(tr.Segment_End[leg]) and round(float(tr.T[moved]), 3) == 11.432
    fault, = Gating_Faults(prompt_insert[0], optimized_built['kinematics'], optimized_built['collision'], cell_configuration)
    to_mg06 = next(t for t in Free_Transfers(program, optimized_built['optimization'], optimized_built['default']) if t.Name == 'To_MG06')
    with pytest.raises(Optimizer_Error_Cls, match='J_INSERT comes where the robot has been .* Declare an exit point for To_MG06 in '
                                                  'Optimization.yaml Exit_Points'):
        Gate(prompt_insert[0], optimized_built['kinematics'], optimized_built['collision'], cell_configuration, fault.Step, to_mg06, 1, fault)
