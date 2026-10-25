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
# File Name: ../test/Program/test_core.py
## =========================================================================== ##

# System (Default)
import os
import math
import copy
import dataclasses
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test
#   ../Program/Core
from Program.Core import (Build_Program, Check_Trajectory, Format_Check_Report, Jaw_Posing_Faults, Movej_Duration, Program_Error_Cls, Pose_Sample,
                          Validate_Constants, Loader_Actual, Loader_Actual_Trajectory)
#   ../Configuration/Core, ../Configuration/Program_Parser, ../Collision/Core, ../Kinematics/Core, ../Trajectory/Core,
#   ../Keypoints/Core
from Configuration.Core import Read_PCB_Meta, CONST_PCB_LOCATIONS, CONST_FORK_PATH
from Configuration.Program_Parser import (Program_Str, Segment_Str, Parse_Commands, Load_Constants, Load_Simulation, Load_Optimization, Free_Steps,
                                          CONST_CONSTANTS_YAML)
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
from Kinematics.Core import Robot_Kinematics_Cls, PCB_For_TCP
from Trajectory.Core import Write_Trajectory
from Keypoints.Core import Resolve_Keypoint

"""
Description:
    Initialization of constants.

    The program build and check (FEATURES.md F8, F10): the motion laws, the recorded fork path, the world state, the check,
    the constants against the model. Expected values come from the legacy planner's rules (git tag
    legacy-planner-reference), the reference plan and SPEC §4, written out here.
"""
# The legacy MoveJ rule's numbers: 60 % of 220 / 230 deg/s, 3 / 6 rad/s^2, at least 0.5 s (SPEC §4.4, §4.10).
#   Unit [Vector<float> 1x6] (rad/s), (rad/s^2), [float] (s)
CONST_V_MAX = 0.6 * np.radians([220, 220, 220, 220, 230, 230])
CONST_A_MAX = np.array([3.0, 3.0, 3.0, 6.0, 6.0, 6.0])
CONST_T_MIN = 0.5


@pytest.fixture(scope='module')
def tools(cell_configuration):
    model = Build_Planning_Model(cell_configuration)
    collision = Robot_Collision_Cls(model)
    return Robot_Kinematics_Cls(model, collision.Data), Read_PCB_Meta(), collision


@pytest.fixture(scope='module')
def built(program, cell_configuration, tools):
    kinematics, meta, _ = tools
    return Build_Program(program, cell_configuration, meta, kinematics, {})


@pytest.fixture(scope='module')
def checked(built, program, cell_configuration, tools):
    kinematics, meta, collision = tools
    free = Free_Steps(program, Load_Optimization(program))
    return Check_Trajectory(built[0], program, built[1], kinematics, collision, cell_configuration, meta, free)


def _Probe(lines, constants, simulation, cell, start='P_HOME', pcb='input_crate', allow=(), loader_jobs=None, tester_jobs=None):
    """
    Description:
        A one-segment program of the given robot lines, from a start point (loader home, gripper open, vacuum off, no end settle).
    """

    probe_simulation = dataclasses.replace(simulation, Start_Robot=start, Start_PCB=pcb, End_Settle=0.0)
    commands = Parse_Commands(lines, 'S1_Probe', constants, probe_simulation, cell, loader_jobs, tester_jobs)
    return Program_Str(Name='probe', Segments=[Segment_Str(Key='S1_Probe', Name='Probe', Number=1, Commands=commands, Allow_Contact=list(allow))],
                       Constants=constants, Simulation=probe_simulation)


def _Check(program, cell, tools):
    """
    Description:
        Build and check a program.
    """

    kinematics, meta, collision = tools
    trajectory, notes = Build_Program(program, cell, meta, kinematics, {})
    return Check_Trajectory(trajectory, program, notes, kinematics, collision, cell, meta)


def _Segment(trajectory, text):
    i = trajectory.Segment_Command.index(text)
    return trajectory.Segment_Start[i], trajectory.Segment_End[i]


def test_movej_follows_the_legacy_rule(built, constants, cell_configuration):
    """
    Description:
        MoveJ P_HOME -> P_INPUT_ABOVE: T = max(0.5, max 1.875 |dq| / v_max, max sqrt(5.774 |dq| / a_max)) with the legacy
        numbers, round(T / 4 ms) samples; min-jerk: at rest at both ends, the analytic speed's peak 1.875 dq / T. (Its samples
        were the legacy plan's first ones to 1.7e-8 rad, checked until step 6 retired the plan: git tag legacy-cycle-reference.)
    """

    q0, q1 = np.array(constants.Points['P_HOME'].Joints), np.array(constants.Points['P_INPUT_ABOVE'].Joints)
    dq = np.abs(q1 - q0)
    expected = max(CONST_T_MIN, np.max(1.875 * dq / CONST_V_MAX), np.max(np.sqrt(5.774 * dq / CONST_A_MAX)))
    assert Movej_Duration(q0, q1, cell_configuration) == pytest.approx(expected, rel=1e-12)
    trajectory, _ = built
    first, last = _Segment(trajectory, 'Robot.MoveJ(P_INPUT_ABOVE)')
    n = int(round(expected / 0.004))
    assert (first, last) == (0, n + 1)
    qd = trajectory.Robot_QD[first:last]
    assert np.all(qd[0] == 0) and np.all(qd[-1] == 0)
    assert np.max(np.abs(qd), axis=0) == pytest.approx(1.875 * (q1 - q0).__abs__() / (n * 0.004), rel=1e-3)


def test_a_movej_with_v_runs_at_that_fraction_and_is_checked_at_it(constants, simulation, cell_configuration, tools):
    """
    Description:
        MoveJ(P_INPUT_ABOVE, v=0.3): the legacy rule with v_max = 0.3 x the datasheet speeds (twice the default's 0.6 time on
        the speed term), the samples measured by the clearance check, and its speed held to 0.3 / 0.6 = 0.5 of the Cell.yaml
        limits; the same move faster than its v= (built at 0.6, checked as v=0.3) fails.
    """

    kinematics, meta, collision = tools
    q0, q1 = np.array(constants.Points['P_HOME'].Joints), np.array(constants.Points['P_INPUT_ABOVE'].Joints)
    dq = np.abs(q1 - q0)
    expected = max(CONST_T_MIN, np.max(1.875 * dq / (0.3 * np.radians([220, 220, 220, 220, 230, 230]))), np.max(np.sqrt(5.774 * dq / CONST_A_MAX)))
    slow = _Probe(['Robot.MoveJ(P_INPUT_ABOVE, v=0.3)'], constants, simulation, cell_configuration)
    trajectory, notes = Build_Program(slow, cell_configuration, meta, kinematics, {})
    assert len(trajectory.T) - 1 == int(round(expected / 0.004)) and expected > Movej_Duration(q0, q1, cell_configuration)
    result = Check_Trajectory(trajectory, slow, notes, kinematics, collision, cell_configuration, meta)
    assert result.Verdict == 'PASS' and result.Steps[0].Checked > 100 and result.Steps[0].Peak_Speed <= 0.5 * 1.002
    fast = _Probe(['Robot.MoveJ(P_INPUT_ABOVE)'], constants, simulation, cell_configuration)
    trajectory, notes = Build_Program(fast, cell_configuration, meta, kinematics, {})
    claimed = _Probe(['Robot.MoveJ(P_INPUT_ABOVE, v=0.3)'], constants, simulation, cell_configuration)
    result = Check_Trajectory(trajectory, claimed, notes, kinematics, collision, cell_configuration, meta)
    assert any(f.startswith('Robot.MoveJ(P_INPUT_ABOVE, v=0.3): peak joint speed') for f in result.Steps[0].Failures)


def test_movel_is_straight_and_ends_on_the_stored_joints(built, constants, tools):
    """
    Description:
        MoveL P_INPUT_APPROACH -> P_INPUT_PICK (15 mm at 30 mm/s): every TCP sample within 0.01 mm of the line, the orientation
        kept, the last sample exactly the point's joints, T = 1.875 L / speed (0.94 s), the TCP's peak speed at most 30 mm/s.
    """

    kinematics, _, _ = tools
    trajectory, _ = built
    first, last = _Segment(trajectory, 'Robot.MoveL(P_INPUT_PICK, v=0.03)')
    p0, R0 = kinematics.Site_Pose(trajectory.Robot_Q[first - 1])
    p1, _ = kinematics.Site_Pose(np.array(constants.Points['P_INPUT_PICK'].Joints))
    axis = (p1 - p0) / np.linalg.norm(p1 - p0)
    points = [kinematics.Site_Pose(q) for q in trajectory.Robot_Q[first:last]]
    off_line = max(np.linalg.norm((p - p0) - ((p - p0) @ axis) * axis) for p, _ in points)
    tilt = max(np.arccos(np.clip((np.trace(R0.T @ R) - 1) / 2, -1, 1)) for _, R in points)
    assert off_line <= 1e-5 and tilt <= 1e-6
    assert np.array_equal(trajectory.Robot_Q[last - 1], constants.Points['P_INPUT_PICK'].Joints)
    assert (last - first) * 0.004 == pytest.approx(1.875 * np.linalg.norm(p1 - p0) / 0.03, abs=0.004)
    speeds = [np.linalg.norm(b[0] - a[0]) / 0.004 for a, b in zip(points, points[1:])]
    assert max(speeds) <= 0.03 * 1.01


def test_a_movel_cannot_switch_the_ik_family(constants, simulation, cell_configuration, tools):
    """
    Description:
        A MoveL from P_HOME to P_MG06_ABOVE (wrist up to wrist down) fails in the build too, naming both wrist_2 angles: a line
        cannot switch families (SPEC §4.7).
    """

    kinematics, meta, _ = tools
    with pytest.raises(Program_Error_Cls, match='a MoveL cannot switch families'):
        Build_Program(_Probe(['Robot.MoveL(P_MG06_ABOVE, v=0.2)'], constants, simulation, cell_configuration), cell_configuration, meta,
                      kinematics, {})


def test_a_movel_over_a_joint_limit_is_stretched_and_noted(constants, simulation, cell_configuration, tools):
    """
    Description:
        MoveL P_HOME -> P_INPUT_ABOVE at 1 m/s (365 mm) would exceed a joint's speed or acceleration limit: it is stretched
        (x1.985, measured 2026-10-24) and noted, and the sampled joint speeds and accelerations then stay within the limits.
    """

    kinematics, meta, _ = tools
    trajectory, notes = Build_Program(_Probe(['Robot.MoveL(P_INPUT_ABOVE, v=1.0)'], constants, simulation, cell_configuration), cell_configuration,
                                      meta, kinematics, {})
    assert len(notes) == 1 and notes[0].Text.startswith('stretched x1.985')
    q = np.vstack([trajectory.Robot_Q, trajectory.Robot_Q[-1:]])
    qd = np.diff(q, axis=0) / 0.004
    assert np.all(np.abs(qd) <= CONST_V_MAX) and np.all(np.abs(np.diff(qd, axis=0) / 0.004) <= CONST_A_MAX)


def test_the_fork_paths_follow_the_recorded_rows_and_join_rule(built, constants):
    """
    Description:
        PATH_FORK_CLOSE follows rows 1-305 (5.684 s after row 0) from P_MG06_FORK_START and ends exactly on P_MG06_CLOSE_END;
        PATH_FORK_OPEN starts at P_MG06_OPEN_JOIN and follows the rows after row 663 (the first OPEN row with handle <= 28.45
        deg: 28.32 deg) to P_MG06_FORK_START; MG06 is the recorded state during the paths, and the opening path's first row
        moves (the noted velocity step, 39.1 mm/s).
    """

    trajectory, notes = built
    log = np.load(CONST_FORK_PATH)['log']
    first, last = _Segment(trajectory, 'Robot.MovePath(PATH_FORK_CLOSE)')
    assert (last - first) * 0.004 == pytest.approx(log[305, 0] - log[0, 0], abs=0.002)
    assert np.array_equal(trajectory.Robot_Q[first - 1], constants.Points['P_MG06_FORK_START'].Joints)
    assert np.array_equal(trajectory.Robot_Q[last - 1], constants.Points['P_MG06_CLOSE_END'].Joints)
    assert trajectory.MG06_Handle[last - 1] == pytest.approx(log[305, 3], abs=1e-12)
    first, last = _Segment(trajectory, 'Robot.MovePath(PATH_FORK_OPEN)')
    assert math.degrees(log[663, 3]) == pytest.approx(28.32, abs=0.01) and math.degrees(log[662, 3]) > 28.45
    assert (last - first) * 0.004 == pytest.approx(log[905, 0] - log[663, 0], abs=0.002)
    assert np.array_equal(trajectory.Robot_Q[first - 1], constants.Points['P_MG06_OPEN_JOIN'].Joints)
    assert np.array_equal(trajectory.Robot_Q[last - 1], constants.Points['P_MG06_FORK_START'].Joints)
    assert [n.Command for n in notes] == ['Robot.MovePath(PATH_FORK_OPEN)'] and '39.1 mm/s' in notes[0].Text


def test_the_pcb_moves_through_the_cycle_by_the_rules(built, cell_configuration, constants, simulation):
    """
    Description:
        The PCB's locations in order: input_crate, cups (vacuum on at 3.276 s, as the reference plan), jaws (vacuum off at
        9.912 s), mg06 (the jaws open at MG06), jaws (clamped at MG06), cups (vacuum on at the take), output_crate (vacuum off)
        - at its rest pose on the floor; the jaws stand at G_CLAMP's Jaw_Actual while the PCB is between them, the command at
        G_CLAMP's Jaw; MG06 is settled closed during the test and open at the end.
    """

    trajectory, _ = built
    locations = [CONST_PCB_LOCATIONS[i] for i in trajectory.PCB_Location]
    order = [locations[0]] + [b for a, b in zip(locations, locations[1:]) if a != b]
    assert order == ['input_crate', 'cups', 'jaws', 'mg06', 'jaws', 'cups', 'output_crate']
    assert [e for e in trajectory.Events if e[1].startswith('vacuum')][:2] == [(3.276, 'vacuum_on'), (9.912, 'vacuum_off')]
    assert trajectory.PCB_Pose[-1][:3] == pytest.approx([0.40620, -0.75160, 0.0049], abs=1e-9)
    clamp = trajectory.Segment_Command.index('Loader.Run(J_CLAMP) > Gripper(G_CLAMP, t=0.35)')
    assert trajectory.Jaw[trajectory.Segment_End[clamp] - 1] == simulation.Jaw_Actual['G_CLAMP']
    assert trajectory.Gripper[trajectory.Segment_End[clamp] - 1] == constants.Gripper['G_CLAMP'].Jaw
    during = trajectory.Step_Index == trajectory.Step_Names.index('S11_Test')
    assert np.all(trajectory.MG06_Lid[during] == cell_configuration.Closed_Lid)
    assert np.all(trajectory.MG06_Handle[during] == cell_configuration.Closed_Handle)
    assert trajectory.MG06_Lid[-1] < -0.69 and trajectory.MG06_Handle[-1] == 0.0


def test_the_posed_jaws_are_the_command_limited_by_the_part(built, simulation):
    """
    Description:
        Item 10 (owner, 2026-10-25): at all four gripper operations (the handover clamp, the release in MG06, the re-grip, the
        release at the take) the posed jaw gap is monotonic in the command's direction and never closer than grip with the PCB in
        the jaws; the release in MG06 opens from grip (17.75 mm), not from the clamp command (5.75 mm).
    """

    trajectory, _ = built
    grip = simulation.Jaw_Actual['G_CLAMP']
    assert Jaw_Posing_Faults(trajectory, grip) == []
    moving = np.flatnonzero(np.abs(np.diff(trajectory.Gripper)) > 0.0)
    operations = np.split(moving, np.flatnonzero(np.diff(moving) > 1) + 1)
    assert len(operations) == 4
    release = operations[1]
    assert trajectory.Jaw[release[0]:release[0] + 3].tolist() == [grip] * 3 and trajectory.Gripper[release[0]] < grip


def test_the_old_jaw_posing_is_refused(built, simulation):
    """
    Description:
        Item 10, forced failure: the old posing of the release in MG06 (the raw command once the PCB is in MG06: the jaws jump
        from grip to the clamp command, through the PCB, then open) closes the posed jaws by 12 mm while the gripper opens;
        a jaw closer than grip with the PCB in the jaws is refused too.
    """

    trajectory, _ = built
    grip = simulation.Jaw_Actual['G_CLAMP']
    old = copy.deepcopy(trajectory)
    moving = np.flatnonzero(np.abs(np.diff(old.Gripper)) > 0.0)
    release = np.split(moving, np.flatnonzero(np.diff(moving) > 1) + 1)[1]
    mg06 = CONST_PCB_LOCATIONS.index('mg06')
    switched = int(release[0]) + int(np.argmax(old.PCB_Location[release[0]:] == mg06))
    old.Jaw = np.array(old.Jaw)
    old.Jaw[switched:release[-1] + 2] = old.Gripper[switched:release[-1] + 2]
    faults = Jaw_Posing_Faults(old, grip)
    assert len(faults) == 1 and faults[0][0] == switched and faults[0][1].startswith('the posed jaws close by 12.00 mm at ')
    squeezed = copy.deepcopy(trajectory)
    squeezed.Jaw = np.array(squeezed.Jaw)
    inside = int(np.flatnonzero(squeezed.PCB_Location == CONST_PCB_LOCATIONS.index('jaws'))[100])
    squeezed.Jaw[inside] = grip - 0.001
    assert [f for _, f in Jaw_Posing_Faults(squeezed, grip) if 'closer than grip' in f] == [
        'the posed jaws are 1.00 mm closer than grip with the PCB in them at %.3f s' % squeezed.T[inside]]


def test_a_waitdi_holds_its_simulated_response_and_the_cycle_ends_with_the_settle(built, simulation):
    """
    Description:
        The WaitDI right after each vacuum on holds Simulation.yaml's 0.3 s response (75 samples); the second one at the take,
        after the jaws opened, costs nothing (the input came long before, F11); the trajectory ends with the simulation's
        End_Settle step (0.8 s, 200 samples) after the program's last segment, S21_Home_End.
    """

    trajectory, _ = built
    waits = [trajectory.Segment_End[i] - trajectory.Segment_Start[i] for i, text in enumerate(trajectory.Segment_Command)
             if text == 'Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)']
    assert waits == [75, 75, 0]
    assert trajectory.Step_Names[-2:] == ['S21_Home_End', 'End_Settle'] and trajectory.Segment_Command[-1] == 'End_Settle(0.8)'
    assert np.sum(trajectory.Step_Index == len(trajectory.Step_Names) - 1) == int(round(simulation.End_Settle / 0.004))


def test_a_waitdi_holds_only_what_is_left_of_the_response(constants, simulation, cell_configuration, tools):
    """
    Description:
        F11: a WaitDI waits for the input to come 0.3 s after its cause (the vacuum on): after a 0.1 s wait it holds the remaining
        0.2 s (50 samples), a second one right after it nothing.
    """

    kinematics, meta, _ = tools
    lines = ['Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveL(P_INPUT_APPROACH, v=0.2)', 'Robot.MoveL(P_INPUT_PICK, v=0.03)', 'Robot.SetDO(DO_VACUUM, ON)',
             'Wait(0.1)', 'Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)', 'Robot.WaitDI(DI_VACUUM_OK, timeout=0.5)']
    trajectory, _ = Build_Program(_Probe(lines, constants, simulation, cell_configuration), cell_configuration, meta, kinematics, {})
    assert [e - s for s, e in (_Segment(trajectory, text) for text in lines[4:])] == [25, 50, 0]


@pytest.mark.parametrize('lines, start, pcb, text', [
    (['Robot.SetDO(DO_VACUUM, ON)'], 'P_HOME', 'input_crate', 'the cups are .* from the PCB'),
    (['Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveL(P_INPUT_APPROACH, v=0.2)', 'Robot.MoveL(P_INPUT_PICK, v=0.2)', 'Robot.SetDO(DO_VACUUM, ON)',
      'Robot.MoveL(P_INPUT_ABOVE, v=0.2)', 'Robot.SetDO(DO_VACUUM, OFF)'], 'P_HOME', 'input_crate',
     'vacuum off 120.0 mm from the PCB.s place on the crate floor would drop it'),
    (['Loader.Run(J_LET_GO)'], 'P_HOME', 'jaws', 'opening the jaws away from MG06'),
    (['Tester.Run(J_TEST)'], 'P_HOME', 'mg06', 'the test needs the PCB in MG06 and MG06 settled closed'),
    (['Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)'], 'P_HOME', 'input_crate', 'DI_VACUUM_OK never comes: DO_VACUUM is off; the wait would time out'),
])
def test_a_broken_world_rule_is_refused(constants, simulation, cell_configuration, tools, lines, start, pcb, text):
    """
    Description:
        Vacuum on with no PCB under the cups, vacuum off 120 mm above the crate floor (a drop), the jaws opened away from MG06
        with the PCB only in them, a test with MG06 open and a WaitDI for the vacuum with the vacuum off are build errors naming
        the segment and the line.
    """

    kinematics, meta, _ = tools
    probe = _Probe(lines, constants, simulation, cell_configuration, start=start, pcb=pcb, loader_jobs={'J_LET_GO': ['Gripper(G_OPEN, t=0.35)']},
                   tester_jobs={'J_TEST': ['Test(Process.Test_Duration)']})
    with pytest.raises(Program_Error_Cls, match='Step S1_Probe, .*' + text):
        Build_Program(probe, cell_configuration, meta, kinematics, {})


def test_the_build_is_deterministic(program, cell_configuration, tools, built, tmp_path):
    """
    Description:
        Building the program twice gives byte-identical trajectory files (npz and CSV) - on one machine and platform (B5).
    """

    kinematics, meta, _ = tools
    again, _ = Build_Program(program, cell_configuration, meta, kinematics, {})
    one = [open(p, 'rb').read() for p in Write_Trajectory(built[0], str(tmp_path / 'one' / 'p.npz'))]
    two = [open(p, 'rb').read() for p in Write_Trajectory(again, str(tmp_path / 'two' / 'p.npz'))]
    assert one == two


def test_posing_a_sample_sets_the_whole_cell(built, tools, cell_configuration):
    """
    Description:
        Pose_Sample in the middle of S11_Test: the robot joints, the parked loader (y -199.5 mm, z +20 mm), MG06 settled closed
        with the press plate on its coupling, the PCB in MG06's place pose.
    """

    kinematics, meta, _ = tools
    trajectory, _ = built
    i = int(np.flatnonzero(trajectory.Step_Index == trajectory.Step_Names.index('S11_Test'))[100])
    Pose_Sample(kinematics, trajectory, i)
    a, q = kinematics.Scene_Address, kinematics.Data.qpos
    assert np.array_equal(q[kinematics.Q_Address], trajectory.Robot_Q[i])
    # The loader's moves end exactly on their targets.
    assert (q[a['y_axis']], q[a['z_axis']]) == (-0.1995, 0.02) and q[a['handle_hinge']] == cell_configuration.Closed_Handle
    assert q[a['press_slide']] == pytest.approx(0.0104554 * cell_configuration.Closed_Handle ** 3, rel=1e-5)
    assert kinematics.Data.xpos[kinematics.Model.body('pcb').id] == pytest.approx(meta.Place_Pos, abs=1e-9)


def test_the_carried_pcb_hangs_at_the_cup_face_once_the_bellows_extend(built, tools):
    """
    Description:
        The carried PCB (SPEC §4.10: sprung bellows): picked with the cups pressed 1.5 mm, it stays on the crate floor (z 4.9
        mm) until the TCP has risen 1.5 mm, then hangs at the cup face; at P_OUTPUT_PLACE, before the vacuum goes off, it is
        1.0 mm above its place on the floor (within the point's 0.05 mm stale tolerance: measured 1.019 mm), then on the floor.
    """

    kinematics, _, _ = tools
    trajectory, _ = built
    first, last = _Segment(trajectory, 'Robot.MoveL(P_INPUT_CLEAR, v=0.20)')
    tcp = np.array([kinematics.Site_Pose(q)[0] for q in trajectory.Robot_Q[first - 1:last]])
    rise = np.linalg.norm(tcp - tcp[0], axis=1)
    z = trajectory.PCB_Pose[first - 1:last, 2]
    assert np.max(np.abs(z[rise <= 0.0015] - 0.0049)) <= 2e-8 and np.sum(rise <= 0.0015) >= 10
    nominal = PCB_For_TCP(*kinematics.Site_Pose(trajectory.Robot_Q[last - 1]))[0]
    assert trajectory.PCB_Pose[last - 1][:3] == pytest.approx(nominal, abs=1e-12)
    place = _Segment(trajectory, 'Robot.MoveL(P_OUTPUT_PLACE, v=0.03)')[1] - 1
    assert trajectory.PCB_Pose[place][2] == pytest.approx(0.0049 + 0.001, abs=0.00005) and trajectory.PCB_Pose[place + 1][2] == pytest.approx(0.0049)


def test_the_default_program_passes_its_check(checked):
    """
    Description:
        The default program's check (measured 2026-10-24): every step passes; S9_Close_MG06's deepest handle_bar overlap is the
        recorded closing path's 0.54 mm, within Cell.yaml's 0.6 mm; every allowed group occurs where declared; the press at the
        pick is 1.48 mm (cups_pcb); holds are skipped (fewer samples measured than exist); the cycle is 69.396 s (F11's take fix).
    """

    assert [s.Name for s in checked.Steps if s.Verdict == 'FAIL'] == [] and checked.Verdict == 'PASS'
    close = next(s for s in checked.Steps if s.Name == 'S9_Close_MG06')
    assert close.Allowed['handle_bar'].Distance == pytest.approx(-0.000539, abs=0.000002)
    pick = next(s for s in checked.Steps if s.Name == 'S2_Pick_Input')
    assert pick.Allowed['cups_pcb'].Distance == pytest.approx(-0.00148, abs=0.00002) and pick.Allowed['pcb_crate'] is not None
    assert all(f is not None for s in checked.Steps for f in s.Allowed.values())
    assert 0 < checked.Checked < 17350 and checked.Cycle_Time == pytest.approx(69.396, abs=1e-9)


def test_the_handover_keeps_the_board_off_the_ledges(checked):
    """
    Description:
        B1 (SPEC §4.7): the PCB-ledge gap with the jaws at grip is 0.299 mm at P_LOADER_PLACE (0.318 mm before the lateral
        re-teach, B work item 2) and while the jaws clamp the PCB on the cups, and never smaller along the MoveL into it (0.3 +-
        0.1 mm): no press; the PCB centre is 0.4 mm from the jaw centreline (its physics position on the cups is on it).
    """

    h = checked.Handover
    assert h.Failures == [] and h.At_Handover == pytest.approx(0.000299, abs=0.000002)
    assert h.Line_Min == pytest.approx(h.At_Handover, abs=2e-6) and h.Clamped_Min == pytest.approx(0.000299, abs=0.000002)
    assert h.Clamped_Max == pytest.approx(h.Clamped_Min, abs=1e-9) and h.Lateral == pytest.approx(0.0004, abs=0.000002)


def test_a_press_at_the_handover_is_reported(constants, simulation, cell_configuration, tools):
    """
    Description:
        A handover whose hold pose is the take's 1.5 mm press (P_LOADER_PLACE's joints replaced by P_LOADER_PICK's) is
        reported: the PCB-ledge gap below 0.2 mm at P_LOADER_PLACE, '(a press)'.
    """

    # A consistent pressed variant: P_LOADER_PLACE taught 1.5 mm deeper (P_LOADER_PICK's joints and pose, its PCB-hold offset
    # +1.5 mm), the points derived from it detached.
    pressed, pressed_simulation = copy.deepcopy(constants), copy.deepcopy(simulation)
    place, pick = pressed.Points['P_LOADER_PLACE'], constants.Points['P_LOADER_PICK']
    place.Joints, place.Pose = list(pick.Joints), list(pick.Pose)
    pressed_simulation.Points['P_LOADER_PLACE'].Hold_Offset = list(simulation.Points['P_LOADER_PLACE'].Hold_Offset[:2]) + [0.0015]
    for point in pressed.Points.values():
        if point.Reference == 'P_LOADER_PLACE':
            point.Reference, point.Offset = None, None
    lines = ['Robot.MoveJ(P_LOADER_ABOVE)', 'Robot.MoveL(P_LOADER_APPROACH, v=0.2)', 'Robot.MoveL(P_LOADER_PLACE, v=0.03)', 'Loader.Run(J_CLAMP)']
    probe = _Probe(lines, pressed, pressed_simulation, cell_configuration, pcb='cups', allow=('cups_pcb', 'pcb_jaws'),
                   loader_jobs={'J_CLAMP': ['Gripper(G_CLAMP, t=0.35)']})
    result = _Check(probe, cell_configuration, tools)
    assert any(f.startswith('the PCB-ledge gap at P_LOADER_PLACE is') and f.endswith('(a press)') for f in result.Handover.Failures)
    assert result.Verdict == 'FAIL'


def _Lateral_Handover(constants, simulation, cell, tools, dx, lines=None):
    """
    Description:
        A handover probe with P_LOADER_PLACE taught dx along its tool x (the jaw axis): its PCB_Hold offset [dx, 0, 0] resolved
        by IK, the points derived from it detached; checked.
    """

    kinematics, meta, _ = tools
    moved, moved_simulation = copy.deepcopy(constants), copy.deepcopy(simulation)
    moved_simulation.Points['P_LOADER_PLACE'].Hold_Offset = [dx, 0.0, 0.0]
    joints, pose = Resolve_Keypoint('P_LOADER_PLACE', kinematics, moved, moved_simulation, meta, cell)
    moved.Points['P_LOADER_PLACE'].Joints, moved.Points['P_LOADER_PLACE'].Pose = list(joints), list(pose)
    for point in moved.Points.values():
        if point.Reference == 'P_LOADER_PLACE':
            point.Reference, point.Offset = None, None
    lines = lines or ['Robot.MoveJ(P_LOADER_ABOVE)', 'Robot.MoveL(P_LOADER_APPROACH, v=0.2)', 'Robot.MoveL(P_LOADER_PLACE, v=0.03)',
                      'Loader.Run(J_CLAMP)']
    probe = _Probe(lines, moved, moved_simulation, cell, pcb='cups', allow=('cups_pcb', 'pcb_jaws'),
                   loader_jobs={'J_CLAMP': ['Gripper(G_CLAMP, t=0.35)']})
    return _Check(probe, cell, tools)


def test_the_descent_measures_the_height_above_the_ledges_only(constants, simulation, cell_configuration, tools):
    """
    Description:
        Item 1 (owner, 2026-10-25): a handover taught 0.4 mm along the jaw axis (the re-teach's size) passes - along the
        descent the jaws are centred on the PCB, so the gap is its height above the finger ledges (with the jaws at grip about
        the uncentred PCB it was -0.186 mm, the board in a jaw's pad corner); the lateral offset is reported, 0.400 mm.
    """

    result = _Lateral_Handover(constants, simulation, cell_configuration, tools, 0.0004)
    h = result.Handover
    assert h.Failures == [] and h.Lateral == pytest.approx(0.0004, abs=2e-6)
    assert h.Line_Min == pytest.approx(h.At_Handover, abs=2e-6) and h.At_Handover == pytest.approx(0.000299, abs=0.000002)


def test_a_dip_below_the_ledges_along_the_descent_is_reported(constants, simulation, cell_configuration, tools):
    """
    Description:
        Item 1, forced failure: a descent that goes 1.5 mm deeper (P_LOADER_PICK) and back up to P_LOADER_PLACE still fails
        along the line with the jaws centred: the gap falls below 0.2 mm, '(a press)'.
    """

    lines = ['Robot.MoveJ(P_LOADER_ABOVE)', 'Robot.MoveL(P_LOADER_PICK, v=0.03)', 'Robot.MoveL(P_LOADER_PLACE, v=0.03)', 'Loader.Run(J_CLAMP)']
    probe = _Probe(lines, constants, simulation, cell_configuration, pcb='cups', allow=('cups_pcb', 'pcb_jaws'),
                   loader_jobs={'J_CLAMP': ['Gripper(G_CLAMP, t=0.35)']})
    h = _Check(probe, cell_configuration, tools).Handover
    assert any(f.startswith('the PCB-ledge gap falls to') and f.endswith('(a press)') for f in h.Failures), h.Failures


def test_a_handover_off_the_jaw_centreline_is_reported(constants, simulation, cell_configuration, tools):
    """
    Description:
        Item 1, forced failure: P_LOADER_PLACE taught 0.6 mm along the jaw axis is more than Handover.Lateral_Max (0.5 mm) from the
        jaw centreline and fails, naming the offset.
    """

    h = _Lateral_Handover(constants, simulation, cell_configuration, tools, 0.0006).Handover
    assert h.Lateral == pytest.approx(0.0006, abs=2e-6)
    found = [f for f in h.Failures if 'jaw centreline' in f]
    assert len(found) == 1 and found[0] == ('the PCB centre is %.3f mm from the jaw centreline at P_LOADER_PLACE, more than Handover.Lateral_Max '
                                            '0.5 mm: the jaws would pull it off the cups as they centre it' % (1000 * abs(h.Lateral)))


def test_a_colliding_step_is_reported_with_its_pair_and_time(constants, simulation, cell_configuration, tools):
    """
    Description:
        A MoveL from 15 mm above the PCB in the input crate straight across to 15 mm above the output crate's floor (through
        both crates' walls) fails, naming a cup and a crate wall, the sample and its time.
    """

    probe = _Probe(['Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveL(P_INPUT_APPROACH, v=0.2)', 'Robot.MoveL(P_OUTPUT_APPROACH, v=0.2)'], constants,
                   simulation, cell_configuration)
    step = _Check(probe, cell_configuration, tools).Steps[0]
    assert step.Verdict == 'FAIL' and step.Closest.Distance < 0 and 'crate_' in step.Closest.Pair
    assert any(f.startswith(step.Closest.Pair) and 'at sample %d (%.3f s)' % (step.Closest.Sample, step.Closest.Time) in f for f in step.Failures)


def test_contacts_are_allowed_only_where_declared_and_as_deep_as_configured(constants, simulation, cell_configuration, tools):
    """
    Description:
        The pick's press without cups_pcb declared fails (group not allowed); declared, with cups_pcb allowed only 1.0 mm deep it
        fails on the depth (1.48 mm); a declared group that never occurs (pcb_jaws at the crate) fails.
    """

    lines = ['Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveL(P_INPUT_APPROACH, v=0.2)', 'Robot.MoveL(P_INPUT_PICK, v=0.03)']
    undeclared = _Check(_Probe(lines, constants, simulation, cell_configuration), cell_configuration, tools)
    assert any('group cups_pcb not allowed in this step' in f for f in undeclared.Steps[0].Failures)
    shallow = copy.deepcopy(cell_configuration)
    shallow.Allowed_Depth['cups_pcb'] = 0.001
    deep = _Check(_Probe(lines, constants, simulation, cell_configuration, allow=('cups_pcb', 'pcb_jaws')), shallow, tools)
    failures = deep.Steps[0].Failures
    assert any(f.startswith('cups_pcb 1.48 mm deep') and 'at most 1.00 mm' in f for f in failures)
    assert any('allowed contact pcb_jaws does not occur in the step' in f for f in failures)


def test_a_movej_over_its_rule_limits_is_reported(constants, simulation, cell_configuration, tools):
    """
    Description:
        Checked against acceleration limits 10 % below the ones the MoveJ was timed with, the MoveJ P_HOME -> P_INPUT_ABOVE
        (acceleration-limited) fails, naming its line, its peak and the allowed fraction.
    """

    kinematics, meta, collision = tools
    probe = _Probe(['Robot.MoveJ(P_INPUT_ABOVE)'], constants, simulation, cell_configuration)
    trajectory, notes = Build_Program(probe, cell_configuration, meta, kinematics, {})
    slower = copy.deepcopy(cell_configuration)
    slower.Acceleration_Limits = [0.9 * a for a in slower.Acceleration_Limits]
    result = Check_Trajectory(trajectory, probe, notes, kinematics, collision, slower, meta)
    assert any(f.startswith('Robot.MoveJ(P_INPUT_ABOVE): peak joint speed') for f in result.Steps[0].Failures)


def test_the_report_lists_steps_groups_handover_and_failures(checked, built, cell_configuration):
    """
    Description:
        The report: a header with the cycle time; a row per step with its peaks and verdict, the closest pair per allowed group
        and of the rest with sample and time; the handover's gaps; the stretched MoveLs; the opening path's note; the events;
        the points used (24: 23 PASS, 1 WARN); no failure; the closing sentence on simulation results.
    """

    lines = Format_Check_Report(checked, built[0], cell_configuration)
    text = '\n'.join(lines)
    assert lines[0].startswith('program Program: 22 steps, 17350 samples (4 ms), cycle time 69.396 s;')
    assert any(line.startswith('S2_Pick_Input ') and line.endswith('PASS') for line in lines)
    assert '    cups_pcb     -1.48 mm at ' in text and '    not allowed  ' in text
    assert ('handover (no press, 0.3 +- 0.1 mm): PCB-ledge gap with the jaws at grip - along the MoveL into P_LOADER_PLACE (jaws centred on the '
            'PCB) min 0.299 mm') in text and 'mm from the jaw centreline (<= 0.5 mm): PASS' in text
    assert 'stretched MoveLs: none' in text and 'note: S13_Open_MG06, Robot.MovePath(PATH_FORK_OPEN): the recorded path starts moving' in text
    assert 'events: 3.276 s vacuum_on, ' in text and 'keypoints used: 24 (0 FAIL, 1 WARN, 23 PASS;' in text
    assert lines[-1] == 'Program: the program check passed. Simulation results only: not validated for the real cell.'


@pytest.mark.parametrize('key, old, new, text', [
    ('  P_HOME:', '    Pose: [0.216200182, ', '    Pose: [0.216220182, ', 'Points.P_HOME.Pose: 0.0200 mm / 0.0000 deg from FK(Joints, T_SUCTION)'),
    ('  P_INPUT_ABOVE:', '    Offset: [0.0, 0.0, -0.1215]', '    Offset: [0.0, 0.0, -0.1216]', 'Points.P_INPUT_ABOVE.Reference: the point is 0.1'),
    ('  T_SUCTION:', '    Flange_Offset: [0.0, 0.0, 0.2606, 0.0, 0.0, 0.0]', '    Flange_Offset: [0.0, 0.0, 0.2607, 0.0, 0.0, 0.0]',
     'Tools.T_SUCTION.Flange_Offset: 0.1000 mm / 0.0000 deg from the model site aubo/tool/suction_tcp'),
    ('  L_PARK:', '    Y: -0.1995', '    Y: -0.25', "Loader_Positions.L_PARK.Y: -0.25 m is outside the model's y_axis range"),
    ('  G_OPEN:', '    Jaw: 0.035', '    Jaw: 0.045', "Gripper_Commands.G_OPEN.Jaw: 0.045 m is outside the jaws' range"),
])
def test_the_constants_are_checked_against_the_model(tmp_path, cell_configuration, tools, key, old, new, text):
    """
    Description:
        Validate_Constants (the build runs it first): a Pose 0.02 mm off FK, an Offset 0.1 mm off its Reference's pose (beyond the
        0.05 mm stale tolerance), a tool offset 0.1 mm off its model site, a loader position beyond its axis, a jaw beyond its
        range - each named with Constants.yaml's line and the constant; the build refuses them.
    """

    kinematics, meta, _ = tools
    path = tmp_path / 'Constants.yaml'
    original = open(CONST_CONSTANTS_YAML, encoding='utf-8').read()
    assert old in original
    path.write_text(original.replace(old, new, 1), encoding='utf-8')
    constants = Load_Constants(str(path))
    simulation = Load_Simulation(constants)
    program = Program_Str(Name='probe', Constants=constants, Simulation=simulation)
    problems = Validate_Constants(program, kinematics, cell_configuration, meta)
    line = next(i for i, l in enumerate(original.splitlines(), start=1) if l == key)
    assert any(text in p and 'Constants.yaml:%d: ' % line in p for p in problems), problems
    with pytest.raises(Program_Error_Cls, match='The constants do not match the model'):
        Build_Program(program, cell_configuration, meta, kinematics, {})


def test_the_shipped_constants_match_the_model(program, cell_configuration, tools):
    """
    Description:
        The shipped Constants.yaml passes Validate_Constants: every Pose, tool offset, derived point, loader position and jaw.
    """

    kinematics, meta, _ = tools
    assert Validate_Constants(program, kinematics, cell_configuration, meta) == []


def test_a_run_is_a_start_and_a_waitdone(tmp_path, monkeypatch, program, constants, simulation, cell_configuration, tools, built):
    """
    Description:
        The loader's lane (F9): the taught program with S6's Loader.Run(J_INSERT) written as Loader.Start(J_INSERT) +
        Loader.WaitDone(J_INSERT) (in a file with an Optimized record, where they are allowed) gives the same samples on
        every channel and the same events; only the trajectory's segments differ (two lines instead of the job's six). (The
        file is not the optimizer's, so the optimizer's waiting points are not required in it.)
    """

    import Configuration.Program_Parser as Program_Parser
    from Configuration.Program_Parser import Load_Program, CONST_PROGRAM_YAML
    monkeypatch.setattr(Program_Parser, 'Optimizer_Points', lambda path=None: {})
    text = open(CONST_PROGRAM_YAML, encoding='utf-8').read().replace('    - Loader.Run(J_INSERT)\n',
                                                                     '    - Loader.Start(J_INSERT)\n    - Loader.WaitDone(J_INSERT)\n', 1)
    text += '\nWaypoints: {}\nOptimized:\n  Inputs: {}\n  Platform: {}\n  Schedule: {}\n  Topp_Grid_Points: 1000\n'
    path = tmp_path / 'Program_Lane.yaml'
    path.write_text(text, encoding='utf-8')
    lane = Load_Program(constants, simulation, cell_configuration, str(path))
    kinematics, meta, _ = tools
    trajectory, _ = Build_Program(lane, cell_configuration, meta, kinematics, {})
    default = built[0]
    for channel in ('T', 'Robot_Q', 'Robot_QD', 'Loader_Y', 'Loader_Z', 'Gripper', 'Vacuum', 'Step_Index', 'PCB_Location', 'PCB_Pose', 'MG06_Lid',
                    'MG06_Handle', 'Jaw'):
        assert np.asarray(getattr(default, channel)).tobytes() == np.asarray(getattr(trajectory, channel)).tobytes(), channel
    assert trajectory.Events == default.Events and len(trajectory.Segment_Command) == len(default.Segment_Command) - 6 + 2


def test_the_optimized_splines_run_within_the_limits(constants, simulation, cell_configuration, tools):
    """
    Description:
        Every MoveSpline of the committed optimized program: its finite-difference joint speeds and accelerations within the
        Cell.yaml limits (its v= is Speed_Fraction), its stored speeds the spline's, at rest at both ends, ending exactly on
        its end point's Joints.
    """

    from Configuration.Program_Parser import Load_Program, CONST_OPTIMIZED_PROGRAM_YAML
    optimized = Load_Program(constants, simulation, cell_configuration, CONST_OPTIMIZED_PROGRAM_YAML)
    kinematics, meta, _ = tools
    trajectory, _ = Build_Program(optimized, cell_configuration, meta, kinematics, {})
    v_max = cell_configuration.Speed_Fraction * np.array(cell_configuration.Datasheet_Speeds)
    a_max = np.array(cell_configuration.Acceleration_Limits)
    splines = [k for k, text in enumerate(trajectory.Segment_Command) if text.startswith('Robot.MoveSpline')]
    assert len(splines) == 7
    for k in splines:
        # Sample 0 is the start state (the first segment's first sample), so a segment's motion begins at sample 1 at the earliest.
        first, end = max(int(trajectory.Segment_Start[k]), 1), int(trajectory.Segment_End[k])
        q = np.vstack([trajectory.Robot_Q[first - 1], trajectory.Robot_Q[first:end], trajectory.Robot_Q[end - 1]])
        qd = np.diff(q, axis=0) / 0.004
        assert np.all(np.abs(qd) <= v_max * 1.001) and np.all(np.abs(np.diff(qd, axis=0) / 0.004) <= a_max * 1.001)
        assert np.all(trajectory.Robot_QD[end - 1] == 0.0)
        # The trajectory's speeds are the spline's (analytic): the central differences of its joints within a sample's
        # acceleration (a_max x 4 ms), and not zero.
        central = np.gradient(trajectory.Robot_Q[first - 1:end], 0.004, axis=0)[1:-1]
        assert np.all(np.abs(trajectory.Robot_QD[first:end - 1] - central) <= a_max * 0.004 + 1e-9)
        assert float(np.max(np.abs(trajectory.Robot_QD[first:end]))) > 0.1
        target = next(c for s in optimized.Segments for c in s.Commands if c.Text == trajectory.Segment_Command[k])
        assert np.array_equal(trajectory.Robot_Q[end - 1], np.array(constants.Points[target.Path_End].Joints))


def test_the_free_steps_are_the_taught_list_and_the_optimized_splines(program, cell_configuration):
    """
    Description:
        F11: Free_Steps gives the taught program's Optimization.yaml Free_Segments (8), the optimized program's MoveSpline
        segments (6), none for a taught program without its settings.
    """

    from Configuration.Program_Parser import Load_Program, CONST_OPTIMIZED_PROGRAM_YAML
    assert Free_Steps(program, Load_Optimization(program)) == {'S1_Approach_Input', 'S3_To_Loader', 'S5_Home_After_Handover', 'S8_To_MG06',
                                                               'S14_Home_After_MG06', 'S17_To_Take', 'S19_To_Output', 'S21_Home_End'}
    assert Free_Steps(program) == set()
    optimized = Load_Program(program.Constants, program.Simulation, cell_configuration, CONST_OPTIMIZED_PROGRAM_YAML)
    assert Free_Steps(optimized) == {'S1_Approach_Input', 'S3_To_Loader', 'S5_To_MG06', 'S11_To_Take', 'S13_To_Output', 'S15_Home_End'}


def test_the_free_transfers_keep_the_loader_clearance(checked, built, program, cell_configuration, tools):
    """
    Description:
        F11: in the taught program's free transfers the robot and the carried PCB keep Loader_Free + Tracking_Margin (14 mm) from
        the loader where its actuators put it: S3_To_Loader 20.59 mm (the PCB / z_carriage), the others beyond the 50 mm cut;
        with Loader_Free 20 mm S3_To_Loader fails naming the pair, the sample and the rule, and only it.
    """

    loader = {s.Name: s.Loader for s in checked.Steps if s.Loader is not None}
    assert set(loader) == Free_Steps(program, Load_Optimization(program))
    assert loader['S3_To_Loader'].Pair == 'pcb / z_carriage' and loader['S3_To_Loader'].Distance == pytest.approx(0.02059, abs=0.00001)
    assert all(found.Sample == -1 for name, found in loader.items() if name != 'S3_To_Loader')
    kinematics, meta, collision = tools
    strict = dataclasses.replace(cell_configuration, Loader_Free=0.02)
    result = Check_Trajectory(built[0], program, built[1], kinematics, collision, strict, meta, {'S3_To_Loader', 'S19_To_Output'})
    failed = {s.Name: s.Failures for s in result.Steps if s.Failures}
    assert list(failed) == ['S3_To_Loader'] and failed['S3_To_Loader'][0].startswith('pcb / z_carriage 20.59 mm from the loader at sample 1399 '
                                                                                     '(5.596 s), the loader at its actuator model')
    assert failed['S3_To_Loader'][0].endswith('a free transfer keeps Clearance.Loader_Free 20.0 mm + Tracking_Margin 4.0 mm')


def test_the_loader_actuator_model_lags_and_settles(built, tools):
    """
    Description:
        F11: Loader_Actual follows the commands through the model's actuators: equal at the start; during J_INSERT's y move
        behind the command by up to tens of millimetres (the physics run's loader lag is 22.6 mm); settled in the test wait (y
        within 0.01 mm, z within its gravity droop, load / kp, 0.44 mm); the PCB in the jaws moves with it (Loader_Actual_Trajectory).
    """

    kinematics = tools[0]
    tr = built[0]
    y, z = Loader_Actual(tr, kinematics.Model)
    assert y[0] == tr.Loader_Y[0] and z[0] == tr.Loader_Z[0]
    lag = np.abs(y - tr.Loader_Y)
    assert 0.01 < float(np.max(lag)) < 0.03
    settled = np.flatnonzero(tr.T > 30.0)[:10]
    assert np.all(lag[settled] < 1e-5) and np.all(np.abs(z - tr.Loader_Z)[settled] < 0.0005)
    actual = Loader_Actual_Trajectory(tr, kinematics.Model)
    jaws = np.flatnonzero(tr.PCB_Location == CONST_PCB_LOCATIONS.index('jaws'))
    i = jaws[np.argmax(lag[jaws])]
    assert actual.PCB_Pose[i, 1] - tr.PCB_Pose[i, 1] == pytest.approx(y[i] - tr.Loader_Y[i], abs=1e-12)
    assert np.array_equal(actual.Robot_Q, tr.Robot_Q)


@pytest.mark.slow
def test_the_loader_actuator_model_matches_the_physics(built, program, cell_configuration):
    """
    Description:
        F11: over the taught program's first 21 s (J_INSERT and J_PARK) the loader in physics is within 0.5 mm (y) and 1.1 mm
        (z) of Loader_Actual (measured 0.17 mm / 1.04 mm on the optimized program, 2026-10-25), where its command is up to
        22.6 mm off.
    """

    from Scene.Core import Build_Cell_Model
    from Simulation.Core import Trajectory_Drive_Cls, Physics_Run_Cls
    tr = built[0]
    model = Build_Cell_Model(cell_configuration)
    run = Physics_Run_Cls(model, Trajectory_Drive_Cls(tr, program), Read_PCB_Meta(), cell_configuration)
    y, z = Loader_Actual(tr, model)
    errors = []
    while run.Data.time < 21.0:
        run.Step()
        i = int(round(run.Data.time / 0.004))
        if abs(run.Data.time - i * 0.004) < 1e-9:
            errors.append((abs(run.Data.qpos[run.qy] - y[i]), abs(run.Data.qpos[run.qz] - z[i]), abs(run.Data.qpos[run.qy] - tr.Loader_Y[i])))
    e = np.array(errors)
    assert e[:, 0].max() < 0.0005 and e[:, 1].max() < 0.0011 and e[:, 2].max() > 0.02, e.max(axis=0)
