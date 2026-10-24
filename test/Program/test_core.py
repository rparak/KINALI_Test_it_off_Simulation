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
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test
#   ../Program/Core
from Program.Core import Build_Program, Movej_Duration, Program_Error_Cls, Pose_Sample
#   ../Configuration/Core, ../Collision/Core, ../Kinematics/Core, ../Trajectory/Core
from Configuration.Core import (Load_Program, Read_PCB_Meta, Program_Str, Program_Step_Str, Program_Command_Str, CONST_PCB_LOCATIONS,
                                CONST_FORK_PATH)
from Collision.Core import Build_Planning_Model
from Kinematics.Core import Robot_Kinematics_Cls
from Trajectory.Core import Write_Trajectory

"""
Description:
    Initialization of constants.

    The program build (FEATURES.md F8): the motion laws, the recorded fork path, the world state. Expected values come
    from the legacy planner's rules (git tag legacy-planner-reference), the reference plan and SPEC §4, written out here.
"""
# The legacy MoveJ rule's numbers: 60 % of 220 / 230 deg/s, 3 / 6 rad/s^2, at least 0.5 s (SPEC §4.4, §4.10).
#   Unit [Vector<float> 1x6] (rad/s), (rad/s^2), [float] (s)
CONST_V_MAX = 0.6 * np.radians([220, 220, 220, 220, 230, 230])
CONST_A_MAX = np.array([3.0, 3.0, 3.0, 6.0, 6.0, 6.0])
CONST_T_MIN = 0.5


@pytest.fixture(scope='module')
def tools(cell_configuration):
    model = Build_Planning_Model(cell_configuration)
    return Robot_Kinematics_Cls(model), Read_PCB_Meta()


@pytest.fixture(scope='module')
def program(keypoints, cell_configuration):
    return Load_Program(keypoints, cell_configuration)


@pytest.fixture(scope='module')
def built(program, keypoints, cell_configuration, tools):
    kinematics, meta = tools
    return Build_Program(program, keypoints, cell_configuration, meta, kinematics, {})


def _Program(commands, start='home', pcb='input_crate', mg06='Open'):
    """
    Description:
        A one-step program of the given commands from a start keypoint (loader home, gripper open, vacuum off).
    """

    return Program_Str(Name='probe', Start_Robot=start, Start_Loader='home', Start_Gripper='open', Start_Vacuum=False, Start_PCB=pcb,
                       Start_MG06=mg06, Steps=[Program_Step_Str(Name='probe', Comment='a probe', Commands=commands)])


def _Command(kind, target, speed=0.0, duration=0.0):
    return Program_Command_Str(Kind=kind, Target=target, Speed=speed, Duration=duration, Text='%s %s' % (kind, target))


def _Segment(trajectory, text):
    i = trajectory.Segment_Command.index(text)
    return trajectory.Segment_Start[i], trajectory.Segment_End[i]


def test_movej_follows_the_legacy_rule_and_reproduces_the_reference_plan(built, keypoints, cell_configuration, root):
    """
    Description:
        MoveJ home -> above_input_pcb: T = max(0.5, max 1.875 |dq| / v_max, max sqrt(5.774 |dq| / a_max)) with the legacy
        numbers, round(T / 4 ms) samples; its samples are the reference plan's first ones to the keypoints' rounding (1e-6 deg
        = 1.7e-8 rad); min-jerk: at rest at both ends, the analytic speed's peak 1.875 dq / T.
    """

    q0, q1 = np.array(keypoints.Robot['home'].Joints), np.array(keypoints.Robot['above_input_pcb'].Joints)
    dq = np.abs(q1 - q0)
    expected = max(CONST_T_MIN, np.max(1.875 * dq / CONST_V_MAX), np.max(np.sqrt(5.774 * dq / CONST_A_MAX)))
    assert Movej_Duration(q0, q1, cell_configuration) == pytest.approx(expected, rel=1e-12)
    trajectory, _ = built
    first, last = _Segment(trajectory, 'Movej above_input_pcb')
    n = int(round(expected / 0.004))
    assert (first, last) == (0, n + 1)
    plan = np.load(os.path.join(root, 'Data', 'Reference', 'cell_plan.npz'))['q']
    assert np.max(np.abs(trajectory.Robot_Q[1:n + 1] - plan[:n])) <= 2e-8
    qd = trajectory.Robot_QD[first:last]
    assert np.all(qd[0] == 0) and np.all(qd[-1] == 0)
    assert np.max(np.abs(qd), axis=0) == pytest.approx(1.875 * (q1 - q0).__abs__() / (n * 0.004), rel=1e-3)


def test_movel_is_straight_and_ends_on_the_stored_joints(built, keypoints, tools):
    """
    Description:
        MoveL input_pcb_approach -> input_pcb_press (15 mm at 30 mm/s): every TCP sample within 0.01 mm of the line, the
        orientation kept, the last sample exactly the stored joints, T = 1.875 L / speed (0.94 s), the TCP's peak speed at
        most the commanded 30 mm/s.
    """

    kinematics, _ = tools
    trajectory, _ = built
    first, last = _Segment(trajectory, 'Movel input_pcb_press 0.03')
    p0, R0 = kinematics.Site_Pose(trajectory.Robot_Q[first - 1])
    p1, _ = kinematics.Site_Pose(np.array(keypoints.Robot['input_pcb_press'].Joints))
    axis = (p1 - p0) / np.linalg.norm(p1 - p0)
    points = [kinematics.Site_Pose(q) for q in trajectory.Robot_Q[first:last]]
    off_line = max(np.linalg.norm((p - p0) - ((p - p0) @ axis) * axis) for p, _ in points)
    tilt = max(np.arccos(np.clip((np.trace(R0.T @ R) - 1) / 2, -1, 1)) for _, R in points)
    assert off_line <= 1e-5 and tilt <= 1e-6
    assert np.array_equal(trajectory.Robot_Q[last - 1], keypoints.Robot['input_pcb_press'].Joints)
    assert (last - first) * 0.004 == pytest.approx(1.875 * np.linalg.norm(p1 - p0) / 0.03, abs=0.004)
    speeds = [np.linalg.norm(b[0] - a[0]) / 0.004 for a, b in zip(points, points[1:])]
    assert max(speeds) <= 0.03 * 1.01


def test_a_movel_cannot_switch_the_ik_family(keypoints, cell_configuration, tools):
    """
    Description:
        A MoveL from home to above_handle (wrist up to wrist down) fails, naming both wrist_2 angles: a line cannot switch
        families (SPEC §4.7). (A stale keypoint is the check's: a MoveL aims at its stored joints' pose.)
    """

    kinematics, meta = tools
    with pytest.raises(Program_Error_Cls, match='a MoveL cannot switch families'):
        Build_Program(_Program([_Command('Movel', 'above_handle', speed=0.2)]), keypoints, cell_configuration, meta, kinematics, {})


def test_a_movel_over_a_joint_limit_is_stretched_and_noted(keypoints, cell_configuration, tools):
    """
    Description:
        MoveL home -> above_input_pcb at 1 m/s (365 mm) would exceed a joint's speed or acceleration limit: it is stretched
        (x1.985, measured 2026-10-24) and noted, and the sampled joint speeds and accelerations then stay within 60 % of the
        datasheet speeds and the assumed accelerations.
    """

    kinematics, meta = tools
    probe = _Program([_Command('Movel', 'above_input_pcb', speed=1.0)])
    trajectory, notes = Build_Program(probe, keypoints, cell_configuration, meta, kinematics, {})
    assert len(notes) == 1 and notes[0].Text.startswith('stretched x1.985')
    q = np.vstack([trajectory.Robot_Q, trajectory.Robot_Q[-1:]])
    qd = np.diff(q, axis=0) / 0.004
    assert np.all(np.abs(qd) <= CONST_V_MAX) and np.all(np.abs(np.diff(qd, axis=0) / 0.004) <= CONST_A_MAX)


def test_the_fork_paths_follow_the_recorded_rows_and_join_rule(built, keypoints, tools):
    """
    Description:
        Path Fork_Close follows rows 1-305 (5.684 s after row 0) from fork_start and ends exactly on fork_closed_end; Path
        Fork_Open starts at fork_open_join and follows the rows after row 663 (the first OPEN row with handle <= 28.45 deg:
        28.32 deg) to fork_start; MG06 is the recorded state during the paths, and Fork_Open's first row moves (the noted
        velocity step, 39.1 mm/s).
    """

    trajectory, notes = built
    log = np.load(CONST_FORK_PATH)['log']
    first, last = _Segment(trajectory, 'Path Fork_Close')
    assert (last - first) * 0.004 == pytest.approx(log[305, 0] - log[0, 0], abs=0.002)
    assert np.array_equal(trajectory.Robot_Q[first - 1], keypoints.Robot['fork_start'].Joints)
    assert np.array_equal(trajectory.Robot_Q[last - 1], keypoints.Robot['fork_closed_end'].Joints)
    assert trajectory.MG06_Handle[last - 1] == pytest.approx(log[305, 3], abs=1e-12)
    first, last = _Segment(trajectory, 'Path Fork_Open')
    assert math.degrees(log[663, 3]) == pytest.approx(28.32, abs=0.01) and math.degrees(log[662, 3]) > 28.45
    assert (last - first) * 0.004 == pytest.approx(log[905, 0] - log[663, 0], abs=0.002)
    assert np.array_equal(trajectory.Robot_Q[first - 1], keypoints.Robot['fork_open_join'].Joints)
    assert np.array_equal(trajectory.Robot_Q[last - 1], keypoints.Robot['fork_start'].Joints)
    assert [n.Command for n in notes] == ['Path Fork_Open'] and '39.1 mm/s' in notes[0].Text


def test_the_pcb_moves_through_the_cycle_by_the_rules(built, cell_configuration, keypoints):
    """
    Description:
        The PCB's locations in order: input_crate, cups (vacuum on at 3.276 s, as the reference plan), jaws (vacuum off at
        9.524 s), mg06 (the jaws open at MG06), jaws (clamped at MG06), cups (vacuum on at the take), output_crate (vacuum
        off) - at its rest pose on the floor; the jaws stand at grip while the PCB is between them; MG06 is settled closed
        during the test and open at the end.
    """

    trajectory, _ = built
    locations = [CONST_PCB_LOCATIONS[i] for i in trajectory.PCB_Location]
    order = [locations[0]] + [b for a, b in zip(locations, locations[1:]) if a != b]
    assert order == ['input_crate', 'cups', 'jaws', 'mg06', 'jaws', 'cups', 'output_crate']
    assert [e for e in trajectory.Events if e[1].startswith('vacuum')][:2] == [(3.276, 'vacuum_on'), (9.524, 'vacuum_off')]
    assert trajectory.PCB_Pose[-1][:3] == pytest.approx([0.40620, -0.75160, 0.0049], abs=1e-9)
    clamp = trajectory.Segment_Command.index('Gripper clamp 0.35')
    assert trajectory.Jaw[trajectory.Segment_End[clamp] - 1] == keypoints.Gripper['grip']
    assert trajectory.Gripper[trajectory.Segment_End[clamp] - 1] == keypoints.Gripper['clamp']
    test = trajectory.Step_Names.index('test')
    during = trajectory.Step_Index == test
    assert np.all(trajectory.MG06_Lid[during] == cell_configuration.Closed_Lid)
    assert np.all(trajectory.MG06_Handle[during] == cell_configuration.Closed_Handle)
    assert trajectory.MG06_Lid[-1] < -0.69 and trajectory.MG06_Handle[-1] == 0.0


@pytest.mark.parametrize('commands, start, pcb, text', [
    ([('Vacuum', 'On')], 'home', 'input_crate', 'the cups are .* from the PCB'),
    ([('Movej', 'above_input_pcb'), ('Movel', 'input_pcb_approach'), ('Movel', 'input_pcb_press'), ('Vacuum', 'On'),
      ('Movel', 'above_input_pcb'), ('Vacuum', 'Off')], 'home', 'input_crate',
     'vacuum off 120.0 mm from the PCB.s place on the crate floor would drop it'),
    ([('Gripper', 'open')], 'home', 'jaws', 'opening the jaws away from MG06'),
    ([('Test', 'Process.Test_Duration')], 'home', 'mg06', 'the test needs the PCB in MG06 and MG06 settled closed'),
])
def test_a_broken_world_rule_is_refused(keypoints, cell_configuration, tools, commands, start, pcb, text):
    """
    Description:
        Vacuum on with no PCB under the cups, vacuum off 120 mm above the crate floor (a drop), the jaws opened away from
        MG06 with the PCB only in them, and a test with MG06 open are build errors naming the step and the command.
    """

    kinematics, meta = tools
    built = []
    for kind, target in commands:
        speed = 0.2 if kind == 'Movel' else 0.0
        built.append(Program_Command_Str(Kind=kind, Target=target, Speed=speed, Duration=0.35 if kind == 'Gripper' else 2.0,
                                         Text='%s %s' % (kind, target)))
    with pytest.raises(Program_Error_Cls, match=text):
        Build_Program(_Program(built, start=start, pcb=pcb), keypoints, cell_configuration, meta, kinematics, {})


def test_the_build_is_deterministic(program, keypoints, cell_configuration, tools, built, tmp_path):
    """
    Description:
        Building the program twice gives byte-identical trajectory files (npz and CSV) - on one machine and platform (B5).
    """

    kinematics, meta = tools
    again, _ = Build_Program(program, keypoints, cell_configuration, meta, kinematics, {})
    one = [open(p, 'rb').read() for p in Write_Trajectory(built[0], str(tmp_path / 'one' / 'p.npz'))]
    two = [open(p, 'rb').read() for p in Write_Trajectory(again, str(tmp_path / 'two' / 'p.npz'))]
    assert one == two


def test_posing_a_sample_sets_the_whole_cell(built, tools, cell_configuration):
    """
    Description:
        Pose_Sample at the middle of the test step: the robot joints, the parked loader (y -199.5 mm, z +20 mm), MG06 settled
        closed with the press plate on its coupling, the PCB in MG06's place pose.
    """

    kinematics, meta = tools
    trajectory, _ = built
    i = int(np.flatnonzero(trajectory.Step_Index == trajectory.Step_Names.index('test'))[100])
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
        The carried PCB (SPEC §4.10: sprung bellows): picked with the cups pressed 1.5 mm, it stays on the crate floor
        (z 4.9 mm) until the TCP has risen 1.5 mm, then hangs at the cup face (the nominal held pose); at output_place,
        before the vacuum goes off, it is 1.0 mm above its place on the floor (SPEC §4.8; within the keypoint's 0.05 mm stale
        tolerance: measured 1.019 mm) and then lies on the floor.
    """

    from Kinematics.Core import PCB_For_TCP
    kinematics, _ = tools
    trajectory, _ = built
    first, last = _Segment(trajectory, 'Movel above_input_pcb 0.2')
    tcp = np.array([kinematics.Site_Pose(q)[0] for q in trajectory.Robot_Q[first - 1:last]])
    rise = np.linalg.norm(tcp - tcp[0], axis=1)
    z = trajectory.PCB_Pose[first - 1:last, 2]
    assert np.max(np.abs(z[rise <= 0.0015] - 0.0049)) <= 2e-8 and np.sum(rise <= 0.0015) >= 10
    nominal = PCB_For_TCP(*kinematics.Site_Pose(trajectory.Robot_Q[last - 1]))[0]
    assert trajectory.PCB_Pose[last - 1][:3] == pytest.approx(nominal, abs=1e-12)
    place = _Segment(trajectory, 'Movel output_place 0.03')[1] - 1
    assert trajectory.PCB_Pose[place][2] == pytest.approx(0.0049 + 0.001, abs=0.00005) and trajectory.PCB_Pose[place + 1][2] == pytest.approx(0.0049)
