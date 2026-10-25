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
import math
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test
#   ../Program/Core
from Program.Core import Build_Program, Movej_Duration, Program_Error_Cls, Validate_Constants
from Program.Check import Check_Trajectory
#   ../Configuration/Core, ../Configuration/Program_Parser, ../Collision/Core, ../Kinematics/Core, ../Trajectory/Core,
#   ../Keypoints/Core
from Configuration.Core import CONST_PCB_LOCATIONS, CONST_FORK_PATH
from Configuration.Program_Parser import Program_Str, Load_Constants, Load_Simulation, Load_Optimization, Free_Steps, CONST_CONSTANTS_YAML
from Kinematics.Core import PCB_For_TCP
from Trajectory.Core import Write_Trajectory

"""
Description:
    Initialization of constants.

    The program build (FEATURES.md F8, F10): the motion laws, the recorded fork path, the world state, the constants against
    the model. Expected values come from the legacy planner's rules (commit f754e2d0379b06ffb43286f4fb7aabf88e25ca5f), the
    reference plan and SPEC §4, written out here.
"""
# The legacy MoveJ rule's numbers: 60 % of 220 / 230 deg/s, 3 / 6 rad/s^2, at least 0.5 s (SPEC §4.4, §4.10).
#   Unit [Vector<float> 1x6] (rad/s), (rad/s^2), [float] (s)
CONST_V_MAX = 0.6 * np.radians([220, 220, 220, 220, 230, 230])
CONST_A_MAX = np.array([3.0, 3.0, 3.0, 6.0, 6.0, 6.0])
CONST_T_MIN = 0.5


def _Segment(trajectory, text):
    i = trajectory.Segment_Command.index(text)
    return trajectory.Segment_Start[i], trajectory.Segment_End[i]


def test_movej_follows_the_legacy_rule(program_built, constants, cell_configuration):
    """
    Description:
        MoveJ P_HOME -> P_INPUT_ABOVE: T = max(0.5, max 1.875 |dq| / v_max, max sqrt(5.774 |dq| / a_max)) with the legacy
        numbers, round(T / 4 ms) samples; min-jerk: at rest at both ends, the analytic speed's peak 1.875 dq / T. (Its samples
        were the legacy plan's first ones to 1.7e-8 rad, checked until step 6 retired the plan: commit 7a8560c8c16806739a02da8e989c2a2f65d1e41d.)
    """

    q0, q1 = np.array(constants.Points['P_HOME'].Joints), np.array(constants.Points['P_INPUT_ABOVE'].Joints)
    dq = np.abs(q1 - q0)
    expected = max(CONST_T_MIN, np.max(1.875 * dq / CONST_V_MAX), np.max(np.sqrt(5.774 * dq / CONST_A_MAX)))
    assert Movej_Duration(q0, q1, cell_configuration) == pytest.approx(expected, rel=1e-12)
    trajectory, _ = program_built
    first, last = _Segment(trajectory, 'Robot.MoveJ(P_INPUT_ABOVE)')
    n = int(round(expected / 0.004))
    assert (first, last) == (0, n + 1)
    qd = trajectory.Robot_QD[first:last]
    assert np.all(qd[0] == 0) and np.all(qd[-1] == 0)
    assert np.max(np.abs(qd), axis=0) == pytest.approx(1.875 * (q1 - q0).__abs__() / (n * 0.004), rel=1e-3)


def test_a_movej_with_v_runs_at_that_fraction_and_is_checked_at_it(constants, simulation, cell_configuration, program_tools, probe_program):
    """
    Description:
        MoveJ(P_INPUT_ABOVE, v=0.3): the legacy rule with v_max = 0.3 x the datasheet speeds (twice the default's 0.6 time on
        the speed term), the samples measured by the clearance check, and its speed held to 0.3 / 0.6 = 0.5 of the Cell.yaml
        limits; the same move faster than its v= (built at 0.6, checked as v=0.3) fails.
    """

    kinematics, meta, collision = program_tools
    q0, q1 = np.array(constants.Points['P_HOME'].Joints), np.array(constants.Points['P_INPUT_ABOVE'].Joints)
    dq = np.abs(q1 - q0)
    expected = max(CONST_T_MIN, np.max(1.875 * dq / (0.3 * np.radians([220, 220, 220, 220, 230, 230]))), np.max(np.sqrt(5.774 * dq / CONST_A_MAX)))
    slow = probe_program(['Robot.MoveJ(P_INPUT_ABOVE, v=0.3)'], constants, simulation, cell_configuration)
    trajectory, notes = Build_Program(slow, cell_configuration, meta, kinematics, {})
    assert len(trajectory.T) - 1 == int(round(expected / 0.004)) and expected > Movej_Duration(q0, q1, cell_configuration)
    result = Check_Trajectory(trajectory, slow, notes, kinematics, collision, cell_configuration, meta)
    assert result.Verdict == 'PASS' and result.Steps[0].Checked > 100 and result.Steps[0].Peak_Speed <= 0.5 * 1.002
    fast = probe_program(['Robot.MoveJ(P_INPUT_ABOVE)'], constants, simulation, cell_configuration)
    trajectory, notes = Build_Program(fast, cell_configuration, meta, kinematics, {})
    claimed = probe_program(['Robot.MoveJ(P_INPUT_ABOVE, v=0.3)'], constants, simulation, cell_configuration)
    result = Check_Trajectory(trajectory, claimed, notes, kinematics, collision, cell_configuration, meta)
    assert any(f.startswith('Robot.MoveJ(P_INPUT_ABOVE, v=0.3): peak joint speed') for f in result.Steps[0].Failures)


def test_movel_is_straight_and_ends_on_the_stored_joints(program_built, constants, program_tools):
    """
    Description:
        MoveL P_INPUT_APPROACH -> P_INPUT_PICK (15 mm at 30 mm/s): every TCP sample within 0.01 mm of the line, the orientation
        kept, the last sample exactly the point's joints, T = 1.875 L / speed (0.94 s), the TCP's peak speed at most 30 mm/s.
    """

    kinematics, _, _ = program_tools
    trajectory, _ = program_built
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


def test_a_movel_cannot_switch_the_ik_family(constants, simulation, cell_configuration, program_tools, probe_program):
    """
    Description:
        A MoveL from P_HOME to P_MG06_ABOVE (wrist up to wrist down) fails in the build too, naming both wrist_2 angles: a line
        cannot switch families (SPEC §4.7).
    """

    kinematics, meta, _ = program_tools
    with pytest.raises(Program_Error_Cls, match='a MoveL cannot switch families'):
        Build_Program(probe_program(['Robot.MoveL(P_MG06_ABOVE, v=0.2)'], constants, simulation, cell_configuration), cell_configuration, meta,
                      kinematics, {})


def test_a_movel_over_a_joint_limit_is_stretched_and_noted(constants, simulation, cell_configuration, program_tools, probe_program):
    """
    Description:
        MoveL P_HOME -> P_INPUT_ABOVE at 1 m/s (365 mm) would exceed a joint's speed or acceleration limit: it is stretched
        (x1.985, measured 2026-10-24) and noted, and the sampled joint speeds and accelerations then stay within the limits.
    """

    kinematics, meta, _ = program_tools
    trajectory, notes = Build_Program(probe_program(['Robot.MoveL(P_INPUT_ABOVE, v=1.0)'], constants, simulation, cell_configuration),
                                      cell_configuration, meta, kinematics, {})
    assert len(notes) == 1 and notes[0].Text.startswith('stretched x1.985')
    q = np.vstack([trajectory.Robot_Q, trajectory.Robot_Q[-1:]])
    qd = np.diff(q, axis=0) / 0.004
    assert np.all(np.abs(qd) <= CONST_V_MAX) and np.all(np.abs(np.diff(qd, axis=0) / 0.004) <= CONST_A_MAX)


def test_the_fork_paths_follow_the_recorded_rows_and_join_rule(program_built, constants):
    """
    Description:
        PATH_FORK_CLOSE follows rows 1-305 (5.684 s after row 0) from P_MG06_FORK_START and ends exactly on P_MG06_CLOSE_END;
        PATH_FORK_OPEN starts at P_MG06_OPEN_JOIN and follows the rows after row 663 (the first OPEN row with handle <= 28.45
        deg: 28.32 deg) to P_MG06_FORK_START; MG06 is the recorded state during the paths, and the opening path's first row
        moves (the noted velocity step, 39.1 mm/s).
    """

    trajectory, notes = program_built
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


def test_the_pcb_moves_through_the_cycle_by_the_rules(program_built, cell_configuration, constants, simulation):
    """
    Description:
        The PCB's locations in order: input_crate, cups (vacuum on at 3.276 s, as the reference plan), jaws (vacuum off at
        10.164 s), mg06 (the jaws open at MG06), jaws (clamped at MG06), cups (vacuum on at the take), output_crate (vacuum off)
        - at its rest pose on the floor; the jaws stand at G_CLAMP's Jaw_Actual while the PCB is between them, the command at
        G_CLAMP's Jaw; MG06 is settled closed during the test and open at the end.
    """

    trajectory, _ = program_built
    locations = [CONST_PCB_LOCATIONS[i] for i in trajectory.PCB_Location]
    order = [locations[0]] + [b for a, b in zip(locations, locations[1:]) if a != b]
    assert order == ['input_crate', 'cups', 'jaws', 'mg06', 'jaws', 'cups', 'output_crate']
    assert [e for e in trajectory.Events if e[1].startswith('vacuum')][:2] == [(3.276, 'vacuum_on'), (10.164, 'vacuum_off')]
    assert trajectory.PCB_Pose[-1][:3] == pytest.approx([0.40620, -0.75160, 0.0049], abs=1e-9)
    clamp = trajectory.Segment_Command.index('Loader.Run(J_CLAMP) > Gripper(G_CLAMP, t=0.6)')
    assert trajectory.Jaw[trajectory.Segment_End[clamp] - 1] == simulation.Jaw_Actual['G_CLAMP']
    assert trajectory.Gripper[trajectory.Segment_End[clamp] - 1] == constants.Gripper['G_CLAMP'].Jaw
    during = trajectory.Step_Index == trajectory.Step_Names.index('S11_Test')
    assert np.all(trajectory.MG06_Lid[during] == cell_configuration.Closed_Lid)
    assert np.all(trajectory.MG06_Handle[during] == cell_configuration.Closed_Handle)
    assert trajectory.MG06_Lid[-1] < -0.69 and trajectory.MG06_Handle[-1] == 0.0


def test_a_waitdi_holds_its_simulated_response_and_the_cycle_ends_with_the_settle(program_built, simulation):
    """
    Description:
        The WaitDI right after each vacuum on holds Simulation.yaml's 0.3 s response (75 samples); the second one at the take,
        after the jaws opened, costs nothing (the input came long before, F11); the trajectory ends with the simulation's
        End_Settle step (0.8 s, 200 samples) after the program's last segment, S21_Home_End.
    """

    trajectory, _ = program_built
    waits = [trajectory.Segment_End[i] - trajectory.Segment_Start[i] for i, text in enumerate(trajectory.Segment_Command)
             if text == 'Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)']
    assert waits == [75, 75, 0]
    assert trajectory.Step_Names[-2:] == ['S21_Home_End', 'End_Settle'] and trajectory.Segment_Command[-1] == 'End_Settle(0.8)'
    assert np.sum(trajectory.Step_Index == len(trajectory.Step_Names) - 1) == int(round(simulation.End_Settle / 0.004))


def test_a_waitdi_holds_only_what_is_left_of_the_response(constants, simulation, cell_configuration, program_tools, probe_program):
    """
    Description:
        F11: a WaitDI waits for the input to come 0.3 s after its cause (the vacuum on): after a 0.1 s wait it holds the remaining
        0.2 s (50 samples), a second one right after it nothing.
    """

    kinematics, meta, _ = program_tools
    lines = ['Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveL(P_INPUT_APPROACH, v=0.2)', 'Robot.MoveL(P_INPUT_PICK, v=0.03)', 'Robot.SetDO(DO_VACUUM, ON)',
             'Wait(0.1)', 'Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)', 'Robot.WaitDI(DI_VACUUM_OK, timeout=0.5)']
    trajectory, _ = Build_Program(probe_program(lines, constants, simulation, cell_configuration), cell_configuration, meta, kinematics, {})
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
def test_a_broken_world_rule_is_refused(constants, simulation, cell_configuration, program_tools, lines, start, pcb, text, probe_program):
    """
    Description:
        Vacuum on with no PCB under the cups, vacuum off 120 mm above the crate floor (a drop), the jaws opened away from MG06
        with the PCB only in them, a test with MG06 open and a WaitDI for the vacuum with the vacuum off are build errors naming
        the segment and the line.
    """

    kinematics, meta, _ = program_tools
    probe = probe_program(lines, constants, simulation, cell_configuration, start=start, pcb=pcb,
                          loader_jobs={'J_LET_GO': ['Gripper(G_OPEN, t=0.35)']}, tester_jobs={'J_TEST': ['Test(Process.Test_Duration)']})
    with pytest.raises(Program_Error_Cls, match='Step S1_Probe, .*' + text):
        Build_Program(probe, cell_configuration, meta, kinematics, {})


def test_the_build_is_deterministic(program, cell_configuration, program_tools, program_built, tmp_path):
    """
    Description:
        Building the program twice gives byte-identical trajectory files (npz and CSV) - on one machine and platform (B5).
    """

    kinematics, meta, _ = program_tools
    again, _ = Build_Program(program, cell_configuration, meta, kinematics, {})
    one = [open(p, 'rb').read() for p in Write_Trajectory(program_built[0], str(tmp_path / 'one' / 'p.npz'))]
    two = [open(p, 'rb').read() for p in Write_Trajectory(again, str(tmp_path / 'two' / 'p.npz'))]
    assert one == two


def test_the_carried_pcb_hangs_at_the_cup_face_once_the_bellows_extend(program_built, program_tools):
    """
    Description:
        The carried PCB (SPEC §4.10: sprung bellows): picked with the cups pressed 1.5 mm, it stays on the crate floor (z 4.9
        mm) until the TCP has risen 1.5 mm, then hangs at the cup face; at P_OUTPUT_PLACE, before the vacuum goes off, it is
        1.0 mm above its place on the floor (within the point's 0.05 mm stale tolerance: measured 1.019 mm), then on the floor.
    """

    kinematics, _, _ = program_tools
    trajectory, _ = program_built
    first, last = _Segment(trajectory, 'Robot.MoveL(P_INPUT_CLEAR, v=0.20)')
    tcp = np.array([kinematics.Site_Pose(q)[0] for q in trajectory.Robot_Q[first - 1:last]])
    rise = np.linalg.norm(tcp - tcp[0], axis=1)
    z = trajectory.PCB_Pose[first - 1:last, 2]
    assert np.max(np.abs(z[rise <= 0.0015] - 0.0049)) <= 2e-8 and np.sum(rise <= 0.0015) >= 10
    nominal = PCB_For_TCP(*kinematics.Site_Pose(trajectory.Robot_Q[last - 1]))[0]
    assert trajectory.PCB_Pose[last - 1][:3] == pytest.approx(nominal, abs=1e-12)
    place = _Segment(trajectory, 'Robot.MoveL(P_OUTPUT_PLACE, v=0.03)')[1] - 1
    assert trajectory.PCB_Pose[place][2] == pytest.approx(0.0049 + 0.001, abs=0.00005) and trajectory.PCB_Pose[place + 1][2] == pytest.approx(0.0049)


@pytest.mark.parametrize('key, old, new, text', [
    ('  P_HOME:', '    Pose: [0.216200182, ', '    Pose: [0.216220182, ', 'Points.P_HOME.Pose: 0.0200 mm / 0.0000 deg from FK(Joints, T_SUCTION)'),
    ('  P_INPUT_ABOVE:', '    Offset: [0.0, 0.0, -0.1215]', '    Offset: [0.0, 0.0, -0.1216]', 'Points.P_INPUT_ABOVE.Reference: the point is 0.1'),
    ('  T_SUCTION:', '    Flange_Offset: [0.0, 0.0, 0.2606, 0.0, 0.0, 0.0]', '    Flange_Offset: [0.0, 0.0, 0.2607, 0.0, 0.0, 0.0]',
     'Tools.T_SUCTION.Flange_Offset: 0.1000 mm / 0.0000 deg from the model site aubo/tool/suction_tcp'),
    ('  L_PARK:', '    Y: -0.1995', '    Y: -0.25', "Loader_Positions.L_PARK.Y: -0.25 m is outside the model's y_axis range"),
    ('  G_OPEN:', '    Jaw: 0.035', '    Jaw: 0.045', "Gripper_Commands.G_OPEN.Jaw: 0.045 m is outside the jaws' range"),
])
def test_the_constants_are_checked_against_the_model(tmp_path, cell_configuration, program_tools, key, old, new, text):
    """
    Description:
        Validate_Constants (the build runs it first): a Pose 0.02 mm off FK, an Offset 0.1 mm off its Reference's pose (beyond the
        0.05 mm stale tolerance), a tool offset 0.1 mm off its model site, a loader position beyond its axis, a jaw beyond its
        range - each named with Constants.yaml's line and the constant; the build refuses them.
    """

    kinematics, meta, _ = program_tools
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


def test_the_shipped_constants_match_the_model(program, cell_configuration, program_tools):
    """
    Description:
        The shipped Constants.yaml passes Validate_Constants: every Pose, tool offset, derived point, loader position and jaw.
    """

    kinematics, meta, _ = program_tools
    assert Validate_Constants(program, kinematics, cell_configuration, meta) == []


def test_a_run_is_a_start_and_a_waitdone(tmp_path, monkeypatch, program, constants, simulation, cell_configuration, program_tools, program_built):
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
    kinematics, meta, _ = program_tools
    trajectory, _ = Build_Program(lane, cell_configuration, meta, kinematics, {})
    default = program_built[0]
    for channel in ('T', 'Robot_Q', 'Robot_QD', 'Loader_Y', 'Loader_Z', 'Gripper', 'Vacuum', 'Step_Index', 'PCB_Location', 'PCB_Pose', 'MG06_Lid',
                    'MG06_Handle', 'Jaw'):
        assert np.asarray(getattr(default, channel)).tobytes() == np.asarray(getattr(trajectory, channel)).tobytes(), channel
    assert trajectory.Events == default.Events and len(trajectory.Segment_Command) == len(default.Segment_Command) - 6 + 2


def test_the_optimized_splines_run_within_the_limits(constants, simulation, cell_configuration, program_tools):
    """
    Description:
        Every MoveSpline of the committed optimized program: its finite-difference joint speeds and accelerations within the
        Cell.yaml limits (its v= is Speed_Fraction), its stored speeds the spline's, at rest at both ends, ending exactly on
        its end point's Joints.
    """

    from Configuration.Program_Parser import Load_Program, CONST_OPTIMIZED_PROGRAM_YAML
    optimized = Load_Program(constants, simulation, cell_configuration, CONST_OPTIMIZED_PROGRAM_YAML)
    kinematics, meta, _ = program_tools
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
