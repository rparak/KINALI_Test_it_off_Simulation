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
# File Name: ../test/Program/test_check.py
## =========================================================================== ##

# System (Default)
import copy
import dataclasses
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test and what it checks
#   ../Program/Check, ../Program/Core
from Program.Core import Build_Program
from Program.Check import (Check_Trajectory, Format_Check_Report, Jaw_Posing_Faults, Jaw_Speed_Faults, Pose_Sample, Loader_Actual,
                           Loader_Actual_Trajectory)
#   ../Configuration/Core, ../Configuration/Program_Parser, ../Collision/Core, ../Kinematics/Core, ../Trajectory/Core,
#   ../Keypoints/Core
from Configuration.Core import Read_PCB_Meta
from Configuration.Program_Parser import CONST_PCB_LOCATIONS, Load_Optimization, Free_Steps
from Keypoints.Core import Resolve_Keypoint

"""
Description:
    The program check (FEATURES.md F8; SPEC §7.9): the default program passes it, every rule it holds is broken once and
    reported - collisions, contact groups and depths, the MoveJ rule limits, the jaws' posing and speed, the handover's
    press, descent and centring, the re-grip - and its report; posing at a sample; the loader's actuator model.
"""


def test_the_posed_jaws_are_the_command_limited_by_the_part(program_built, simulation):
    """
    Description:
        Item 10 (owner, 2026-10-25): at all four gripper operations (the handover clamp, the release in MG06, the re-grip, the
        release at the take) the posed jaw gap is monotonic in the command's direction and never closer than grip with the PCB in
        the jaws; the release in MG06 opens from grip (Jaw_Actual), not from the clamp command (5.75 mm).
    """

    trajectory, _ = program_built
    grip = simulation.Jaw_Actual['G_CLAMP']
    assert Jaw_Posing_Faults(trajectory, grip) == []
    moving = np.flatnonzero(np.abs(np.diff(trajectory.Gripper)) > 0.0)
    operations = np.split(moving, np.flatnonzero(np.diff(moving) > 1) + 1)
    assert len(operations) == 4
    release = operations[1]
    assert trajectory.Jaw[release[0]:release[0] + 3].tolist() == [grip] * 3 and trajectory.Gripper[release[0]] < grip


def test_the_old_jaw_posing_is_refused(program_built, simulation):
    """
    Description:
        Item 10, forced failure: the old posing of the release in MG06 (the raw command once the PCB is in MG06: the jaws jump
        from grip, 17.54 mm, to the clamp command, 5.75 mm, through the PCB, then open) closes the posed jaws by 11.79 mm while
        the gripper opens;
        a jaw closer than grip with the PCB in the jaws is refused too.
    """

    trajectory, _ = program_built
    grip = simulation.Jaw_Actual['G_CLAMP']
    old = copy.deepcopy(trajectory)
    moving = np.flatnonzero(np.abs(np.diff(old.Gripper)) > 0.0)
    release = np.split(moving, np.flatnonzero(np.diff(moving) > 1) + 1)[1]
    mg06 = CONST_PCB_LOCATIONS.index('mg06')
    switched = int(release[0]) + int(np.argmax(old.PCB_Location[release[0]:] == mg06))
    old.Jaw = np.array(old.Jaw)
    old.Jaw[switched:release[-1] + 2] = old.Gripper[switched:release[-1] + 2]
    faults = Jaw_Posing_Faults(old, grip)
    assert len(faults) == 1 and faults[0][0] == switched and faults[0][1].startswith('the posed jaws close by 11.79 mm at ')
    squeezed = copy.deepcopy(trajectory)
    squeezed.Jaw = np.array(squeezed.Jaw)
    inside = int(np.flatnonzero(squeezed.PCB_Location == CONST_PCB_LOCATIONS.index('jaws'))[100])
    squeezed.Jaw[inside] = grip - 0.001
    assert [f for _, f in Jaw_Posing_Faults(squeezed, grip) if 'closer than grip' in f] == [
        'the posed jaws are 1.00 mm closer than grip with the PCB in them at %.3f s' % squeezed.T[inside]]


def test_the_gripper_ramps_within_the_datasheet_jaw_speed(program_built, cell_configuration):
    """
    Description:
        Item 3 (owner, 2026-10-25): every Gripper command ramps (29.25 mm per jaw in 0.6 s, min-jerk) at most at
        Motion.Jaw_Speed_Max (100 mm/s, the SCHUNK EGI 040-EI): its peak 91.4 mm/s = 1.875 x 29.25 mm / 0.6 s.
    """

    trajectory, _ = program_built
    assert Jaw_Speed_Faults(trajectory, cell_configuration.Jaw_Speed_Max) == []
    peak = np.max(np.abs(np.diff(trajectory.Gripper))) / 0.004
    assert peak == pytest.approx(1.875 * 0.02925 / 0.6, rel=2e-3) and peak <= cell_configuration.Jaw_Speed_Max


def test_a_gripper_ramp_faster_than_the_jaw_speed_is_refused(constants, simulation, cell_configuration, program_tools, probe_program, check_program):
    """
    Description:
        Item 3, forced failure: the legacy 0.35 s clamp ramps at 157.6 mm/s and fails the check, naming the speed and the limit.
    """

    lines = ['Robot.MoveJ(P_LOADER_ABOVE)', 'Robot.MoveL(P_LOADER_APPROACH, v=0.2)', 'Robot.MoveL(P_LOADER_PLACE, v=0.03)', 'Loader.Run(J_CLAMP)']
    probe = probe_program(lines, constants, simulation, cell_configuration, pcb='cups', allow=('cups_pcb', 'pcb_jaws'),
                          loader_jobs={'J_CLAMP': ['Gripper(G_CLAMP, t=0.35)']})
    failures = [f for s in check_program(probe, cell_configuration, program_tools).Steps for f in s.Failures]
    assert len(failures) == 1 and failures[0].startswith('the gripper command ramps at 157.6 mm/s at ')
    assert 'faster than Motion.Jaw_Speed_Max 100 mm/s' in failures[0]


def test_posing_a_sample_sets_the_whole_cell(program_built, program_tools, cell_configuration):
    """
    Description:
        Pose_Sample in the middle of S11_Test: the robot joints, the parked loader (y -199.5 mm, z +20 mm), MG06 settled closed
        with the press plate on its coupling, the PCB in MG06's place pose.
    """

    kinematics, meta, _ = program_tools
    trajectory, _ = program_built
    i = int(np.flatnonzero(trajectory.Step_Index == trajectory.Step_Names.index('S11_Test'))[100])
    Pose_Sample(kinematics, trajectory, i)
    a, q = kinematics.Scene_Address, kinematics.Data.qpos
    assert np.array_equal(q[kinematics.Q_Address], trajectory.Robot_Q[i])
    # The loader's moves end exactly on their targets.
    assert (q[a['y_axis']], q[a['z_axis']]) == (-0.1995, 0.02) and q[a['handle_hinge']] == cell_configuration.Closed_Handle
    assert q[a['press_slide']] == pytest.approx(0.0104554 * cell_configuration.Closed_Handle ** 3, rel=1e-5)
    assert kinematics.Data.xpos[kinematics.Model.body('pcb').id] == pytest.approx(meta.Place_Pos, abs=1e-9)


def test_the_default_program_passes_its_check(program_checked):
    """
    Description:
        The default program's check (measured 2026-10-24): every step passes; S9_Close_MG06's deepest handle_bar overlap is the
        recorded closing path's 0.54 mm, within Cell.yaml's 0.6 mm; every allowed group occurs where declared; the press at the
        pick is 1.48 mm (cups_pcb); holds are skipped (fewer samples measured than exist); the cycle is 70.404 s (the gripper's
        0.6 s ramps, B work item 3).
    """

    assert [s.Name for s in program_checked.Steps if s.Verdict == 'FAIL'] == [] and program_checked.Verdict == 'PASS'
    close = next(s for s in program_checked.Steps if s.Name == 'S9_Close_MG06')
    assert close.Allowed['handle_bar'].Distance == pytest.approx(-0.000539, abs=0.000002)
    pick = next(s for s in program_checked.Steps if s.Name == 'S2_Pick_Input')
    assert pick.Allowed['cups_pcb'].Distance == pytest.approx(-0.00148, abs=0.00002) and pick.Allowed['pcb_crate'] is not None
    assert all(f is not None for s in program_checked.Steps for f in s.Allowed.values())
    assert 0 < program_checked.Checked < 17602 and program_checked.Cycle_Time == pytest.approx(70.404, abs=1e-9)


def test_the_handover_keeps_the_board_off_the_ledges(program_checked):
    """
    Description:
        B1 (SPEC §4.7): the PCB-ledge gap with the jaws at grip is 0.299 mm at P_LOADER_PLACE (0.318 mm before the lateral
        re-teach, B work item 2) and while the jaws clamp the PCB on the cups, and never smaller along the MoveL into it (0.3 +-
        0.1 mm): no press; the PCB centre is 0.4 mm from the jaw centreline (its physics position on the cups is on it).
    """

    h = program_checked.Handover
    assert h.Failures == [] and h.At_Handover == pytest.approx(0.000299, abs=0.000002)
    assert h.Line_Min == pytest.approx(h.At_Handover, abs=2e-6) and h.Clamped_Min == pytest.approx(0.000299, abs=0.000002)
    assert h.Clamped_Max == pytest.approx(h.Clamped_Min, abs=1e-9) and h.Lateral == pytest.approx(0.0004, abs=0.000002)


def test_a_press_at_the_handover_is_reported(constants, simulation, cell_configuration, program_tools, probe_program, check_program):
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
    probe = probe_program(lines, pressed, pressed_simulation, cell_configuration, pcb='cups', allow=('cups_pcb', 'pcb_jaws'),
                          loader_jobs={'J_CLAMP': ['Gripper(G_CLAMP, t=0.6)']})
    result = check_program(probe, cell_configuration, program_tools)
    assert any(f.startswith('the PCB-ledge gap at P_LOADER_PLACE is') and f.endswith('(a press)') for f in result.Handover.Failures)
    assert result.Verdict == 'FAIL'


def _Lateral_Handover(constants, simulation, cell, program_tools, probe_program, check_program, dx, lines=None):
    """
    Description:
        A handover probe with P_LOADER_PLACE taught dx along its tool x (the jaw axis): its PCB_Hold offset [dx, 0, 0] resolved
        by IK, the points derived from it detached; checked.
    """

    kinematics, meta, _ = program_tools
    moved, moved_simulation = copy.deepcopy(constants), copy.deepcopy(simulation)
    moved_simulation.Points['P_LOADER_PLACE'].Hold_Offset = [dx, 0.0, 0.0]
    joints, pose = Resolve_Keypoint('P_LOADER_PLACE', kinematics, moved, moved_simulation, meta, cell)
    moved.Points['P_LOADER_PLACE'].Joints, moved.Points['P_LOADER_PLACE'].Pose = list(joints), list(pose)
    for point in moved.Points.values():
        if point.Reference == 'P_LOADER_PLACE':
            point.Reference, point.Offset = None, None
    lines = lines or ['Robot.MoveJ(P_LOADER_ABOVE)', 'Robot.MoveL(P_LOADER_APPROACH, v=0.2)', 'Robot.MoveL(P_LOADER_PLACE, v=0.03)',
                      'Loader.Run(J_CLAMP)']
    probe = probe_program(lines, moved, moved_simulation, cell, pcb='cups', allow=('cups_pcb', 'pcb_jaws'),
                          loader_jobs={'J_CLAMP': ['Gripper(G_CLAMP, t=0.6)']})
    return check_program(probe, cell, program_tools)


def test_the_descent_measures_the_height_above_the_ledges_only(constants, simulation, cell_configuration, program_tools, probe_program,
                                                               check_program):
    """
    Description:
        Item 1 (owner, 2026-10-25): a handover taught 0.4 mm along the jaw axis (the re-teach's size) passes - along the
        descent the jaws are centred on the PCB, so the gap is its height above the finger ledges (with the jaws at grip about
        the uncentred PCB it was -0.186 mm, the board in a jaw's pad corner): the descent's gap is the gap at the point plus the
        PCB's height above it along the plate normal (with the jaws at the physics grip, Jaw_Actual 17.54 mm, beside the board
        the lead-in faces gave 0.078 mm); the lateral offset is reported, 0.400 mm.
    """

    result = _Lateral_Handover(constants, simulation, cell_configuration, program_tools, probe_program, check_program, 0.0004)
    h = result.Handover
    assert h.Failures == [] and h.Lateral == pytest.approx(0.0004, abs=2e-6)
    assert h.Line_Min == pytest.approx(h.At_Handover, abs=2e-6) and h.At_Handover == pytest.approx(0.000299, abs=0.000002)


def test_a_dip_below_the_ledges_along_the_descent_is_reported(constants, simulation, cell_configuration, program_tools, probe_program, check_program):
    """
    Description:
        Item 1, forced failure: a descent that goes 1.5 mm deeper (P_LOADER_PICK) and back up to P_LOADER_PLACE still fails
        along the line with the jaws centred: the gap falls below 0.2 mm, '(a press)'.
    """

    lines = ['Robot.MoveJ(P_LOADER_ABOVE)', 'Robot.MoveL(P_LOADER_PICK, v=0.03)', 'Robot.MoveL(P_LOADER_PLACE, v=0.03)', 'Loader.Run(J_CLAMP)']
    probe = probe_program(lines, constants, simulation, cell_configuration, pcb='cups', allow=('cups_pcb', 'pcb_jaws'),
                          loader_jobs={'J_CLAMP': ['Gripper(G_CLAMP, t=0.6)']})
    h = check_program(probe, cell_configuration, program_tools).Handover
    assert any(f.startswith('the PCB-ledge gap falls to') and f.endswith('(a press)') for f in h.Failures), h.Failures


def test_a_handover_off_the_jaw_centreline_is_reported(constants, simulation, cell_configuration, program_tools, probe_program, check_program):
    """
    Description:
        Item 1, forced failure: P_LOADER_PLACE taught 0.6 mm along the jaw axis is more than Handover.Lateral_Max (0.5 mm) from the
        jaw centreline and fails, naming the offset.
    """

    h = _Lateral_Handover(constants, simulation, cell_configuration, program_tools, probe_program, check_program, 0.0006).Handover
    assert h.Lateral == pytest.approx(0.0006, abs=2e-6)
    found = [f for f in h.Failures if 'jaw centreline' in f]
    assert len(found) == 1 and found[0] == ('the PCB centre is %.3f mm from the jaw centreline at P_LOADER_PLACE, more than Handover.Lateral_Max '
                                            '0.5 mm: the jaws would pull it off the cups as they centre it' % (1000 * abs(h.Lateral)))


def _Regrip(constants, simulation, cell, program_tools, probe_program, check_program, z):
    """
    Description:
        A probe of the re-grip from MG06: the PCB resting in MG06, the loader's J_FETCH down to L_FETCH at height z (m); checked.
    """

    moved = copy.deepcopy(constants)
    moved.Loader['L_FETCH'].Z = z
    jobs = {'J_FETCH': ['MoveZ(L_TRAVEL, t=0.6)', 'MoveY(L_TRAVEL, t=2.2)', 'MoveZ(L_FETCH, t=0.6)', 'Wait(0.1)', 'Gripper(G_CLAMP, t=0.6)']}
    probe = probe_program(['Loader.Run(J_FETCH)'], moved, simulation, cell, pcb='mg06', allow=('pcb_jaws',), loader_jobs=jobs)
    return check_program(probe, cell, program_tools).Regrip


def test_the_regrip_keeps_the_board_off_the_ledges(program_checked):
    """
    Description:
        Item 5 (owner, 2026-10-25): the re-grip from MG06 at L_FETCH has the handover's no-press rule - the PCB-ledge gap with the
        jaws at grip 0.265 mm about the PCB at its MG06 rest pose (0.3 +- 0.1 mm; 0.359 mm settled in physics) - and the open jaws
        keep 7.13 mm from the PCB and the holder over the descent.
    """

    r = program_checked.Regrip
    assert r.Failures == [] and r.Gap == pytest.approx(0.000265, abs=0.000002)
    assert r.Descent_Min == pytest.approx(0.00713, abs=0.00002) and r.Descent_Pair.startswith('jaw_R / mg06_base at ')


@pytest.mark.parametrize('lift, failure', [
    (0.00037 - 0.000035, 'the PCB-ledge gap at the re-grip from MG06 is -0.066 mm, not 0.3 +- 0.1 mm (the jaws clamp on the ledge corners)'),
    (-0.0085, 'the open jaws touch '),
])
def test_a_regrip_on_the_ledges_or_into_the_holder_is_reported(constants, simulation, cell_configuration, program_tools, lift, failure, probe_program,
                                                               check_program):
    """
    Description:
        Item 5, forced failures: L_FETCH at the raised L_PLACE (the jaws close with the ledges 0.066 mm above the board's
        underside) fails the gap; L_FETCH 8.5 mm below the shipped height drives the open jaws into the holder during the descent.
    """

    regrip = _Regrip(constants, simulation, cell_configuration, program_tools, probe_program, check_program, constants.Loader['L_FETCH'].Z + lift)
    assert any(f.startswith(failure) for f in regrip.Failures), regrip.Failures


def test_a_colliding_step_is_reported_with_its_pair_and_time(constants, simulation, cell_configuration, program_tools, probe_program, check_program):
    """
    Description:
        A MoveL from 15 mm above the PCB in the input crate straight across to 15 mm above the output crate's floor (through
        both crates' walls) fails, naming a cup and a crate wall, the sample and its time.
    """

    probe = probe_program(['Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveL(P_INPUT_APPROACH, v=0.2)', 'Robot.MoveL(P_OUTPUT_APPROACH, v=0.2)'], constants,
                          simulation, cell_configuration)
    step = check_program(probe, cell_configuration, program_tools).Steps[0]
    assert step.Verdict == 'FAIL' and step.Closest.Distance < 0 and 'crate_' in step.Closest.Pair
    assert any(f.startswith(step.Closest.Pair) and 'at sample %d (%.3f s)' % (step.Closest.Sample, step.Closest.Time) in f for f in step.Failures)


def test_contacts_are_allowed_only_where_declared_and_as_deep_as_configured(constants, simulation, cell_configuration, program_tools, probe_program,
                                                                            check_program):
    """
    Description:
        The pick's press without cups_pcb declared fails (group not allowed); declared, with cups_pcb allowed only 1.0 mm deep it
        fails on the depth (1.48 mm); a declared group that never occurs (pcb_jaws at the crate) fails.
    """

    lines = ['Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveL(P_INPUT_APPROACH, v=0.2)', 'Robot.MoveL(P_INPUT_PICK, v=0.03)']
    undeclared = check_program(probe_program(lines, constants, simulation, cell_configuration), cell_configuration, program_tools)
    assert any('group cups_pcb not allowed in this step' in f for f in undeclared.Steps[0].Failures)
    shallow = copy.deepcopy(cell_configuration)
    shallow.Allowed_Depth['cups_pcb'] = 0.001
    deep = check_program(probe_program(lines, constants, simulation, cell_configuration, allow=('cups_pcb', 'pcb_jaws')), shallow, program_tools)
    failures = deep.Steps[0].Failures
    assert any(f.startswith('cups_pcb 1.48 mm deep') and 'at most 1.00 mm' in f for f in failures)
    assert any('allowed contact pcb_jaws does not occur in the step' in f for f in failures)


def test_a_movej_over_its_rule_limits_is_reported(constants, simulation, cell_configuration, program_tools, probe_program):
    """
    Description:
        Checked against acceleration limits 10 % below the ones the MoveJ was timed with, the MoveJ P_HOME -> P_INPUT_ABOVE
        (acceleration-limited) fails, naming its line, its peak and the allowed fraction.
    """

    kinematics, meta, collision = program_tools
    probe = probe_program(['Robot.MoveJ(P_INPUT_ABOVE)'], constants, simulation, cell_configuration)
    trajectory, notes = Build_Program(probe, cell_configuration, meta, kinematics, {})
    slower = copy.deepcopy(cell_configuration)
    slower.Acceleration_Limits = [0.9 * a for a in slower.Acceleration_Limits]
    result = Check_Trajectory(trajectory, probe, notes, kinematics, collision, slower, meta)
    assert any(f.startswith('Robot.MoveJ(P_INPUT_ABOVE): peak joint speed') for f in result.Steps[0].Failures)


def test_the_report_lists_steps_groups_handover_and_failures(program_checked, program_built, cell_configuration):
    """
    Description:
        The report: a header with the cycle time; a row per step with its peaks and verdict, the closest pair per allowed group
        and of the rest with sample and time; the handover's gaps; the stretched MoveLs; the opening path's note; the events;
        the points used (24: 23 PASS, 1 WARN); no failure; the closing sentence on simulation results.
    """

    lines = Format_Check_Report(program_checked, program_built[0], cell_configuration)
    text = '\n'.join(lines)
    assert lines[0].startswith('program Program: 22 steps, 17602 samples (4 ms), cycle time 70.404 s;')
    assert any(line.startswith('S2_Pick_Input ') and line.endswith('PASS') for line in lines)
    assert '    cups_pcb     -1.48 mm at ' in text and '    not allowed  ' in text
    assert ('handover (no press, 0.3 +- 0.1 mm): PCB-ledge gap with the jaws at grip - along the MoveL into P_LOADER_PLACE (the PCB\'s height '
            'above the ledges) min 0.299 mm') in text and 'mm from the jaw centreline (<= 0.5 mm): PASS' in text
    assert 'stretched MoveLs: none' in text and 'note: S13_Open_MG06, Robot.MovePath(PATH_FORK_OPEN): the recorded path starts moving' in text
    assert 'events: 3.276 s vacuum_on, ' in text and 'keypoints used: 24 (0 FAIL, 1 WARN, 23 PASS;' in text
    assert lines[-1] == 'Program: the program check passed. Simulation results only: not validated for the real cell.'


def test_the_free_transfers_keep_the_loader_clearance(program_checked, program_built, program, cell_configuration, program_tools):
    """
    Description:
        F11: in the taught program's free transfers the robot and the carried PCB keep Loader_Free + Tracking_Margin (14 mm) from
        the loader where its actuators put it: S3_To_Loader 20.59 mm (the PCB / z_carriage), the others beyond the 50 mm cut;
        with Loader_Free 20 mm S3_To_Loader fails naming the pair, the sample and the rule, and only it.
    """

    loader = {s.Name: s.Loader for s in program_checked.Steps if s.Loader is not None}
    assert set(loader) == Free_Steps(program, Load_Optimization(program))
    assert loader['S3_To_Loader'].Pair == 'pcb / z_carriage' and loader['S3_To_Loader'].Distance == pytest.approx(0.02059, abs=0.00001)
    assert all(found.Sample == -1 for name, found in loader.items() if name != 'S3_To_Loader')
    kinematics, meta, collision = program_tools
    strict = dataclasses.replace(cell_configuration, Loader_Free=0.02)
    result = Check_Trajectory(program_built[0], program, program_built[1], kinematics, collision, strict, meta, {'S3_To_Loader', 'S19_To_Output'})
    failed = {s.Name: s.Failures for s in result.Steps if s.Failures}
    assert list(failed) == ['S3_To_Loader'] and failed['S3_To_Loader'][0].startswith('pcb / z_carriage 20.59 mm from the loader at sample 1399 '
                                                                                     '(5.596 s), the loader at its actuator model')
    assert failed['S3_To_Loader'][0].endswith('a free transfer keeps Clearance.Loader_Free 20.0 mm + Tracking_Margin 4.0 mm')


def test_the_loader_actuator_model_lags_and_settles(program_built, program_tools):
    """
    Description:
        F11: Loader_Actual follows the commands through the model's actuators: equal at the start; during J_INSERT's y move
        behind the command by up to tens of millimetres (the physics run's loader lag is 22.6 mm); settled in the test wait (y
        within 0.01 mm, z within its gravity droop, load / kp, 0.44 mm); the PCB in the jaws moves with it (Loader_Actual_Trajectory).
    """

    kinematics = program_tools[0]
    tr = program_built[0]
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
def test_the_loader_actuator_model_matches_the_physics(program_built, program, cell_configuration):
    """
    Description:
        F11: over the taught program's first 21 s (J_INSERT and J_PARK) the loader in physics is within 0.5 mm (y) and 1.1 mm
        (z) of Loader_Actual (measured 0.17 mm / 1.04 mm on the optimized program, 2026-10-25), where its command is up to
        22.6 mm off.
    """

    from Scene.Core import Build_Cell_Model
    from Simulation.Core import Trajectory_Drive_Cls, Physics_Run_Cls
    tr = program_built[0]
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
