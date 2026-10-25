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
# File Name: ../test/Report/test_core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test and what it works on
#   ../Report/Core
from Report.Core import (Moving, Spans, Travel, Channel_Peaks, Segment_Kind, Critical_Path, Robot_Waits, Signal_Order, Wrist_Approach, Job_Window,
                         Stretch_Job, Stretched_Distance, Handovers, Custody_Gaps, Tilt, Location_Spans, Geom_Points, Cell_Geometry_Cls, By_Design,
                         Overhang, Clear_Height, Physics_Record_Str, CONST_PAIRS, CONST_ROBOT_LINKS)
#   ../Program/Check - the loader's actual position, its distance to the robot
from Program.Check import Loader_Actual_Trajectory, Robot_Loader_Distance
#   ../Trajectory/Core - the format
from Trajectory.Core import Trajectory_Str, CONST_DT

"""
Description:
    The comparison's measurements (FEATURES.md F14, SPEC §7.12): on small made-up inputs where the answer is known, and on the
    shipped programs built in process (the optimized_built fixture).
"""


def _Taught(optimized_built):
    return optimized_built['default']


def _Optimized(optimized_built):
    return optimized_built['trajectory']


def test_moving_marks_the_samples_reached_by_a_change_and_spans_join_them():
    """
    Description:
        A sample moves when it differs from the one before (beyond 1e-9); sample 0 never moves. A span of moving samples i..j
        covers t[i] - dt .. t[j].
    """

    values = np.array([0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 3.0, 3.0])
    t = np.arange(8) * 0.5
    mask = Moving(values)
    assert mask.tolist() == [False, False, True, True, False, False, True, False]
    assert Spans(mask, t) == [(0.5, 1.5), (2.5, 3.0)]
    assert not Moving(1e-10 * np.arange(8)).any()


def test_travel_and_peaks_of_a_sine():
    """
    Description:
        sin(2 t) over 0..2 pi travels 8 (four swings of 2); its peak speed is 2 and its peak acceleration 4 (central
        differences, at 1 ms).
    """

    t = np.arange(0.0, 2 * math.pi, 1e-3)
    x = np.sin(2 * t)
    assert Travel(x)[0] == pytest.approx(8.0, abs=1e-3)
    speed, acceleration = Channel_Peaks(x, 1e-3)
    assert speed[0] == pytest.approx(2.0, abs=1e-3) and acceleration[0] == pytest.approx(4.0, abs=1e-2)


@pytest.mark.parametrize('text, parallel, kind', [('Robot.MoveL(P_INPUT_PICK, v=0.03)', False, 'robot motion'),
                                                  ('Robot.MoveSpline(W_TO_TAKE, v=0.6)', True, 'robot motion'),
                                                  ('Loader.WaitDone(J_RETURN)', True, 'loader'),
                                                  ('Loader.Run(J_CLAMP) > Gripper(G_CLAMP, t=0.6)', False, 'loader'),
                                                  ('Wait(1.648)', True, 'loader (timed wait)'), ('Wait(0.25)', False, 'robot dwell'),
                                                  ('Tester.Run(J_TEST) > Test(Process.Test_Duration)', False, 'tester'),
                                                  ('Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)', False, 'vacuum signal'),
                                                  ('Loader.Start(J_FETCH)', True, 'loader start'), ('End_Settle(0.8)', False, 'robot dwell')])
def test_a_segment_is_spent_on_what_the_robot_does_or_waits_for(text, parallel, kind):
    """
    Description:
        Segment_Kind: a motion, or what the robot waits for; a Wait in a step that runs a loader job in parallel waits for the
        loader by its time.
    """

    assert Segment_Kind(text, parallel) == kind


def test_the_critical_path_adds_up_to_each_cycle(optimized_built):
    """
    Description:
        The robot's timeline covers the cycle: the kinds sum to the last sample's time in both programs (taught 70.404 s,
        optimized 58.716 s); only the optimized program has the timed wait, 1.648 s.
    """

    for trajectory, cycle in ((_Taught(optimized_built), 70.404), (_Optimized(optimized_built), 58.716)):
        path = Critical_Path(trajectory)
        assert sum(path.values()) == pytest.approx(trajectory.T[-1], abs=1e-9) and trajectory.T[-1] == pytest.approx(cycle, abs=1e-9)
    assert 'loader (timed wait)' not in Critical_Path(_Taught(optimized_built))
    assert Critical_Path(_Optimized(optimized_built))['loader (timed wait)'] == pytest.approx(1.648, abs=1e-9)


def test_the_waits_and_the_handshake_of_the_optimized_program(optimized_built):
    """
    Description:
        Robot_Waits lists the timed Wait(1.648) after Loader.Start(J_RETURN) at 41.864 s; Signal_Order has the job's start
        before its WaitDone.
    """

    tr = _Optimized(optimized_built)
    waits = {text: (t, d, kind) for t, d, kind, text in Robot_Waits(tr)}
    assert waits['Wait(1.648)'][1:] == (pytest.approx(1.648), 'loader (timed wait)') and waits['Wait(1.648)'][0] == pytest.approx(41.864)
    lines = [text for _, text in Signal_Order(tr)]
    assert lines.index('Loader.Start(J_RETURN)') < lines.index('Loader.WaitDone(J_RETURN)')


def test_the_wrist_singularity_is_met_only_in_the_family_switch(optimized_built):
    """
    Description:
        Both programs cross wrist_2 = 0 only where the IK family switches (taught 0.00 deg in its MoveJ to P_HOME, optimized
        0.02 deg in W_TO_TAKE_WAIT); outside it the taught program comes to 0.94 deg (it stops at P_VIA_HOME_MG06), the
        optimized one to 72.5 deg; the Cartesian moves stay 72.5 deg away in both.
    """

    taught, optimized = Wrist_Approach(_Taught(optimized_built)), Wrist_Approach(_Optimized(optimized_built))
    assert optimized['whole cycle'][0] < math.radians(0.05) and optimized['whole cycle'][2].startswith('Robot.MoveSpline(W_TO_TAKE_WAIT')
    assert math.degrees(taught['outside the family switch'][0]) == pytest.approx(0.94, abs=0.01)
    assert math.degrees(optimized['outside the family switch'][0]) == pytest.approx(72.46, abs=0.01)
    assert taught['Cartesian moves'][0] == optimized['Cartesian moves'][0]


def test_a_job_slowed_by_one_is_the_trajectory_itself(optimized_built):
    """
    Description:
        Stretch_Job at factor 1: every channel from sample 0 to the job's window end is the trajectory's own, sample by sample.
    """

    tr = _Optimized(optimized_built)
    stretched, window = Stretch_Job(tr, 'J_RETURN', 1.0)
    n = window.stop
    for name in ('T', 'Robot_Q', 'Loader_Y', 'Loader_Z', 'Gripper', 'Jaw', 'PCB_Location', 'PCB_Pose', 'MG06_Lid', 'MG06_Handle'):
        assert np.array_equal(getattr(stretched, name), getattr(tr, name)[:n]), name


def test_a_slowed_job_holds_the_robot_at_its_waitdone(optimized_built):
    """
    Description:
        At factor 1.5 the job's window is 1.5 times its loader part; the robot runs its own lines as programmed and then holds
        its WaitDone's joints to the window's end; the loader's channels are the job's at 1 / 1.5 of the time since its start
        (linear between samples), so its Y reaches home 50 % later.
    """

    tr = _Optimized(optimized_built)
    a, w, last = Job_Window(tr, 'J_RETURN')
    stretched, window = Stretch_Job(tr, 'J_RETURN', 1.5)
    assert len(window) == int(math.ceil(1.5 * (last - a))) + 1
    assert np.array_equal(stretched.Robot_Q[a:w + 1], tr.Robot_Q[a:w + 1]) and np.all(stretched.Robot_Q[w:] == tr.Robot_Q[w])
    z = tr.Loader_Z
    assert stretched.Loader_Z[a + 1] == pytest.approx(z[a] + (z[a + 1] - z[a]) / 1.5, abs=1e-15) and z[a + 1] != z[a]
    home = np.flatnonzero(stretched.Loader_Y[a:] == tr.Loader_Y[last])[0]
    assert home == pytest.approx(1.5 * (np.flatnonzero(tr.Loader_Y[a:] == tr.Loader_Y[last])[0]), abs=2)


def test_the_slowed_return_breaks_i8_within_three_percent(optimized_built, cell_configuration):
    """
    Description:
        The optimized program's timed Wait(1.648): as programmed, the closest robot - loader distance over J_RETURN is the
        interlock check's own (19.39 mm, the same function on the same samples); with J_RETURN 3 % slower it falls below
        Loader_Free + Tracking_Margin (14 mm), 10 % slower the robot meets the Z carriage (measured 2026-10-25: 11.78 mm,
        -4.95 mm).
    """

    tr, kinematics, collision = _Optimized(optimized_built), optimized_built['kinematics'], optimized_built['collision']
    a, _, _ = Job_Window(tr, 'J_RETURN')
    as_is = Stretched_Distance(tr, 'J_RETURN', 1.0, kinematics, collision)
    stretched, window = Stretch_Job(tr, 'J_RETURN', 1.0)
    direct = Robot_Loader_Distance(Loader_Actual_Trajectory(stretched, kinematics.Model), kinematics, collision, window)
    assert as_is[0] == direct[0] and as_is[0] == pytest.approx(0.01939, abs=1e-4) and as_is[2] == 'aubo/tool/ee_tool / z_carriage'
    assert Stretched_Distance(tr, 'J_RETURN', 1.03, kinematics, collision)[0] < cell_configuration.Loader_Free + cell_configuration.Tracking_Margin
    assert Stretched_Distance(tr, 'J_RETURN', 1.10, kinematics, collision)[0] < 0.0


def test_the_take_spline_is_clear_with_the_loader_y_at_home(optimized_built, cell_configuration):
    """
    Description:
        The proposal's premise: with the loader at its home Y, the whole W_TO_TAKE spline keeps more than 50 mm (the measure's
        cut) from it at every Z from home to the travel height.
    """

    tr = _Optimized(optimized_built)
    need = cell_configuration.Loader_Free + cell_configuration.Tracking_Margin
    best, found = Clear_Height(tr, 'Robot.MoveSpline(W_TO_TAKE, v=0.6)', optimized_built['kinematics'], optimized_built['collision'],
                               np.linspace(0.0, 0.0792, 5), need)
    assert best == pytest.approx(0.0792) and all(d >= 0.05 for _, d in found)


def _Record(cups, holders, z=None):
    n = len(cups)
    qpos = np.zeros((n, 7))
    qpos[:, 3] = 1.0
    if z is not None:
        qpos[:, 2] = z
    return Physics_Record_Str(T=np.arange(n) * CONST_DT, Qpos=qpos, Qvel=np.zeros((n, 6)), Cups=np.array(cups, np.int8),
                              Holders=np.array(holders, bool), Vacuum=np.zeros(n, np.int8), Tcp=np.zeros((n, 3)))


def _Locations(locations):
    n = len(locations)
    pose = np.zeros((n, 7))
    pose[:, 3] = 1.0
    return Trajectory_Str(T=np.arange(n) * CONST_DT, PCB_Location=np.array(locations, np.int8), PCB_Pose=pose)


def test_a_handover_with_both_holders_on_the_pcb():
    """
    Description:
        Crate to cups at sample 4: the cups touch from sample 2, the crate until sample 5: both hold it 4 samples, none never;
        no pose error.
    """

    record = _Record([0, 0, 4, 4, 4, 4, 4, 4, 4, 4], [[0, 0, 1]] * 6 + [[0, 0, 0]] * 4)
    found = Handovers(_Locations([0, 0, 0, 0, 1, 1, 1, 1, 1, 1]), record, 0)
    assert len(found) == 1
    h = found[0]
    assert (h.Before, h.After, h.T, h.New_Touch, h.Old_Release) == ('crate', 'cups', 4 * CONST_DT, 2 * CONST_DT, 5 * CONST_DT)
    assert h.Both == pytest.approx(4 * CONST_DT) and h.Free == 0.0 and h.Position_Error == 0.0 and h.Angle_Error == 0.0


def test_a_handover_with_a_drop_between_the_holders():
    """
    Description:
        Jaws to the MG06 holder: the jaws release at sample 3, the holder first touches at sample 6: none holds it for samples
        4-5, and Custody_Gaps reports that span with the PCB's fall.
    """

    record = _Record([0] * 10, [[1, 0, 0]] * 4 + [[0, 0, 0]] * 2 + [[0, 1, 0]] * 4, z=[1.0] * 4 + [0.999, 0.998] + [0.998] * 4)
    h = Handovers(_Locations([2] * 5 + [3] * 5), record, 0)[0]
    assert (h.Before, h.After, h.Old_Release, h.New_Touch) == ('jaws', 'MG06 holder', 3 * CONST_DT, 6 * CONST_DT)
    assert h.Both == 0.0 and h.Free == pytest.approx(2 * CONST_DT)
    (a, b, drop), = Custody_Gaps(record, 0)
    assert (a, b) == (3 * CONST_DT, 6 * CONST_DT) and drop == pytest.approx(0.002)


def test_tilt_from_the_vertical():
    """
    Description:
        A rotation of 8.94 deg about x tilts the z axis 8.94 deg; the identity 0.
    """

    a = math.radians(8.94)
    q = np.array([[1.0, 0.0, 0.0, 0.0], [math.cos(a / 2), math.sin(a / 2), 0.0, 0.0]])
    assert np.degrees(Tilt(q)) == pytest.approx([0.0, 8.94])


def test_the_pcb_passes_seven_stages_in_both_programs(optimized_built):
    """
    Description:
        Input crate, cups, jaws, MG06, jaws, cups, output crate: the same stages in the same order in both programs.
    """

    stages = ['input_crate', 'cups', 'jaws', 'mg06', 'jaws', 'cups', 'output_crate']
    for trajectory in (_Taught(optimized_built), _Optimized(optimized_built)):
        assert [name for name, _, _ in Location_Spans(trajectory)] == stages


def test_geom_points_are_a_mesh_s_vertices_and_a_box_s_corners(optimized_built):
    """
    Description:
        A mesh geom gives its compiled vertices, a primitive the 8 corners of its own box; grouped by geom, ascending.
    """

    m = optimized_built['model']
    mesh = next(g for g in range(m.ngeom) if m.geom_type[g] == 7 and m.geom_contype[g])
    box = next(g for g in range(m.ngeom) if m.geom_type[g] == 6 and m.geom_contype[g])
    owners, points = Geom_Points(m, np.array([mesh, box]))
    assert np.sum(owners == mesh) == m.mesh_vertnum[m.geom_dataid[mesh]] and np.sum(owners == box) == 8
    assert np.all(np.diff(owners) >= 0)
    corners = points[owners == box]
    assert np.allclose(corners.min(axis=0), m.geom_aabb[box][:3] - m.geom_aabb[box][3:])
    assert np.allclose(corners.max(axis=0), m.geom_aabb[box][:3] + m.geom_aabb[box][3:])


def test_the_cell_geometry_at_the_start(optimized_built, program, cell_configuration):
    """
    Description:
        At the taught program's first sample (the PCB in the input crate, the loader home): every pair measured (the robot and
        the PCB separate), the jaw zone holds the loader and not the PCB, the robot's own links are apart; the PCB resting in
        the crate touches by design; the fork on the handle is by design only in the closing and opening; the PCB and the
        jaws are by design while the loader is down in MG06.
    """

    tr = _Taught(optimized_built)
    geometry = Cell_Geometry_Cls(optimized_built['kinematics'], optimized_built['collision'], cell_configuration.Loader_Free)
    geometry.Pose_Sample(tr, 0)
    geometry.Grow_Zone('jaw zone')
    distances, boxes, occupied = geometry.Measure('input_crate')
    names = [name for name, _, _ in CONST_PAIRS]
    assert not np.isnan(distances).any() and distances[names.index('robot - own links')] > 0.01
    assert occupied['loader in jaw zone'] and not occupied['PCB in jaw zone']
    assert math.isnan(geometry.Pair_Distance('robot - PCB', 'cups')[0]) and math.isnan(geometry.Pair_Distance('PCB - loader', 'jaws')[0])
    design = By_Design(tr, program)
    close = int(np.flatnonzero(tr.Step_Index == tr.Step_Names.index('S9_Close_MG06'))[0])
    down = int(np.flatnonzero((tr.Loader_Y == program.Constants.Loader['L_PLACE'].Y) & (tr.Loader_Z < program.Constants.Loader['L_TRAVEL'].Z))[0])
    assert design('PCB - environment', 0) and not design('robot - MG06 lid/handle/press', 0)
    assert design('robot - MG06 lid/handle/press', close) and design('PCB - loader', down) and not design('robot - loader', down)


def test_the_pcb_overhangs_the_cups_on_every_side(optimized_built, cell_configuration):
    """
    Description:
        Carried on the cups (mid S3), the board reaches 10-20 mm beyond the cups on each side (measured 2026-10-25: +x 19.4,
        -x 16.9, +y 10.4, -y 19.6 mm).
    """

    tr = _Taught(optimized_built)
    name, a, b = next(s for s in Location_Spans(tr) if s[0] == 'cups')
    Cell_Geometry_Cls(optimized_built['kinematics'], optimized_built['collision'], cell_configuration.Loader_Free).Pose_Sample(tr, (a + b) // 2)
    found = Overhang(optimized_built['model'], optimized_built['kinematics'].Data, set(CONST_ROBOT_LINKS[-1][1:]))
    assert {k: round(1000 * v, 1) for k, v in found.items()} == {'+x': 19.4, '-x': 16.9, '+y': 10.4, '-y': 19.6}


@pytest.mark.slow
def test_the_recording_does_not_change_the_run(optimized_built, cell_model, cell_configuration):
    """
    Description:
        Record_Physics only reads the run's data: on the optimized trajectory its report is the one Run_Physics gives, line for
        line, and it holds one record per trajectory sample, at the sample's time.
    """

    from Configuration.Program_Parser import Free_Steps
    from Simulation.Core import Trajectory_Drive_Cls, Run_Physics
    from Simulation.Report import Format_Program_Report
    from Report.Core import Record_Physics
    tr, program, meta = _Optimized(optimized_built), optimized_built['optimized'], optimized_built['meta']
    free = Free_Steps(program)
    plain = Run_Physics(cell_model, Trajectory_Drive_Cls(tr, program, free), meta, cell_configuration)
    recorded, record = Record_Physics(cell_model, Trajectory_Drive_Cls(tr, program, free), meta, cell_configuration)
    assert Format_Program_Report(recorded, cell_configuration, program.Name) == Format_Program_Report(plain, cell_configuration, program.Name)
    assert len(record.T) == len(tr.T) and np.allclose(record.T, tr.T, atol=1e-9)
