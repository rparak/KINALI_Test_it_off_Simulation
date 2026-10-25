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
# File Name: ../test/Simulation/test_core.py
## =========================================================================== ##

# System (Default)
import os
import sys
import copy
# Types (Dynamic type creation)
from types import SimpleNamespace
# Subprocess (Subprocess management)
import subprocess
# Regular expressions (Default)
import re
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test
#   ../Simulation/Core
from Simulation.Core import (Legacy_Plan_Str, Cycle_Result_Str, Vacuum_Event_Str, Read_Legacy_Plan, Run_Cycle, Format_Report, Trajectory_Drive_Cls,
                             Legacy_Drive_Cls, Physics_Run_Cls, Hold_Str, Check_Program_Acceptance,
                             Step_Physics_Str, Program_Warnings, Format_Program_Report,
                             Check_Acceptance, Min_Jerk, Loader_Target, Robot_Target, Phase_At, Simulation_Error_Cls)
#   ../Configuration/Core
from Configuration.Core import Read_PCB_Meta, Load_Cell_Configuration
#   ../Scene/Core
from Scene.Core import Build_Cell_Model
#   ../Trajectory/Core
from Trajectory.Core import Trajectory_Str

"""
Description:
    Initialization of constants.

    The acceptance limits of SPEC §10.1 and the reference values of SPEC §4.6 are written out here, not
    imported from the module under test.
"""
# The five contact categories of the report (SPEC §4.6).
#   Unit [List<string>]
CONST_CHECKS = ['robot <-> MG06 / workstation / crates', 'robot tool <-> handle (outside closing / opening)', 'robot <-> loader',
                'loader <-> MG06 / workstation', 'PCB <-> lid / press plate']


def _Good_Result():
    """
    Description:
        A result with the values of the reference run (SPEC §4.6), in SI units.
    """

    return Cycle_Result_Str(Cycle_Time=65.8, Vacuum=[Vacuum_Event_Str(3.28, True, 4), Vacuum_Event_Str(9.52, False, None),
                                                    Vacuum_Event_Str(54.76, True, 4), Vacuum_Event_Str(62.44, False, None)],
                            Placed_T=16.4, Placed_Error=np.array([0.00004, -0.00029, 0.00006]), Placed_Tilt=np.radians(0.02),
                            Closed=(np.radians(-0.5), np.radians(28.7), 31.6),
                            Final_PCB=np.array([0.4063, -0.7524, 0.0049, 1.0, 0.0, 0.0, 0.0]), Crate_Centre=np.array([0.4062, -0.7516]),
                            Lid_Open=-0.6974,
                            Final_Tilt=0.0, Track_Error=0.0126, Slip=0.00132, Loader_Lag=0.0226, Fork_Force=294.0,
                            Penetration={'MG06 holder': (-0.0002, 16.14), 'crate': (-0.00014, 62.46), 'loader jaws': (-0.00011, 9.13)},
                            Contacts={}, Lid_End=np.radians(-40.0), Handle_End=0.0)


def _Failed(result):
    return [c.Name for c in Check_Acceptance(result, Load_Cell_Configuration()) if not c.Passed]


def test_the_reference_values_pass_every_criterion():
    """
    Description:
        The reference run's values meet all criteria of SPEC §10.1 (10 checks: 5 contacts, picks, MG06,
        crate, MG06 open, penetration).
    """

    checks = Check_Acceptance(_Good_Result(), Load_Cell_Configuration())
    assert len(checks) == 10 and all(c.Passed for c in checks), [(c.Name, c.Detail) for c in checks if not c.Passed]


@pytest.mark.parametrize('change, criterion', [
    (lambda r: r.Contacts.update({'robot <-> loader': (-0.0005, 'a', 'b', 1.0)}), 'robot <-> loader'),
    (lambda r: setattr(r.Vacuum[2], 'Cups', 3), 'vacuum ON 2x with 4/4 cups'),
    (lambda r: r.Vacuum.pop(2), 'vacuum ON 2x with 4/4 cups'),
    (lambda r: setattr(r, 'Placed_Error', np.array([0.0005, 0.0, 0.0])), 'PCB in MG06 error < 0.5 mm, tilt < 0.2 deg'),
    (lambda r: setattr(r, 'Placed_Tilt', np.radians(0.2)), 'PCB in MG06 error < 0.5 mm, tilt < 0.2 deg'),
    (lambda r: setattr(r, 'Final_PCB', np.array([0.4082, -0.7516, 0.0049, 1, 0, 0, 0])), 'PCB in the output crate < 2.0 mm from the centre'),
    (lambda r: setattr(r, 'Lid_End', np.radians(-0.5)), 'MG06 ends open'),
    (lambda r: setattr(r, 'Handle_End', np.radians(28.7)), 'MG06 ends open'),
    (lambda r: r.Penetration.update({'crate': (-0.00051, 62.46)}), 'PCB penetration <= 0.5 mm'),
])
def test_each_violation_fails_exactly_its_criterion(change, criterion):
    """
    Description:
        A value at or beyond one limit (the limits are strict, 0.5 mm penetration is allowed) fails that
        criterion and no other.
    """

    result = copy.deepcopy(_Good_Result())
    change(result)
    assert _Failed(result) == [criterion]


def test_half_a_millimetre_of_penetration_is_allowed():
    """
    Description:
        The penetration bound is inclusive (owner: PCB penetration <= 0.5 mm).
    """

    result = _Good_Result()
    result.Penetration['crate'] = (-0.0005, 62.46)
    assert _Failed(result) == []


def test_the_report_lines_have_the_reference_format():
    """
    Description:
        Format_Report prints the lines of SPEC §4.6 for given values (mm and deg, the same rounding).
    """

    result = _Good_Result()
    result.Contacts['robot <-> loader'] = (-0.00123, 'jaw_pad', 'cup0', 9.13)
    lines = Format_Report(result)
    assert lines[:5] == ['cycle time 65.8 s', '  3.28s vacuum ON  (4 of 4 cups touch the PCB)', '  9.52s vacuum OFF',
                         ' 54.76s vacuum ON  (4 of 4 cups touch the PCB)', ' 62.44s vacuum OFF']
    assert lines[5] == 'PCB in MG06 (t=16.4s): error 0.30 mm (dx 0.04 dy -0.29 dz 0.06), tilt 0.02 deg'
    assert lines[6] == 'MG06 closed with the robot away (t=31.6s): lid -0.5 deg, handle 28.7 deg'
    assert lines[10] == 'deepest PCB penetration: MG06 holder -0.20 mm (t=16.14), crate -0.14 mm (t=62.46), loader jaws -0.11 mm (t=9.13)'
    assert lines[13] == 'robot <-> loader                                  : deepest -1.23 mm  jaw_pad / cup0  t=9.13'
    assert lines[-1] == 'MG06 at the end: lid -40.0 deg, handle 0.0 deg'


def test_the_profiles_and_the_plan_lookups():
    """
    Description:
        Min-jerk is 0, 0.5, 1 at the start, middle and end and clipped outside; a loader move reaches its
        target; the servo command adds kv/kp times the planned velocity; the phase is the last started.
    """

    assert [Min_Jerk(s) for s in (-1, 0, 0.5, 1, 2)] == [0, 0, 0.5, 1, 1]
    plan = Legacy_Plan_Str(Dt=0.5, Q=np.array([[0.0] * 6, [1.0] * 6, [2.0] * 6]), T_End=1.0, Loader={'y': [[1.0, 2.0, 0.3]]},
                           Phases=[[0.0, 'a'], [2.0, 'b']])
    assert Loader_Target(plan, {'y': -0.1}, 'y', 0.5) == -0.1
    assert Loader_Target(plan, {'y': -0.1}, 'y', 2.0) == pytest.approx(0.1)
    assert Loader_Target(plan, {'y': -0.1}, 'y', 5.0) == pytest.approx(0.3)
    command, q = Robot_Target(plan, np.full((3, 6), 2.0), np.full(6, 0.03), 0.25)
    assert q == pytest.approx([0.5] * 6) and command == pytest.approx([0.56] * 6)
    assert [Phase_At(plan, t) for t in (-1.0, 0.0, 1.9, 2.0)] == ['', 'a', 'a', 'b']


def test_a_plan_that_cannot_be_read_is_named(tmp_path):
    """
    Description:
        A missing plan file is a Simulation_Error_Cls naming it.
    """

    with pytest.raises(Simulation_Error_Cls, match='absent.npz cannot be read'):
        Read_Legacy_Plan(str(tmp_path / 'absent.npz'))


def test_an_unstable_simulation_fails_instead_of_continuing(root):
    """
    Description:
        With a 0.05 s timestep MuJoCo resets the state; the run raises Simulation_Error_Cls naming the time
        instead of running on from the reset state (SPEC §4.7), with the line and the PCB's last position. It runs in a fresh
        interpreter with a timeout: without the guard the time restarts at each reset and the run never ends. The PCB floats
        (gravity compensated), so it does not fall through the crate floor at that step and stop the run as lost first.
    """

    code = """import sys
sys.path.insert(0, %r)
from Configuration.Core import Configure_MuJoCo_Log, Read_PCB_Meta, Load_Cell_Configuration
from Configuration.Program_Parser import Load_Constants
Configure_MuJoCo_Log()
from Scene.Core import Build_Cell_Model
#   ../Trajectory/Core
from Trajectory.Core import Trajectory_Str
from Simulation.Core import Legacy_Plan_Str, Read_Legacy_Plan, Run_Cycle, Simulation_Error_Cls
cell = Load_Cell_Configuration()
model = Build_Cell_Model(cell)
model.opt.timestep = 0.05
model.body_gravcomp[model.body('pcb').id] = 1.0
real = Read_Legacy_Plan()
plan = Legacy_Plan_Str(Dt=real.Dt, Q=real.Q[:500], T_End=2.0, Loader={'y': [], 'z': [], 'jaw': []}, Events=[],
                       Phases=[[0.0, 'loader: up, to park']], Jaw_Open=real.Jaw_Open)
try:
    Run_Cycle(model, plan, Read_PCB_Meta(), cell, Load_Constants().Loader['L_HOME'])
    print('NO ERROR')
except Simulation_Error_Cls as error:
    print('ERROR', error)
""" % os.path.join(root, 'src')
    try:
        result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired:
        pytest.fail('The unstable run did not end within 120 s: the instability guard is missing.')
    assert re.search(r'^ERROR The simulation became unstable at t = [0-9.]+ s .*MuJoCo reset the state \(line: .*; the PCB was at x '
                     r'-?[0-9.]+ y -?[0-9.]+ z -?[0-9.]+ mm\)', result.stdout, re.M), \
        result.stdout + result.stderr


@pytest.fixture(scope='module')
def reference_run(cell_configuration, constants):
    return Run_Cycle(Build_Cell_Model(cell_configuration), Read_Legacy_Plan(), Read_PCB_Meta(), cell_configuration, constants.Loader['L_HOME'])


@pytest.mark.slow
def test_the_reference_plan_reproduces_the_baseline_report(root, reference_run):
    """
    Description:
        The full run of Data/Reference/cell_plan.npz prints test/Simulation/data/baseline_report.txt line for line
        (SPEC §10.2).
    """

    with open(os.path.join(root, 'test', 'Simulation', 'data', 'baseline_report.txt'), encoding='utf-8') as handle:
        assert Format_Report(reference_run) == handle.read().splitlines()


@pytest.mark.slow
def test_the_reference_plan_meets_the_acceptance_criteria(reference_run):
    """
    Description:
        SPEC §10.1 on the full run, with the limits written here: no contact in any category, 4 of 4 cups at
        both picks, PCB in MG06 < 0.5 mm and < 0.2 deg, < 2 mm from the output crate centre, MG06 ends open
        (lid within 2 deg of -0.6974 rad, handle within 2 deg of 0), penetration <= 0.5 mm. No instability:
        the run returned.
    """

    r = reference_run
    for check in CONST_CHECKS:
        assert check not in r.Contacts, (check, r.Contacts.get(check))
    assert [v.Cups for v in r.Vacuum if v.On] == [4, 4]
    assert np.linalg.norm(r.Placed_Error) < 0.0005 and np.degrees(r.Placed_Tilt) < 0.2
    assert np.linalg.norm(r.Final_PCB[:2] - r.Crate_Centre) < 0.002
    assert abs(np.degrees(r.Lid_End - (-0.6974))) < 2.0 and abs(np.degrees(r.Handle_End)) < 2.0
    assert all(-v[0] <= 0.0005 for v in r.Penetration.values()), r.Penetration


@pytest.mark.slow
def test_the_raw_values_of_the_reference_run_are_pinned(reference_run):
    """
    Description:
        Regression pin of the unrounded values (measured 2026-10-24 on the code proven bit-identical to the
        legacy run, tag legacy-planner-reference, FEATURES.md F3.5): the report rounds them, so a change below its precision (the
        phase sampling, the cup-touch threshold) would pass the report test. A pin, not a bound.
    """

    r = reference_run
    assert r.Placed_T == pytest.approx(16.42, abs=1e-6) and r.Closed[2] == pytest.approx(31.65, abs=1e-6)
    assert [v.T for v in r.Vacuum] == pytest.approx([3.276, 9.524, 54.764, 62.444], abs=1e-6)
    assert r.Track_Error == pytest.approx(0.012575964178209498, rel=1e-9)
    assert r.Slip == pytest.approx(0.0013238278119356037, rel=1e-9)
    assert r.Loader_Lag == pytest.approx(0.022590242934832326, rel=1e-9)
    assert r.Fork_Force == pytest.approx(293.694615772405, rel=1e-9)
    assert np.linalg.norm(r.Placed_Error) == pytest.approx(0.0003017790990516208, rel=1e-9)
    assert r.Placed_Tilt == pytest.approx(0.00040800224934629315, rel=1e-9)


def _Small_Trajectory():
    """
    Description:
        A four-sample trajectory of three steps: a MoveSpline step that starts a loader job, the handle step with J_PARK, the
        Test step; and the program's segments (only Key and Allow_Contact are read) and its vacuum switch's data.
    """

    tr = Trajectory_Str(T=np.arange(4) * 0.004, Robot_Q=np.array([[0.0] * 6, [1.0] * 6, [2.0] * 6, [3.0] * 6]), Robot_QD=np.full((4, 6), 2.0),
                        Loader_Y=np.array([0.0, 0.1, 0.2, 0.3]), Loader_Z=np.zeros(4), Gripper=np.full(4, 0.01),
                        Step_Index=np.array([0, 0, 1, 2], np.int32), Segment_Id=np.array([0, 1, 2, 3], np.int32),
                        Step_Names=['S1_Move', 'S2_Close', 'S3_Test'], Segment_Step=np.array([0, 0, 1, 2], np.int32),
                        Segment_Command=['Robot.MoveSpline(W_A, v=0.6)', 'Loader.Start(J_FETCH)', 'Loader.Run(J_PARK) > Path(F)', 'Wait(2.0)'],
                        Segment_Start=np.array([0, 1, 2, 3], np.int32), Segment_End=np.array([1, 2, 3, 3], np.int32),
                        Events=[(0.004, 'vacuum_on'), (0.008, 'gripper_open'), (0.012, 'vacuum_off')])
    program = SimpleNamespace(Segments=[SimpleNamespace(Key='S1_Move', Allow_Contact=[]),
                                        SimpleNamespace(Key='S2_Close', Allow_Contact=['handle_bar'])],
                              Simulation=SimpleNamespace(Signal_Response={'DI_VACUUM_OK': 0.3}, Vacuum_OK_Min_Cups=4))
    return tr, program


def test_the_trajectory_drive_interpolates_its_samples():
    """
    Description:
        Trajectory_Drive_Cls (F11): the servo command is q + kv/kp x qd, linear between the 4 ms samples like Robot_Target; the
        loader channels likewise; the start is sample 0; the label and the line are those of the sample at or before t; past the
        end it holds the last sample (the last interval's label); only the vacuum events drive the cups.
    """

    tr, program = _Small_Trajectory()
    drive = Trajectory_Drive_Cls(tr, program)
    drive.Bind(np.full(6, 0.03))
    command, q = drive.Robot(0.006)
    assert q == pytest.approx([1.5] * 6) and command == pytest.approx([1.56] * 6)
    assert drive.Loader(0.002) == pytest.approx([0.05, 0.0, 0.01])
    assert drive.Robot(1.0)[1] == pytest.approx([3.0] * 6, abs=0.01)
    assert [s.tolist() if isinstance(s, np.ndarray) else s for s in drive.Start()] == [[0.0] * 6, 0.0, 0.0, 0.01]
    assert [drive.Label(t) for t in (0.0, 0.0079, 0.008, 0.0119)] == ['S1_Move', 'S1_Move', 'S2_Close', 'S2_Close']
    assert drive.Command(0.005) == 'Loader.Start(J_FETCH)' and drive.T_End == pytest.approx(0.012)
    assert drive.Events == [(0.004, True), (0.012, False)]


def test_the_trajectory_drive_classifies_its_steps():
    """
    Description:
        The fork may touch the handle only in a step whose segment allows handle_bar; the PCB counts as placed from J_PARK's
        first sample and MG06 as closed from the Test step's first sample; a MoveSpline step is FREE, a Loader.Start step
        parallel.
    """

    drive = Trajectory_Drive_Cls(*_Small_Trajectory())
    assert [drive.Fork_Allowed(s) for s in ('S1_Move', 'S2_Close', 'S3_Test')] == [False, True, False]
    assert drive.Placed_Time == pytest.approx(0.008) and drive.Closed_Time == pytest.approx(0.012)
    assert [drive.Placed('', t) for t in (0.0079, 0.008)] == [False, True]
    assert [drive.Free(s) for s in ('S1_Move', 'S2_Close')] == [True, False]
    assert [drive.Parallel(s) for s in ('S1_Move', 'S3_Test')] == [True, False]


def _Short_Run(cell_configuration, constants, events):
    """
    Description:
        A physics run of the reference plan's first 0.3 s (the robot at home, the PCB in the input crate) with the given vacuum
        events.
    """

    real = Read_Legacy_Plan()
    plan = Legacy_Plan_Str(Dt=real.Dt, Q=real.Q[:100], T_End=0.3, Loader={'y': [], 'z': [], 'jaw': []}, Events=events,
                           Phases=[[0.0, 'robot: to the input crate']], Jaw_Open=real.Jaw_Open)
    drive = Legacy_Drive_Cls(plan, constants.Loader['L_HOME'])
    return Physics_Run_Cls(Build_Cell_Model(cell_configuration), drive, Read_PCB_Meta(), cell_configuration)


def test_the_vacuum_on_with_no_cup_on_the_pcb_loses_it(cell_configuration, constants):
    """
    Description:
        F11: with the vacuum on and no cup touching the PCB for 20 ms (the first such step at 0.1005 s) the run stops (Done)
        with a fault naming the time, the step and the line; the result carries it and the placement is unknown (nan), so the MG06 criterion fails.
    """

    run = _Short_Run(cell_configuration, constants, [[0.1, 'suction_on']])
    while not run.Done:
        run.Step()
    result = run.Result()
    assert run.Data.time == pytest.approx(0.1205, abs=1e-6), run.Data.time
    assert re.fullmatch(r'PCB lost at t = 0\.12[01] s, step robot: to the input crate, line "robot: to the input crate": no cup has touched '
                        r'it for 20 ms with the vacuum on\.', result.Fault), result.Fault
    assert 'PCB in MG06 error < 0.5 mm, tilt < 0.2 deg' in _Failed(result)


def test_a_pcb_below_the_table_top_is_lost(cell_configuration, constants):
    """
    Description:
        F11: a PCB 20 mm or more below the table top (z = 0) is lost even with the vacuum off (it fell through the cell).
    """

    run = _Short_Run(cell_configuration, constants, [])
    run.Data.qpos[run.qpcb + 2] = -0.021
    run.Step()
    assert run.Done and run.Fault.endswith(': it is 21 mm below the table top.'), run.Fault
    run = _Short_Run(cell_configuration, constants, [])
    run.Data.qpos[run.qpcb + 2] = -0.019
    run.Step()
    assert not run.Done and run.Fault is None


def _Program_Result():
    """
    Description:
        The reference values with a program run's two vacuum holds of 4 of 4 cups (F11).
    """

    result = _Good_Result()
    result.Holds = [Hold_Str(T_On=3.28, T_Off=9.91), Hold_Str(T_On=57.85, T_Off=66.07)]
    result.Steps = [Step_Physics_Str(Name='S3_To_Loader', Start=5.19, End=7.03, Free=True, Track_Error=0.0014, Slip=0.00128, Slip_T=6.864,
                                     Slip_Tilt=np.radians(8.8), Slip_Acceleration=1.75, Tilt_Max=np.radians(8.9), Acceleration_Max=2.23,
                                     Robot_Loader=(0.03514, 5.6, 'aubo/tool/ee_tool / z_carriage'), PCB_Loader=(0.02002, 5.6, 'pcb / z_carriage')),
                    Step_Physics_Str(Name='S4_Handover', Start=7.03, End=11.62, Track_Error=0.0011, Slip=0.0013,
                                     Robot_Loader=(-0.00001, 9.5, 'a / jaw_L'))]
    return result


@pytest.mark.parametrize('change, criterion', [
    (lambda r: setattr(r.Holds[1], 'Min_Cups', 3), 'every vacuum hold keeps 4/4 cups'),
    (lambda r: r.Holds.pop(1), 'every vacuum hold keeps 4/4 cups'),
    (lambda r: (setattr(r, 'Fault', 'PCB lost at ...'), setattr(r, 'Fault_Kind', 'lost')), 'the PCB is never lost'),
    (lambda r: (setattr(r, 'Fault', 'vacuum not OK at ...'), setattr(r, 'Fault_Kind', 'input')), 'every WaitDI gets DI_VACUUM_OK in time'),
    (lambda r: setattr(r.Steps[0], 'PCB_Loader', (0.0009, 5.6, 'pcb / z_carriage')), 'free transfers >= 1.0 mm from the loader'),
])
def test_a_program_run_fails_exactly_its_new_criterion(change, criterion):
    """
    Description:
        F11: a program's run has SPEC §10.1's ten checks and four more - every vacuum hold keeps 4 of 4 cups (both holds), the PCB
        is never lost, every WaitDI gets the vacuum switch's input, the free transfers keep 1 mm from the loader (a LOCKED step
        at the loader may touch it); each violation fails that one.
    """

    result = _Program_Result()
    checks = Check_Program_Acceptance(result, Load_Cell_Configuration())
    assert len(checks) == 14 and all(c.Passed for c in checks), [(c.Name, c.Detail) for c in checks if not c.Passed]
    change(result)
    assert [c.Name for c in Check_Program_Acceptance(result, Load_Cell_Configuration()) if not c.Passed] == [criterion]


def _Build(program, cell_configuration):
    """
    Description:
        The trajectory of a program (on the planning model).
    """

    from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
    from Kinematics.Core import Robot_Kinematics_Cls
    from Program.Core import Build_Program
    model = Build_Planning_Model(cell_configuration)
    trajectory, _ = Build_Program(program, cell_configuration, Read_PCB_Meta(), Robot_Kinematics_Cls(model, Robot_Collision_Cls(model).Data), {})
    return trajectory


def _Run_To_End(program, trajectory, cell_configuration, inputs=None):
    """
    Description:
        A program's physics run to its end or its fault; inputs replaces the drive's vacuum switch checks.
    """

    drive = Trajectory_Drive_Cls(trajectory, program)
    drive.Inputs = drive.Inputs if inputs is None else inputs(drive.Inputs)
    run = Physics_Run_Cls(Build_Cell_Model(cell_configuration), drive, Read_PCB_Meta(), cell_configuration)
    while not run.Done:
        run.Step()
    return run


def test_the_vacuum_switch_fails_a_waitdi_whose_input_has_not_come(program, cell_configuration):
    """
    Description:
        F11: the taught program's first WaitDI ends 0.304 s after the vacuum went on with 4 of 4 cups on the PCB; asked for a 0.5 s
        response, the switch is still false there, so the run stops at that line with the evidence (exit 1 in the scripts).
    """

    trajectory = _Build(program, cell_configuration)
    run = _Run_To_End(program, trajectory, cell_configuration, lambda inputs: [(t, s, x, 0.5, n, o) for t, s, x, _, n, o in inputs])
    assert run.Fault_Kind == 'input' and run.Data.time == pytest.approx(3.580, abs=0.001), (run.Data.time, run.Fault)
    assert re.fullmatch(r'vacuum not OK at t = 3\.580 s, step S2_Pick_Input, line "Robot\.WaitDI\(DI_VACUUM_OK, timeout=1\.0\)": 4 of 4 cups '
                        r'touch the PCB \(DI_VACUUM_OK needs 4 for 0\.5 s: for 0\.30[34] s\); the WaitDI would time out after 1\.0 s\.', run.Fault), \
        run.Fault


@pytest.mark.slow
def test_the_old_take_loses_the_board_at_the_release(program, constants, simulation, cell_configuration, tmp_path):
    """
    Description:
        F11: the take of before the fix - the jaws opened with the bellows pressed (the touch move after the release) - stops
        the run in S18_Take_From_Loader: the PCB lost while the jaws open, or the second WaitDI's input missing after it; the
        taught program as shipped passes every new criterion.
    """

    from Configuration.Program_Parser import Load_Program, CONST_PROGRAM_YAML
    text = open(CONST_PROGRAM_YAML, encoding='utf-8').read()
    touch = '    - Robot.MoveL(P_LOADER_TAKE_TOUCH, v=0.01)            # owner, 2026-10-24 (F11): relieve the bellows before the jaws open\n'
    release = '    - Loader.Run(J_RELEASE)\n'
    assert text.count(touch + release) == 1
    path = tmp_path / 'Program.yaml'
    path.write_text(text.replace(touch + release, release + touch), encoding='utf-8')
    old = Load_Program(constants, simulation, cell_configuration, str(path))
    run = _Run_To_End(old, _Build(old, cell_configuration), cell_configuration)
    assert run.Fault is not None and ', step S18_Take_From_Loader, line ' in run.Fault, run.Fault
    assert (run.Fault_Kind == 'lost' and 'line "Loader.Run(J_RELEASE) > Gripper(G_OPEN, t=0.35)"' in run.Fault) or \
        (run.Fault_Kind == 'input' and 'line "Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)"' in run.Fault), run.Fault
    shipped = _Run_To_End(program, _Build(program, cell_configuration), cell_configuration).Result()
    assert [c.Name for c in Check_Program_Acceptance(shipped, cell_configuration) if not c.Passed] == []


def test_a_program_run_warns_above_the_owners_thresholds():
    """
    Description:
        F11: no warning at the measured values; a slip above 1.98 mm, a tracking error above 0.0189 rad (1.5 x the legacy run's)
        and a free transfer within Loader_Free (10 mm) of the loader each warn once, naming the step; a LOCKED step at the
        loader does not warn; none of them fails the acceptance.
    """

    cell = Load_Cell_Configuration()
    result = _Program_Result()
    assert Program_Warnings(result, cell) == []
    result.Steps[0].Slip, result.Steps[0].Track_Error, result.Steps[0].PCB_Loader = 0.00199, 0.0190, (0.0099, 5.6, 'pcb / z_carriage')
    warnings = Program_Warnings(result, cell)
    assert warnings == ['S3_To_Loader: PCB slip 1.99 mm at 6.864 s above 1.98 mm (1.5 x the legacy run)',
                        'S3_To_Loader: robot tracking error 0.0190 rad above 0.0189 rad (1.5 x the legacy run)',
                        'S3_To_Loader: carried PCB 9.90 mm from the loader at 5.600 s (pcb / z_carriage), within Clearance.Loader_Free 10.0 mm']
    assert all(c.Passed for c in Check_Program_Acceptance(result, cell))


def test_the_program_report_has_its_sections():
    """
    Description:
        F11: Format_Program_Report: the program line, SPEC §4.6's lines, the holds, a row per step (FREE / LOCKED, tracking, slip,
        the loader distances), the carrying free transfers' slip with tilt and acceleration, the warnings, the 14 checks, the
        closing sentence.
    """

    lines = Format_Program_Report(_Program_Result(), Load_Cell_Configuration(), 'Program')
    assert lines[0] == 'program Program: physics run of its trajectory, cycle 65.800 s' and lines[1] == 'cycle time 65.8 s'
    assert 'vacuum holds: 3.280-9.910 s min 4/4 cups; 57.850-66.070 s min 4/4 cups' in lines
    row = next(line for line in lines if line.startswith('S3_To_Loader '))
    assert row.split()[1:7] == ['5.190-', '7.030', 'FREE', '0.0014', '1.28', '35.14'] and row.split()[7] == '20.02'
    assert ('  S3_To_Loader           slip 1.28 mm at 6.864 s: tilt 8.8 deg, acceleration 1.75 m/s^2; max tilt 8.9 deg, max acceleration '
            '2.23 m/s^2') in lines
    assert 'warnings: none' in lines and sum(1 for line in lines if line.endswith(('PASS', 'FAIL')) or '  PASS  ' in line) == 14
    assert lines[-1] == 'Program: the physics run meets SPEC §10.1 (14 of 14). Simulation results only: not validated for the real cell.'


@pytest.mark.slow
def test_the_fork_force_is_reported_as_net_squeeze_and_hinge_load(program, cell_configuration):
    """
    Description:
        F11: through the taught program's closing of MG06 the run measures the fork on the handle bar three ways: the summed
        normal force, the net force on the bar (no larger than the sum; its direction a unit vector in the fork's frame, mostly
        along the slot's pushing face, +x), the squeeze ((sum - |vector sum of the normals|) / 2, never negative) and the handle
        hinge load; measured 2026-10-25 291 N / 229 N / 37 N / 9.4 N m.
    """

    trajectory = _Build(program, cell_configuration)
    drive = Trajectory_Drive_Cls(trajectory, program)
    run = Physics_Run_Cls(Build_Cell_Model(cell_configuration), drive, Read_PCB_Meta(), cell_configuration)
    while run.Data.time < 31.0:
        run.Step()
    r = run.fork_force, run.fork_net, run.fork_squeeze, run.handle_load
    assert 0.0 < run.fork_net <= run.fork_force and 0.0 <= run.fork_squeeze <= 0.5 * run.fork_force and run.handle_load > 0.0, r
    assert np.linalg.norm(run.fork_net_direction) == pytest.approx(1.0) and run.fork_net_direction[0] > 0.5, run.fork_net_direction
    assert [round(v, 1) for v in r] == pytest.approx([291.0, 229.0, 37.0, 9.4], abs=1.0), r
