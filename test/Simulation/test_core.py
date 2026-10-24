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
from Simulation.Core import (Legacy_Plan_Str, Cycle_Result_Str, Vacuum_Event_Str, Read_Legacy_Plan, Run_Cycle, Format_Report,
                             Check_Acceptance, Min_Jerk, Loader_Target, Robot_Target, Phase_At, Simulation_Error_Cls)
#   ../Configuration/Core
from Configuration.Core import Read_PCB_Meta, Load_Cell_Configuration, Load_Keypoints
#   ../Scene/Core
from Scene.Core import Build_Cell_Model

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
        instead of running on from the reset state (SPEC §4.7). It runs in a fresh interpreter with a
        timeout: without the guard the time restarts at each reset and the run never ends.
    """

    code = """import sys
sys.path.insert(0, %r)
from Configuration.Core import Configure_MuJoCo_Log, Read_PCB_Meta, Load_Cell_Configuration, Load_Keypoints
Configure_MuJoCo_Log()
from Scene.Core import Build_Cell_Model
from Simulation.Core import Legacy_Plan_Str, Read_Legacy_Plan, Run_Cycle, Simulation_Error_Cls
cell = Load_Cell_Configuration()
model = Build_Cell_Model(cell)
model.opt.timestep = 0.05
real = Read_Legacy_Plan()
plan = Legacy_Plan_Str(Dt=real.Dt, Q=real.Q[:500], T_End=2.0, Loader={'y': [], 'z': [], 'jaw': []}, Events=[],
                       Phases=[[0.0, 'loader: up, to park']], Jaw_Open=real.Jaw_Open)
try:
    Run_Cycle(model, plan, Read_PCB_Meta(), cell, Load_Keypoints().Loader['home'])
    print('NO ERROR')
except Simulation_Error_Cls as error:
    print('ERROR', error)
""" % os.path.join(root, 'src')
    try:
        result = subprocess.run([sys.executable, '-c', code], capture_output=True, text=True, timeout=120)
    except subprocess.TimeoutExpired:
        pytest.fail('The unstable run did not end within 120 s: the instability guard is missing.')
    assert re.search(r'^ERROR The simulation became unstable at t = [0-9.]+ s .*MuJoCo reset the state', result.stdout, re.M), \
        result.stdout + result.stderr


@pytest.fixture(scope='module')
def reference_run(cell_configuration, keypoints):
    return Run_Cycle(Build_Cell_Model(cell_configuration), Read_Legacy_Plan(), Read_PCB_Meta(), cell_configuration, keypoints.Loader['home'])


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
