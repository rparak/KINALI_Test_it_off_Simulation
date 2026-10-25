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
# File Name: ../test/Simulation/test_report.py
## =========================================================================== ##

# System (Default)
import copy
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test and the results it reads
#   ../Simulation/Report, ../Simulation/Core
from Simulation.Core import Cycle_Result_Str, Vacuum_Event_Str, Hold_Str, Place_Physics_Str, Step_Physics_Str
from Simulation.Report import Format_Report, Check_Program_Acceptance, Program_Warnings, Format_Program_Report, Check_Acceptance
#   ../Configuration/Core
from Configuration.Core import Load_Cell_Configuration

"""
Description:
    A run's acceptance and report (SPEC §10.1, §4.6) on given results: the acceptance limits of SPEC §10.1 and the reference
    values of SPEC §4.6 are written out here, not imported from the module under test.
"""


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
def test_each_violation_fails_exactly_its_criterion(change, criterion, failed_criteria):
    """
    Description:
        A value at or beyond one limit (the limits are strict, 0.5 mm penetration is allowed) fails that
        criterion and no other.
    """

    result = copy.deepcopy(_Good_Result())
    change(result)
    assert failed_criteria(result) == [criterion]


def test_half_a_millimetre_of_penetration_is_allowed(failed_criteria):
    """
    Description:
        The penetration bound is inclusive (owner: PCB penetration <= 0.5 mm).
    """

    result = _Good_Result()
    result.Penetration['crate'] = (-0.0005, 62.46)
    assert failed_criteria(result) == []


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
    result.Fork_Force, result.Fork_Net_Force, result.Handle_Load = 291.0, 229.0, 9.38
    result.Joint_Torque, result.Torque_Limit = np.array([25.1, 52.3, 40.9, 18.2, 7.0, 20.5]), np.array([180.0, 180.0, 150.0, 60.0, 60.0, 60.0])
    # The place into MG06 as measured 2026-10-25 (the taught program, B work).
    result.Place = Place_Physics_Str(Release=17.130, Release_Z=0.217041, Seated_Z=0.216889, Drop=0.000152, Guide_Min=0.000177,
                                     Guide_Pair='holder_14 at 17.128 s', Jaw_Closing=0.0, Holder_Force=0.0, Transient=-0.0000400, Settled=-0.0000002,
                                     Supports=['holder_03', 'holder_04', 'holder_10'],
                                     Support_Gaps={'holder_03': 0.0, 'holder_04': 0.0, 'holder_06': 0.000013, 'holder_10': 0.0}, Speed=0.0,
                                     Tilt=np.radians(0.0117), Other_Contacts=[], Settle_Time=0.014, Centring=-0.000026, Regrip_Gap=0.000359)
    return result


@pytest.mark.parametrize('change, criterion', [
    (lambda r: setattr(r.Holds[1], 'Min_Cups', 3), 'every vacuum hold keeps 4/4 cups'),
    (lambda r: r.Holds.pop(1), 'every vacuum hold keeps 4/4 cups'),
    (lambda r: (setattr(r, 'Fault', 'PCB lost at ...'), setattr(r, 'Fault_Kind', 'lost')), 'the PCB is never lost'),
    (lambda r: (setattr(r, 'Fault', 'vacuum not OK at ...'), setattr(r, 'Fault_Kind', 'input')), 'every WaitDI gets DI_VACUUM_OK in time'),
    (lambda r: setattr(r.Steps[0], 'PCB_Loader', (0.0009, 5.6, 'pcb / z_carriage')), ['free transfers >= 1.0 mm from the loader',
                                                                                       'free transfers >= Loader_Free 10.0 mm']),
    (lambda r: setattr(r.Steps[0], 'PCB_Loader', (0.0099, 5.6, 'pcb / z_carriage')), 'free transfers >= Loader_Free 10.0 mm'),
    (lambda r: setattr(r, 'Slip', 0.00199), 'PCB slip <= 1.98 mm'),
    (lambda r: setattr(r, 'Track_Error', 0.0190), 'tracking error <= 0.0189 rad'),
    (lambda r: setattr(r, 'Fork_Net_Force', 290.1), 'fork net force <= 290 N (sim. regression bound)'),
    (lambda r: setattr(r, 'Handle_Load', 13.51), 'hinge load <= 13.5 N m (sim. regression bound)'),
    (lambda r: r.Joint_Torque.__setitem__(5, 60.0), 'no robot actuator saturated (model limits)'),
    (lambda r: setattr(r.Place, 'Jaw_Closing', 0.000011), 'place: no jaw closing at L_PLACE'),
    (lambda r: setattr(r.Place, 'Holder_Force', 1.01), 'place: holder force <= 1.0 N (jaws closed)'),
    (lambda r: setattr(r.Place, 'Transient', -0.00011), 'release: penetration <= 0.10 mm while dropping'),
    (lambda r: setattr(r.Place, 'Settled', -0.0000103), 'release: settled penetration <= 0.0102 mm'),
    (lambda r: setattr(r.Place, 'Guide_Min', -0.000001), 'place: no side guide touched on the way down'),
    (lambda r: setattr(r.Place, 'Supports', ['holder_03', 'holder_04']), 'PCB seated in MG06 within 0.2 s'),
    (lambda r: r.Place.Support_Gaps.update(holder_06=0.0000501, holder_10=0.0000501), 'PCB seated in MG06 within 0.2 s'),
    (lambda r: setattr(r.Place, 'Speed', 0.001), 'PCB seated in MG06 within 0.2 s'),
    (lambda r: setattr(r.Place, 'Tilt', np.radians(0.1)), 'PCB seated in MG06 within 0.2 s'),
    (lambda r: setattr(r.Place, 'Other_Contacts', ['holder_17']), 'PCB seated in MG06 within 0.2 s'),
    (lambda r: setattr(r.Place, 'Other_Contacts', ['jaw_R']), 'PCB seated in MG06 within 0.2 s'),
    (lambda r: setattr(r.Place, 'Centring', -0.000501), 'handover: centring <= 0.5 mm'),
    (lambda r: setattr(r.Place, 'Regrip_Gap', 0.000199), 're-grip: ledge gap 0.3 +- 0.1 mm'),
    (lambda r: setattr(r.Place, 'Regrip_Gap', 0.000401), 're-grip: ledge gap 0.3 +- 0.1 mm'),
    (lambda r: setattr(r, 'Place', None), ['place: no jaw closing at L_PLACE', 'place: holder force <= 1.0 N (jaws closed)',
                                           'release: penetration <= 0.10 mm while dropping', 'release: settled penetration <= 0.0102 mm',
                                           'place: no side guide touched on the way down', 'PCB seated in MG06 within 0.2 s',
                                           'handover: centring <= 0.5 mm', 're-grip: ledge gap 0.3 +- 0.1 mm']),
])
def test_a_program_run_fails_exactly_its_new_criterion(change, criterion):
    """
    Description:
        F11 and step 6: a program's run has SPEC §10.1's ten checks and ten more - every vacuum hold keeps 4 of 4 cups (both
        holds), the PCB is never lost, every WaitDI gets the vacuum switch's input, the free transfers keep 1 mm from the loader (a
        LOCKED step at the loader may touch it), the slip <= 1.98 mm, the tracking error <= 0.0189 rad, the fork's net force <=
        290 N, the handle hinge load <= 13.5 N m, the free transfers >= Loader_Free (10 mm), no robot actuator saturated (a wrist
        3 torque at its 60 N m limit); B work (owner, 2026-10-25), the place into MG06: no jaw closing at L_PLACE, the holder
        force <= 1.0 N, the release's penetration <= 0.10 mm (0.09 mm passes) and settled <= 0.0102 mm, no side guide touched, the PCB seated (too
        few supports, a support too far, too fast, too tilted, touching a guide or a jaw), the handover's centring <= 0.5 mm,
        the re-grip's ledge gap 0.3 +- 0.1 mm (both sides), and no place measured fails all eight; each violation (just beyond
        its bound) fails that one (the 1 mm floor also fails Loader_Free).
    """

    result = _Program_Result()
    checks = Check_Program_Acceptance(result, Load_Cell_Configuration())
    assert len(checks) == 28 and all(c.Passed for c in checks), [(c.Name, c.Detail) for c in checks if not c.Passed]
    change(result)
    expected = criterion if isinstance(criterion, list) else [criterion]
    assert [c.Name for c in Check_Program_Acceptance(result, Load_Cell_Configuration()) if not c.Passed] == expected


def test_the_release_bound_passes_0_09_mm():
    """
    Description:
        Item 2 (owner, 2026-10-25): the release's transient penetration bound is 0.10 mm - 0.09 mm passes (the margin runs' worst
        was 0.068 mm), 0.11 mm fails (the old pressed place was 0.157 / 0.181 mm).
    """

    result = _Program_Result()
    result.Place.Transient = -0.00009
    assert all(c.Passed for c in Check_Program_Acceptance(result, Load_Cell_Configuration()))


def test_a_program_run_warns_above_the_owners_thresholds():
    """
    Description:
        F11: no warning at the measured values; a slip above 1.98 mm, a tracking error above 0.0189 rad (1.5 x the legacy run's)
        and a free transfer within Loader_Free (10 mm) of the loader each warn once, naming the step; a LOCKED step at the
        loader does not warn; the free transfer within Loader_Free also fails the acceptance (step 6; the slip and the tracking
        bounds are on the run's maxima, which this step's values do not change) and names the step, the time and the pair (A5).
    """

    cell = Load_Cell_Configuration()
    result = _Program_Result()
    assert Program_Warnings(result, cell) == []
    result.Steps[0].Slip, result.Steps[0].Track_Error, result.Steps[0].PCB_Loader = 0.00199, 0.0190, (0.0099, 5.6, 'pcb / z_carriage')
    warnings = Program_Warnings(result, cell)
    assert warnings == ['S3_To_Loader: PCB slip 1.99 mm at 6.864 s above 1.98 mm (1.5 x the legacy run)',
                        'S3_To_Loader: robot tracking error 0.0190 rad above 0.0189 rad (1.5 x the legacy run)',
                        'S3_To_Loader: carried PCB 9.90 mm from the loader at 5.600 s (pcb / z_carriage), within Clearance.Loader_Free 10.0 mm']
    assert [(c.Name, c.Detail) for c in Check_Program_Acceptance(result, cell) if not c.Passed] == [
        ('free transfers >= Loader_Free 10.0 mm', '9.90 mm at 5.600 s, S3_To_Loader (pcb / z_carriage)')]


def test_the_program_report_has_its_sections():
    """
    Description:
        F11: Format_Program_Report: the program line, SPEC §4.6's lines, the holds, a row per step (FREE / LOCKED, tracking, slip,
        the loader distances), the carrying free transfers' slip with tilt and acceleration, the warnings, the 20 checks, the
        closing sentence.
    """

    lines = Format_Program_Report(_Program_Result(), Load_Cell_Configuration(), 'Program')
    assert lines[0] == 'program Program: physics run of its trajectory, cycle 65.800 s' and lines[1] == 'cycle time 65.8 s'
    assert 'vacuum holds: 3.280-9.910 s min 4/4 cups; 57.850-66.070 s min 4/4 cups' in lines
    row = next(line for line in lines if line.startswith('S3_To_Loader '))
    assert row.split()[1:7] == ['5.190-', '7.030', 'FREE', '0.0014', '1.28', '35.14'] and row.split()[7] == '20.02'
    assert ('  S3_To_Loader           slip 1.28 mm at 6.864 s: tilt 8.8 deg, acceleration 1.75 m/s^2; max tilt 8.9 deg, max acceleration '
            '2.23 m/s^2') in lines
    assert 'warnings: none' in lines and sum(1 for line in lines if line.endswith(('PASS', 'FAIL')) or '  PASS  ' in line) == 28
    assert lines[-1] == 'Program: the physics run meets SPEC §10.1 (28 of 28). Simulation results only: not validated for the real cell.'
