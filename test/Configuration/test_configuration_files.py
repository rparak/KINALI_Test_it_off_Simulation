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
# File Name: ../test/Configuration/test_configuration_files.py
## =========================================================================== ##

# System (Default)
import math
# Copy (Shallow and deep copy operations)
import copy
# PyYAML (YAML parser and emitter) [pip3 install pyyaml]
import yaml
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the loaders under test
#   ../Configuration/Core
from Configuration.Core import (Load_Cell_Configuration, Load_Keypoints, Load_Program, Configuration_Error_Cls, CONST_CELL_YAML, CONST_KEYPOINTS_YAML,
                                CONST_PROGRAM_DEFAULT_YAML)

"""
Description:
    Initialization of constants.

    The configuration files and their loader (SPEC §8, project-style-rpx references/yaml.md). The required
    fields and the SPEC §4.8 table are written out here, not read from the loader or the SPEC.
"""
# Every field of Configuration/Cell.yaml (all REQUIRED).
#   Unit [List<Tuple<string>>]
CONST_CELL_FIELDS = [('Robot', 'Base_Position'), ('Robot', 'Base_Yaw'), ('Vacuum', 'Pressure'), ('Vacuum', 'Cup_Diameter'),
                     ('MG06', 'Closed_Lid'), ('MG06', 'Closed_Handle'), ('Clearance', 'Robot'), ('Clearance', 'Allowed_Depth'),
                     ('Clearance', 'Check_Position'), ('Clearance', 'Stale_Position'), ('Clearance', 'Stale_Angle'),
                     ('Clearance', 'Family_Margin'), ('Clearance', 'Check_Step'), ('MG06', 'Open_Join_Margin'), ('Handover', 'Ledge_Gap'),
                     ('Handover', 'Ledge_Tolerance'), ('Motion', 'Speed_Fraction'), ('Motion', 'Datasheet_Speeds'),
                     ('Motion', 'Acceleration_Limits'), ('Motion', 'Movej_Min_Duration'), ('Motion', 'Movel_Min_Duration'),
                     ('Process', 'Test_Duration'), ('Acceptance', 'Place_Error'), ('Acceptance', 'Place_Tilt'),
                     ('Acceptance', 'Crate_Distance'), ('Acceptance', 'Open_Tolerance'), ('Acceptance', 'Penetration'),
                     ('Simulation', 'Phase_Steps'), ('Simulation', 'Cup_Touch')]
# The default program's steps in order (FEATURES.md F8, the owner's approval 2026-10-24).
#   Unit [List<string>]
CONST_PROGRAM_STEPS = ['approach_input', 'pick_input', 'to_loader', 'handover', 'home_after_handover', 'loader_insert', 'loader_park',
                       'to_mg06', 'close_mg06', 'leave_mg06', 'test', 'return_mg06', 'open_mg06', 'home_after_mg06', 'loader_fetch',
                       'loader_return', 'to_take', 'take_from_loader', 'to_output', 'place_output', 'home_end', 'settle']
# The auxiliary keypoints the default program adds (FEATURES.md F8.5): not in SPEC §4.8 or the reference plan; the keypoint
# each follows in the file.
#   Unit [Dict<string, string>]
CONST_AUXILIARY = {'input_pcb_clear': 'input_pcb_press'}
# Every field of a robot keypoint (all REQUIRED; Relative_To may be null, Allow_Contact empty).
#   Unit [List<string>]
CONST_KEYPOINT_FIELDS = ['Joints', 'Family', 'Check', 'Relative_To', 'Fixture', 'Context', 'Allow_Contact', 'Note']
# SPEC §4.8: the 22 keypoints' joints as tabulated (rad, rounded to 1e-4).
#   Unit [Dict<string, List<float>>] (rad)
CONST_SPEC_TABLE = {
    'home': [0.7896, 0.6910, 1.8333, -0.4286, 1.5708, 0.0042], 'above_input_pcb': [0.4649, -0.1335, 2.0022, 0.5650, 1.5708, -0.3205],
    'input_pcb_approach': [0.4649, -0.3261, 2.1191, 0.8743, 1.5708, -0.3205], 'input_pcb_press': [0.4649, -0.3633, 2.1304, 0.9229, 1.5708, -0.3205],
    'via_input_to_loader': [0.0465, 0.0565, 1.4637, -0.1952, 1.4994, -0.7367], 'above_loader': [-0.3720, 0.1896, 1.7926, -0.0310, 1.4280, -1.1529],
    'loader_approach': [-0.4242, 0.1057, 2.1193, 0.3873, 1.4249, -1.2055], 'loader_handover': [-0.4296, 0.0867, 2.1469, 0.4347, 1.4246, -1.2110],
    'loader_take_press': [-0.4302, 0.0847, 2.1496, 0.4395, 1.4246, -1.2116], 'loader_out_of_jaws': [-0.4187, 0.1225, 2.0906, 0.3409, 1.4252, -1.2000],
    'via_mg06_high': [-0.0138, 0.2020, 0.7805, -1.6993, -0.0164, 0.4996], 'above_handle': [-0.8205, 0.2056, 0.9667, -2.3805, -1.6059, 1.1519],
    'fork_start': [-1.1328, 0.0136, 1.1340, -2.0213, -1.9182, 1.1519], 'fork_closed_end': [-0.5157, 0.0227, 2.4098, -0.7545, -1.3011, 1.1518],
    'fork_bar_centred': [-0.5118, 0.0507, 2.3961, -0.7961, -1.2972, 1.1518], 'fork_out_of_bar': [-0.8243, 0.0582, 2.2746, -0.9252, -1.6097, 1.1518],
    'fork_retreat_up': [-0.8243, 0.2137, 2.0471, -1.3081, -1.6097, 1.1518], 'fork_open_join': [-0.5034, 0.0542, 2.3958, -0.8000, -1.2887, 1.1520],
    'via_loader_to_output': [0.7093, 0.7032, 1.8349, -0.4944, 1.5158, -0.0734], 'above_output': [1.0463, 0.0611, 2.2028, 0.5709, 1.5708, 0.2609],
    'output_approach': [1.0462, -0.1697, 2.3385, 0.9374, 1.5708, 0.2609], 'output_place': [1.0462, -0.2120, 2.3510, 0.9922, 1.5708, 0.2609]}


def _Mutated(tmp_path, source, change):
    """
    Description:
        A copy of a shipped YAML file with change(tree) applied, written to tmp_path.
    """

    with open(source, encoding='utf-8') as handle:
        tree = yaml.safe_load(handle)
    change(tree)
    path = tmp_path / 'mutated.yaml'
    path.write_text(yaml.safe_dump(tree, sort_keys=False), encoding='utf-8')
    return str(path)


def _Error(loader, path):
    with pytest.raises(Configuration_Error_Cls) as error:
        loader(path)
    return str(error.value)


def test_the_shipped_cell_configuration_loads_in_si_units():
    """
    Description:
        Configuration/Cell.yaml loads without notes; degrees are radians, the values are the verified ones (the
        base pose and yaw, 40 kPa, D15, the closed MG06, 3 mm, the acceptance limits, 20 steps, 0.5 mm).
    """

    cell = Load_Cell_Configuration()
    assert cell.Notes == []
    assert cell.Base_Position == [0.42072, -0.33497, 0.162] and cell.Base_Yaw == pytest.approx(math.radians(-135.0), abs=0)
    assert (cell.Vacuum_Pressure, cell.Cup_Diameter, cell.Clearance) == (40000.0, 0.015, 0.003)
    assert cell.Closed_Lid == pytest.approx(math.radians(-0.5)) and cell.Closed_Handle == pytest.approx(math.radians(28.65))
    assert cell.Allowed_Depth == {'cups_pcb': 0.0016, 'handle_bar': 0.0006, 'pcb_crate': 0.0005, 'pcb_jaws': 0.0005}
    assert (cell.Place_Error, cell.Crate_Distance, cell.Penetration, cell.Phase_Steps, cell.Cup_Touch) == (0.0005, 0.002, 0.0005, 20, 0.0005)


@pytest.mark.parametrize('keys', CONST_CELL_FIELDS)
def test_a_missing_cell_field_is_named(tmp_path, keys):
    """
    Description:
        Removing any field of Cell.yaml fails the load naming its key path.
    """

    path = _Mutated(tmp_path, CONST_CELL_YAML, lambda t: t[keys[0]].pop(keys[1]))
    assert '%s: missing (REQUIRED)' % '.'.join(keys) in _Error(Load_Cell_Configuration, path)


def test_every_invalid_cell_value_is_named_at_once(tmp_path):
    """
    Description:
        A wrong vector length, a negative pressure, a zero phase period and an unknown group are all named in one
        error; nothing falls back to a default.
    """

    def Change(t):
        t['Robot']['Base_Position'] = [0.4, -0.3]
        t['Vacuum']['Pressure'] = -1.0
        t['Simulation']['Phase_Steps'] = 0
        t['Clearance']['Allowed_Depth'].pop('pcb_crate')
    message = _Error(Load_Cell_Configuration, _Mutated(tmp_path, CONST_CELL_YAML, Change))
    assert '4 problem(s)' in message
    for text in ('Robot.Base_Position: 2 values, 3 required', 'Vacuum.Pressure: -1.0 is outside', 'Simulation.Phase_Steps: must be a whole number',
                 'Clearance.Allowed_Depth.pcb_crate: missing (REQUIRED)'):
        assert text in message, text


def test_an_unknown_key_is_a_warning_note(tmp_path):
    """
    Description:
        An unknown key does not fail the load; it is returned as a WARNING note naming it.
    """

    cell = Load_Cell_Configuration(_Mutated(tmp_path, CONST_CELL_YAML, lambda t: t['Robot'].update(Base_Yaww=0.0)))
    assert len(cell.Notes) == 1 and cell.Notes[0][0] == 'WARNING'
    assert cell.Notes[0][1].endswith('Robot.Base_Yaww: unknown key, ignored (misspelt?)')


def test_the_shipped_keypoints_load(keypoints):
    """
    Description:
        Configuration/Keypoints.yaml loads without notes: the 22 robot keypoints of SPEC §4.8 and the default program's
        auxiliary point input_pcb_clear, in the cycle's order; the four loader positions and the three gripper states; the
        fork keypoints carry their recorded MG06 state.
    """

    expected = list(CONST_SPEC_TABLE)
    for name, after in CONST_AUXILIARY.items():
        expected.insert(expected.index(after) + 1, name)
    assert keypoints.Notes == [] and list(keypoints.Robot) == expected
    assert list(keypoints.Loader) == ['home', 'travel_place', 'place', 'park'] and list(keypoints.Gripper) == ['open', 'grip', 'clamp']
    assert (keypoints.Loader['home'].Y, keypoints.Loader['park'].Y, keypoints.Loader['park'].Z) == (-0.1, -0.1995, 0.02)
    end = keypoints.Robot['fork_closed_end']
    assert end.Fixture == 'Recorded' and math.degrees(end.Fixture_Handle) == pytest.approx(29.125260, abs=1e-6)
    assert keypoints.Robot['fork_bar_centred'].Fixture == 'Closed' and keypoints.Robot['home'].Fixture == 'Open'
    assert keypoints.Robot['home'].Relative_Target is None and keypoints.Robot['home'].Allow_Contact == []


@pytest.mark.parametrize('field', CONST_KEYPOINT_FIELDS)
def test_a_missing_keypoint_field_is_named(tmp_path, field):
    """
    Description:
        Removing any field of a robot keypoint fails the load naming its key path (no optional fields).
    """

    path = _Mutated(tmp_path, CONST_KEYPOINTS_YAML, lambda t: t['Robot']['home'].pop(field))
    assert 'Robot.home.%s: missing (REQUIRED)' % field in _Error(Load_Keypoints, path)


@pytest.mark.parametrize('change, text', [
    (lambda t: t['Robot']['home'].update(Joints=[0, 0, 0, 0, 0]), 'Robot.home.Joints: 5 values, 6 required'),
    (lambda t: t['Robot']['home'].update(Family='wrist_sideways'), "Robot.home.Family: 'wrist_sideways' is not one of wrist_up, wrist_down"),
    (lambda t: t['Robot']['home']['Context'].update(Loader='garage'), "Robot.home.Context.Loader: 'garage' is not one of"),
    (lambda t: t['Robot']['home']['Check'].update(Frame='elbow'), "Robot.home.Check.Frame: 'elbow' is not one of"),
    (lambda t: t['Robot']['home'].update(Fixture='Ajar'), "Robot.home.Fixture: 'Ajar' is not one of Open, Closed"),
    (lambda t: t['Robot']['home'].update(Allow_Contact=['elbow_table']), 'Robot.home.Allow_Contact:'),
    (lambda t: t['Robot']['loader_handover'].update(Allow_Contact=['cups_pcb', 'jaws_pcb']),
     "Robot.loader_handover.Allow_Contact: ['cups_pcb', 'jaws_pcb'] must be a list of cups_pcb, handle_bar, pcb_crate, pcb_jaws"),
    (lambda t: t['Robot']['input_pcb_press']['Relative_To'].update(Target='pcb_hold@garage'),
     "'garage' is not one of input_crate, jaws, output_crate"),
    (lambda t: t['Robot']['input_pcb_press']['Relative_To'].update(Target='nowhere'), "'nowhere' is neither pcb_hold@<location> nor a keypoint"),
    (lambda t: t['Robot']['fork_closed_end']['Fixture'].pop('Source'), 'Robot.fork_closed_end.Fixture.Source: missing (REQUIRED)'),
    (lambda t: t['Loader']['park'].pop('Z'), 'Loader.park.Z: missing (REQUIRED)'),
])
def test_an_invalid_keypoint_value_is_named(tmp_path, change, text):
    """
    Description:
        Each kind of invalid value is named with its key path.
    """

    assert text in _Error(Load_Keypoints, _Mutated(tmp_path, CONST_KEYPOINTS_YAML, change))


def test_the_joints_are_the_spec_table_at_full_precision(keypoints):
    """
    Description:
        Every keypoint's joints are within the SPEC §4.8 table's rounding (1e-4 rad): the full-precision values
        (FEATURES.md F6) are the tabulated keypoints.
    """

    for name, table in CONST_SPEC_TABLE.items():
        assert np.max(np.abs(np.array(keypoints.Robot[name].Joints) - table)) <= 1e-4, name


def test_the_joints_are_the_reference_plans_segment_ends(root, keypoints):
    """
    Description:
        Every keypoint's joints are a stationary row of Data/Reference/cell_plan.npz (q_home for home) to the
        file's rounding (1e-6 deg = 1.7e-8 rad): full precision, not the table's 1e-4. (The program's auxiliary points are
        not in the plan.)
    """

    import os
    plan = np.load(os.path.join(root, 'Data', 'Reference', 'cell_plan.npz'))
    q, q_home = plan['q'], plan['q_home']
    for name, keypoint in keypoints.Robot.items():
        if name in CONST_AUXILIARY:
            continue
        joints = np.array(keypoint.Joints)
        nearest = q_home if name == 'home' else q[np.argmin(np.max(np.abs(q - joints), axis=1))]
        assert np.max(np.abs(nearest - joints)) <= math.radians(0.5e-6) + 1e-12, name


def test_the_motion_process_and_handover_fields_load_in_si_units(cell_configuration):
    """
    Description:
        The step 4 fields of Cell.yaml: 60 % of the datasheet speeds (220 / 230 deg/s), the assumed 3 / 6 rad/s^2 exactly,
        MoveJ / MoveL at least 0.5 / 0.6 s, the 2.0 s test, the 0.2 deg open join, the 1 mm check step, the 0.3 +- 0.1 mm
        handover gap.
    """

    c = cell_configuration
    assert c.Speed_Fraction == 0.6 and c.Datasheet_Speeds == pytest.approx(np.radians([220, 220, 220, 220, 230, 230]), abs=1e-15)
    assert c.Acceleration_Limits == [3.0, 3.0, 3.0, 6.0, 6.0, 6.0]
    assert (c.Movej_Min_Duration, c.Movel_Min_Duration, c.Test_Duration, c.Check_Step) == (0.5, 0.6, 2.0, 0.001)
    assert c.Open_Join_Margin == pytest.approx(math.radians(0.2)) and (c.Ledge_Gap, c.Ledge_Tolerance) == (0.0003, 0.0001)


def test_the_shipped_default_program_loads(keypoints, cell_configuration):
    """
    Description:
        Configuration/Program_Default.yaml loads without notes: the start state, the 22 steps in order, 69 commands; a
        Movel, a Loader and the Test command parsed with their values.
    """

    program = Load_Program(keypoints, cell_configuration)
    assert program.Notes == [] and program.Name == 'Program_Default'
    assert (program.Start_Robot, program.Start_Loader, program.Start_Gripper, program.Start_Vacuum, program.Start_PCB,
            program.Start_MG06) == ('home', 'home', 'open', False, 'input_crate', 'Open')
    assert [s.Name for s in program.Steps] == CONST_PROGRAM_STEPS and sum(len(s.Commands) for s in program.Steps) == 69
    pick = program.Steps[1]
    assert pick.Allow_Contact == ['cups_pcb', 'pcb_crate'] and pick.Commands[1].Text == 'Movel input_pcb_press 0.03'
    assert (pick.Commands[1].Kind, pick.Commands[1].Target, pick.Commands[1].Speed) == ('Movel', 'input_pcb_press', 0.03)
    insert = program.Steps[5].Commands[1]
    assert (insert.Kind, insert.Axis, insert.Target, insert.Duration) == ('Loader', 'Y', 'travel_place', 2.0)
    test = program.Steps[10].Commands[0]
    assert (test.Kind, test.Target, test.Duration) == ('Test', 'Process.Test_Duration', 2.0)


@pytest.mark.parametrize('keys', [('Start', 'Robot'), ('Start', 'Loader'), ('Start', 'Gripper'), ('Start', 'Vacuum'), ('Start', 'PCB'),
                                  ('Start', 'MG06'), (1, 'Name'), (1, 'Comment'), (1, 'Allow_Contact'), (1, 'Commands')])
def test_a_missing_program_field_is_named(tmp_path, keypoints, cell_configuration, keys):
    """
    Description:
        Removing a field of the start state or of a step fails the load naming its key path (no optional fields).
    """

    def Change(t):
        (t['Start'] if keys[0] == 'Start' else t['Steps'][keys[0]]).pop(keys[1])
    path = _Mutated(tmp_path, CONST_PROGRAM_DEFAULT_YAML, Change)
    name = 'Start' if keys[0] == 'Start' else 'Steps[%d]' % keys[0]
    assert '%s.%s: missing (REQUIRED)' % (name, keys[1]) in _Error(lambda x: Load_Program(keypoints, cell_configuration, x), path)


@pytest.mark.parametrize('change, text', [
    (lambda t: t['Steps'][1]['Commands'][1]['Movel'].pop('Speed'), 'Steps[1].Commands[1].Movel.Speed: missing (REQUIRED)'),
    (lambda t: t['Steps'][1]['Commands'][1]['Movel'].update(Speed=3.0), 'Steps[1].Commands[1].Movel.Speed: 3.0 is outside'),
    (lambda t: t['Steps'][0]['Commands'].__setitem__(0, {'Movej': 'nowhere'}), "Steps[0].Commands[0].Movej: 'nowhere' is not one of"),
    (lambda t: t['Steps'][0]['Commands'].__setitem__(0, {'Jump': 'home'}), "Steps[0].Commands[0]: 'Jump' is not a command"),
    (lambda t: t['Steps'][5]['Commands'][0]['Loader'].update(Axis='X'), "Steps[5].Commands[0].Loader.Axis: 'X' is not one of Y, Z"),
    (lambda t: t['Steps'][5]['Commands'][0]['Loader'].update(To='garage'), "Steps[5].Commands[0].Loader.To: 'garage' is not one of"),
    (lambda t: t['Steps'][5]['Commands'][0]['Loader'].pop('Duration'), 'Steps[5].Commands[0].Loader.Duration: missing (REQUIRED)'),
    (lambda t: t['Steps'][3]['Commands'][2]['Gripper'].update(To='squeeze'), "Steps[3].Commands[2].Gripper.To: 'squeeze' is not one of"),
    (lambda t: t['Steps'][1]['Commands'].__setitem__(2, {'Vacuum': 'maybe'}), "Steps[1].Commands[2].Vacuum: must be On or Off"),
    (lambda t: t['Steps'][10]['Commands'].__setitem__(0, {'Test': 'Process.Coffee_Break'}),
     "Steps[10].Commands[0].Test: 'Process.Coffee_Break' is not one of Process.Test_Duration"),
    (lambda t: t['Steps'][8]['Commands'].__setitem__(0, {'Path': 'Fork_Wiggle'}), "Steps[8].Commands[0].Path: 'Fork_Wiggle' is not one of"),
    (lambda t: t['Steps'][1].update(Allow_Contact=['elbow_table']), 'Steps[1].Allow_Contact:'),
    (lambda t: t['Steps'][2].update(Name='pick_input'), "Steps[2].Name: 'pick_input' is used by an earlier step"),
    (lambda t: t['Steps'][2].update(Name='To Loader'), 'Steps[2].Name: must be a lowercase name without spaces'),
    (lambda t: t['Start'].update(PCB='pocket'), "Start.PCB: 'pocket' is not one of"),
])
def test_an_invalid_program_value_is_named(tmp_path, keypoints, cell_configuration, change, text):
    """
    Description:
        Each kind of invalid program value - a command field missing or out of range, an unknown keypoint, command, axis,
        loader position, gripper state, path, contact group or duration reference, a duplicate or malformed step name, an
        unknown PCB location - is named with its key path.
    """

    assert text in _Error(lambda x: Load_Program(keypoints, cell_configuration, x), _Mutated(tmp_path, CONST_PROGRAM_DEFAULT_YAML, change))


def test_every_invalid_program_value_is_named_at_once(tmp_path, keypoints, cell_configuration):
    """
    Description:
        Three problems in three steps fail the load once, naming all three; an unknown key in a step is a WARNING note.
    """

    def Change(t):
        t['Steps'][0]['Commands'][0] = {'Movej': 'nowhere'}
        t['Steps'][4]['Comment'] = ''
        t['Steps'][6]['Commands'][0]['Loader']['Axis'] = 'W'
    message = _Error(lambda x: Load_Program(keypoints, cell_configuration, x), _Mutated(tmp_path, CONST_PROGRAM_DEFAULT_YAML, Change))
    assert '3 problem(s)' in message and 'Steps[4].Comment: must say what the step does' in message
    program = Load_Program(keypoints, cell_configuration, _Mutated(tmp_path, CONST_PROGRAM_DEFAULT_YAML, lambda t: t['Steps'][0].update(Speed=1)))
    assert program.Notes == [('WARNING', program.Notes[0][1])] and program.Notes[0][1].endswith('Steps[0].Speed: unknown key, ignored (misspelt?)')
