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
from Configuration.Core import Load_Cell_Configuration, Load_Keypoints, Configuration_Error_Cls, CONST_CELL_YAML, CONST_KEYPOINTS_YAML

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
                     ('Clearance', 'Family_Margin'), ('Acceptance', 'Place_Error'), ('Acceptance', 'Place_Tilt'),
                     ('Acceptance', 'Crate_Distance'), ('Acceptance', 'Open_Tolerance'), ('Acceptance', 'Penetration'),
                     ('Simulation', 'Phase_Steps'), ('Simulation', 'Cup_Touch')]
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
    assert cell.Allowed_Depth == {'cups_pcb': 0.0016, 'handle_bar': 0.0005, 'pcb_crate': 0.0005, 'pcb_jaws': 0.0005}
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
        Configuration/Keypoints.yaml loads without notes: the 22 robot keypoints of SPEC §4.8 in the cycle's order,
        the four loader positions and the three gripper states; the fork keypoints carry their recorded MG06 state.
    """

    assert keypoints.Notes == [] and list(keypoints.Robot) == list(CONST_SPEC_TABLE)
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
        file's rounding (1e-6 deg = 1.7e-8 rad): full precision, not the table's 1e-4.
    """

    import os
    plan = np.load(os.path.join(root, 'Data', 'Reference', 'cell_plan.npz'))
    q, q_home = plan['q'], plan['q_home']
    for name, keypoint in keypoints.Robot.items():
        joints = np.array(keypoint.Joints)
        nearest = q_home if name == 'home' else q[np.argmin(np.max(np.abs(q - joints), axis=1))]
        assert np.max(np.abs(nearest - joints)) <= math.radians(0.5e-6) + 1e-12, name
