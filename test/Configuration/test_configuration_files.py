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
# PyYAML (YAML parser and emitter) [pip3 install pyyaml]
import yaml
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the loaders under test
#   ../Configuration/Core
from Configuration.Core import Load_Cell_Configuration, Configuration_Error_Cls, CONST_CELL_YAML

"""
Description:
    Initialization of constants.

    Configuration/Cell.yaml and its loader (SPEC §8, project-style-rpx references/yaml.md). The required fields are written
    out here, not read from the loader. (The program's three files: test_program_parser.py.)
"""
# Every field of Configuration/Cell.yaml (all REQUIRED).
#   Unit [List<Tuple<string>>]
CONST_CELL_FIELDS = [('Robot', 'Base_Position'), ('Robot', 'Base_Yaw'), ('Vacuum', 'Pressure'), ('Vacuum', 'Cup_Diameter'),
                     ('MG06', 'Closed_Lid'), ('MG06', 'Closed_Handle'), ('Clearance', 'Robot'), ('Clearance', 'Allowed_Depth'),
                     ('Clearance', 'Stale_Position'), ('Clearance', 'Stale_Angle'),
                     ('Clearance', 'Family_Margin'), ('Clearance', 'Check_Step'), ('MG06', 'Open_Join_Margin'), ('Handover', 'Ledge_Gap'),
                     ('Handover', 'Ledge_Tolerance'), ('Handover', 'Lateral_Max'), ('Motion', 'Speed_Fraction'), ('Motion', 'Datasheet_Speeds'),
                     ('Motion', 'Acceleration_Limits'), ('Motion', 'Movej_Min_Duration'), ('Motion', 'Movel_Min_Duration'),
                     ('Process', 'Test_Duration'), ('Acceptance', 'Place_Error'), ('Acceptance', 'Place_Tilt'),
                     ('Acceptance', 'Crate_Distance'), ('Acceptance', 'Open_Tolerance'), ('Acceptance', 'Penetration'),
                     ('Simulation', 'Phase_Steps'), ('Simulation', 'Cup_Touch')]


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
    assert c.Open_Join_Margin == pytest.approx(math.radians(0.2)) and (c.Ledge_Gap, c.Ledge_Tolerance, c.Lateral_Max) == (0.0003, 0.0001, 0.0005)
