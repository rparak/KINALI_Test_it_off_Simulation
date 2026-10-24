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
# File Name: ../test/Configuration/test_program_parser.py
## =========================================================================== ##

# System (Default)
import os
import math
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the parser under test, the YAML reader
#   ../Configuration/Program_Parser, ../Configuration/Core
from Configuration.Program_Parser import (Load_Constants, Load_Simulation, Load_Program, Parse_Line, CONST_CONSTANTS_YAML, CONST_SIMULATION_YAML,
                                          CONST_PROGRAM_YAML)
from Configuration.Core import Read_YAML, Configuration_Error_Cls

"""
Description:
    Initialization of constants.

    The cell program's three files and their parser (SPEC §8, FEATURES.md F10). Each validation error is made by a
    minimal text edit of a shipped file (its lines keep their numbers) and its message must name the file, the line, the
    segment, job or key and - for a command - the line's text. Expected values are written out here.
"""
# SPEC §4.8: the 22 points' joints as tabulated (rad, rounded to 1e-4), by their names in Constants.yaml (SPEC §4.8's name table).
#   Unit [Dict<string, List<float>>] (rad)
CONST_SPEC_TABLE = {
    'P_HOME': [0.7896, 0.6910, 1.8333, -0.4286, 1.5708, 0.0042], 'P_INPUT_ABOVE': [0.4649, -0.1335, 2.0022, 0.5650, 1.5708, -0.3205],
    'P_INPUT_APPROACH': [0.4649, -0.3261, 2.1191, 0.8743, 1.5708, -0.3205], 'P_INPUT_PICK': [0.4649, -0.3633, 2.1304, 0.9229, 1.5708, -0.3205],
    'P_VIA_INPUT_LOADER': [0.0465, 0.0565, 1.4637, -0.1952, 1.4994, -0.7367], 'P_LOADER_ABOVE': [-0.3720, 0.1896, 1.7926, -0.0310, 1.4280, -1.1529],
    'P_LOADER_APPROACH': [-0.4242, 0.1057, 2.1193, 0.3873, 1.4249, -1.2055], 'P_LOADER_PLACE': [-0.4296, 0.0867, 2.1469, 0.4347, 1.4246, -1.2110],
    'P_LOADER_PICK': [-0.4302, 0.0847, 2.1496, 0.4395, 1.4246, -1.2116], 'P_LOADER_EXIT': [-0.4187, 0.1225, 2.0906, 0.3409, 1.4252, -1.2000],
    'P_VIA_HOME_MG06': [-0.0138, 0.2020, 0.7805, -1.6993, -0.0164, 0.4996], 'P_MG06_ABOVE': [-0.8205, 0.2056, 0.9667, -2.3805, -1.6059, 1.1519],
    'P_MG06_FORK_START': [-1.1328, 0.0136, 1.1340, -2.0213, -1.9182, 1.1519], 'P_MG06_CLOSE_END': [-0.5157, 0.0227, 2.4098, -0.7545, -1.3011, 1.1518],
    'P_MG06_BAR_CENTRE': [-0.5118, 0.0507, 2.3961, -0.7961, -1.2972, 1.1518], 'P_MG06_SLOT_EXIT': [-0.8243, 0.0582, 2.2746, -0.9252, -1.6097, 1.1518],
    'P_MG06_WAIT': [-0.8243, 0.2137, 2.0471, -1.3081, -1.6097, 1.1518], 'P_MG06_OPEN_JOIN': [-0.5034, 0.0542, 2.3958, -0.8000, -1.2887, 1.1520],
    'P_VIA_LOADER_OUTPUT': [0.7093, 0.7032, 1.8349, -0.4944, 1.5158, -0.0734], 'P_OUTPUT_ABOVE': [1.0463, 0.0611, 2.2028, 0.5709, 1.5708, 0.2609],
    'P_OUTPUT_APPROACH': [1.0462, -0.1697, 2.3385, 0.9374, 1.5708, 0.2609], 'P_OUTPUT_PLACE': [1.0462, -0.2120, 2.3510, 0.9922, 1.5708, 0.2609]}
# The program's segments in order (FEATURES.md F10, the owner's naming).
#   Unit [List<string>]
CONST_SEGMENTS = ['S1_Approach_Input', 'S2_Pick_Input', 'S3_To_Loader', 'S4_Handover', 'S5_Home_After_Handover', 'S6_Loader_Insert', 'S7_Loader_Park',
                  'S8_To_MG06', 'S9_Close_MG06', 'S10_Leave_MG06', 'S11_Test', 'S12_Return_MG06', 'S13_Open_MG06', 'S14_Home_After_MG06',
                  'S15_Loader_Fetch', 'S16_Loader_Return', 'S17_To_Take', 'S18_Take_From_Loader', 'S19_To_Output', 'S20_Place_Output',
                  'S21_Home_End']


def _Edited(tmp_path, source, old, new, name=None):
    """
    Description:
        A copy of a shipped file with one text replaced (exactly once present), in tmp_path under its own name (or name).
    """

    text = open(source, encoding='utf-8').read()
    assert text.count(old) >= 1, old
    path = tmp_path / (name or os.path.basename(source))
    path.write_text(text.replace(old, new, 1), encoding='utf-8')
    return str(path)


def _Line_Of(source, text):
    """
    Description:
        The line number (1-based) of the first line of a shipped file containing text.
    """

    return next(i for i, line in enumerate(open(source, encoding='utf-8'), start=1) if text in line)


def _Load(cell, constants=CONST_CONSTANTS_YAML, simulation=CONST_SIMULATION_YAML, program=CONST_PROGRAM_YAML):
    """
    Description:
        The three files loaded, as a script loads them.
    """

    c = Load_Constants(constants)
    return Load_Program(c, Load_Simulation(c, simulation), cell, program)


def _Error(call):
    with pytest.raises(Configuration_Error_Cls) as error:
        call()
    return str(error.value)


def test_the_yaml_loader_rejects_duplicate_keys_and_keeps_on_off_as_text(tmp_path):
    """
    Description:
        The project's YAML loader: a duplicate key fails naming it and both lines; ON, OFF, On, Off, yes, no stay strings; only
        true / false are booleans; every string carries its line.
    """

    path = tmp_path / 'probe.yaml'
    path.write_text('a: ON\nb: OFF\nc: On\nd: Off\ne: yes\nf: no\ng: true\nh: false\ni: P_HOME\n', encoding='utf-8')
    document = Read_YAML(str(path))
    assert [document[k] for k in 'abcdef'] == ['ON', 'OFF', 'On', 'Off', 'yes', 'no'] and document['g'] is True and document['h'] is False
    assert document['i'].Line == 9
    path.write_text('Program:\n  S1_A: []\n  S1_A: []\n', encoding='utf-8')
    message = _Error(lambda: Read_YAML(str(path)))
    assert "found duplicate key 'S1_A' (first at line 2)" in message and 'line 3' in message


def test_a_command_line_is_parsed_never_evaluated():
    """
    Description:
        Parse_Line: a call with constant names, literals and keywords; ON stays a name; a Cell.yaml reference is a dotted name;
        anything that is not a plain call (an attribute of a call, a computed argument, ** keywords, a syntax error) fails.
    """

    assert Parse_Line('Robot.MoveL(P_INPUT_PICK, v=0.03)') == ('Robot.MoveL', [('name', 'P_INPUT_PICK')], {'v': 0.03})
    assert Parse_Line('Robot.SetDO(DO_VACUUM, ON)') == ('Robot.SetDO', [('name', 'DO_VACUUM'), ('name', 'ON')], {})
    assert Parse_Line('Test(Process.Test_Duration)') == ('Test', [('name', 'Process.Test_Duration')], {})
    assert Parse_Line('Wait(0.25)') == ('Wait', [('value', 0.25)], {})
    for text, message in (("__import__('os').system('echo')", 'must be a name or Station.Command'), ('Wait(1 + 1)', 'a constant name or a literal'),
                          ('Robot.MoveJ(P_HOME, **x)', '** arguments'), ('Robot.MoveJ(P_HOME', 'not a command call'), ('P_HOME',
                                                                                                                       'not a command call'),
                          ('Robot.MoveJ(P_HOME, v=abs(1))', 'v= must be a literal')):
        with pytest.raises(ValueError, match=message.replace('*', '[*]')):
            Parse_Line(text)


def test_the_shipped_files_load(program, constants, simulation):
    """
    Description:
        The three shipped files load without notes: 2 tools, 23 points, 4 loader positions, 2 gripper commands, 2 paths, 2
        signals; the 21 segments in order with 52 lines; the loader's 6 jobs and the tester's one; the start state.
    """

    assert program.Notes == []
    assert [len(t) for t in (constants.Tools, constants.Points, constants.Loader, constants.Gripper, constants.Paths,
                             constants.Signals)] == [2, 23, 4, 2, 2, 2]
    assert [s.Key for s in program.Segments] == CONST_SEGMENTS and sum(len(s.Commands) for s in program.Segments) == 52
    assert list(program.Loader_Jobs) == ['J_CLAMP', 'J_INSERT', 'J_PARK', 'J_FETCH', 'J_RETURN',
                                         'J_RELEASE'] and list(program.Tester_Jobs) == ['J_TEST']
    assert (simulation.Start_Robot, simulation.Start_Loader, simulation.Start_Gripper, simulation.Start_Vacuum, simulation.Start_PCB,
            simulation.Start_MG06) == ('P_HOME', 'L_HOME', 'G_OPEN', False, 'input_crate', 'Open')
    assert simulation.Signal_Response == {'DI_VACUUM_OK': 0.3} and simulation.End_Settle == 0.8
    assert simulation.Jaw_Actual == {'G_CLAMP': 0.017750000000000002}


def test_the_commands_are_resolved(program, constants, cell_configuration):
    """
    Description:
        The intermediate representation: a MoveL with its joint target (rad, the point's), tool site and speed; a MoveJ
        without v= at Cell.yaml's Speed_Fraction; the vacuum output with its actuator; the WaitDI with the simulated
        response; a Run with its job's lines resolved (axis, position, duration); the path with its start and end.
    """

    pick = program.Segments[1].Commands
    assert (pick[1].Kind, pick[1].Target, pick[1].Speed, pick[1].Site) == ('MoveL', 'P_INPUT_PICK', 0.03, 'aubo/tool/suction_tcp')
    assert pick[1].Joints == constants.Points['P_INPUT_PICK'].Joints and pick[1].Owner == 'S2_Pick_Input'
    assert pick[1].Line == _Line_Of(CONST_PROGRAM_YAML, 'Robot.MoveL(P_INPUT_PICK, v=0.03)')
    assert program.Segments[0].Commands[0].Speed == cell_configuration.Speed_Fraction
    assert (pick[2].Kind, pick[2].Value, pick[2].Actuator) == ('SetDO', 'ON', 'vacuum')
    assert (pick[3].Kind, pick[3].Duration, pick[3].Timeout) == ('WaitDI', 0.3, 1.0)
    insert = program.Segments[5].Commands[0]
    assert insert.Kind == 'Run' and [(c.Kind, c.Target, c.Duration) for c in insert.Job[:2]] == [('MoveZ', 'L_TRAVEL', 0.8), ('MoveY',
                                                                                                                              'L_TRAVEL', 2.0)]
    assert insert.Job[1].Position == constants.Loader['L_TRAVEL'].Y and insert.Job[4].Jaw == 0.035
    close = program.Segments[8].Commands[0]
    assert (close.Kind, close.Phase, close.Path_Start, close.Path_End) == ('MovePath', 'CLOSE', 'P_MG06_FORK_START', 'P_MG06_CLOSE_END')
    assert program.Segments[1].Allow_Contact == ['cups_pcb', 'pcb_crate'] and program.Segments[0].Allow_Contact == []


def test_the_points_are_the_spec_table_and_the_reference_plans_rows(constants, root):
    """
    Description:
        Every point's joints are within the SPEC §4.8 table's rounding (1e-4 rad) and a stationary row of
        Data/Reference/cell_plan.npz to the file's rounding (1e-6 deg) - except P_INPUT_CLEAR, the default program's own point.
    """

    plan = np.load(os.path.join(root, 'Data', 'Reference', 'cell_plan.npz'))
    for name, table in CONST_SPEC_TABLE.items():
        joints = np.array(constants.Points[name].Joints)
        assert np.max(np.abs(joints - table)) <= 1e-4, name
        nearest = plan['q_home'] if name == 'P_HOME' else plan['q'][np.argmin(np.max(np.abs(plan['q'] - joints), axis=1))]
        assert np.max(np.abs(nearest - joints)) <= math.radians(0.5e-6) + 1e-12, name
    assert set(constants.Points) - set(CONST_SPEC_TABLE) == {'P_INPUT_CLEAR'}


@pytest.mark.parametrize('old, new, text', [
    ('    Offset: [0.0, 0.0, -0.1215]', '    Offset: null', 'Points.P_INPUT_ABOVE: Reference and Offset are both null or both given'),
    ('  P_HOME:\n    Description: "Start and end of the cycle, tool down."\n    Tool: T_SUCTION\n    Reference: null\n    Offset: null',
     '  P_HOME:\n    Description: "Start and end of the cycle, tool down."\n    Tool: T_SUCTION\n    Reference: null\n    Offset: [0.0, 0.0, 0.1]',
     'Points.P_HOME: Reference and Offset are both null or both given'),
    ('    Reference: P_INPUT_PICK\n    Offset: [0.0, 0.0, -0.1215]', '    Reference: P_NOWHERE\n    Offset: [0.0, 0.0, -0.1215]',
     "Points.P_INPUT_ABOVE.Reference: 'P_NOWHERE' is not a point"),
    ('    Offset: [0.0, 0.0, -0.1215]', '    Offset: [0.0, -0.1215]', 'Points.P_INPUT_ABOVE.Offset: must be 3 numbers'),
    ('    Tool: T_SUCTION\n    Reference: null', '    Tool: T_FINGER\n    Reference: null', "Points.P_HOME.Tool: 'T_FINGER' is not a tool"),
    ('  L_PARK:', '  P_PARK:', 'Loader_Positions.P_PARK: a name in Loader_Positions is UPPERCASE and starts with L_'),
    ('  L_PARK:', '  L_park:', 'Loader_Positions.L_park: a name in Loader_Positions is UPPERCASE and starts with L_'),
    ('  L_PARK:', '  G_OPEN:', 'Gripper_Commands.G_OPEN: already declared in Loader_Positions'),
    ('    Phase: CLOSE', '    Phase: SHUT', "Paths.PATH_FORK_CLOSE.Phase: 'SHUT' is not one of CLOSE, OPEN"),
    ('    File: Data/Paths/fork_close_open_path.npz', '    File: Data/Paths/none.npz',
     "Paths.PATH_FORK_CLOSE.File: 'Data/Paths/none.npz' does not exist"),
    ('    Joints: [45.242793, 39.593430, 105.038309, -24.555084, 89.999924, 0.242831]\n', '', 'Points.P_HOME.Joints: missing (REQUIRED)'),
    ('    Description: "Fully open."\n', '', 'Gripper_Commands.G_OPEN.Description: missing (REQUIRED)'),
])
def test_an_invalid_constant_is_named_with_its_file_and_line(tmp_path, old, new, text):
    """
    Description:
        Each kind of invalid constant - Reference and Offset not both null or both given (each way round), an unknown
        Reference, a wrong Offset, an unknown tool, a name with the wrong prefix or not UPPERCASE, a name declared twice, a
        wrong path phase or file, a missing field - fails naming Constants.yaml, the line and the constant.
    """

    message = _Error(lambda: Load_Constants(_Edited(tmp_path, CONST_CONSTANTS_YAML, old, new)))
    assert text in message and 'Constants.yaml:' in message


def test_an_offset_cycle_is_rejected(tmp_path):
    """
    Description:
        P_INPUT_PICK made derived from P_INPUT_ABOVE, which is derived from it: an offset cycle, named with the chain.
    """

    old = ('  P_INPUT_PICK:\n    Description: "Cups pressed 1.5 mm onto the PCB, then vacuum on (SPEC §4.7)."\n    Tool: T_SUCTION\n'
           '    Reference: null\n    Offset: null')
    new = old.replace('Reference: null\n    Offset: null', 'Reference: P_INPUT_ABOVE\n    Offset: [0.0, 0.0, 0.1215]')
    message = _Error(lambda: Load_Constants(_Edited(tmp_path, CONST_CONSTANTS_YAML, old, new)))
    assert 'an offset cycle: P_INPUT_ABOVE -> P_INPUT_PICK -> P_INPUT_ABOVE' in message


@pytest.mark.parametrize('old, new, text', [
    ('  Vacuum: OFF', '  Vacuum: maybe', "Start.Vacuum: 'maybe' is not one of ON, OFF"),
    ('  Robot: P_HOME', '  Robot: P_GARAGE', "Start.Robot: 'P_GARAGE' is not one of"),
    ('  T_FLANGE: aubo/tool/tool_flange\n', '', 'Tool_Site.T_FLANGE: missing: every tool needs its model site'),
    ('  G_CLAMP: 0.017750000000000002', '  G_SQUEEZE: 0.017750000000000002', 'Jaw_Actual.G_SQUEEZE: is not a declared gripper command'),
    ('  P_HOME:\n    Context', '  P_HOMME:\n    Context', 'Points.P_HOMME: is not a point of Constants.yaml (unknown point)'),
    ('    Fixture: Open\n    Contacts: []\n    PCB_Hold: null\n  P_INPUT_ABOVE',
     '    Fixture: Ajar\n    Contacts: []\n    PCB_Hold: null\n  P_INPUT_ABOVE',
     "Points.P_HOME.Fixture: 'Ajar' is not one of Open, Closed"),
    ('End_Settle: 0.8', 'End_Settle: -1.0', 'End_Settle: -1.0 is outside'),
])
def test_invalid_simulation_data_is_named_with_its_file_and_line(tmp_path, constants, old, new, text):
    """
    Description:
        Each kind of invalid simulation data - the start state, a tool without its model site, a jaw stop of an undeclared
        command, an unknown point, a wrong MG06 state, a negative settle - fails naming Simulation.yaml, the line and the key.
    """

    message = _Error(lambda: Load_Simulation(constants, _Edited(tmp_path, CONST_SIMULATION_YAML, old, new)))
    assert text in message and 'Simulation.yaml:' in message


@pytest.mark.parametrize('old, new, text', [
    ('Robot.MoveL(P_INPUT_PICK, v=0.03)', 'Robot.MoveL(P_INPUT_PICK)', 'Robot.MoveL needs v= (REQUIRED)'),
    ('Robot.MoveL(P_LOADER_PLACE, v=0.03)', 'Robot.MoveL(L_PLACE, v=0.03)',
     'wrong constant type: Robot.MoveL takes a P_ point, L_PLACE is an L_ loader position'),
    ('Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveJ(P_NOWHERE)', "'P_NOWHERE' is not a declared constant (Constants.yaml); Robot.MoveJ takes a P_ point"),
    ('Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveJ(P_INPUT_ABOVE, speed=0.3)', 'Robot.MoveJ has no argument speed='),
    ('Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveJ(P_INPUT_ABOVE, a=0.5)', 'a= is reserved (no blending yet): only 0 or omitted, not 0.5'),
    ('Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveJ(P_INPUT_ABOVE, r=0.01)', 'r= is reserved (no blending yet): only 0 or omitted, not 0.01'),
    ('Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveJ(P_INPUT_ABOVE, v=1.5)', 'MoveJ v= is a fraction of the datasheet joint speeds in (0, 1], not 1.5'),
    ('Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveJ(P_INPUT_ABOVE, v=0)', 'MoveJ v= is a fraction of the datasheet joint speeds in (0, 1], not 0'),
    ('Robot.MoveL(P_INPUT_PICK, v=0.03)', 'Robot.MoveL(P_INPUT_PICK, v=2.0)', 'MoveL v= is the tool speed in m/s within [0.001, 1], not 2.0'),
    ('Loader.Run(J_INSERT)', 'Loader.Start(J_INSERT)', 'Loader.Start is reserved for F9'),
    ('Tester.Run(J_TEST)', 'Tester.WaitDone(J_TEST)', 'Tester.WaitDone is reserved for F9'),
    ('Loader.Run(J_INSERT)', 'Loader.Run(J_NOPE)',
     "Loader.Run takes a job of Loader_Jobs (J_CLAMP, J_INSERT, J_PARK, J_FETCH, J_RETURN, J_RELEASE), not 'J_NOPE'"),
    ('Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)', 'Robot.WaitDI(DI_VACUUM_OK)', 'Robot.WaitDI needs timeout= (REQUIRED)'),
    ('Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)', 'Robot.WaitDI(DI_VACUUM_OK, timeout=0.2)',
     'the simulated response of DI_VACUUM_OK (0.3 s) exceeds the timeout 0.2 s: the wait would fail'),
    ('Robot.SetDO(DO_VACUUM, ON)', 'Robot.SetDO(DO_VACUUM, MAYBE)', "Robot.SetDO takes ON or OFF, not 'MAYBE'"),
    ('Robot.MoveJ(P_INPUT_ABOVE)', 'MoveZ(L_TRAVEL, t=0.8)', "'MoveZ' is not a command here"),
    ('Robot.MoveJ(P_INPUT_ABOVE)', 'Robot.MoveJ(P_INPUT_ABOVE', 'not a command call'),
    ('Robot.MoveJ(P_INPUT_ABOVE)', "__import__('os').system('echo')", 'the command must be a name or Station.Command'),
])
def test_an_invalid_program_line_is_named_with_file_segment_line_and_text(tmp_path, cell_configuration, old, new, text):
    """
    Description:
        Each kind of invalid robot line - a missing v= / timeout=, a constant of the wrong type, an unknown constant, job or
        argument, a reserved a= / r= / Start / WaitDone, a speed out of range, a response beyond the timeout, a wrong state, a
        job's command in the robot's program, a syntax error, code - fails naming Program.yaml, the line, the segment and the
        line's text.
    """

    line = _Line_Of(CONST_PROGRAM_YAML, old)
    message = _Error(lambda: _Load(cell_configuration, program=_Edited(tmp_path, CONST_PROGRAM_YAML, old, new)))
    assert text in message, message
    assert any(part.startswith('Program.yaml:%d: S' % line) and "'%s'" % new in part for part in message.split(os.sep)), message


@pytest.mark.parametrize('old, new, text', [
    ('    - MoveZ(L_TRAVEL, t=0.8)', '    - MoveZ(L_TRAVEL)', 'J_INSERT: \'MoveZ(L_TRAVEL)\': MoveZ needs t= (REQUIRED)'),
    ('    - MoveZ(L_TRAVEL, t=0.8)', '    - MoveZ(P_HOME, t=0.8)', 'wrong constant type: MoveZ takes an L_ loader position, P_HOME is a P_ point'),
    ('    - MoveZ(L_TRAVEL, t=0.8)', '    - Robot.MoveJ(P_HOME)', "'Robot.MoveJ' is not a command here"),
    ('    - Test(Process.Test_Duration)', '    - Test(Process.Coffee)', 'Test takes Process.Test_Duration (a Cell.yaml duration)'),
])
def test_an_invalid_job_line_is_named(tmp_path, cell_configuration, old, new, text):
    """
    Description:
        A loader job's line without t=, with a constant of the wrong type or a robot command (the robot never drives the
        loader's axes, the loader never the robot), and a tester job's unknown duration, each named with the job and the line.
    """

    message = _Error(lambda: _Load(cell_configuration, program=_Edited(tmp_path, CONST_PROGRAM_YAML, old, new)))
    assert text in message and 'Program.yaml:%d: J_' % _Line_Of(CONST_PROGRAM_YAML, old.strip()) in message, message


@pytest.mark.parametrize('old, new, text', [
    ('  S3_To_Loader:', '  S4_To_Loader:', 'Program.S4_To_Loader: the segments are numbered S1..Sn in order: this is segment 3, so S3_To_Loader'),
    ('  S5_Home_After_Handover:', '  S5_Handover:', 'Program.S5_Handover: the name Handover is already used by S4_Handover'),
    ('  S5_Home_After_Handover:', '  Home_After_Handover:', 'Program.Home_After_Handover: a segment is S<n>_<Name>'),
    ('  S5_Home_After_Handover:', '  S4_Handover:', "found duplicate key 'S4_Handover'"),
    ('    - Loader.Run(J_RELEASE)', '    - Wait(0.35)', 'Loader_Jobs.J_RELEASE: a job no segment runs'),
])
def test_the_segment_and_job_rules(tmp_path, cell_configuration, old, new, text):
    """
    Description:
        The segments are S1..Sn in order with unique names (a gap, a reused name, a key without S<n>_, a duplicate key), and
        every job is run.
    """

    assert text in _Error(lambda: _Load(cell_configuration, program=_Edited(tmp_path, CONST_PROGRAM_YAML, old, new)))


def test_a_program_using_a_value_v_and_blank_a_r_loads(tmp_path, cell_configuration):
    """
    Description:
        a=0 and r=0 are accepted (no blending), and a MoveJ with v=0.3 carries it.
    """

    program = _Load(cell_configuration, program=_Edited(tmp_path, CONST_PROGRAM_YAML, 'Robot.MoveJ(P_INPUT_ABOVE)',
                                                        'Robot.MoveJ(P_INPUT_ABOVE, v=0.3, a=0, r=0)'))
    assert program.Segments[0].Commands[0].Speed == 0.3


@pytest.mark.parametrize('file, old, new, text', [
    ('simulation', '  To_Output: [cups_pcb]', '  To_Outlet: [cups_pcb]',
     'Allow_Contact.To_Outlet: is not a segment of Program.yaml (unknown segment'),
    ('simulation', 'Actuators:\n  DO_VACUUM: vacuum', 'Actuators: {}', 'DO_VACUUM has no simulated actuator (Simulation.yaml Actuators)'),
    ('simulation', 'Actuators:\n  DO_VACUUM: vacuum', 'Actuators: {}', 'Signals.DO_VACUUM: an output without a simulated actuator'),
    ('simulation', 'Signal_Response:\n  DI_VACUUM_OK: 0.3', 'Signal_Response: {}',
     'Signals.DI_VACUUM_OK: an input without a simulated response time'),
    ('constants', 'Loader_Positions:\n', 'Loader_Positions:\n  L_SPARE:\n    Description: "Never used."\n    Y: 0.0\n    Z: 0.0\n',
     'Loader_Positions.L_SPARE: declared but used by neither Program.yaml nor Simulation.yaml'),
])
def test_signals_contacts_and_constants_are_complete(tmp_path, cell_configuration, file, old, new, text):
    """
    Description:
        Across the files: Allow_Contact names segments; every output has a simulated actuator and every input a response
        time; every declared constant is used (here a loader position nobody uses).
    """

    if file == 'simulation':
        message = _Error(lambda: _Load(cell_configuration, simulation=_Edited(tmp_path, CONST_SIMULATION_YAML, old, new)))
    else:
        message = _Error(lambda: _Load(cell_configuration, constants=_Edited(tmp_path, CONST_CONSTANTS_YAML, old, new)))
    assert text in message, message


def test_the_family_changes_only_across_the_switch_point(tmp_path, cell_configuration):
    """
    Description:
        Without P_VIA_HOME_MG06 on the way to MG06 the family changes between P_HOME and P_MG06_ABOVE (not the switch point);
        a MoveL into P_VIA_HOME_MG06 changes the family by a line - both named with the segment and the line.
    """

    via = '    - Robot.MoveJ(P_VIA_HOME_MG06)                        # legacy: the via joint_move inserted (SPEC §4.10)\n'
    message = _Error(lambda: _Load(cell_configuration, program=_Edited(tmp_path, CONST_PROGRAM_YAML, via, '')))
    assert 'S8_To_MG06' in message and 'the IK family changes (P_HOME to P_MG06_ABOVE) away from P_VIA_HOME_MG06, the only switch point' in message
    message = _Error(lambda: _Load(cell_configuration, program=_Edited(tmp_path, CONST_PROGRAM_YAML, 'Robot.MoveJ(P_VIA_HOME_MG06)',
                                                                         'Robot.MoveL(P_VIA_HOME_MG06, v=0.2)')))
    assert 'a MoveL cannot change the IK family (P_HOME to P_VIA_HOME_MG06); use MoveJ' in message


def test_a_path_must_start_where_the_robot_stands(tmp_path, cell_configuration):
    """
    Description:
        Without the MoveJ to P_MG06_FORK_START the robot stands at P_MG06_ABOVE when PATH_FORK_CLOSE starts: named with its
        start point; after a path the robot stands at its End (the next MoveL from P_MG06_CLOSE_END passes).
    """

    start = "    - Robot.MoveJ(P_MG06_FORK_START)                      # legacy: joint_move(q_f0, '-> fork start pose')\n"
    message = _Error(lambda: _Load(cell_configuration, program=_Edited(tmp_path, CONST_PROGRAM_YAML, start, '')))
    assert 'S9_Close_MG06' in message and 'PATH_FORK_CLOSE starts at P_MG06_FORK_START, but the robot stands at P_MG06_ABOVE' in message
