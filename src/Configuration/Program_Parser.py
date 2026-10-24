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
# File Name: ../Configuration/Program_Parser.py
## =========================================================================== ##

# System (Default)
import os
# AST (Abstract syntax trees: the command lines are parsed, never evaluated)
import ast
# Math (Mathematical functions)
import math
# Re (Regular expressions)
import re
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Custom Lib.:
#   ../Configuration/Core - the YAML reader, the repository's files, the cell's closed sets and errors
from Configuration.Core import (Read_YAML, Configuration_Error_Cls, Cell_Configuration_Str, CONST_ROOT, CONST_CONFIGURATION, CONST_PCB_LOCATIONS,
                                CONST_CONTACT_GROUPS, CONST_FIXTURE_STATES, CONST_PROCESS_KEYS)

"""
Description:
    Initialization of constants.

    The cell program's three files (SPEC §8, FEATURES.md F10): Configuration/Constants.yaml (every named value of the
    real cell), Configuration/Simulation.yaml (what only the simulation needs) and Configuration/Program.yaml (what the
    cell's controllers run) - their only reader. Load_Constants, Load_Simulation and Load_Program validate everything
    that needs no model (names, types, references, the program's grammar and flow) and return a typed intermediate
    representation that Program and Keypoints use directly; the checks against the model (a Pose against FK, an
    offset's consistency, the loader's axis ranges) are Program.Core.Validate_Constants. A command line is parsed with
    Python's ast and never evaluated. Every error names the file, the line, the segment or job and the line's text.
"""
# The three files.
#   Unit [string]
CONST_CONSTANTS_YAML = os.path.join(CONST_CONFIGURATION, 'Constants.yaml')
CONST_SIMULATION_YAML = os.path.join(CONST_CONFIGURATION, 'Simulation.yaml')
CONST_PROGRAM_YAML = os.path.join(CONST_CONFIGURATION, 'Program.yaml')
# The sections of Constants.yaml and the prefixes of their names.
#   Unit [Dict<string, Tuple<string>>]
CONST_SECTIONS = {'Tools': ('T_',), 'Points': ('P_',), 'Loader_Positions': ('L_',), 'Gripper_Commands': ('G_',), 'Paths': ('PATH_',),
                  'Signals': ('DO_', 'DI_')}
# A constant's name: uppercase letters, digits and underscores after its prefix.
#   Unit [string]
CONST_NAME_PATTERN = r'^[A-Z][A-Z0-9_]*$'
# The recorded paths' phases (SPEC §4.9) and the digital states.
#   Unit [Tuple<string>]
CONST_PHASES, CONST_STATES = ('CLOSE', 'OPEN'), ('ON', 'OFF')
# The simulated actuators an output may drive (the four cups' adhesion).
#   Unit [Tuple<string>]
CONST_ACTUATORS = ('vacuum',)
# The point where the robot switches its IK family (wrist_2's sign), the only one (SPEC §4.7): a single legal value.
#   Unit [string]
CONST_FAMILY_SWITCH_POINT = 'P_VIA_HOME_MG06'
# wrist_2's index in the joint vector.
#   Unit [int]
CONST_WRIST_2 = 4
# The commands: per call name, its positional arguments' kinds and its keywords (True = required). Kinds: P point,
# PATH path, DO / DI signal, STATE ON / OFF, JL / JT a loader / tester job, L loader position, G gripper command,
# NUMBER a literal, PROCESS a Cell.yaml Process duration.
#   Unit [Dict<string, Tuple<List<string>, Dict<string, bool>>>]
CONST_ROBOT_COMMANDS = {'Robot.MoveJ': (['P'], {'v': False, 'a': False, 'r': False}), 'Robot.MoveL': (['P'], {'v': True, 'a': False, 'r': False}),
                        'Robot.MovePath': (['PATH'], {}), 'Robot.SetDO': (['DO', 'STATE'], {}), 'Robot.WaitDI': (['DI'], {'timeout': True}),
                        'Loader.Run': (['JL'], {}), 'Tester.Run': (['JT'], {}), 'Wait': (['NUMBER'], {})}
CONST_LOADER_COMMANDS = {'MoveY': (['L'], {'t': True}), 'MoveZ': (['L'], {'t': True}), 'Gripper': (['G'], {'t': True}), 'Wait': (['NUMBER'], {})}
CONST_TESTER_COMMANDS = {'Test': (['PROCESS'], {})}
# The station calls reserved for F9's synchronisation (Run = Start + WaitDone).
#   Unit [Tuple<string>]
CONST_RESERVED = ('Loader.Start', 'Loader.WaitDone', 'Tester.Start', 'Tester.WaitDone')
# What each argument kind is, for the messages; the section a constant kind is declared in.
#   Unit [Dict<string, string>]
CONST_KIND_WORDS = {'P': 'a P_ point', 'PATH': 'a PATH_ path', 'DO': 'a DO_ output', 'DI': 'a DI_ input', 'L': 'an L_ loader position',
                    'G': 'a G_ gripper command', 'T': 'a T_ tool'}
# The ranges of the literal values: a MoveJ's fraction of the datasheet speeds, a MoveL's speed [m/s], a time [s].
#   Unit [Tuple<float>]
CONST_MOVEJ_SPEED, CONST_MOVEL_SPEED, CONST_TIME = (0.0, 1.0), (0.001, 1.0), (0.0, 600.0)


class Program_Format_Error_Cls(Configuration_Error_Cls):
    """
    Description:
        A program file (Constants.yaml, Simulation.yaml, Program.yaml) is invalid. The message lists every problem at once,
        each with its file, line, segment or job and the line's text; a script prints it verbatim as an [ERROR] line.
    """


@dataclass
class Tool_Str:
    """
    Description:
        A tool on the flange (Constants.yaml Tools).
    """

    # The name, its description and its line.
    #   Unit [string], [string], [int]
    Name: str = ''
    Description: str = ''
    Line: int = 0
    # The tool frame in the flange frame: x, y, z, roll, pitch, yaw.
    #   Unit [Vector<float> 1x6] (m, m, m, rad, rad, rad)
    Flange_Offset: tp.List[float] = field(default_factory=list)


@dataclass
class Point_Str:
    """
    Description:
        A robot point (Constants.yaml Points).
    """

    # The name, its description and its line.
    #   Unit [string], [string], [int]
    Name: str = ''
    Description: str = ''
    Line: int = 0
    # The tool whose frame the Pose is.
    #   Unit [string]
    Tool: str = ''
    # An offset point's parent and its offset along the parent's tool axes; None for a point that is not one.
    #   Unit [string], [Vector<float> 1x3] (m)
    Reference: tp.Optional[str] = None
    Offset: tp.Optional[tp.List[float]] = None
    # The joints, shoulder_pan .. wrist_3 (authoritative), and the tool frame in the robot base frame (a check value).
    #   Unit [Vector<float> 1x6] (rad), [Vector<float> 1x6] (m, m, m, rad, rad, rad)
    Joints: tp.List[float] = field(default_factory=list)
    Pose: tp.List[float] = field(default_factory=list)


@dataclass
class Loader_Position_Str:
    """
    Description:
        A loader position (Constants.yaml Loader_Positions).
    """

    # The name, its description and its line.
    #   Unit [string], [string], [int]
    Name: str = ''
    Description: str = ''
    Line: int = 0
    # The Y axis (towards MG06) and the Z axis (up).
    #   Unit [float] (m)
    Y: float = 0.0
    Z: float = 0.0


@dataclass
class Gripper_Command_Str:
    """
    Description:
        A gripper command (Constants.yaml Gripper_Commands).
    """

    # The name, its description and its line.
    #   Unit [string], [string], [int]
    Name: str = ''
    Description: str = ''
    Line: int = 0
    # The jaw set-point, per jaw.
    #   Unit [float] (m)
    Jaw: float = 0.0


@dataclass
class Path_Str:
    """
    Description:
        A recorded flange path (Constants.yaml Paths).
    """

    # The name, its description and its line.
    #   Unit [string], [string], [int]
    Name: str = ''
    Description: str = ''
    Line: int = 0
    # The recording (absolute path), its phase (CLOSE or OPEN), the points it starts and ends at.
    #   Unit [string]
    File: str = ''
    Phase: str = ''
    Start: str = ''
    End: str = ''


@dataclass
class Signal_Str:
    """
    Description:
        A digital signal (Constants.yaml Signals).
    """

    # The name, its description and its line; DO or DI; its port (a placeholder until the real I/O map exists).
    #   Unit [string], [string], [int], [string], [string]
    Name: str = ''
    Description: str = ''
    Line: int = 0
    Kind: str = ''
    Port: str = ''


@dataclass
class Constants_Str:
    """
    Description:
        Constants.yaml: every named value, by section, in the file's order. Built by Load_Constants.
    """

    # The file.
    #   Unit [string]
    Path: str = ''
    # The constants by name.
    #   Unit [Dict<string, Tool_Str>], [Dict<string, Point_Str>], [Dict<string, Loader_Position_Str>],
    #        [Dict<string, Gripper_Command_Str>], [Dict<string, Path_Str>], [Dict<string, Signal_Str>]
    Tools: tp.Dict[str, Tool_Str] = field(default_factory=dict)
    Points: tp.Dict[str, Point_Str] = field(default_factory=dict)
    Loader: tp.Dict[str, Loader_Position_Str] = field(default_factory=dict)
    Gripper: tp.Dict[str, Gripper_Command_Str] = field(default_factory=dict)
    Paths: tp.Dict[str, Path_Str] = field(default_factory=dict)
    Signals: tp.Dict[str, Signal_Str] = field(default_factory=dict)
    # The loader's notes: (level, message) for each unknown key.
    #   Unit [List<Tuple<string>>]
    Notes: tp.List[tp.Tuple[str, str]] = field(default_factory=list)


@dataclass
class Point_Scene_Str:
    """
    Description:
        A point's scene in the simulation (Simulation.yaml Points): for the keypoint check, show and step tools.
    """

    # The point and its line.
    #   Unit [string], [int]
    Name: str = ''
    Line: int = 0
    # The loader position, the gripper command and the PCB location around it.
    #   Unit [string]
    Loader: str = ''
    Gripper: str = ''
    PCB: str = ''
    # MG06: 'Open', 'Closed' or 'Recorded' (then the lid and handle angles and their source).
    #   Unit [string], [float] (rad), [float] (rad), [string]
    Fixture: str = ''
    Fixture_Lid: tp.Optional[float] = None
    Fixture_Handle: tp.Optional[float] = None
    Fixture_Source: tp.Optional[str] = None
    # The contact groups allowed within the clearance there.
    #   Unit [List<string>]
    Contacts: tp.List[str] = field(default_factory=list)
    # The PCB location whose hold pose the point is, and the offset along the tool axes; None when it has none.
    #   Unit [string], [Vector<float> 1x3] (m)
    Hold_Location: tp.Optional[str] = None
    Hold_Offset: tp.Optional[tp.List[float]] = None


@dataclass
class Simulation_Str:
    """
    Description:
        Simulation.yaml: the start state, the tools' model sites, the outputs' actuators, the inputs' response times, the
        jaws' real stop, the end settle, the segments' contacts, the points' scenes. Built by Load_Simulation.
    """

    # The file.
    #   Unit [string]
    Path: str = ''
    # The start state: the robot point, the loader position, the gripper command, the vacuum, the PCB location, MG06.
    #   Unit [string], [bool]
    Start_Robot: str = ''
    Start_Loader: str = ''
    Start_Gripper: str = ''
    Start_Vacuum: bool = False
    Start_PCB: str = ''
    Start_MG06: str = ''
    # The model site of each tool; the actuator of each output; the response time of each input.
    #   Unit [Dict<string, string>], [Dict<string, string>], [Dict<string, float>] (s)
    Tool_Site: tp.Dict[str, str] = field(default_factory=dict)
    Actuators: tp.Dict[str, str] = field(default_factory=dict)
    Signal_Response: tp.Dict[str, float] = field(default_factory=dict)
    # Where the jaws really stop on the PCB per closing gripper command.
    #   Unit [Dict<string, float>] (m)
    Jaw_Actual: tp.Dict[str, float] = field(default_factory=dict)
    # The hold after the program's last segment.
    #   Unit [float] (s)
    End_Settle: float = 0.0
    # The contact groups per segment name (without S<n>_); the lines of the names.
    #   Unit [Dict<string, List<string>>], [Dict<string, int>]
    Allow_Contact: tp.Dict[str, tp.List[str]] = field(default_factory=dict)
    Allow_Contact_Lines: tp.Dict[str, int] = field(default_factory=dict)
    # The points' scenes.
    #   Unit [Dict<string, Point_Scene_Str>]
    Points: tp.Dict[str, Point_Scene_Str] = field(default_factory=dict)
    # The loader's notes.
    #   Unit [List<Tuple<string>>]
    Notes: tp.List[tp.Tuple[str, str]] = field(default_factory=list)


@dataclass
class Command_Str:
    """
    Description:
        One command line, its values resolved: the robot's (MoveJ, MoveL, MovePath, SetDO, WaitDI, Run, Wait) or a
        job's (MoveY, MoveZ, Gripper, Wait, Test). Fields a command does not have stay at their defaults.
    """

    # The command, the station that runs it (Robot, Loader, Tester), the constant or job it names.
    #   Unit [string]
    Kind: str = ''
    Station: str = ''
    Target: str = ''
    # The line's text, its file and line, and the segment or job it is in.
    #   Unit [string], [string], [int], [string]
    Text: str = ''
    File: str = ''
    Line: int = 0
    Owner: str = ''
    # A motion's joint target and its tool's model site.
    #   Unit [Vector<float> 1x6] (rad), [string]
    Joints: tp.List[float] = field(default_factory=list)
    Site: str = ''
    # MoveJ: the fraction of the datasheet joint speeds; MoveL: the peak tool speed.
    #   Unit [float] (-), (m/s)
    Speed: float = 0.0
    # A wait, a loader or gripper move, a test, a WaitDI's simulated response; a WaitDI's timeout.
    #   Unit [float] (s)
    Duration: float = 0.0
    Timeout: float = 0.0
    # SetDO: ON / OFF and its simulated actuator; MoveY / MoveZ: the axis and its position; Gripper: the jaw command
    # and where the jaws really stop on the PCB (None when the command has no Jaw_Actual).
    #   Unit [string], [string], [string], [float] (m), [float] (m), [float] (m)
    Value: str = ''
    Actuator: str = ''
    Axis: str = ''
    Position: float = 0.0
    Jaw: float = 0.0
    Jaw_Actual: tp.Optional[float] = None
    # MovePath: the recording, its phase, its start and end points and their joints.
    #   Unit [string], [string], [string], [string], [Vector<float> 1x6] (rad)
    Path_File: str = ''
    Phase: str = ''
    Path_Start: str = ''
    Path_End: str = ''
    End_Joints: tp.List[float] = field(default_factory=list)
    # Run: the job's commands.
    #   Unit [List<Command_Str>]
    Job: tp.List['Command_Str'] = field(default_factory=list)


@dataclass
class Segment_Str:
    """
    Description:
        One segment of the robot's program: S<n>_<Name>, its commands, its allowed contacts.
    """

    # The key (S<n>_<Name>), the name, the number, the line.
    #   Unit [string], [string], [int], [int]
    Key: str = ''
    Name: str = ''
    Number: int = 0
    Line: int = 0
    # The commands, in order.
    #   Unit [List<Command_Str>]
    Commands: tp.List[Command_Str] = field(default_factory=list)
    # The contact groups (Simulation.yaml Allow_Contact).
    #   Unit [List<string>]
    Allow_Contact: tp.List[str] = field(default_factory=list)


@dataclass
class Program_Str:
    """
    Description:
        Program.yaml with its constants and simulation data: the intermediate representation Program.Core builds a
        trajectory from. Built by Load_Program.
    """

    # The program's name (the file name without .yaml) and its file.
    #   Unit [string]
    Name: str = ''
    Path: str = ''
    # The segments, in order; the loader's and the tester's jobs.
    #   Unit [List<Segment_Str>], [Dict<string, List<Command_Str>>]
    Segments: tp.List[Segment_Str] = field(default_factory=list)
    Loader_Jobs: tp.Dict[str, tp.List[Command_Str]] = field(default_factory=dict)
    Tester_Jobs: tp.Dict[str, tp.List[Command_Str]] = field(default_factory=dict)
    # The constants and the simulation data it uses.
    #   Unit [Constants_Str(object)], [Simulation_Str(object)]
    Constants: Constants_Str = field(default_factory=Constants_Str)
    Simulation: Simulation_Str = field(default_factory=Simulation_Str)
    # The loader's notes.
    #   Unit [List<Tuple<string>>]
    Notes: tp.List[tp.Tuple[str, str]] = field(default_factory=list)


class _Problems_Cls(object):
    """
    Description:
        Collects a file's problems, each named by its line and place, and the unknown-key notes; fails once with all.

    Initialization of the Class:
        Args:
            (1) path [string]: The file.

        Example:
            Initialization:
                # Initialization of the class.
                Cls = _Problems_Cls(path)

            Features:
                # A problem at a line, then fail if there was any.
                Cls.Add(12, 'Points.P_HOME', 'Joints: 5 values, 6 required')
                Cls.Finish()
    """

    def __init__(self, path: str) -> None:
        # << PRIVATE >> #
        self.__path = os.path.relpath(path, CONST_ROOT) if os.path.isabs(path) else path
        self.Problems: tp.List[str] = []
        self.Notes: tp.List[tp.Tuple[str, str]] = []

    @property
    def File(self) -> str:
        """
        Description:
            The file, repository-relative.

        Returns:
            (1) parameter [string]: The file.
        """

        return self.__path

    def Add(self, line: int, where: str, message: str, text: tp.Optional[str] = None, file: tp.Optional[str] = None) -> None:
        """
        Description:
            Record a problem: '<file>:<line>: <where>: [<'line text'>: ]<message>'.

        Args:
            (1) line [int]: The line (0 when unknown).
            (2) where [string]: The key path, segment or job.
            (3) message [string]: What is wrong.
            (4) text [string]: The command line's text, or None.
            (5) file [string]: Another file the problem is in (a cross-file check), or None for this one.
        """

        file = self.__path if file is None else (os.path.relpath(file, CONST_ROOT) if os.path.isabs(file) else file)
        self.Problems.append('%s:%d: %s: %s%s' % (file, line, where, '' if text is None else "'%s': " % text, message))

    def Finish(self) -> None:
        """
        Description:
            Fail once with every problem, if there is one.
        """

        if self.Problems:
            raise Program_Format_Error_Cls('The program file %s is invalid (%d problem(s)); correct it:\n  %s'
                                           % (self.__path, len(self.Problems), '\n  '.join(self.Problems)))


def _Line(tree: tp.Any, key: str, fallback: int = 0) -> int:
    """
    Description:
        The line of a key in a mapping read by the project's loader (its keys carry their lines).

    Args:
        (1) tree [Dict<string, Any>]: The mapping.
        (2) key [string]: The key.
        (3) fallback [int]: The line when the key is not there.

    Returns:
        (1) parameter [int]: The line.
    """

    if isinstance(tree, dict):
        for k in tree:
            if k == key:
                return getattr(k, 'Line', fallback)
    return fallback


def _Mapping(p: _Problems_Cls, tree: tp.Any, key: str, where: str, line: int, known: tp.Optional[tp.Sequence[str]] = None) -> tp.Dict[str, tp.Any]:
    """
    Description:
        A required mapping at a key; unknown keys become WARNING notes.

    Args:
        (1) p [_Problems_Cls(object)]: The problems.
        (2) tree [Dict<string, Any>]: The mapping holding it.
        (3) key [string]: Its key.
        (4) where [string]: Its key path, for the messages.
        (5) line [int]: The line to name when it is missing.
        (6) known [List<string>]: The keys it may hold, or None for any.

    Returns:
        (1) parameter [Dict<string, Any>]: The mapping, or {} with a problem.
    """

    if not isinstance(tree, dict) or key not in tree:
        p.Add(line, where, 'missing (REQUIRED)')
        return {}
    value = tree[key]
    if not isinstance(value, dict):
        p.Add(_Line(tree, key, line), where, 'must be a mapping, not %r' % (value,))
        return {}
    for k in value:
        if known is not None and k not in known:
            p.Notes.append(('WARNING', '%s:%d: %s.%s: unknown key, ignored (misspelt?)' % (p.File, getattr(k, 'Line', 0), where, k)))
    return value


def _Field(p: _Problems_Cls, tree: tp.Dict[str, tp.Any], key: str, where: str, line: int) -> tp.Any:
    """
    Description:
        A required field's value, or None with a problem.

    Args:
        (1) p [_Problems_Cls(object)]: The problems.
        (2) tree [Dict<string, Any>]: The mapping.
        (3) key [string]: The field.
        (4) where [string]: The mapping's key path.
        (5) line [int]: The mapping's line.

    Returns:
        (1) parameter [Any]: The value, or None.
    """

    if key not in tree:
        p.Add(line, '%s.%s' % (where, key), 'missing (REQUIRED)')
        return None
    return tree[key]


def _Number(p: _Problems_Cls, value: tp.Any, where: str, line: int, low: float, high: float) -> float:
    """
    Description:
        A number within [low, high], or NaN with a problem.

    Args:
        (1) p [_Problems_Cls(object)]: The problems.
        (2) value [Any]: The value.
        (3) where [string]: Its key path.
        (4) line [int]: Its line.
        (5 - 6) low, high [float]: The range.

    Returns:
        (1) parameter [float]: The value.
    """

    if isinstance(value, bool) or not isinstance(value, (int, float)):
        p.Add(line, where, 'must be a number, not %r' % (value,))
        return math.nan
    if not low <= value <= high:
        p.Add(line, where, '%r is outside [%g, %g]' % (value, low, high))
    return float(value)


def _Vector(p: _Problems_Cls, value: tp.Any, where: str, line: int, n: int, low: float, high: float) -> tp.List[float]:
    """
    Description:
        A list of n numbers within [low, high], or NaNs with a problem.

    Args:
        (1) p [_Problems_Cls(object)]: The problems.
        (2) value [Any]: The value.
        (3) where [string]: Its key path.
        (4) line [int]: Its line.
        (5) n [int]: The length.
        (6 - 7) low, high [float]: The range of each value.

    Returns:
        (1) parameter [List<float>]: The values.
    """

    if not isinstance(value, list) or len(value) != n or any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value):
        p.Add(line, where, 'must be %d numbers, not %r' % (n, value))
        return [math.nan] * n
    if any(not low <= v <= high for v in value):
        p.Add(line, where, '%r has a value outside [%g, %g]' % (value, low, high))
    return [float(v) for v in value]


def _Text(p: _Problems_Cls, value: tp.Any, where: str, line: int) -> str:
    """
    Description:
        A non-empty string, or '' with a problem.

    Args:
        (1) p [_Problems_Cls(object)]: The problems.
        (2) value [Any]: The value.
        (3) where [string]: Its key path.
        (4) line [int]: Its line.

    Returns:
        (1) parameter [string]: The text.
    """

    if not isinstance(value, str) or not value.strip():
        p.Add(line, where, 'must be a non-empty text, not %r' % (value,))
        return ''
    return str(value)


def _Pose(p: _Problems_Cls, value: tp.Any, where: str, line: int) -> tp.List[float]:
    """
    Description:
        [x, y, z (m), R, P, Y (deg)] as metres and radians.

    Args:
        (1) p [_Problems_Cls(object)]: The problems.
        (2) value [Any]: The value.
        (3) where [string]: Its key path.
        (4) line [int]: Its line.

    Returns:
        (1) parameter [List<float>]: x, y, z (m), R, P, Y (rad).
    """

    v = _Vector(p, value, where, line, 6, -360.0, 360.0)
    return v[:3] + [math.radians(a) for a in v[3:]]


def Load_Constants(path: str = CONST_CONSTANTS_YAML) -> Constants_Str:
    """
    Description:
        Load Configuration/Constants.yaml, its only reader: the six sections, each name UPPERCASE with its section's prefix and
        declared once in the file, every field REQUIRED and validated (Reference / Offset null on a point that is not an
        offset point), the references (a point's tool and parent, a path's points, its file) checked, offset cycles found,
        degrees converted to radians; every problem at once with its line.

    Args:
        (1) path [string]: The file (default Configuration/Constants.yaml).

    Returns:
        (1) parameter [Constants_Str(object)]: The constants.
    """

    p = _Problems_Cls(path)
    root = Read_YAML(path)
    if not isinstance(root, dict):
        p.Add(1, 'the file', 'must be a mapping of the sections %s' % ', '.join(CONST_SECTIONS))
        p.Finish()
    for k in root:
        if k not in CONST_SECTIONS:
            p.Notes.append(('WARNING', '%s:%d: %s: unknown section, ignored (misspelt?)' % (p.File, getattr(k, 'Line', 0), k)))
    c = Constants_Str(Path=path)
    sections, seen = {}, {}
    for section, prefixes in CONST_SECTIONS.items():
        sections[section] = _Mapping(p, root, section, section, 1)
        for name in sections[section]:
            line = getattr(name, 'Line', 0)
            if not re.match(CONST_NAME_PATTERN, str(name)) or not str(name).startswith(prefixes):
                p.Add(line, '%s.%s' % (section, name), 'a name in %s is UPPERCASE and starts with %s' % (section, ' or '.join(prefixes)))
            if name in seen:
                p.Add(line, '%s.%s' % (section, name), 'already declared in %s (line %d); a name is declared once' % seen[name])
            seen[name] = (section, line)
    known = {'Tools': ('Description', 'Flange_Offset'), 'Points': ('Description', 'Tool', 'Reference', 'Offset', 'Joints', 'Pose'),
             'Loader_Positions': ('Description', 'Y', 'Z'), 'Gripper_Commands': ('Description', 'Jaw'),
             'Paths': ('Description', 'File', 'Phase', 'Start', 'End'), 'Signals': ('Description', 'Port')}
    entries = {}
    for section in CONST_SECTIONS:
        for name in sections[section]:
            line = getattr(name, 'Line', 0)
            where = '%s.%s' % (section, name)
            tree = _Mapping(p, sections[section], name, where, line, known[section])
            entries[(section, name)] = (tree, where, line)
            for key in known[section]:
                if key not in tree:
                    p.Add(line, '%s.%s' % (where, key), 'missing (REQUIRED)')
    for (section, name), (tree, where, line) in entries.items():
        name = str(name)
        description = _Text(p, tree.get('Description', 'x'), where + '.Description', _Line(tree, 'Description', line))
        if section == 'Tools':
            c.Tools[name] = Tool_Str(Name=name, Description=description, Line=line,
                                     Flange_Offset=_Pose(p, tree.get('Flange_Offset', [0.0] * 6), where + '.Flange_Offset',
                                                         _Line(tree, 'Flange_Offset', line)))
        elif section == 'Points':
            point = Point_Str(Name=name, Description=description, Line=line, Tool=str(tree.get('Tool', '')),
                              Joints=[math.radians(v) for v in _Vector(p, tree.get('Joints', [0.0] * 6), where + '.Joints',
                                                                           _Line(tree, 'Joints', line), 6, -360.0, 360.0)],
                              Pose=_Pose(p, tree.get('Pose', [0.0] * 6), where + '.Pose', _Line(tree, 'Pose', line)))
            if point.Tool not in sections['Tools']:
                p.Add(_Line(tree, 'Tool', line), where + '.Tool', '%r is not a tool (Tools: %s)' % (point.Tool, ', '.join(sections['Tools'])))
            parent, offset = tree.get('Reference'), tree.get('Offset')
            if (parent is None) != (offset is None):
                p.Add(_Line(tree, 'Reference', line), where, 'Reference and Offset are both null or both given')
            elif parent is not None:
                if parent not in sections['Points']:
                    p.Add(_Line(tree, 'Reference', line), where + '.Reference', '%r is not a point' % (parent,))
                point.Reference = str(parent)
                point.Offset = _Vector(p, offset, where + '.Offset', _Line(tree, 'Offset', line), 3, -1.0, 1.0)
            c.Points[name] = point
        elif section == 'Loader_Positions':
            c.Loader[name] = Loader_Position_Str(Name=name, Description=description, Line=line,
                                                 Y=_Number(p, tree.get('Y', 0.0), where + '.Y', _Line(tree, 'Y', line), -1.0, 1.0),
                                                 Z=_Number(p, tree.get('Z', 0.0), where + '.Z', _Line(tree, 'Z', line), -1.0, 1.0))
        elif section == 'Gripper_Commands':
            c.Gripper[name] = Gripper_Command_Str(Name=name, Description=description, Line=line,
                                                  Jaw=_Number(p, tree.get('Jaw', 0.0), where + '.Jaw', _Line(tree, 'Jaw', line), 0.0, 0.05))
        elif section == 'Paths':
            path_ = Path_Str(Name=name, Description=description, Line=line, File=os.path.join(CONST_ROOT, str(tree.get('File', ''))),
                             Phase=str(tree.get('Phase', '')), Start=str(tree.get('Start', '')), End=str(tree.get('End', '')))
            if 'File' in tree and not os.path.isfile(path_.File):
                p.Add(_Line(tree, 'File', line), where + '.File', '%r does not exist (a repository-relative file)' % (tree['File'],))
            if 'Phase' in tree and path_.Phase not in CONST_PHASES:
                p.Add(_Line(tree, 'Phase', line), where + '.Phase', '%r is not one of %s' % (path_.Phase, ', '.join(CONST_PHASES)))
            for key in ('Start', 'End'):
                if key in tree and tree[key] not in sections['Points']:
                    p.Add(_Line(tree, key, line), '%s.%s' % (where, key), '%r is not a point' % (tree[key],))
            c.Paths[name] = path_
        else:
            c.Signals[name] = Signal_Str(Name=name, Description=description, Line=line, Kind=name[:2],
                                         Port=_Text(p, tree.get('Port', 'x'), where + '.Port', _Line(tree, 'Port', line)))
    for name, point in c.Points.items():
        chain, parent = [name], point.Reference
        while parent is not None and parent in c.Points:
            if parent in chain:
                p.Add(point.Line, 'Points.%s.Reference' % name, 'an offset cycle: %s' % ' -> '.join(chain + [parent]))
                break
            chain.append(parent)
            parent = c.Points[parent].Reference
    p.Finish()
    c.Notes = p.Notes
    return c


def _Constant_Kind(constants: Constants_Str, name: str) -> tp.Optional[str]:
    """
    Description:
        The kind of a declared constant (P, PATH, DO, DI, L, G, T), or None when it is not declared.

    Args:
        (1) constants [Constants_Str(object)]: The constants.
        (2) name [string]: The name.

    Returns:
        (1) parameter [string]: The kind, or None.
    """

    for kind, table in (('P', constants.Points), ('PATH', constants.Paths), ('L', constants.Loader), ('G', constants.Gripper), ('T',
                                                                                                                                constants.Tools)):
        if name in table:
            return kind
    return constants.Signals[name].Kind if name in constants.Signals else None


def Load_Simulation(constants: Constants_Str, path: str = CONST_SIMULATION_YAML) -> Simulation_Str:
    """
    Description:
        Load Configuration/Simulation.yaml, its only reader: every field REQUIRED and validated against the constants (the
        start state, a model site for every tool, an actuator for every output, a response time for every input, a scene
        for every point and no other), degrees converted to radians; every problem at once with its line. The segment
        names of Allow_Contact are checked by Load_Program.

    Args:
        (1) constants [Constants_Str(object)]: The constants it names.
        (2) path [string]: The file (default Configuration/Simulation.yaml).

    Returns:
        (1) parameter [Simulation_Str(object)]: The simulation data.
    """

    p = _Problems_Cls(path)
    root = Read_YAML(path)
    keys = ('Start', 'Tool_Site', 'Actuators', 'Signal_Response', 'Jaw_Actual', 'End_Settle', 'Allow_Contact', 'Points')
    root = root if isinstance(root, dict) else {}
    for k in root:
        if k not in keys:
            p.Notes.append(('WARNING', '%s:%d: %s: unknown section, ignored (misspelt?)' % (p.File, getattr(k, 'Line', 0), k)))
    s = Simulation_Str(Path=path)
    start = _Mapping(p, root, 'Start', 'Start', 1, ('Robot', 'Loader', 'Gripper', 'Vacuum', 'PCB', 'MG06'))
    line = _Line(root, 'Start', 1)

    def Choice(tree: tp.Dict[str, tp.Any], key: str, where: str, choices: tp.Sequence[str], at: int) -> str:
        """
        Description:
            A required value out of choices.

        Args:
            (1) tree [Dict<string, Any>]: The mapping.
            (2) key [string]: The field.
            (3) where [string]: Its key path.
            (4) choices [List<string>]: The allowed values.
            (5) at [int]: The mapping's line.

        Returns:
            (1) parameter [string]: The value, or ''.
        """

        value = _Field(p, tree, key, where, at)
        if value is not None and value not in choices:
            p.Add(_Line(tree, key, at), '%s.%s' % (where, key), '%r is not one of %s' % (value, ', '.join(choices)))
            return ''
        return '' if value is None else str(value)

    s.Start_Robot = Choice(start, 'Robot', 'Start', tuple(constants.Points), line)
    s.Start_Loader = Choice(start, 'Loader', 'Start', tuple(constants.Loader), line)
    s.Start_Gripper = Choice(start, 'Gripper', 'Start', tuple(constants.Gripper), line)
    s.Start_Vacuum = Choice(start, 'Vacuum', 'Start', CONST_STATES, line) == 'ON'
    s.Start_PCB = Choice(start, 'PCB', 'Start', CONST_PCB_LOCATIONS, line)
    s.Start_MG06 = Choice(start, 'MG06', 'Start', CONST_FIXTURE_STATES, line)
    for section, table, what in (('Tool_Site', constants.Tools, 'tool'), ('Actuators', {n: v for n,
                                                                                        v in constants.Signals.items() if v.Kind == 'DO'}, 'output'),
                                 ('Signal_Response', {n: v for n, v in constants.Signals.items() if v.Kind == 'DI'}, 'input'),
                                 ('Jaw_Actual', constants.Gripper, 'gripper command')):
        tree = _Mapping(p, root, section, section, 1)
        for name, value in tree.items():
            at = getattr(name, 'Line', 0)
            if name not in table:
                p.Add(at, '%s.%s' % (section, name), 'is not a declared %s of Constants.yaml' % what)
            elif section == 'Tool_Site':
                s.Tool_Site[str(name)] = _Text(p, value, '%s.%s' % (section, name), at)
            elif section == 'Actuators':
                if value not in CONST_ACTUATORS:
                    p.Add(at, '%s.%s' % (section, name), '%r is not a simulated actuator (%s)' % (value, ', '.join(CONST_ACTUATORS)))
                s.Actuators[str(name)] = str(value)
            elif section == 'Signal_Response':
                s.Signal_Response[str(name)] = _Number(p, value, '%s.%s' % (section, name), at, 0.0, 60.0)
            else:
                s.Jaw_Actual[str(name)] = _Number(p, value, '%s.%s' % (section, name), at, 0.0, 0.05)
        if section == 'Tool_Site':
            for name in table:
                if name not in tree:
                    p.Add(_Line(root, section, 1), '%s.%s' % (section, name), 'missing: every tool needs its model site (REQUIRED)')
    settle = _Field(p, root, 'End_Settle', 'End_Settle', 1)
    s.End_Settle = _Number(p, settle, 'End_Settle', _Line(root, 'End_Settle', 1), 0.0, 60.0) if settle is not None else 0.0
    for name, groups in _Mapping(p, root, 'Allow_Contact', 'Allow_Contact', 1).items():
        at = getattr(name, 'Line', 0)
        if not isinstance(groups, list) or not groups or any(g not in CONST_CONTACT_GROUPS for g in groups):
            p.Add(at, 'Allow_Contact.%s' % name, '%r must be a list of %s (an unlisted segment is contact-free)' % (groups,
                                                                                                                    ', '.join(CONST_CONTACT_GROUPS)))
            continue
        s.Allow_Contact[str(name)] = [str(g) for g in groups]
        s.Allow_Contact_Lines[str(name)] = at
    points = _Mapping(p, root, 'Points', 'Points', 1)
    for name in points:
        at = getattr(name, 'Line', 0)
        where = 'Points.%s' % name
        if name not in constants.Points:
            p.Add(at, where, 'is not a point of Constants.yaml (unknown point)')
            continue
        tree = _Mapping(p, points, name, where, at, ('Context', 'Fixture', 'Contacts', 'PCB_Hold'))
        context = _Mapping(p, tree, 'Context', where + '.Context', at, ('Loader', 'Gripper', 'PCB'))
        scene = Point_Scene_Str(Name=str(name), Line=at, Loader=Choice(context, 'Loader', where + '.Context', tuple(constants.Loader), at),
                                Gripper=Choice(context, 'Gripper', where + '.Context', tuple(constants.Gripper), at),
                                PCB=Choice(context, 'PCB', where + '.Context', CONST_PCB_LOCATIONS, at))
        fixture = _Field(p, tree, 'Fixture', where, at)
        if isinstance(fixture, dict):
            state = _Mapping(p, tree, 'Fixture', where + '.Fixture', at, ('Lid', 'Handle', 'Source'))
            scene.Fixture = 'Recorded'
            # A recorded lid may stand slightly past its closed stop (0 deg): the contact solver presses it in.
            scene.Fixture_Lid = math.radians(_Number(p, _Field(p, state, 'Lid', where + '.Fixture', at), where + '.Fixture.Lid', at, -40.0, 1.0))
            scene.Fixture_Handle = math.radians(_Number(p, _Field(p, state, 'Handle', where + '.Fixture', at), where + '.Fixture.Handle',
                                                        at, -1.0, 60.0))
            scene.Fixture_Source = _Text(p, state.get('Source', ''), where + '.Fixture.Source', at)
        elif fixture is not None:
            if fixture not in CONST_FIXTURE_STATES:
                p.Add(_Line(tree, 'Fixture', at), where + '.Fixture', '%r is not one of Open, Closed or {Lid, Handle, Source}' % (fixture,))
            scene.Fixture = str(fixture)
        contacts = _Field(p, tree, 'Contacts', where, at)
        if contacts is not None and (not isinstance(contacts, list) or any(g not in CONST_CONTACT_GROUPS for g in contacts)):
            p.Add(_Line(tree, 'Contacts', at), where + '.Contacts', '%r must be a list of %s ([] for none)' % (contacts,
                                                                                                               ', '.join(CONST_CONTACT_GROUPS)))
        scene.Contacts = [str(g) for g in contacts] if isinstance(contacts, list) else []
        hold = _Field(p, tree, 'PCB_Hold', where, at)
        if isinstance(hold, dict):
            location = _Field(p, hold, 'Location', where + '.PCB_Hold', at)
            if location not in ('input_crate', 'jaws', 'output_crate'):
                p.Add(_Line(tree, 'PCB_Hold', at), where + '.PCB_Hold.Location', '%r is not one of input_crate, jaws, output_crate' % (location,))
            scene.Hold_Location = str(location)
            scene.Hold_Offset = _Vector(p, _Field(p, hold, 'Offset', where + '.PCB_Hold', at), where + '.PCB_Hold.Offset', at, 3, -1.0, 1.0)
        elif hold is not None and 'PCB_Hold' in tree:
            p.Add(_Line(tree, 'PCB_Hold', at), where + '.PCB_Hold', 'must be null or {Location, Offset}')
        s.Points[str(name)] = scene
    for name in constants.Points:
        if name not in points:
            p.Add(_Line(root, 'Points', 1), 'Points.%s' % name, 'missing: every point of Constants.yaml needs its scene (REQUIRED)')
    p.Finish()
    s.Notes = p.Notes
    return s


def Parse_Line(text: str) -> tp.Tuple[str, tp.List[tp.Tuple[str, tp.Any]], tp.Dict[str, tp.Any]]:
    """
    Description:
        One command line parsed with Python's ast, never evaluated: a call whose function is a name or a name's attribute;
        positional arguments that are names (constants, ON / OFF), a name's attribute (Process.Test_Duration) or literals;
        keyword arguments whose values are literals (ast.literal_eval).

    Args:
        (1) text [string]: The line.

    Returns:
        (1) parameter [string]: The call's name, e.g. 'Robot.MoveL'.
        (2) parameter [List<Tuple<string, Any>>]: The positional arguments: ('name', 'P_HOME'), ('value', 0.3).
        (3) parameter [Dict<string, Any>]: The keyword arguments' values.
    """

    try:
        tree = ast.parse(str(text).strip(), mode='eval')
    except SyntaxError as error:
        raise ValueError('not a command call (%s)' % error.msg)
    call = tree.body
    if not isinstance(call, ast.Call):
        raise ValueError('not a command call: write Command(arguments)')
    if isinstance(call.func, ast.Name):
        name = call.func.id
    elif isinstance(call.func, ast.Attribute) and isinstance(call.func.value, ast.Name):
        name = '%s.%s' % (call.func.value.id, call.func.attr)
    else:
        raise ValueError('the command must be a name or Station.Command')
    args = []
    for arg in call.args:
        if isinstance(arg, ast.Name):
            args.append(('name', arg.id))
        elif isinstance(arg, ast.Attribute) and isinstance(arg.value, ast.Name):
            args.append(('name', '%s.%s' % (arg.value.id, arg.attr)))
        else:
            try:
                args.append(('value', ast.literal_eval(arg)))
            except ValueError:
                raise ValueError('an argument must be a constant name or a literal, not %s' % ast.unparse(arg))
    kwargs = {}
    for keyword in call.keywords:
        if keyword.arg is None:
            raise ValueError('** arguments are not allowed')
        try:
            kwargs[keyword.arg] = ast.literal_eval(keyword.value)
        except ValueError:
            raise ValueError('%s= must be a literal, not %s' % (keyword.arg, ast.unparse(keyword.value)))
    return name, args, kwargs


class _Program_Parser_Cls(object):
    """
    Description:
        Parses Program.yaml's lines into commands against the constants and the simulation data, collecting every problem
        and the constants each line uses.

    Initialization of the Class:
        Args:
            (1) constants [Constants_Str(object)]: The constants.
            (2) simulation [Simulation_Str(object)]: The simulation data.
            (3) cell [Cell_Configuration_Str(object)]: The cell (the default MoveJ speed, the test duration).
            (4) problems [_Problems_Cls(object)]: The problems.

        Example:
            Initialization:
                # Initialization of the class.
                Cls = _Program_Parser_Cls(constants, simulation, cell, problems)

            Features:
                # One line of a segment.
                command = Cls.Command(text, 'S2_Pick_Input', CONST_ROBOT_COMMANDS, '')
    """

    def __init__(self, constants: Constants_Str, simulation: Simulation_Str, cell: Cell_Configuration_Str, problems: _Problems_Cls) -> None:
        # << PRIVATE >> #
        self.__c, self.__s, self.__cell, self.__p = constants, simulation, cell, problems
        self.Used: tp.Set[str] = set()
        self.Loader_Jobs: tp.Dict[str, tp.List[Command_Str]] = {}
        self.Tester_Jobs: tp.Dict[str, tp.List[Command_Str]] = {}

    def Command(self, text: tp.Any, owner: str, grammar: tp.Dict[str, tp.Tuple[tp.List[str], tp.Dict[str, bool]]],
                station: str) -> tp.Optional[Command_Str]:
        """
        Description:
            One line: parsed, its arguments checked against the grammar (their kinds, the required and reserved keywords,
            the literal ranges) and resolved into a command.

        Args:
            (1) text [string]: The line (a Located_Text_Cls: its line is known).
            (2) owner [string]: The segment or job.
            (3) grammar [Dict]: The commands allowed here.
            (4) station [string]: The station a job line runs on ('' in the robot's program).

        Returns:
            (1) parameter [Command_Str(object)]: The command, or None with a problem.
        """

        p, c, s = self.__p, self.__c, self.__s
        line = getattr(text, 'Line', 0)

        def Fail(message: str) -> None:
            """
            Description:
                A problem of this line.

            Args:
                (1) message [string]: What is wrong.
            """

            p.Add(line, owner, message, str(text))

        if not isinstance(text, str):
            p.Add(line, owner, 'a command is one line of text, not %r' % (text,))
            return None
        try:
            name, args, kwargs = Parse_Line(text)
        except ValueError as error:
            Fail(str(error))
            return None
        if name in CONST_RESERVED:
            Fail('%s is reserved for F9 (the robot and the stations in parallel); use %s.Run (Run = Start + WaitDone)' % (name, name.split('.')[0]))
            return None
        if name not in grammar:
            Fail("'%s' is not a command here; the commands are %s" % (name, ', '.join(grammar)))
            return None
        kinds, keywords = grammar[name]
        if len(args) != len(kinds):
            Fail('%s takes %d positional argument(s), not %d' % (name, len(kinds), len(args)))
            return None
        for key in kwargs:
            if key not in keywords:
                Fail("%s has no argument %s= (it takes %s)" % (name, key, ', '.join('%s=' % k for k in keywords) or 'none'))
                return None
        for key, required in keywords.items():
            if required and key not in kwargs:
                Fail('%s needs %s= (REQUIRED)' % (name, key))
                return None
        for key in ('a', 'r'):
            if key in kwargs and kwargs[key] != 0:
                Fail('%s= is reserved (no blending yet): only 0 or omitted, not %r' % (key, kwargs[key]))
                return None
        command = Command_Str(Kind=name.split('.')[-1], Station=name.split('.')[0] if '.' in name else station, Text=str(text).strip(),
                              File=p.File, Line=line, Owner=owner)
        values = []
        for (form, value), kind in zip(args, kinds):
            if kind == 'NUMBER':
                if form != 'value' or isinstance(value, bool) or not isinstance(value, (int, float)) or not CONST_TIME[0] <= value <= CONST_TIME[1]:
                    Fail('%s takes a time in seconds within [%g, %g], not %r' % (name, CONST_TIME[0], CONST_TIME[1], value))
                    return None
                values.append(float(value))
            elif kind == 'STATE':
                if form != 'name' or value not in CONST_STATES:
                    Fail('%s takes ON or OFF, not %r' % (name, value))
                    return None
                values.append(value)
            elif kind == 'PROCESS':
                references = tuple('Process.' + k for k in CONST_PROCESS_KEYS)
                if form != 'name' or value not in references:
                    Fail('%s takes %s (a Cell.yaml duration), not %r' % (name, ' or '.join(references), value))
                    return None
                values.append(value)
            elif kind in ('JL', 'JT'):
                jobs = self.Loader_Jobs if kind == 'JL' else self.Tester_Jobs
                if form != 'name' or value not in jobs:
                    Fail('%s takes a job of %s (%s), not %r' % (name, 'Loader_Jobs' if kind == 'JL' else 'Tester_Jobs', ', '.join(jobs), value))
                    return None
                self.Used.add(value)
                values.append(value)
            else:
                found = _Constant_Kind(c, value) if form == 'name' else None
                if found is None:
                    Fail('%r is not a declared constant (Constants.yaml); %s takes %s' % (value, name, CONST_KIND_WORDS[kind])
                         if form == 'name' else '%s takes %s, not the literal %r' % (name, CONST_KIND_WORDS[kind], value))
                    return None
                if found != kind:
                    Fail('wrong constant type: %s takes %s, %s is %s' % (name, CONST_KIND_WORDS[kind], value, CONST_KIND_WORDS[found]))
                    return None
                self.Used.add(value)
                values.append(value)
        if command.Kind in ('MoveJ', 'MoveL'):
            point = c.Points[values[0]]
            command.Target, command.Joints, command.Site = point.Name, list(point.Joints), s.Tool_Site.get(point.Tool, '')
            if command.Kind == 'MoveJ':
                v = kwargs.get('v', self.__cell.Speed_Fraction)
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not CONST_MOVEJ_SPEED[0] < v <= CONST_MOVEJ_SPEED[1]:
                    Fail('MoveJ v= is a fraction of the datasheet joint speeds in (0, 1], not %r' % (v,))
                    return None
            else:
                v = kwargs['v']
                if isinstance(v, bool) or not isinstance(v, (int, float)) or not CONST_MOVEL_SPEED[0] <= v <= CONST_MOVEL_SPEED[1]:
                    Fail('MoveL v= is the tool speed in m/s within [%g, %g], not %r' % (CONST_MOVEL_SPEED[0], CONST_MOVEL_SPEED[1], v))
                    return None
            command.Speed = float(v)
        elif command.Kind == 'MovePath':
            path = c.Paths[values[0]]
            command.Target, command.Path_File, command.Phase = path.Name, path.File, path.Phase
            command.Path_Start, command.Path_End = path.Start, path.End
            command.End_Joints = list(c.Points[path.End].Joints) if path.End in c.Points else []
            self.Used.update((path.Start, path.End))
        elif command.Kind == 'SetDO':
            command.Target, command.Value = values
            if command.Target not in s.Actuators:
                Fail('%s has no simulated actuator (Simulation.yaml Actuators)' % command.Target)
                return None
            command.Actuator = s.Actuators[command.Target]
        elif command.Kind == 'WaitDI':
            command.Target, timeout = values[0], kwargs['timeout']
            if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or not CONST_TIME[0] < timeout <= CONST_TIME[1]:
                Fail('WaitDI timeout= is a time in seconds within (0, %g], not %r' % (CONST_TIME[1], timeout))
                return None
            if command.Target not in s.Signal_Response:
                Fail('%s has no simulated response time (Simulation.yaml Signal_Response)' % command.Target)
                return None
            command.Timeout, command.Duration = float(timeout), s.Signal_Response[command.Target]
            if command.Duration > command.Timeout:
                Fail('the simulated response of %s (%g s) exceeds the timeout %g s: the wait would fail' % (command.Target,
                                                                                                            command.Duration, command.Timeout))
                return None
        elif command.Kind == 'Run':
            command.Target = values[0]
            command.Job = (self.Loader_Jobs if command.Station == 'Loader' else self.Tester_Jobs)[command.Target]
        elif command.Kind == 'Wait':
            command.Duration = values[0]
        elif command.Kind in ('MoveY', 'MoveZ'):
            position = c.Loader[values[0]]
            command.Target, command.Axis = position.Name, command.Kind[-1]
            command.Position = position.Y if command.Axis == 'Y' else position.Z
        elif command.Kind == 'Gripper':
            command.Target, command.Jaw = values[0], c.Gripper[values[0]].Jaw
            command.Jaw_Actual = s.Jaw_Actual.get(values[0])
        elif command.Kind == 'Test':
            command.Target, command.Duration = values[0], self.__cell.Test_Duration
        if 't' in kwargs:
            t = kwargs['t']
            if isinstance(t, bool) or not isinstance(t, (int, float)) or not 0.05 <= t <= CONST_TIME[1]:
                Fail('t= is a time in seconds within [0.05, %g], not %r' % (CONST_TIME[1], t))
                return None
            command.Duration = float(t)
        return command



def _Family(point: Point_Str) -> int:
    """
    Description:
        A point's IK family: the sign of its wrist_2 (+1 wrist up, -1 wrist down; SPEC §4.7).

    Args:
        (1) point [Point_Str(object)]: The point.

    Returns:
        (1) parameter [int]: +1 or -1.
    """

    return 1 if point.Joints[CONST_WRIST_2] > 0 else -1


def Load_Program(constants: Constants_Str, simulation: Simulation_Str, cell: Cell_Configuration_Str, path: str = CONST_PROGRAM_YAML) -> Program_Str:
    """
    Description:
        Load Configuration/Program.yaml, its only reader, into the intermediate representation: the loader's and the tester's
        jobs, the segments S1..Sn in order, every line parsed (Parse_Line) and resolved against the constants and the
        simulation data. Validated: the grammar (the commands, their constant types, the required v= / t= / timeout=, the
        reserved a= / r= and Start / WaitDone, the literal ranges); the segment numbering and names; Simulation.yaml's
        segment names; an actuator for every output and a response time within the timeout for every input; every constant
        and job declared and used (across Program.yaml and Simulation.yaml); the IK family changing only across
        CONST_FAMILY_SWITCH_POINT and never in a MoveL; a MovePath starting where the robot stands (its Start) and the robot
        standing at its End after it. Every problem at once with its file, line, segment or job and the line's text.

    Args:
        (1) constants [Constants_Str(object)]: The constants.
        (2) simulation [Simulation_Str(object)]: The simulation data.
        (3) cell [Cell_Configuration_Str(object)]: The cell (the default MoveJ speed, the test duration).
        (4) path [string]: The file (default Configuration/Program.yaml).

    Returns:
        (1) parameter [Program_Str(object)]: The program.
    """

    p = _Problems_Cls(path)
    root = Read_YAML(path)
    root = root if isinstance(root, dict) else {}
    for k in root:
        if k not in ('Program', 'Loader_Jobs', 'Tester_Jobs'):
            p.Notes.append(('WARNING', '%s:%d: %s: unknown section, ignored (misspelt?)' % (p.File, getattr(k, 'Line', 0), k)))
    parser = _Program_Parser_Cls(constants, simulation, cell, p)
    program = Program_Str(Name=os.path.splitext(os.path.basename(path))[0], Path=path, Constants=constants, Simulation=simulation)
    for section, jobs, grammar, station in (('Loader_Jobs', parser.Loader_Jobs, CONST_LOADER_COMMANDS, 'Loader'),
                                            ('Tester_Jobs', parser.Tester_Jobs, CONST_TESTER_COMMANDS, 'Tester')):
        for name, lines in _Mapping(p, root, section, section, 1).items():
            at = getattr(name, 'Line', 0)
            if not re.match(r'^J_[A-Z0-9_]+$', str(name)):
                p.Add(at, '%s.%s' % (section, name), 'a job name is J_ and UPPERCASE')
            if not isinstance(lines, list) or not lines:
                p.Add(at, '%s.%s' % (section, name), 'must be a list of command lines')
                continue
            jobs[str(name)] = [command for command in (parser.Command(text, str(name), grammar, station) for text in lines) if command is not None]
    names = {}
    for number, (key, lines) in enumerate(_Mapping(p, root, 'Program', 'Program', 1).items(), start=1):
        at = getattr(key, 'Line', 0)
        match = re.match(r'^S([0-9]+)_([A-Za-z][A-Za-z0-9_]*)$', str(key))
        if match is None:
            p.Add(at, 'Program.%s' % key, 'a segment is S<n>_<Name>')
            continue
        if int(match.group(1)) != number:
            p.Add(at, 'Program.%s' % key, 'the segments are numbered S1..Sn in order: this is segment %d, so S%d_%s' % (number, number,
                                                                                                                        match.group(2)))
        name = match.group(2)
        if name in names:
            p.Add(at, 'Program.%s' % key, "the name %s is already used by %s; segment names are unique" % (name, names[name]))
        names[name] = str(key)
        if not isinstance(lines, list) or not lines:
            p.Add(at, 'Program.%s' % key, 'must be a list of command lines')
            continue
        commands = [command for command in (parser.Command(text, str(key), CONST_ROBOT_COMMANDS, '') for text in lines) if command is not None]
        program.Segments.append(Segment_Str(Key=str(key), Name=name, Number=number, Line=at, Commands=commands,
                                            Allow_Contact=list(simulation.Allow_Contact.get(name, []))))
    program.Loader_Jobs, program.Tester_Jobs = parser.Loader_Jobs, parser.Tester_Jobs
    for name, at in simulation.Allow_Contact_Lines.items():
        if name not in names:
            p.Add(at, 'Allow_Contact.%s' % name, 'is not a segment of Program.yaml (unknown segment; the name is without S<n>_)',
                  file=simulation.Path)
    for name, signal in constants.Signals.items():
        if signal.Kind == 'DO' and name not in simulation.Actuators:
            p.Add(signal.Line, 'Signals.%s' % name, 'an output without a simulated actuator (Simulation.yaml Actuators)', file=constants.Path)
        if signal.Kind == 'DI' and name not in simulation.Signal_Response:
            p.Add(signal.Line, 'Signals.%s' % name, 'an input without a simulated response time (Simulation.yaml Signal_Response)',
                  file=constants.Path)
    # Every constant used: by a line, by the start state; a used point's tool and Reference chain with it.
    used = set(parser.Used) | {simulation.Start_Robot, simulation.Start_Loader, simulation.Start_Gripper}
    for name in list(used):
        while name in constants.Points:
            used.add(constants.Points[name].Tool)
            name = constants.Points[name].Reference
            used.add(name)
    for section, table in (('Tools', constants.Tools), ('Points', constants.Points), ('Loader_Positions', constants.Loader),
                           ('Gripper_Commands', constants.Gripper), ('Paths', constants.Paths), ('Signals', constants.Signals)):
        for name, entry in table.items():
            if name not in used:
                p.Add(entry.Line, '%s.%s' % (section, name), 'declared but used by neither Program.yaml nor Simulation.yaml; remove it or use it',
                      file=constants.Path)
    for section, jobs in (('Loader_Jobs', program.Loader_Jobs), ('Tester_Jobs', program.Tester_Jobs)):
        for name in jobs:
            if name not in parser.Used:
                p.Add(_Line(root.get(section, {}), name), '%s.%s' % (section, name), 'a job no segment runs; remove it or run it')
    # The robot's flow: where it stands before each motion (by name).
    here = simulation.Start_Robot
    for segment in program.Segments:
        for command in segment.Commands:
            if command.Kind in ('MoveJ', 'MoveL') and here in constants.Points:
                target = constants.Points[command.Target]
                if _Family(constants.Points[here]) != _Family(target):
                    if command.Kind == 'MoveL':
                        p.Add(command.Line, segment.Key, 'a MoveL cannot change the IK family (%s to %s); use MoveJ' % (here, target.Name),
                              command.Text)
                    elif CONST_FAMILY_SWITCH_POINT not in (here, target.Name):
                        p.Add(command.Line, segment.Key, 'the IK family changes (%s to %s) away from %s, the only switch point'
                              % (here, target.Name, CONST_FAMILY_SWITCH_POINT), command.Text)
                here = target.Name
            elif command.Kind == 'MovePath':
                if here != command.Path_Start:
                    p.Add(command.Line, segment.Key, '%s starts at %s, but the robot stands at %s' % (command.Target, command.Path_Start,
                                                                                                      here), command.Text)
                here = command.Path_End
    p.Finish()
    program.Notes = constants.Notes + simulation.Notes + p.Notes
    return program


def Parse_Commands(lines: tp.Sequence[str], owner: str, constants: Constants_Str, simulation: Simulation_Str, cell: Cell_Configuration_Str,
                   loader_jobs: tp.Optional[tp.Dict[str, tp.Sequence[str]]] = None,
                   tester_jobs: tp.Optional[tp.Dict[str, tp.Sequence[str]]] = None) -> tp.List[Command_Str]:
    """
    Description:
        Command lines in the robot's program grammar, parsed and resolved like Load_Program's (with the given jobs),
        without a whole file's rules (the segment numbering, every constant used, the flow): a building block for a program
        assembled in code, e.g. one segment.

    Args:
        (1) lines [List<string>]: The robot's lines.
        (2) owner [string]: The segment's name, for the messages.
        (3) constants [Constants_Str(object)]: The constants.
        (4) simulation [Simulation_Str(object)]: The simulation data.
        (5) cell [Cell_Configuration_Str(object)]: The cell.
        (6 - 7) loader_jobs, tester_jobs [Dict<string, List<string>>]: The jobs the lines may run, or None.

    Returns:
        (1) parameter [List<Command_Str>]: The commands.
    """

    p = _Problems_Cls('<lines>')
    parser = _Program_Parser_Cls(constants, simulation, cell, p)
    for name, job in (loader_jobs or {}).items():
        parser.Loader_Jobs[name] = [command for command in (parser.Command(text, name, CONST_LOADER_COMMANDS, 'Loader') for text in job) if command]
    for name, job in (tester_jobs or {}).items():
        parser.Tester_Jobs[name] = [command for command in (parser.Command(text, name, CONST_TESTER_COMMANDS, 'Tester') for text in job) if command]
    commands = [command for command in (parser.Command(text, owner, CONST_ROBOT_COMMANDS, '') for text in lines) if command is not None]
    p.Finish()
    return commands
