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
# File Name: ../Configuration/Core.py
## =========================================================================== ##

# System (Default)
import os
# JSON (JSON encoder and decoder)
import json
# Math (Mathematical functions)
import math
# Re (Regular expressions)
import re
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# PyYAML (YAML parser and emitter) [pip3 install pyyaml]
import yaml
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco

"""
Description:
    Initialization of constants.

    The repository's folders and files, built from this module's location (src/Configuration/Core.py),
    never from the working directory (SPEC §7.1); the project's YAML loader (Read_YAML) and the only reader of
    Configuration/Cell.yaml (the program's three files: Configuration/Program_Parser.py) and of Data/PCB/pcb_meta.json; the route of
    MuJoCo's log and of a script's lines into logger/. It prints nothing.
"""
# The repository root: two folders above src/Configuration/.
#   Unit [string]
CONST_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
# The hand-written configuration (SPEC §8).
#   Unit [string]
CONST_CONFIGURATION = os.path.join(CONST_ROOT, 'Configuration')
CONST_CELL_YAML = os.path.join(CONST_CONFIGURATION, 'Cell.yaml')
# The MJCF files and their meshes (style project page mjcf.md).
#   Unit [string]
CONST_MODELS = os.path.join(CONST_ROOT, 'models')
# Input data by purpose: PCB/ (the PCB poses), Paths/ (recorded robot paths), Reference/ (the regression plan).
#   Unit [string]
CONST_DATA = os.path.join(CONST_ROOT, 'Data')
# Generated artefacts, one subfolder per kind (git-ignored; design-decisions: logs in logger/, generated
# artefacts in Data/Generated/): Reports/ (the cycle and program reports), Images/ (proof images), Trajectories/ (the
# program trajectories, step 4).
#   Unit [string]
CONST_GENERATED = os.path.join(CONST_ROOT, 'Data', 'Generated')
CONST_REPORTS = os.path.join(CONST_GENERATED, 'Reports')
CONST_IMAGES = os.path.join(CONST_GENERATED, 'Images')
CONST_TRAJECTORIES = os.path.join(CONST_GENERATED, 'Trajectories')
# Run logs: each script's printed lines and MuJoCo's own log (git-ignored, created on first use).
#   Unit [string]
CONST_LOGGER = os.path.join(CONST_ROOT, 'logger')
# The finished scene (workstation, MG06, loader, PCB), built from Blender; never regenerated.
#   Unit [string]
CONST_SCENE_XML = os.path.join(CONST_MODELS, 'scene.xml')
# The whole cell in one MJCF for MuJoCo's viewer, written by Scene.Core.Export_Cell_Scene.
#   Unit [string]
CONST_CELL_SCENE_XML = os.path.join(CONST_MODELS, 'cell_scene.xml')
# The vendored AUBO i5 (ian-chuang/Manipulator-Mujoco, MIT) and the VGC10 + fork tool.
#   Unit [string]
CONST_ROBOT_XML = os.path.join(CONST_MODELS, 'aubo_i5', 'aubo_i5.xml')
CONST_TOOL_XML = os.path.join(CONST_MODELS, 'robot_tool', 'vgc10_opener_free.xml')
# The PCB poses and the loader axis positions (SPEC §4.4), and the recorded flange path that closes
# and opens MG06 (SPEC §4.9).
#   Unit [string]
CONST_PCB_META = os.path.join(CONST_DATA, 'PCB', 'pcb_meta.json')
CONST_FORK_PATH = os.path.join(CONST_DATA, 'Paths', 'fork_close_open_path.npz')
# The verified IK plan the cycle run is proven on (SPEC §4.6), the regression input until step 6 replaces it
# with the keypoint program; its planner is in git tag legacy-planner-reference (design-decisions).
#   Unit [string]
CONST_REFERENCE_PLAN = os.path.join(CONST_DATA, 'Reference', 'cell_plan.npz')
# The file name of the cycle report (written into Data/Generated/Reports/ or a script's --out folder).
#   Unit [string]
CONST_REPORT_NAME = 'report.txt'
# MuJoCo's log file; MuJoCo writes MUJOCO_LOG.TXT into the working directory unless told otherwise.
#   Unit [string]
CONST_MUJOCO_LOG = os.path.join(CONST_LOGGER, 'MUJOCO_LOG.TXT')
# The keys of Data/PCB/pcb_meta.json, each a position [m] (3 values) or a quaternion (4 values, w first); every
# key is required. (The loader positions are constants of Configuration/Constants.yaml.)
#   Unit [Dict<string, int>] (key -> number of values)
CONST_PCB_META_KEYS = {'cad_pos': 3, 'place_pos': 3, 'hold_pos': 3, 'quat': 4}
# The closed sets the configuration files share: the PCB's locations, the contact groups, MG06's states.
#   Unit [Tuple<string>]
CONST_PCB_LOCATIONS = ('input_crate', 'cups', 'jaws', 'mg06', 'output_crate')
CONST_CONTACT_GROUPS = ('cups_pcb', 'handle_bar', 'pcb_crate', 'pcb_jaws')
CONST_FIXTURE_STATES = ('Open', 'Closed')
# The durations of Cell.yaml's Process section a program's Test command may name (Test: Process.<key>).
#   Unit [Tuple<string>]
CONST_PROCESS_KEYS = ('Test_Duration',)


class Configuration_Error_Cls(ValueError):
    """
    Description:
        A project data file is missing or malformed. The message names the file and every problem
        found; a script prints it verbatim as an [ERROR] line.
    """


@dataclass
class PCB_Meta_Str:
    """
    Description:
        The PCB poses of Data/PCB/pcb_meta.json (SPEC §4.4), in the MG06 fixture frame. Built by Read_PCB_Meta.
    """

    # The PCB centre as placed in the CAD.
    #   Unit [Vector<float> 1x3] (m)
    Cad_Pos: tp.List[float] = field(default_factory=list)
    # The PCB centre placed in the MG06 holder (the place target).
    #   Unit [Vector<float> 1x3] (m)
    Place_Pos: tp.List[float] = field(default_factory=list)
    # The PCB centre held in the loader jaws at loader home.
    #   Unit [Vector<float> 1x3] (m)
    Hold_Pos: tp.List[float] = field(default_factory=list)
    # The PCB orientation in the holder and in the jaws (8.94 deg about X), w first.
    #   Unit [Vector<float> 1x4] (-)
    Quat: tp.List[float] = field(default_factory=list)


def Read_PCB_Meta(path: str = CONST_PCB_META) -> PCB_Meta_Str:
    """
    Description:
        Read Data/PCB/pcb_meta.json, the only reader of that file. Every problem is collected first and
        reported once: a missing file, invalid JSON, a missing key, a value of the wrong length or type.

    Args:
        (1) path [string]: The file (default: Data/PCB/pcb_meta.json).

    Returns:
        (1) parameter [PCB_Meta_Str(object)]: The poses and positions, in m.
    """

    try:
        with open(path, encoding='utf-8') as handle:
            raw = json.load(handle)
    except FileNotFoundError as error:
        raise Configuration_Error_Cls('The PCB data file %s does not exist. Restore it from git: git checkout -- %s'
                                      % (path, os.path.relpath(path, CONST_ROOT))) from error
    except json.JSONDecodeError as error:
        raise Configuration_Error_Cls('The PCB data file %s is not valid JSON (%s). Restore it from git.' % (path, error)) from error
    problems, values = [], {}
    for key, n in CONST_PCB_META_KEYS.items():
        if key not in raw:
            problems.append('%s is missing' % key)
            continue
        value = raw[key]
        if isinstance(value, list) and len(value) == n and all(isinstance(v, (int, float)) for v in value):
            values[key] = [float(v) for v in value]
        else:
            problems.append('%s must be a list of %d numbers, not %r' % (key, n, value))
    if problems:
        raise Configuration_Error_Cls('The PCB data file %s is invalid: %s. Correct it or restore it from git.'
                                      % (path, '; '.join(problems)))
    return PCB_Meta_Str(Cad_Pos=values['cad_pos'], Place_Pos=values['place_pos'], Hold_Pos=values['hold_pos'], Quat=values['quat'])


def Configure_MuJoCo_Log(path: str = CONST_MUJOCO_LOG) -> str:
    """
    Description:
        Point MuJoCo's log file at path (default logger/MUJOCO_LOG.TXT), creating its folder, so that
        no MUJOCO_LOG.TXT is written into the working directory. MuJoCo keeps printing its warnings on
        the console. The setting is process-wide; scripts and the test suite call it once at start.

    Args:
        (1) path [string]: The log file.

    Returns:
        (1) parameter [string]: The log file.
    """

    os.makedirs(os.path.dirname(path), exist_ok=True)
    log_config = mujoco.MjLogConfig.get()
    log_config.logfile = path
    mujoco.MjLogConfig.set(log_config)
    return path


def Script_Log_Path(script_name: str) -> str:
    """
    Description:
        The log file of a script, logger/<script name>.log, creating logger/ if needed.

    Args:
        (1) script_name [string]: The script's file name without .py.

    Returns:
        (1) parameter [string]: The log file path.
    """

    os.makedirs(CONST_LOGGER, exist_ok=True)
    return os.path.join(CONST_LOGGER, script_name + '.log')


class Script_Log_Cls(object):
    """
    Description:
        The log of one script run, logger/<script name>.log, rewritten on each run. Line() writes a line
        and returns it unchanged, so the script prints it: print(log.Line('[INFO] ...')). It prints
        nothing by itself.

    Initialization of the Class:
        Args:
            (1) script_name [string]: The script's file name without .py.

        Example:
            Initialization:
                # Initialization of the class.
                Cls = Script_Log_Cls('run_cycle')

            Features:
                # The line goes to logger/run_cycle.log and to the screen.
                print(Cls.Line('[INFO] Done.'))
                Cls.Close()
    """

    def __init__(self, script_name: str) -> None:
        # << PRIVATE >> #
        self.__path = Script_Log_Path(script_name)
        self.__handle = open(self.__path, 'w', encoding='utf-8')

    @property
    def Path(self) -> str:
        """
        Description:
            The log file.

        Returns:
            (1) parameter [string]: Its path.
        """

        return self.__path

    def Line(self, text: str) -> str:
        """
        Description:
            Append a line to the log (flushed at once, so an aborted run keeps what it wrote).

        Args:
            (1) text [string]: The line, without its newline.

        Returns:
            (1) parameter [string]: The same text, for the caller to print.
        """

        self.__handle.write(text + '\n')
        self.__handle.flush()
        return text

    def Close(self) -> None:
        """
        Description:
            Close the log file.
        """

        self.__handle.close()


@dataclass
class Cell_Configuration_Str:
    """
    Description:
        The cell configuration of Configuration/Cell.yaml in canonical units (SPEC §8). Built by
        Load_Cell_Configuration.
    """

    # The robot base on the pedestal plate, and its rotation about Z.
    #   Unit [Vector<float> 1x3] (m), [float] (rad)
    Base_Position: tp.List[float] = field(default_factory=list)
    Base_Yaw: float = 0.0
    # The vacuum and the suction cup diameter (the adhesion force is derived from them).
    #   Unit [float] (Pa), [float] (m)
    Vacuum_Pressure: float = 0.0
    Cup_Diameter: float = 0.0
    # MG06's settled closed state.
    #   Unit [float] (rad)
    Closed_Lid: float = 0.0
    Closed_Handle: float = 0.0
    # The point and program checks: clearance, allowed contact depth per group, the stale tolerances, the family margin.
    #   Unit [float] (m), [Dict<string, float>] (m), [float] (m), [float] (rad), [float] (rad)
    Clearance: float = 0.0
    Allowed_Depth: tp.Dict[str, float] = field(default_factory=dict)
    Stale_Position: float = 0.0
    Stale_Angle: float = 0.0
    Family_Margin: float = 0.0
    # The acceptance limits of a cycle run (SPEC §10.1).
    #   Unit [float] (m), [float] (rad), [float] (m), [float] (rad), [float] (m)
    Place_Error: float = 0.0
    Place_Tilt: float = 0.0
    Crate_Distance: float = 0.0
    Open_Tolerance: float = 0.0
    Penetration: float = 0.0
    # A program's physics run (F11, step 6): the free transfers' smallest distance to the loader; the largest slip and
    # tracking error; the largest net force of the fork on the handle bar and handle hinge load.
    #   Unit [float] (m), [float] (m), [float] (rad), [float] (N), [float] (N m)
    Loader_Distance: float = 0.0
    Slip_Max: float = 0.0
    Tracking_Max: float = 0.0
    Fork_Net_Force_Max: float = 0.0
    Handle_Load_Max: float = 0.0
    # The run's phase-label period and the cup-touch distance.
    #   Unit [int] (-), [float] (m)
    Phase_Steps: int = 20
    Cup_Touch: float = 0.0
    # The robot's motion limits (step 4): the fraction of the datasheet joint speeds used, the datasheet speeds, the
    # assumed acceleration limits, the shortest MoveJ and MoveL.
    #   Unit [float] (-), [Vector<float> 1x6] (rad/s), [Vector<float> 1x6] (rad/s^2), [float] (s), [float] (s)
    Speed_Fraction: float = 0.0
    Datasheet_Speeds: tp.List[float] = field(default_factory=list)
    Acceleration_Limits: tp.List[float] = field(default_factory=list)
    Movej_Min_Duration: float = 0.0
    Movel_Min_Duration: float = 0.0
    # The MG06 test duration (the program's Test command).
    #   Unit [float] (s)
    Test_Duration: float = 0.0
    # PATH_FORK_OPEN joins the recorded path where its handle is this far below the closed handle.
    #   Unit [float] (rad)
    Open_Join_Margin: float = 0.0
    # The segment check: the largest move of a robot or PCB point between two checked samples.
    #   Unit [float] (m)
    Check_Step: float = 0.0
    # The free transfers' clearance to the loader, and the margin the kinematic checks and the planner add to it (F11).
    #   Unit [float] (m)
    Loader_Free: float = 0.0
    Tracking_Margin: float = 0.0
    # The handover's no-press rule: the PCB-ledge gap and its tolerance.
    #   Unit [float] (m)
    Ledge_Gap: float = 0.0
    Ledge_Tolerance: float = 0.0
    # The loader's notes: (level, message) for each unknown key.
    #   Unit [List<Tuple<string>>]
    Notes: tp.List[tp.Tuple[str, str]] = field(default_factory=list)


class _Reader_Cls(object):
    """
    Description:
        Reads values out of a parsed YAML tree by key path, collecting every problem (a missing key, a wrong
        type or length, a value out of range) and every unknown key instead of stopping at the first.

    Initialization of the Class:
        Args:
            (1) path [string]: The file, for the messages.

        Example:
            Initialization:
                # Initialization of the class.
                Cls = _Reader_Cls('Configuration/Cell.yaml')

            Features:
                # A required number at a key path; Cls.Problems lists what was wrong.
                value = Cls.Number(tree, ('Robot', 'Base_Yaw'), -180.0, 180.0)
    """

    def __init__(self, path: str) -> None:
        # << PRIVATE >> #
        self.__path = os.path.relpath(path, CONST_ROOT) if os.path.isabs(path) else path
        self.Problems: tp.List[str] = []
        self.Notes: tp.List[tp.Tuple[str, str]] = []

    def Fail(self, keys: tp.Sequence[str], message: str) -> None:
        """
        Description:
            Record a problem at a key path.

        Args:
            (1) keys [List<string>]: The key path.
            (2) message [string]: What is wrong.
        """

        self.Problems.append('%s: %s: %s' % (self.__path, '.'.join(keys), message))

    def Get(self, tree: tp.Any, keys: tp.Sequence[str]) -> tp.Any:
        """
        Description:
            The value at the last key of keys inside tree (a mapping), or None with a problem when missing.

        Args:
            (1) tree [Dict<string, Any>]: The mapping holding the last key.
            (2) keys [List<string>]: The full key path.

        Returns:
            (1) parameter [Any]: The value, or None.
        """

        if not isinstance(tree, dict) or keys[-1] not in tree:
            self.Fail(keys, 'missing (REQUIRED)')
            return None
        return tree[keys[-1]]

    def Mapping(self, tree: tp.Any, keys: tp.Sequence[str], known: tp.Optional[tp.Sequence[str]] = None) -> tp.Dict[str, tp.Any]:
        """
        Description:
            A required mapping; with known, every other key is reported as an unknown-key WARNING note.

        Args:
            (1) tree [Dict<string, Any>]: The mapping holding it (or the document for an empty key path).
            (2) keys [List<string>]: The key path.
            (3) known [List<string>]: The keys it may hold, or None for any.

        Returns:
            (1) parameter [Dict<string, Any>]: The mapping, or {} with a problem.
        """

        value = self.Get(tree, keys) if keys else tree
        if value is None:
            return {}
        if not isinstance(value, dict):
            self.Fail(keys, 'must be a mapping, not %r' % (value,))
            return {}
        for key in value:
            if known is not None and key not in known:
                self.Notes.append(('WARNING', '%s: %s: unknown key, ignored (misspelt?)' % (self.__path, '.'.join(list(keys) + [str(key)]))))
        return value

    def Number(self, tree: tp.Any, keys: tp.Sequence[str], low: float, high: float) -> float:
        """
        Description:
            A required number within [low, high].

        Args:
            (1) tree [Dict<string, Any>]: The mapping holding it.
            (2) keys [List<string>]: The key path.
            (3 - 4) low, high [float]: The allowed range.

        Returns:
            (1) parameter [float]: The value, or NaN with a problem.
        """

        value = self.Get(tree, keys)
        if value is None:
            return math.nan
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            self.Fail(keys, 'must be a number, not %r' % (value,))
            return math.nan
        if not low <= value <= high:
            self.Fail(keys, '%r is outside [%g, %g]' % (value, low, high))
        return float(value)

    def Vector(self, tree: tp.Any, keys: tp.Sequence[str], n: int, low: float, high: float) -> tp.List[float]:
        """
        Description:
            A required list of n numbers, each within [low, high].

        Args:
            (1) tree [Dict<string, Any>]: The mapping holding it.
            (2) keys [List<string>]: The key path.
            (3) n [int]: The length.
            (4 - 5) low, high [float]: The allowed range of each value.

        Returns:
            (1) parameter [List<float>]: The values, or NaNs with a problem.
        """

        value = self.Get(tree, keys)
        if value is None:
            return [math.nan] * n
        if not isinstance(value, list) or len(value) != n:
            self.Fail(keys, '%s values, %d required' % (len(value) if isinstance(value, list) else 'not a list of', n))
            return [math.nan] * n
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) for v in value):
            self.Fail(keys, 'must be numbers, not %r' % (value,))
            return [math.nan] * n
        if any(not low <= v <= high for v in value):
            self.Fail(keys, '%r has a value outside [%g, %g]' % (value, low, high))
        return [float(v) for v in value]

    def Choice(self, tree: tp.Any, keys: tp.Sequence[str], choices: tp.Sequence[str]) -> str:
        """
        Description:
            A required string out of choices.

        Args:
            (1) tree [Dict<string, Any>]: The mapping holding it.
            (2) keys [List<string>]: The key path.
            (3) choices [List<string>]: The allowed values.

        Returns:
            (1) parameter [string]: The value, or '' with a problem.
        """

        value = self.Get(tree, keys)
        if value is None:
            return ''
        if value not in choices:
            self.Fail(keys, '%r is not one of %s' % (value, ', '.join(choices)))
            return ''
        return value


class Located_Text_Cls(str):
    """
    Description:
        A string read from a configuration file that remembers its line, so a message can name where it is.

    Initialization of the Class:
        Args:
            (1) text [string]: The text (the str value).

        Example:
            Initialization:
                # Built by the YAML loader for every string scalar and key.
                Cls = Located_Text_Cls('P_HOME')

            Features:
                # The line in its file (0 when unknown).
                Cls.Line
    """

    Line: int = 0


class _YAML_Loader_Cls(yaml.SafeLoader):
    """
    Description:
        The configuration's YAML loader: a duplicate key in a mapping fails with its line; only true / false are
        booleans (ON, OFF, On, Off, yes, no stay strings: the program's words, not YAML 1.1's booleans); every string
        scalar and key is a Located_Text_Cls carrying its line.

    Initialization of the Class:
        Args:
            (1) stream [file]: The open file.

        Example:
            Initialization:
                # Initialization of the class (through yaml.load).
                document = yaml.load(handle, Loader=_YAML_Loader_Cls)

            Features:
                # A string's line.
                document['Program']['S1_Approach_Input'][0].Line
    """


def _Construct_Mapping(loader: yaml.SafeLoader, node: yaml.MappingNode) -> tp.Dict[tp.Any, tp.Any]:
    """
    Description:
        A mapping, failing on a key it already has (PyYAML keeps the last one silently).

    Args:
        (1) loader [yaml.SafeLoader]: The loader.
        (2) node [yaml.MappingNode]: The node.

    Returns:
        (1) parameter [Dict<Any, Any>]: The mapping.
    """

    loader.flatten_mapping(node)
    seen = {}
    for key_node, _ in node.value:
        key = loader.construct_object(key_node, deep=True)
        if key in seen:
            raise yaml.constructor.ConstructorError('while constructing a mapping', node.start_mark,
                                                    "found duplicate key '%s' (first at line %d)" % (key, seen[key]), key_node.start_mark)
        seen[key] = key_node.start_mark.line + 1
    return loader.construct_mapping(node, deep=True)


def _Construct_Text(loader: yaml.SafeLoader, node: yaml.ScalarNode) -> Located_Text_Cls:
    """
    Description:
        A string scalar as a Located_Text_Cls with its line (1-based).

    Args:
        (1) loader [yaml.SafeLoader]: The loader.
        (2) node [yaml.ScalarNode]: The node.

    Returns:
        (1) parameter [Located_Text_Cls]: The text.
    """

    text = Located_Text_Cls(loader.construct_scalar(node))
    text.Line = node.start_mark.line + 1
    return text


# The loader's resolvers: SafeLoader's, with the boolean resolver narrowed to true / false.
_YAML_Loader_Cls.yaml_implicit_resolvers = {first: [(tag, regexp) for tag, regexp in resolvers if tag != 'tag:yaml.org,2002:bool']
                                            for first, resolvers in yaml.SafeLoader.yaml_implicit_resolvers.items()}
_YAML_Loader_Cls.add_implicit_resolver('tag:yaml.org,2002:bool', re.compile(r'^(?:true|True|TRUE|false|False|FALSE)$'), list('tTfF'))
_YAML_Loader_Cls.add_constructor('tag:yaml.org,2002:str', _Construct_Text)
_YAML_Loader_Cls.add_constructor('tag:yaml.org,2002:map', _Construct_Mapping)


def Read_YAML(path: str) -> tp.Any:
    """
    Description:
        A configuration file parsed by the project's YAML loader (_YAML_Loader_Cls: duplicate keys fail, only true /
        false are booleans, strings carry their line), or a Configuration_Error_Cls naming the file and the line.

    Args:
        (1) path [string]: The file.

    Returns:
        (1) parameter [Any]: The document.
    """

    try:
        with open(path, encoding='utf-8') as handle:
            return yaml.load(handle, Loader=_YAML_Loader_Cls)
    except FileNotFoundError as error:
        raise Configuration_Error_Cls('The configuration file %s does not exist. Restore it from git.' % path) from error
    except yaml.YAMLError as error:
        raise Configuration_Error_Cls('The configuration file %s is not valid YAML (%s). Correct it.' % (path, error)) from error


def _Finish(reader: _Reader_Cls, path: str) -> None:
    """
    Description:
        Fail once with every collected problem, if there is one.

    Args:
        (1) reader [_Reader_Cls(object)]: The reader.
        (2) path [string]: The file.
    """

    if reader.Problems:
        raise Configuration_Error_Cls('The configuration file %s is invalid (%d problem(s)); correct it:\n  %s'
                                      % (path, len(reader.Problems), '\n  '.join(reader.Problems)))


def Load_Cell_Configuration(path: str = CONST_CELL_YAML) -> Cell_Configuration_Str:
    """
    Description:
        Load Configuration/Cell.yaml, the only reader of that file: every field REQUIRED and validated, every
        problem collected and reported at once with its key path, unknown keys returned as WARNING notes,
        degrees converted to radians.

    Args:
        (1) path [string]: The file (default Configuration/Cell.yaml).

    Returns:
        (1) parameter [Cell_Configuration_Str(object)]: The configuration.
    """

    r = _Reader_Cls(path)
    root = r.Mapping(Read_YAML(path), (), ('Robot', 'Vacuum', 'MG06', 'Clearance', 'Handover', 'Motion', 'Process', 'Acceptance', 'Simulation'))
    robot = r.Mapping(root, ('Robot',), ('Base_Position', 'Base_Yaw'))
    vacuum = r.Mapping(root, ('Vacuum',), ('Pressure', 'Cup_Diameter'))
    mg06 = r.Mapping(root, ('MG06',), ('Closed_Lid', 'Closed_Handle', 'Open_Join_Margin'))
    clearance = r.Mapping(root, ('Clearance',), ('Robot', 'Allowed_Depth', 'Stale_Position', 'Stale_Angle', 'Family_Margin',
                                                 'Check_Step', 'Loader_Free', 'Tracking_Margin'))
    handover = r.Mapping(root, ('Handover',), ('Ledge_Gap', 'Ledge_Tolerance'))
    motion = r.Mapping(root, ('Motion',), ('Speed_Fraction', 'Datasheet_Speeds', 'Acceleration_Limits', 'Movej_Min_Duration', 'Movel_Min_Duration'))
    process = r.Mapping(root, ('Process',), CONST_PROCESS_KEYS)
    acceptance = r.Mapping(root, ('Acceptance',), ('Place_Error', 'Place_Tilt', 'Crate_Distance', 'Open_Tolerance', 'Penetration', 'Loader_Distance',
                                                   'Slip_Max', 'Tracking_Max', 'Fork_Net_Force_Max', 'Handle_Load_Max'))
    simulation = r.Mapping(root, ('Simulation',), ('Phase_Steps', 'Cup_Touch'))
    depth = r.Mapping(clearance, ('Clearance', 'Allowed_Depth'), CONST_CONTACT_GROUPS)
    allowed = {g: r.Number(depth, ('Clearance', 'Allowed_Depth', g), 0.0, 0.01) for g in CONST_CONTACT_GROUPS}
    steps = r.Get(simulation, ('Simulation', 'Phase_Steps'))
    if steps is not None and (isinstance(steps, bool) or not isinstance(steps, int) or steps < 1):
        r.Fail(('Simulation', 'Phase_Steps'), 'must be a whole number >= 1, not %r' % (steps,))
    cell = Cell_Configuration_Str(
        Base_Position=r.Vector(robot, ('Robot', 'Base_Position'), 3, -5.0, 5.0),
        Base_Yaw=math.radians(r.Number(robot, ('Robot', 'Base_Yaw'), -180.0, 180.0)),
        Vacuum_Pressure=r.Number(vacuum, ('Vacuum', 'Pressure'), 1.0, 101325.0),
        Cup_Diameter=r.Number(vacuum, ('Vacuum', 'Cup_Diameter'), 1e-4, 0.1),
        Closed_Lid=math.radians(r.Number(mg06, ('MG06', 'Closed_Lid'), -40.0, 0.0)),
        Closed_Handle=math.radians(r.Number(mg06, ('MG06', 'Closed_Handle'), 0.0, 60.0)),
        Clearance=r.Number(clearance, ('Clearance', 'Robot'), 1e-4, 0.05), Allowed_Depth=allowed,
        Stale_Position=r.Number(clearance, ('Clearance', 'Stale_Position'), 1e-7, 0.01),
        Stale_Angle=math.radians(r.Number(clearance, ('Clearance', 'Stale_Angle'), 1e-6, 5.0)),
        Family_Margin=math.radians(r.Number(clearance, ('Clearance', 'Family_Margin'), 0.0, 30.0)),
        Place_Error=r.Number(acceptance, ('Acceptance', 'Place_Error'), 1e-6, 0.01),
        Place_Tilt=math.radians(r.Number(acceptance, ('Acceptance', 'Place_Tilt'), 1e-4, 10.0)),
        Crate_Distance=r.Number(acceptance, ('Acceptance', 'Crate_Distance'), 1e-6, 0.1),
        Open_Tolerance=math.radians(r.Number(acceptance, ('Acceptance', 'Open_Tolerance'), 1e-4, 30.0)),
        Penetration=r.Number(acceptance, ('Acceptance', 'Penetration'), 1e-6, 0.01),
        Loader_Distance=r.Number(acceptance, ('Acceptance', 'Loader_Distance'), 0.0, 0.05),
        Slip_Max=r.Number(acceptance, ('Acceptance', 'Slip_Max'), 1e-6, 0.05),
        Tracking_Max=r.Number(acceptance, ('Acceptance', 'Tracking_Max'), 1e-6, 1.0),
        Fork_Net_Force_Max=r.Number(acceptance, ('Acceptance', 'Fork_Net_Force_Max'), 1.0, 10000.0),
        Handle_Load_Max=r.Number(acceptance, ('Acceptance', 'Handle_Load_Max'), 0.01, 1000.0),
        Phase_Steps=steps if isinstance(steps, int) and not isinstance(steps, bool) else 0,
        Cup_Touch=r.Number(simulation, ('Simulation', 'Cup_Touch'), 1e-6, 0.01),
        Speed_Fraction=r.Number(motion, ('Motion', 'Speed_Fraction'), 0.01, 1.0),
        Datasheet_Speeds=[math.radians(v) for v in r.Vector(motion, ('Motion', 'Datasheet_Speeds'), 6, 1.0, 720.0)],
        Acceleration_Limits=[math.radians(v) for v in r.Vector(motion, ('Motion', 'Acceleration_Limits'), 6, 1.0, 10000.0)],
        Movej_Min_Duration=r.Number(motion, ('Motion', 'Movej_Min_Duration'), 0.0, 10.0),
        Movel_Min_Duration=r.Number(motion, ('Motion', 'Movel_Min_Duration'), 0.0, 10.0),
        Test_Duration=r.Number(process, ('Process', 'Test_Duration'), 0.0, 600.0),
        Open_Join_Margin=math.radians(r.Number(mg06, ('MG06', 'Open_Join_Margin'), 0.0, 5.0)),
        Check_Step=r.Number(clearance, ('Clearance', 'Check_Step'), 1e-5, 0.01),
        Loader_Free=r.Number(clearance, ('Clearance', 'Loader_Free'), 1e-4, 0.1),
        Tracking_Margin=r.Number(clearance, ('Clearance', 'Tracking_Margin'), 0.0, 0.05),
        Ledge_Gap=r.Number(handover, ('Handover', 'Ledge_Gap'), 0.0, 0.005),
        Ledge_Tolerance=r.Number(handover, ('Handover', 'Ledge_Tolerance'), 1e-6, 0.005))
    _Finish(r, path)
    cell.Notes = r.Notes
    return cell
