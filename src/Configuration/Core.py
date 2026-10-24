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
    never from the working directory (SPEC §7.1); the only reader of the YAML configuration
    (Configuration/Cell.yaml, Configuration/Keypoints.yaml) and of Data/PCB/pcb_meta.json; the route of
    MuJoCo's log and of a script's lines into logger/. It prints nothing.
"""
# The repository root: two folders above src/Configuration/.
#   Unit [string]
CONST_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
# The hand-written configuration (SPEC §8).
#   Unit [string]
CONST_CONFIGURATION = os.path.join(CONST_ROOT, 'Configuration')
CONST_CELL_YAML = os.path.join(CONST_CONFIGURATION, 'Cell.yaml')
CONST_KEYPOINTS_YAML = os.path.join(CONST_CONFIGURATION, 'Keypoints.yaml')
# The MJCF files and their meshes (style project page mjcf.md).
#   Unit [string]
CONST_MODELS = os.path.join(CONST_ROOT, 'models')
# Input data by purpose: PCB/ (the PCB poses), Paths/ (recorded robot paths), Reference/ (the regression plan).
#   Unit [string]
CONST_DATA = os.path.join(CONST_ROOT, 'Data')
# Generated artefacts, one subfolder per kind (git-ignored; design-decisions: logs in logger/, generated
# artefacts in Data/Generated/): Reports/ (the cycle report), Images/ (proof images); step 4 adds Trajectories/.
#   Unit [string]
CONST_GENERATED = os.path.join(CONST_ROOT, 'Data', 'Generated')
CONST_REPORTS = os.path.join(CONST_GENERATED, 'Reports')
CONST_IMAGES = os.path.join(CONST_GENERATED, 'Images')
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
# key is required. (The loader positions moved to Configuration/Keypoints.yaml, owner 2026-10-24.)
#   Unit [Dict<string, int>] (key -> number of values)
CONST_PCB_META_KEYS = {'cad_pos': 3, 'place_pos': 3, 'hold_pos': 3, 'quat': 4}
# The values the keypoint file's closed sets allow.
#   Unit [Tuple<string>]
CONST_FAMILIES = ('wrist_up', 'wrist_down')
CONST_FRAMES = ('suction_tcp', 'tool_flange')
CONST_PCB_LOCATIONS = ('input_crate', 'cups', 'jaws', 'mg06', 'output_crate')
CONST_HOLD_LOCATIONS = ('input_crate', 'jaws', 'output_crate')
CONST_CONTACT_GROUPS = ('cups_pcb', 'handle_bar', 'pcb_jaws', 'pcb_crate')
CONST_FIXTURE_STATES = ('Open', 'Closed')


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
    # The keypoint check: clearance, allowed contact depth per group, the Check and stale tolerances, the
    # family margin.
    #   Unit [float] (m), [Dict<string, float>] (m), [float] (m), [float] (m), [float] (rad), [float] (rad)
    Clearance: float = 0.0
    Allowed_Depth: tp.Dict[str, float] = field(default_factory=dict)
    Check_Position: float = 0.0
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
    # The run's phase-label period and the cup-touch distance.
    #   Unit [int] (-), [float] (m)
    Phase_Steps: int = 20
    Cup_Touch: float = 0.0
    # The loader's notes: (level, message) for each unknown key.
    #   Unit [List<Tuple<string>>]
    Notes: tp.List[tp.Tuple[str, str]] = field(default_factory=list)


@dataclass
class Robot_Keypoint_Str:
    """
    Description:
        One robot keypoint of Configuration/Keypoints.yaml in canonical units. Built by Load_Keypoints.
    """

    # The keypoint's name.
    #   Unit [string]
    Name: str = ''
    # The six joint angles, shoulder_pan .. wrist_3 (the source of truth).
    #   Unit [Vector<float> 1x6] (rad)
    Joints: tp.List[float] = field(default_factory=list)
    # The IK family: wrist_up or wrist_down.
    #   Unit [string]
    Family: str = ''
    # The check value: a site and the position the joints must reach.
    #   Unit [string], [Vector<float> 1x3] (m)
    Check_Frame: str = ''
    Check_Position: tp.List[float] = field(default_factory=list)
    # Relative_To: the target (pcb_hold@<location> or a keypoint), the frame and the offset in it; None when
    # the keypoint has none.
    #   Unit [string], [string], [Vector<float> 1x3] (m)
    Relative_Target: tp.Optional[str] = None
    Relative_Frame: tp.Optional[str] = None
    Relative_Offset: tp.Optional[tp.List[float]] = None
    # The MG06 state: 'Open', 'Closed' or 'Recorded' (then the lid and handle angles and their source).
    #   Unit [string], [float] (rad), [float] (rad), [string]
    Fixture: str = ''
    Fixture_Lid: tp.Optional[float] = None
    Fixture_Handle: tp.Optional[float] = None
    Fixture_Source: tp.Optional[str] = None
    # The scene: the loader position, the gripper state and the PCB location.
    #   Unit [string]
    Loader: str = ''
    Gripper: str = ''
    PCB: str = ''
    # The contact groups allowed within the clearance.
    #   Unit [List<string>]
    Allow_Contact: tp.List[str] = field(default_factory=list)
    # What the keypoint is for.
    #   Unit [string]
    Note: str = ''


@dataclass
class Loader_Position_Str:
    """
    Description:
        A named loader position (the Y and Z axis set-points).
    """

    # The Y axis (towards MG06) and the Z axis (up).
    #   Unit [float] (m)
    Y: float = 0.0
    Z: float = 0.0


@dataclass
class Keypoints_Str:
    """
    Description:
        The keypoint file: robot keypoints in file order, loader positions, gripper states.
    """

    # The robot keypoints by name, in the order of the file (the cycle's order).
    #   Unit [Dict<string, Robot_Keypoint_Str>]
    Robot: tp.Dict[str, Robot_Keypoint_Str] = field(default_factory=dict)
    # The loader positions by name.
    #   Unit [Dict<string, Loader_Position_Str>]
    Loader: tp.Dict[str, Loader_Position_Str] = field(default_factory=dict)
    # The gripper states by name: the jaw set-point per jaw.
    #   Unit [Dict<string, float>] (m)
    Gripper: tp.Dict[str, float] = field(default_factory=dict)
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


def _Read_YAML(path: str) -> tp.Any:
    """
    Description:
        A YAML file parsed, or a Configuration_Error_Cls naming the file.

    Args:
        (1) path [string]: The file.

    Returns:
        (1) parameter [Any]: The document.
    """

    try:
        with open(path, encoding='utf-8') as handle:
            return yaml.safe_load(handle)
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
    root = r.Mapping(_Read_YAML(path), (), ('Robot', 'Vacuum', 'MG06', 'Clearance', 'Acceptance', 'Simulation'))
    robot = r.Mapping(root, ('Robot',), ('Base_Position', 'Base_Yaw'))
    vacuum = r.Mapping(root, ('Vacuum',), ('Pressure', 'Cup_Diameter'))
    mg06 = r.Mapping(root, ('MG06',), ('Closed_Lid', 'Closed_Handle'))
    clearance = r.Mapping(root, ('Clearance',), ('Robot', 'Allowed_Depth', 'Check_Position', 'Stale_Position', 'Stale_Angle', 'Family_Margin'))
    acceptance = r.Mapping(root, ('Acceptance',), ('Place_Error', 'Place_Tilt', 'Crate_Distance', 'Open_Tolerance', 'Penetration'))
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
        Check_Position=r.Number(clearance, ('Clearance', 'Check_Position'), 1e-7, 0.01),
        Stale_Position=r.Number(clearance, ('Clearance', 'Stale_Position'), 1e-7, 0.01),
        Stale_Angle=math.radians(r.Number(clearance, ('Clearance', 'Stale_Angle'), 1e-6, 5.0)),
        Family_Margin=math.radians(r.Number(clearance, ('Clearance', 'Family_Margin'), 0.0, 30.0)),
        Place_Error=r.Number(acceptance, ('Acceptance', 'Place_Error'), 1e-6, 0.01),
        Place_Tilt=math.radians(r.Number(acceptance, ('Acceptance', 'Place_Tilt'), 1e-4, 10.0)),
        Crate_Distance=r.Number(acceptance, ('Acceptance', 'Crate_Distance'), 1e-6, 0.1),
        Open_Tolerance=math.radians(r.Number(acceptance, ('Acceptance', 'Open_Tolerance'), 1e-4, 30.0)),
        Penetration=r.Number(acceptance, ('Acceptance', 'Penetration'), 1e-6, 0.01),
        Phase_Steps=steps if isinstance(steps, int) and not isinstance(steps, bool) else 0,
        Cup_Touch=r.Number(simulation, ('Simulation', 'Cup_Touch'), 1e-6, 0.01))
    _Finish(r, path)
    cell.Notes = r.Notes
    return cell


def Load_Keypoints(path: str = CONST_KEYPOINTS_YAML) -> Keypoints_Str:
    """
    Description:
        Load Configuration/Keypoints.yaml, the only reader of that file: every field REQUIRED and validated
        (Relative_To may be null; Allow_Contact may be an empty list), the references between sections checked
        (a Context's loader position and gripper state, a Relative_To keypoint), every problem collected and
        reported at once with its key path, unknown keys returned as WARNING notes, degrees converted to radians.

    Args:
        (1) path [string]: The file (default Configuration/Keypoints.yaml).

    Returns:
        (1) parameter [Keypoints_Str(object)]: The keypoints.
    """

    r = _Reader_Cls(path)
    root = r.Mapping(_Read_YAML(path), (), ('Robot', 'Loader', 'Gripper'))
    loader = {}
    for name, value in r.Mapping(root, ('Loader',)).items():
        position = r.Mapping(root['Loader'], ('Loader', name), ('Y', 'Z'))
        loader[name] = Loader_Position_Str(Y=r.Number(position, ('Loader', name, 'Y'), -1.0, 1.0),
                                           Z=r.Number(position, ('Loader', name, 'Z'), -1.0, 1.0))
    gripper = {name: r.Number(root['Gripper'], ('Gripper', name), 0.0, 0.05) for name in r.Mapping(root, ('Gripper',))}
    robot_tree = r.Mapping(root, ('Robot',))
    known = ('Joints', 'Family', 'Check', 'Relative_To', 'Fixture', 'Context', 'Allow_Contact', 'Note')
    robot = {}
    for name in robot_tree:
        k = ('Robot', str(name))
        tree = r.Mapping(robot_tree, k, known)
        joints = [math.radians(v) for v in r.Vector(tree, k + ('Joints',), 6, -360.0, 360.0)]
        check = r.Mapping(tree, k + ('Check',), ('Frame', 'Position'))
        keypoint = Robot_Keypoint_Str(Name=str(name), Joints=joints, Family=r.Choice(tree, k + ('Family',), CONST_FAMILIES),
                                      Check_Frame=r.Choice(check, k + ('Check', 'Frame'), CONST_FRAMES),
                                      Check_Position=r.Vector(check, k + ('Check', 'Position'), 3, -5.0, 5.0))
        relative = r.Get(tree, k + ('Relative_To',))
        if relative is not None:
            relative = r.Mapping(tree, k + ('Relative_To',), ('Target', 'Frame', 'Offset'))
            target = r.Get(relative, k + ('Relative_To', 'Target'))
            location = str(target)[len('pcb_hold@'):] if str(target).startswith('pcb_hold@') else None
            if target is not None and location is None and target not in robot_tree:
                r.Fail(k + ('Relative_To', 'Target'), '%r is neither pcb_hold@<location> nor a keypoint' % (target,))
            if location is not None and location not in CONST_HOLD_LOCATIONS:
                r.Fail(k + ('Relative_To', 'Target'), "'%s' is not one of %s" % (location, ', '.join(CONST_HOLD_LOCATIONS)))
            if target == name:
                r.Fail(k + ('Relative_To', 'Target'), 'a keypoint cannot be relative to itself')
            keypoint.Relative_Target = str(target)
            keypoint.Relative_Frame = r.Choice(relative, k + ('Relative_To', 'Frame'), CONST_FRAMES)
            keypoint.Relative_Offset = r.Vector(relative, k + ('Relative_To', 'Offset'), 3, -1.0, 1.0)
        fixture = r.Get(tree, k + ('Fixture',))
        if isinstance(fixture, dict):
            state = r.Mapping(tree, k + ('Fixture',), ('Lid', 'Handle', 'Source'))
            keypoint.Fixture = 'Recorded'
            # A recorded lid may stand slightly past its closed stop (0 deg): the contact solver presses it in
            # (row 286: +0.0024 deg).
            keypoint.Fixture_Lid = math.radians(r.Number(state, k + ('Fixture', 'Lid'), -40.0, 1.0))
            keypoint.Fixture_Handle = math.radians(r.Number(state, k + ('Fixture', 'Handle'), -1.0, 60.0))
            source = r.Get(state, k + ('Fixture', 'Source'))
            if source is not None and (not isinstance(source, str) or not source.strip()):
                r.Fail(k + ('Fixture', 'Source'), 'must name where the state comes from')
            keypoint.Fixture_Source = source
        elif fixture is not None:
            keypoint.Fixture = r.Choice(tree, k + ('Fixture',), CONST_FIXTURE_STATES)
        context = r.Mapping(tree, k + ('Context',), ('Loader', 'Gripper', 'PCB'))
        keypoint.Loader = r.Choice(context, k + ('Context', 'Loader'), tuple(loader))
        keypoint.Gripper = r.Choice(context, k + ('Context', 'Gripper'), tuple(gripper))
        keypoint.PCB = r.Choice(context, k + ('Context', 'PCB'), CONST_PCB_LOCATIONS)
        allow = r.Get(tree, k + ('Allow_Contact',))
        if allow is not None:
            if not isinstance(allow, list) or any(a not in CONST_CONTACT_GROUPS for a in allow):
                r.Fail(k + ('Allow_Contact',), '%r must be a list of %s ([] for none)' % (allow, ', '.join(CONST_CONTACT_GROUPS)))
            else:
                keypoint.Allow_Contact = list(allow)
        note = r.Get(tree, k + ('Note',))
        if note is not None and (not isinstance(note, str) or not note.strip()):
            r.Fail(k + ('Note',), 'must say what the keypoint is for')
        keypoint.Note = note if isinstance(note, str) else ''
        robot[str(name)] = keypoint
    _Finish(r, path)
    return Keypoints_Str(Robot=robot, Loader=loader, Gripper=gripper, Notes=r.Notes)
