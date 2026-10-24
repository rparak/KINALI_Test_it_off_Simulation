## =========================================================================== ##
# MIT License
# Copyright (c) 2026 Roman Parak
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
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco

"""
Description:
    Initialization of constants.

    The repository's folders and files, built from this module's location (src/Configuration/Core.py),
    never from the working directory (SPEC §7.1); the reader of Data/pcb_meta.json; the route of
    MuJoCo's log and of a script's lines into logger/. It prints nothing.
"""
# The repository root: two folders above src/Configuration/.
#   Unit [string]
CONST_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
# The MJCF files and their meshes (style project page mjcf.md).
#   Unit [string]
CONST_MODELS = os.path.join(CONST_ROOT, 'models')
# Hand-written input data: the PCB poses and the recorded fork path.
#   Unit [string]
CONST_DATA = os.path.join(CONST_ROOT, 'Data')
# Generated reports and images (git-ignored, created by install.sh or on first use).
#   Unit [string]
CONST_OUTPUT = os.path.join(CONST_ROOT, 'output')
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
CONST_PCB_META = os.path.join(CONST_DATA, 'pcb_meta.json')
CONST_FORK_PATH = os.path.join(CONST_DATA, 'fork_close_open_path.npz')
# The verified IK plan the cycle run is proven on (SPEC §4.6), the regression input until step 6 replaces it
# with the keypoint program; its planner is in git tag legacy-planner-reference (design-decisions).
#   Unit [string]
CONST_REFERENCE_PLAN = os.path.join(CONST_DATA, 'Reference', 'cell_plan.npz')
# The file name of the cycle report (written into output/ or a script's --out folder).
#   Unit [string]
CONST_REPORT_NAME = 'report.txt'
# MuJoCo's log file; MuJoCo writes MUJOCO_LOG.TXT into the working directory unless told otherwise.
#   Unit [string]
CONST_MUJOCO_LOG = os.path.join(CONST_LOGGER, 'MUJOCO_LOG.TXT')
# The keys of Data/pcb_meta.json, each a position [m] (3 values), a quaternion (4 values, w first) or a
# scalar [m]; every key is required.
#   Unit [Dict<string, int>] (key -> number of values, 1 = scalar)
CONST_PCB_META_KEYS = {'cad_pos': 3, 'place_pos': 3, 'hold_pos': 3, 'quat': 4, 'y_home': 1, 'y_place': 1,
                       'z_place': 1, 'jaw_grip': 1, 'jaw_open': 1, 'machine_shift': 1}


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
        The PCB poses and loader axis positions of Data/pcb_meta.json (SPEC §4.4), in the MG06 fixture
        frame. Built by Read_PCB_Meta.
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
    # Loader Y axis at home and at the place position; Z axis at the place position.
    #   Unit [float] (m)
    Y_Home: float = 0.0
    Y_Place: float = 0.0
    Z_Place: float = 0.0
    # Jaw opening that grips the PCB, and fully open.
    #   Unit [float] (m)
    Jaw_Grip: float = 0.0
    Jaw_Open: float = 0.0
    # The loader's shift towards MG06 against the CAD (a compensated CAD deviation, SPEC §4.7).
    #   Unit [float] (m)
    Machine_Shift: float = 0.0


def Read_PCB_Meta(path: str = CONST_PCB_META) -> PCB_Meta_Str:
    """
    Description:
        Read Data/pcb_meta.json, the only reader of that file. Every problem is collected first and
        reported once: a missing file, invalid JSON, a missing key, a value of the wrong length or type.

    Args:
        (1) path [string]: The file (default: Data/pcb_meta.json).

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
        if n == 1 and isinstance(value, (int, float)) and not isinstance(value, bool):
            values[key] = float(value)
        elif n > 1 and isinstance(value, list) and len(value) == n and all(isinstance(v, (int, float)) for v in value):
            values[key] = [float(v) for v in value]
        else:
            problems.append('%s must be %s, not %r' % (key, 'a number' if n == 1 else 'a list of %d numbers' % n, value))
    if problems:
        raise Configuration_Error_Cls('The PCB data file %s is invalid: %s. Correct it or restore it from git.'
                                      % (path, '; '.join(problems)))
    return PCB_Meta_Str(Cad_Pos=values['cad_pos'], Place_Pos=values['place_pos'], Hold_Pos=values['hold_pos'],
                        Quat=values['quat'], Y_Home=values['y_home'], Y_Place=values['y_place'], Z_Place=values['z_place'],
                        Jaw_Grip=values['jaw_grip'], Jaw_Open=values['jaw_open'], Machine_Shift=values['machine_shift'])


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
