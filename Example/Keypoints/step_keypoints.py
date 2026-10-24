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
# File Name: ../Example/Keypoints/step_keypoints.py
## =========================================================================== ##

# System (Default)
import sys
# OS (Operating system interfaces)
import os
# The repository root, built from this file's location, so the script runs from any working directory.
#   Unit [string]
CONST_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
#   Add access if it is not in the system path.
if os.path.join(CONST_REPO_ROOT, 'src') not in sys.path:
    sys.path.append(os.path.join(CONST_REPO_ROOT, 'src'))
# Argparse (Command-line parsing)
import argparse
# Time (Time access)
import time
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Configuration/Core - the configuration, the logs
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls, Configuration_Error_Cls
#   ../Configuration/Program_Parser - the points and their scenes
from Configuration.Program_Parser import Load_Constants, Load_Simulation
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model, Scene_Error_Cls
#   ../Kinematics/Core - the kinematics posing uses
from Kinematics.Core import Robot_Kinematics_Cls
#   ../Keypoints/Core - posing
from Keypoints.Core import Pose_Cell
#   ../Render/Core - the viewer
from Render.Core import Launch_Passive_Viewer, Render_Error_Cls

"""
Description:
    Initialization of constants.
"""
# The keys (GLFW codes): Page Down and '.' next, Page Up previous. Every letter and , / ; ' \\ ` toggles a MuJoCo
# visual or render flag (mjVISSTRING, mjRNDSTRING) and the viewer binds Space, Backspace, the arrows, Tab, Esc,
# [ ], - =, F1-F5, 0-5; Page Up / Page Down and '.' are in none of them (FEATURES.md F6).
#   Unit [Dict<int, int>] (key code -> step)
CONST_KEYS = {267: 1, 46: 1, 266: -1}


def main():
    """
    Description:
        Step through the robot points of Configuration/Constants.yaml in MuJoCo's viewer, the whole cell posed at
        each (static, not simulated): Page Down or '.' next, Page Up previous; the keypoint and its note are printed.
        Needs mjpython on macOS (the passive viewer); --headless poses every keypoint once and exits.

            .venv/bin/mjpython Example/Keypoints/step_keypoints.py [--start <keypoint>]
    """

    parser = argparse.ArgumentParser(description='Step through the keypoints in the viewer (Page Down / Page Up).')
    parser.add_argument('--start', help='the first keypoint (default: the first of the file)')
    parser.add_argument('--headless', action='store_true', help='pose every keypoint once, no window')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('step_keypoints')
    try:
        cell, constants, meta = Load_Cell_Configuration(), Load_Constants(), Read_PCB_Meta()
        simulation = Load_Simulation(constants)
        model = Build_Cell_Model(cell)
    except (Configuration_Error_Cls, Scene_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    kinematics = Robot_Kinematics_Cls(model)
    names = list(constants.Points)
    if args.start is not None and args.start not in names:
        print(log.Line("[ERROR] There is no point '%s'; the points are %s." % (args.start, ', '.join(names))))
        log.Close()
        return 1
    state = {'index': names.index(args.start) if args.start else 0, 'pending': True}

    def Show(index: int) -> None:
        """
        Description:
            Pose the cell at the keypoint of index and print it.

        Args:
            (1) index [int]: The keypoint's position in the file.
        """

        point = Pose_Cell(kinematics, constants, simulation, cell, meta, names[index])
        print(log.Line('[INFO] %2d/%d %s: %s' % (index + 1, len(names), point.Name, point.Description)), flush=True)

    if args.headless:
        for index in range(len(names)):
            Show(index)
        log.Close()
        return 0

    def On_Key(key: int) -> None:
        """
        Description:
            Move to the next or previous keypoint on Page Down / '.' / Page Up (posing happens in the main loop).

        Args:
            (1) key [int]: The GLFW key code.
        """

        if key in CONST_KEYS:
            state['index'] = (state['index'] + CONST_KEYS[key]) % len(names)
            state['pending'] = True

    try:
        viewer = Launch_Passive_Viewer(model, kinematics.Data, On_Key, 'Example/Keypoints/step_keypoints.py')
    except Render_Error_Cls as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 2
    print(log.Line('[INFO] Page Down or . = next keypoint, Page Up = previous. Simulation results only: not validated for the real cell.'))
    with viewer:
        while viewer.is_running():
            if state['pending']:
                with viewer.lock():
                    Show(state['index'])
                state['pending'] = False
            viewer.sync()
            time.sleep(0.05)
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
