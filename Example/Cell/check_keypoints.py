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
# File Name: ../Example/Cell/check_keypoints.py
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
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Configuration/Core - the configuration, the logs
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls, Configuration_Error_Cls
#   ../Configuration/Program_Parser - the points and their scenes
from Configuration.Program_Parser import Load_Constants, Load_Simulation
#   ../Scene/Core - the cell's errors
from Scene.Core import Scene_Error_Cls
#   ../Kinematics/Core, ../Collision/Core - FK, the check model (the planning model: the table included) and exact distances
from Kinematics.Core import Robot_Kinematics_Cls
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
#   ../Keypoints/Core - the check
from Keypoints.Core import Check_Keypoint, Format_Check_Table


def main(argv: tp.Optional[tp.List[str]] = None, log_name: tp.Optional[str] = None) -> int:
    """
    Description:
        The keypoint check (SPEC §7.7), headless: every robot keypoint (or one) posed in its context, a table of the
        rules and the verdicts, every failure named; exit 1 when a keypoint FAILs (a WARN passes).

            Called by Example/cell.py check [--name <keypoint>] and Example/cell.py point <name>.

    Args:
        (1) argv [List<string>]: The arguments (default: the command line).
        (2) log_name [string]: The log to append to (cell.py's logger/cell_<subcommand>.log), or None: logger/check_keypoints.log,
            rewritten.

    Returns:
        (1) parameter [int]: The exit code.
    """

    parser = argparse.ArgumentParser(description='Check the keypoints (limits, family, Check, stale, clearance).')
    parser.add_argument('--name', help='check one keypoint only')
    args = parser.parse_args(argv)

    Configure_MuJoCo_Log()
    log = Script_Log_Cls(log_name or 'check_keypoints', append=log_name is not None)
    try:
        cell, constants, meta = Load_Cell_Configuration(), Load_Constants(), Read_PCB_Meta()
        simulation = Load_Simulation(constants)
        model = Build_Planning_Model(cell)
    except (Configuration_Error_Cls, Scene_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    for level, message in cell.Notes + constants.Notes + simulation.Notes:
        print(log.Line('[%s] %s' % (level, message)))
    names = list(constants.Points)
    if args.name is not None and args.name not in names:
        print(log.Line("[ERROR] There is no point '%s'; the points are %s." % (args.name, ', '.join(names))))
        log.Close()
        return 1
    collision = Robot_Collision_Cls(model)
    kinematics = Robot_Kinematics_Cls(model, collision.Data)
    results = [Check_Keypoint(name, kinematics, collision, constants, simulation, cell, meta) for name in ([args.name] if args.name else names)]
    for line in Format_Check_Table(results):
        print(log.Line(line))
    failed = [r.Name for r in results if r.Verdict == 'FAIL']
    print(log.Line('[%s] %d keypoint(s): %d FAIL, %d WARN, %d PASS. Simulation results only: not validated for the real cell.'
                   % ('ERROR' if failed else 'INFO', len(results), len(failed), sum(r.Verdict == 'WARN' for r in results),
                      sum(r.Verdict == 'PASS' for r in results))))
    log.Close()
    return 1 if failed else 0
