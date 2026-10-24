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
# File Name: ../Example/Keypoints/check_keypoints.py
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
# Custom Lib.:
#   ../Configuration/Core - the configuration, the logs
from Configuration.Core import (Configure_MuJoCo_Log, Load_Cell_Configuration, Load_Keypoints, Read_PCB_Meta, Script_Log_Cls,
                                Configuration_Error_Cls)
#   ../Scene/Core - the cell's errors
from Scene.Core import Scene_Error_Cls
#   ../Kinematics/Core, ../Collision/Core - FK, the check model (the planning model: the table included) and exact distances
from Kinematics.Core import Robot_Kinematics_Cls
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
#   ../Keypoints/Core - the check
from Keypoints.Core import Check_Keypoint, Format_Check_Table


def main():
    """
    Description:
        The keypoint check (SPEC §7.7), headless: every robot keypoint (or one) posed in its context, a table of the
        rules and the verdicts, every failure named; exit 1 when a keypoint FAILs (a WARN passes).

            .venv/bin/python Example/Keypoints/check_keypoints.py [--name <keypoint>]
    """

    parser = argparse.ArgumentParser(description='Check the keypoints (limits, family, Check, stale, clearance).')
    parser.add_argument('--name', help='check one keypoint only')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('check_keypoints')
    try:
        cell, keypoints, meta = Load_Cell_Configuration(), Load_Keypoints(), Read_PCB_Meta()
        model = Build_Planning_Model(cell)
    except (Configuration_Error_Cls, Scene_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    for level, message in cell.Notes + keypoints.Notes:
        print(log.Line('[%s] %s' % (level, message)))
    names = list(keypoints.Robot)
    if args.name is not None and args.name not in names:
        print(log.Line("[ERROR] There is no keypoint '%s'; the keypoints are %s." % (args.name, ', '.join(names))))
        log.Close()
        return 1
    collision = Robot_Collision_Cls(model)
    kinematics = Robot_Kinematics_Cls(model, collision.Data)
    results = [Check_Keypoint(name, kinematics, collision, keypoints, cell, meta) for name in ([args.name] if args.name else names)]
    for line in Format_Check_Table(results, keypoints):
        print(log.Line(line))
    failed = [r.Name for r in results if r.Verdict == 'FAIL']
    print(log.Line('[%s] %d keypoint(s): %d FAIL, %d WARN, %d PASS. Simulation results only: not validated for the real cell.'
                   % ('ERROR' if failed else 'INFO', len(results), len(failed), sum(r.Verdict == 'WARN' for r in results),
                      sum(r.Verdict == 'PASS' for r in results))))
    log.Close()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
