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
# File Name: ../Example/Keypoints/resolve_keypoint.py
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
# Math (Mathematical functions)
import math
# Custom Lib.:
#   ../Configuration/Core - the configuration, the logs
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Load_Keypoints, Read_PCB_Meta, Script_Log_Cls, Configuration_Error_Cls
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model, Scene_Error_Cls
#   ../Kinematics/Core - IK
from Kinematics.Core import Robot_Kinematics_Cls
#   ../Keypoints/Core - the resolution
from Keypoints.Core import Resolve_Keypoint, Keypoints_Error_Cls


def main():
    """
    Description:
        Resolve a keypoint's joints from its Relative_To target (IK on its frame's site, by its family, seeded from
        its own joints or --seed) and print the Joints line to paste into Configuration/Keypoints.yaml. It writes
        nothing (project-style-rpx references/yaml.md); the check names it when a keypoint is stale.

            .venv/bin/python Example/Keypoints/resolve_keypoint.py --name <keypoint> [--seed <keypoint>]
    """

    parser = argparse.ArgumentParser(description='Print the resolved Joints line of a keypoint.')
    parser.add_argument('--name', required=True, help='the keypoint (with a Relative_To)')
    parser.add_argument('--seed', help='the keypoint whose joints seed the IK (default: its own)')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('resolve_keypoint')
    try:
        cell, keypoints, meta = Load_Cell_Configuration(), Load_Keypoints(), Read_PCB_Meta()
        if args.name not in keypoints.Robot or (args.seed is not None and args.seed not in keypoints.Robot):
            raise Keypoints_Error_Cls("Unknown keypoint '%s'; the keypoints are %s." % (args.seed if args.name in keypoints.Robot else args.name,
                                                                                        ', '.join(keypoints.Robot)))
        joints = Resolve_Keypoint(args.name, Robot_Kinematics_Cls(Build_Cell_Model(cell)), keypoints, meta, args.seed)
    except (Configuration_Error_Cls, Scene_Error_Cls, Keypoints_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    print(log.Line('[INFO] %s resolved; paste this line into Configuration/Keypoints.yaml (Robot.%s):' % (args.name, args.name)))
    print(log.Line('    Joints: [%s]' % ', '.join('%.6f' % math.degrees(q) for q in joints)))
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
