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
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls, Configuration_Error_Cls
#   ../Configuration/Program_Parser - the points and their scenes
from Configuration.Program_Parser import Load_Constants, Load_Simulation
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model, Scene_Error_Cls
#   ../Kinematics/Core - IK, the Pose angles' spelling
from Kinematics.Core import Robot_Kinematics_Cls, Degrees_Text
#   ../Keypoints/Core - the resolution
from Keypoints.Core import Resolve_Keypoint, Keypoints_Error_Cls


def main():
    """
    Description:
        Resolve a point's joints from its Reference (or PCB_Hold) target (IK on its tool's site, in its family, seeded from
        its own joints or --seed) and print the Joints and Pose lines to paste into Configuration/Constants.yaml. It writes
        nothing (project-style-rpx references/yaml.md); the check names it when a point is stale.

            .venv/bin/python Example/Keypoints/resolve_keypoint.py --name <P_point> [--seed <P_point>]
    """

    parser = argparse.ArgumentParser(description='Print the resolved Joints and Pose lines of a point.')
    parser.add_argument('--name', required=True, help='the point (with a Reference or a PCB_Hold)')
    parser.add_argument('--seed', help='the point whose joints seed the IK (default: its own)')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('resolve_keypoint')
    try:
        cell, constants, meta = Load_Cell_Configuration(), Load_Constants(), Read_PCB_Meta()
        simulation = Load_Simulation(constants)
        if args.name not in constants.Points or (args.seed is not None and args.seed not in constants.Points):
            raise Keypoints_Error_Cls("Unknown point '%s'; the points are %s." % (args.seed if args.name in constants.Points else args.name,
                                                                                ', '.join(constants.Points)))
        joints, pose = Resolve_Keypoint(args.name, Robot_Kinematics_Cls(Build_Cell_Model(cell)), constants, simulation, meta, cell, args.seed)
    except (Configuration_Error_Cls, Scene_Error_Cls, Keypoints_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    print(log.Line('[INFO] %s resolved; paste these lines into Configuration/Constants.yaml (Points.%s):' % (args.name, args.name)))
    print(log.Line('    Joints: [%s]' % ', '.join('%.6f' % math.degrees(q) for q in joints)))
    print(log.Line('    Pose: [%s]' % ', '.join(['%.9f' % v for v in pose[:3]] + [Degrees_Text(a) for a in pose[3:]])))
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
