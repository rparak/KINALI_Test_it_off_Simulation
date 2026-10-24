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
# File Name: ../Example/Keypoints/show_keypoint.py
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
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Configuration/Core - the configuration, the logs
from Configuration.Core import (Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls, Configuration_Error_Cls, CONST_IMAGES,
                                CONST_PCB_LOCATIONS, CONST_FIXTURE_STATES)
#   ../Configuration/Program_Parser - the points and their scenes
from Configuration.Program_Parser import Load_Constants, Load_Simulation
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model, Scene_Error_Cls
#   ../Kinematics/Core - the TCP pose
from Kinematics.Core import Robot_Kinematics_Cls
#   ../Keypoints/Core - posing
from Keypoints.Core import Pose_Cell, Keypoints_Error_Cls
#   ../Render/Core - the viewer and the image
from Render.Core import Launch_Passive_Viewer, Render_Free_Image, Save_Image, Render_Error_Cls


def main():
    """
    Description:
        Pose the whole cell at one point of Configuration/Constants.yaml (robot, loader, jaws, PCB, MG06 from its scene in
        Configuration/Simulation.yaml, each overridable) and show it in MuJoCo's viewer, static (not simulated). --image writes
        an offscreen render looking at the TCP instead; --headless only poses and prints. The viewer needs mjpython on
        macOS (the passive viewer).

            .venv/bin/mjpython Example/Keypoints/show_keypoint.py --name <P_point> [--loader <L_position>] [--gripper <G_command>]
                                                                  [--pcb <location>] [--mg06 Open|Closed]
            .venv/bin/python Example/Keypoints/show_keypoint.py --name <P_point> --image [FILE] [--azimuth 150]
    """

    parser = argparse.ArgumentParser(description='Show the cell posed at one point.')
    parser.add_argument('--name', required=True, help='the point (Configuration/Constants.yaml Points)')
    parser.add_argument('--loader', help='a loader position (L_...) instead of the point\'s scene')
    parser.add_argument('--gripper', help='a gripper command (G_...) instead of the point\'s scene')
    parser.add_argument('--pcb', choices=CONST_PCB_LOCATIONS, help='a PCB location instead of the point\'s scene')
    parser.add_argument('--mg06', choices=CONST_FIXTURE_STATES, help='an MG06 state instead of the point\'s Fixture')
    parser.add_argument('--headless', action='store_true', help='pose and print, no window')
    parser.add_argument('--image', nargs='?', const='', help='write an offscreen render (default Data/Generated/Images/keypoint_<name>.png)')
    parser.add_argument('--azimuth', type=float, default=150.0, help='image camera azimuth [deg] (default 150)')
    parser.add_argument('--elevation', type=float, default=-25.0, help='image camera elevation [deg] (default -25)')
    parser.add_argument('--distance', type=float, default=1.1, help='image camera distance from the TCP [m] (default 1.1)')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('show_keypoint')
    try:
        cell, constants, meta = Load_Cell_Configuration(), Load_Constants(), Read_PCB_Meta()
        simulation = Load_Simulation(constants)
        for level, message in cell.Notes + constants.Notes + simulation.Notes:
            print(log.Line('[%s] %s' % (level, message)))
        model = Build_Cell_Model(cell)
        kinematics = Robot_Kinematics_Cls(model)
        point = Pose_Cell(kinematics, constants, simulation, cell, meta, args.name, args.loader, args.gripper, args.pcb, args.mg06)
    except (Configuration_Error_Cls, Scene_Error_Cls, Keypoints_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    scene = simulation.Points[point.Name]
    tcp, _ = kinematics.Site_Pose(np.array(point.Joints))
    print(log.Line('[INFO] %s: %s' % (point.Name, point.Description)))
    print(log.Line('[INFO] joints [deg] %s; TCP [m] %s' % (np.round(np.degrees(point.Joints), 6).tolist(), np.round(tcp, 5).tolist())))
    print(log.Line('[INFO] loader %s, gripper %s, PCB %s, MG06 %s' % (args.loader or scene.Loader, args.gripper or scene.Gripper,
                                                                       args.pcb or scene.PCB, args.mg06 or scene.Fixture)))
    print(log.Line('[INFO] Simulation result only: not validated for the real cell.'))
    if args.image is not None:
        path = args.image or os.path.join(CONST_IMAGES, 'point_%s.png' % point.Name)
        path = Save_Image(Render_Free_Image(model, kinematics.Data, tcp, args.distance, args.azimuth, args.elevation), path)
        print(log.Line('[INFO] Wrote %s.' % path))
    if args.headless or args.image is not None:
        log.Close()
        return 0
    try:
        viewer = Launch_Passive_Viewer(model, kinematics.Data, command='Example/Keypoints/show_keypoint.py --name %s' % args.name)
    except Render_Error_Cls as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 2
    with viewer:
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.05)
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
