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
# File Name: ../Example/Render/render_image.py
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
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core - paths, the logs
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Configuration_Error_Cls, Script_Log_Cls, CONST_IMAGES
#   ../Configuration/Const - the repository root
from Configuration.Const import CONST_ROOT
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model, Scene_Error_Cls
#   ../Render/Core - the image
from Render.Core import Render_Image, Save_Image, Render_Error_Cls

"""
Description:
    Initialization of constants.
"""
# The default camera and file name: the proof image of the green PCB (design-decisions).
#   Unit [string]
CONST_CAMERA, CONST_IMAGE_NAME = 'cam_pcb', 'pcb_green.png'


def main():
    """
    Description:
        An offscreen image of the cell at its initial state from a camera, through Render.Core, saved as a
        PNG (matplotlib) into DIR (default Data/Generated/Images/pcb_green.png from cam_pcb).

            .venv/bin/python Example/Render/render_image.py [--camera cam_pcb] [--name pcb_green.png] [--out DIR]
    """

    parser = argparse.ArgumentParser(description='Offscreen image of the cell from a camera.')
    parser.add_argument('--camera', default=CONST_CAMERA, help='the camera (default: %s)' % CONST_CAMERA)
    parser.add_argument('--name', default=CONST_IMAGE_NAME, help='the file name (default: %s)' % CONST_IMAGE_NAME)
    parser.add_argument('--out', default=CONST_IMAGES, help='write into this directory (default: %s)' % os.path.relpath(CONST_IMAGES, CONST_ROOT))
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('render_image')
    try:
        model = Build_Cell_Model(Load_Cell_Configuration())
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        path = Save_Image(Render_Image(model, data, args.camera), os.path.join(args.out, args.name))
    except (Configuration_Error_Cls, Scene_Error_Cls, Render_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    print(log.Line('[INFO] Wrote %s (camera %s).' % (path, args.camera)))
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
