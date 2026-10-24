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
# File Name: ../Example/Render/show_cell.py
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
#   ../Configuration/Core - the logs
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Configuration_Error_Cls, Script_Log_Cls
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model, Scene_Error_Cls
#   ../Render/Core - the viewer
from Render.Core import Launch_Viewer


def main():
    """
    Description:
        Open MuJoCo's viewer on the cell built by Scene.Core (the initial state: MG06 open, PCB in the
        loader jaws). Blocks until the window is closed; runs under python on macOS. --headless builds the
        cell and exits (used by the tests).

            .venv/bin/python Example/Render/show_cell.py [--headless]
    """

    parser = argparse.ArgumentParser(description='Open the MuJoCo viewer on the cell.')
    parser.add_argument('--headless', action='store_true', help='build the cell and exit without a window')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('show_cell')
    try:
        model = Build_Cell_Model(Load_Cell_Configuration())
    except (Configuration_Error_Cls, Scene_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    print(log.Line('[INFO] Cell built: %d bodies, %d geoms, %d actuators.' % (model.nbody, model.ngeom, model.nu)))
    if not args.headless:
        print(log.Line('[INFO] Opening the viewer; close its window to exit.'))
        Launch_Viewer(model, data)
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
