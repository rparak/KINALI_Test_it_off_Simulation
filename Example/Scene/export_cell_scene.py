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
# File Name: ../Example/Scene/export_cell_scene.py
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
#   ../Configuration/Core - paths, the logs
from Configuration.Core import Configure_MuJoCo_Log, Script_Log_Cls, CONST_CELL_SCENE_XML
#   ../Scene/Core - the cell and its export
from Scene.Core import Export_Cell_Scene, Scene_Error_Cls


def main():
    """
    Description:
        Write the whole cell as one MJCF for MuJoCo's viewer, through Scene.Core.Export_Cell_Scene, with the
        mesh paths relative to the file (default models/cell_scene.xml).

            .venv/bin/python Example/Scene/export_cell_scene.py [--out DIR]
    """

    parser = argparse.ArgumentParser(description='Write the whole cell as one MJCF (cell_scene.xml).')
    parser.add_argument('--out', default=None, help='write cell_scene.xml into this directory (default: models/)')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('export_cell_scene')
    path = CONST_CELL_SCENE_XML if args.out is None else os.path.join(args.out, os.path.basename(CONST_CELL_SCENE_XML))
    try:
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        written = Export_Cell_Scene(path)
    except Scene_Error_Cls as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    print(log.Line('[INFO] Wrote %s. View it with: .venv/bin/python -m mujoco.viewer --mjcf=%s' % (written, written)))
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
