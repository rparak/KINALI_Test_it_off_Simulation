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
# File Name: ../test/Example/Cell/test_show_keypoint.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Pytest (Testing framework) [pip3 install pytest]
import pytest


def test_headless_posing_prints_the_keypoint_and_its_context(tmp_path, run_script):
    """
    Description:
        --headless poses the cell at a point, prints its description, joints, TCP and scene (overrides applied) and
        exits 0.
    """

    result = run_script('Cell/show_keypoint.py', ['--name', 'P_MG06_CLOSE_END', '--mg06', 'Closed', '--loader', 'L_HOME',
                                                       '--headless'], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '[INFO] P_MG06_CLOSE_END: End of PATH_FORK_CLOSE' in result.stdout
    assert '[INFO] loader L_HOME, gripper G_OPEN, PCB mg06, MG06 Closed' in result.stdout and os.listdir(tmp_path) == []


def test_an_unknown_keypoint_is_an_error_line(tmp_path, run_script):
    """
    Description:
        An unknown point is an [ERROR] line listing the points, exit 1.
    """

    result = run_script('Cell/show_keypoint.py', ['--name', 'nowhere', '--headless'], tmp_path)
    assert result.returncode == 1 and "[ERROR] There is no point 'nowhere'" in result.stdout


@pytest.mark.skipif(sys.platform != 'darwin', reason='the mjpython requirement exists on macOS only')
def test_the_viewer_under_python_on_macos_names_mjpython(tmp_path, run_script):
    """
    Description:
        The static viewer needs the passive viewer, which macOS runs only under mjpython: under python the script
        exits 2 with the command to use.
    """

    result = run_script('Cell/show_keypoint.py', ['--name', 'P_HOME'], tmp_path)
    assert result.returncode == 2
    assert 'On macOS run it under mjpython: .venv/bin/mjpython Example/Keypoints/show_keypoint.py --name P_HOME' in result.stdout
