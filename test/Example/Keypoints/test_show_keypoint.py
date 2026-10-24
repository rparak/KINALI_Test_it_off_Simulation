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
# File Name: ../test/Example/Keypoints/test_show_keypoint.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Matplotlib (Plotting) [pip3 install matplotlib]
import matplotlib.image as mpimg


def test_headless_posing_prints_the_keypoint_and_its_context(tmp_path, run_script):
    """
    Description:
        --headless poses the cell at a keypoint, prints its note, joints, TCP and context (overrides applied) and
        exits 0.
    """

    result = run_script('Keypoints/show_keypoint.py', ['--name', 'fork_closed_end', '--mg06', 'Closed', '--loader', 'home', '--headless'], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '[INFO] fork_closed_end: End of path fork_close' in result.stdout
    assert '[INFO] loader home, gripper open, PCB mg06, MG06 Closed' in result.stdout and os.listdir(tmp_path) == []


def test_the_image_is_written(tmp_path, run_script):
    """
    Description:
        --image FILE writes the offscreen render of the posed cell (960 x 720).
    """

    path = tmp_path / 'home.png'
    result = run_script('Keypoints/show_keypoint.py', ['--name', 'home', '--image', str(path)], tmp_path)
    assert result.returncode == 0 and mpimg.imread(str(path)).shape[:2] == (720, 960)


def test_an_unknown_keypoint_is_an_error_line(tmp_path, run_script):
    """
    Description:
        An unknown keypoint is an [ERROR] line listing the keypoints, exit 1.
    """

    result = run_script('Keypoints/show_keypoint.py', ['--name', 'nowhere', '--headless'], tmp_path)
    assert result.returncode == 1 and "[ERROR] There is no keypoint 'nowhere'" in result.stdout


@pytest.mark.skipif(sys.platform != 'darwin', reason='the mjpython requirement exists on macOS only')
def test_the_viewer_under_python_on_macos_names_mjpython(tmp_path, run_script):
    """
    Description:
        The static viewer needs the passive viewer, which macOS runs only under mjpython: under python the script
        exits 2 with the command to use.
    """

    result = run_script('Keypoints/show_keypoint.py', ['--name', 'home'], tmp_path)
    assert result.returncode == 2
    assert 'On macOS run it under mjpython: .venv/bin/mjpython Example/Keypoints/show_keypoint.py --name home' in result.stdout
