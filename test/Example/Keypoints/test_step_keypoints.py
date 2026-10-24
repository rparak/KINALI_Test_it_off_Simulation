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
# File Name: ../test/Example/Keypoints/test_step_keypoints.py
## =========================================================================== ##


def test_headless_stepping_poses_every_keypoint_in_order(tmp_path, run_script):
    """
    Description:
        --headless poses the 24 points once, in the file's order (P_HOME first, P_OUTPUT_PLACE last), exit 0.
    """

    result = run_script('Keypoints/step_keypoints.py', ['--headless'], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    lines = [line for line in result.stdout.splitlines() if line.startswith('[INFO] ')]
    assert len(lines) == 24 and lines[0].startswith('[INFO]  1/24 P_HOME:') and lines[-1].startswith('[INFO] 24/24 P_OUTPUT_PLACE:')


def test_an_unknown_start_is_an_error_line(tmp_path, run_script):
    """
    Description:
        --start with an unknown point is an [ERROR] line, exit 1.
    """

    result = run_script('Keypoints/step_keypoints.py', ['--start', 'nowhere', '--headless'], tmp_path)
    assert result.returncode == 1 and "[ERROR] There is no point 'nowhere'" in result.stdout
