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
# File Name: ../test/Example/Keypoints/test_check_keypoints.py
## =========================================================================== ##

# System (Default)
import os


def test_the_shipped_keypoints_pass_and_the_table_lists_them_all(tmp_path, run_script):
    """
    Description:
        From an unrelated directory the check exits 0 with a table row per keypoint (23: SPEC §4.8's 22 and the program's
        input_pcb_clear), the summary 0 FAIL / 1 WARN (via_mg06_high, the family switch) / 22 PASS, and nothing written to the working directory.
    """

    result = run_script('Keypoints/check_keypoints.py', [], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    rows = [line for line in result.stdout.splitlines() if line.split() and line.split()[-1] in ('PASS', 'WARN', 'FAIL')]
    assert len(rows) == 23
    assert '[INFO] 23 keypoint(s): 0 FAIL, 1 WARN, 22 PASS.' in result.stdout
    assert os.listdir(tmp_path) == []


def test_one_keypoint_and_an_unknown_name(tmp_path, run_script):
    """
    Description:
        --name checks one keypoint; an unknown name is an [ERROR] line listing the keypoints, exit 1.
    """

    result = run_script('Keypoints/check_keypoints.py', ['--name', 'fork_closed_end'], tmp_path)
    assert result.returncode == 0 and '1 keypoint(s): 0 FAIL, 0 WARN, 1 PASS' in result.stdout and 'handle_bar -0.33' in result.stdout
    result = run_script('Keypoints/check_keypoints.py', ['--name', 'nowhere'], tmp_path)
    assert result.returncode == 1 and "[ERROR] There is no keypoint 'nowhere'" in result.stdout
