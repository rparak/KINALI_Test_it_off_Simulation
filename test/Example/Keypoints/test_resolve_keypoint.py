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
# File Name: ../test/Example/Keypoints/test_resolve_keypoint.py
## =========================================================================== ##

# System (Default)
import re
# Numpy (Array computing) [pip3 install numpy]
import numpy as np


def test_the_resolved_line_is_printed_close_to_the_shipped_joints(tmp_path, run_script, keypoints):
    """
    Description:
        The resolver prints a Joints line (6 values, 1e-6 deg) for input_pcb_press within 0.01 deg of the shipped
        joints (they differ by the legacy IK's 24 um residue only); it writes nothing.
    """

    result = run_script('Keypoints/resolve_keypoint.py', ['--name', 'input_pcb_press'], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    line = next(l for l in result.stdout.splitlines() if l.strip().startswith('Joints: ['))
    values = [float(v) for v in re.findall(r'-?\d+\.\d{6}', line)]
    assert len(values) == 6
    assert np.max(np.abs(np.radians(values) - keypoints.Robot['input_pcb_press'].Joints)) < np.radians(0.01)


def test_a_keypoint_without_relative_to_is_refused(tmp_path, run_script):
    """
    Description:
        A keypoint defined by its joints only (home) has nothing to resolve: [ERROR], exit 1.
    """

    result = run_script('Keypoints/resolve_keypoint.py', ['--name', 'home'], tmp_path)
    assert result.returncode == 1 and '[ERROR] Keypoint home has no Relative_To target to resolve' in result.stdout
