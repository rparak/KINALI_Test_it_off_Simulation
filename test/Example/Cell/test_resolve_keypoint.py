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
# File Name: ../test/Example/Cell/test_resolve_keypoint.py
## =========================================================================== ##

# System (Default)
import os
import re
# Numpy (Array computing) [pip3 install numpy]
import numpy as np


def test_the_resolved_lines_are_printed_close_to_the_shipped_point(tmp_path, run_script, constants):
    """
    Description:
        The resolver prints the Joints line (6 values, 1e-6 deg) of P_INPUT_ABOVE from its Reference within 0.01 deg of the
        shipped joints (they differ by the legacy IK's residue, 0.024 mm), and its Pose line (6 values, 9 decimals) within
        0.05 mm of the shipped Pose; it writes nothing.
    """

    result = run_script('Cell/resolve_keypoint.py', ['--name', 'P_INPUT_ABOVE'], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    joints = next(l for l in result.stdout.splitlines() if l.strip().startswith('Joints: ['))
    values = [float(v) for v in re.findall(r'-?\d+\.\d{6}', joints)]
    assert len(values) == 6 and np.max(np.abs(np.radians(values) - constants.Points['P_INPUT_ABOVE'].Joints)) < np.radians(0.01)
    pose = next(l for l in result.stdout.splitlines() if l.strip().startswith('Pose: ['))
    values = [float(v) for v in re.findall(r'-?\d+\.\d{9}', pose)]
    assert len(values) == 6 and np.max(np.abs(np.array(values[:3]) - constants.Points['P_INPUT_ABOVE'].Pose[:3])) < 0.00005
    assert os.listdir(tmp_path) == []


def test_a_point_without_a_target_is_refused(tmp_path, run_script):
    """
    Description:
        A point defined by its joints only (P_HOME: no Reference, no PCB_Hold) has nothing to resolve: [ERROR], exit 1.
    """

    result = run_script('Cell/resolve_keypoint.py', ['--name', 'P_HOME'], tmp_path)
    assert result.returncode == 1 and '[ERROR] Point P_HOME has no Reference or PCB_Hold target to resolve' in result.stdout
