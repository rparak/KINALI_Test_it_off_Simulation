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
# File Name: ../test/Example/Program/test_compare_programs.py
## =========================================================================== ##

# System (Default)
import os
# Matplotlib (Plotting) [pip3 install matplotlib]
import matplotlib.image as mpimg
# Pytest (Testing framework) [pip3 install pytest]
import pytest


@pytest.mark.slow
def test_the_programs_are_compared_in_a_table_and_a_figure(tmp_path_factory, run_script):
    """
    Description:
        From an unrelated directory the script writes the table and the figure into --out: the cycle times (legacy 65.768 s,
        taught 69.396 s, optimized 57.652 s), the per-joint travel, the paths, the loader's travel, the stops, a row per part
        of the process; exit 0, nothing in the working directory.
    """

    out, cwd = tmp_path_factory.mktemp('compare'), tmp_path_factory.mktemp('cwd')
    result = run_script('Program/compare_programs.py', ['--out', str(out)], cwd)
    assert result.returncode == 0, result.stdout + result.stderr
    assert sorted(os.listdir(out)) == ['compare_programs.png', 'compare_programs.txt'] and os.listdir(cwd) == []
    table = open(os.path.join(out, 'compare_programs.txt'), encoding='utf-8').read()
    assert 'cycle time [s]                     65.768       69.396       57.652        83.1%' in table
    assert 'robot stops (>= 0.1 s)                 11           12           12       100.0%' in table
    assert 'robot at rest in them [s]          18.600       21.256       16.796' in table
    assert '  To_MG06 (FREE: S5, S6, S7, S8)' in table and '  Close_MG06 ' in table and '  joint 6 [rad]' in table
    image = mpimg.imread(os.path.join(out, 'compare_programs.png'))
    assert image.shape[1] > 800
