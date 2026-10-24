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
# File Name: ../test/Example/Render/test_show_cell.py
## =========================================================================== ##

# System (Default)
import os


def test_the_viewer_script_builds_the_cell_headless(root, tmp_path, run_script):
    """
    Description:
        --headless builds the cell, names its size in an [INFO] line, exits 0, writes its log to logger/ and
        nothing to the working directory (the window itself is not opened in the tests).
    """

    result = run_script('Render/show_cell.py', ['--headless'], tmp_path)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == '[INFO] Cell built: 25 bodies, 509 geoms, 13 actuators.'
    with open(os.path.join(root, 'logger', 'show_cell.log'), encoding='utf-8') as handle:
        assert handle.read().strip() == result.stdout.strip()
    assert os.listdir(tmp_path) == []
