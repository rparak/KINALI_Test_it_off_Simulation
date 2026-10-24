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
# File Name: ../test/Example/Program/test_show_program.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Matplotlib (Plotting) [pip3 install matplotlib]
import matplotlib.image as mpimg
# Pytest (Testing framework) [pip3 install pytest]
import pytest


@pytest.fixture(scope='module')
def trajectory(tmp_path_factory, run_script):
    out = tmp_path_factory.mktemp('built')
    run_script('Program/build_program.py', ['--out', str(out)], tmp_path_factory.mktemp('cwd'))
    return str(out / 'Program_Default.npz')


@pytest.mark.slow
def test_headless_posing_at_a_step_and_at_a_time(trajectory, tmp_path, run_script):
    """
    Description:
        --step test (or 11) poses the cell at the end of the test step, --time 3.276 at the press (the PCB still in the input
        crate); the step, the time and the comment are printed; exit 0, nothing written to the working directory.
    """

    result = run_script('Program/show_program.py', ['--trajectory', trajectory, '--step', 'test', '--headless'], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '[INFO] step 11/22 test   t = 35.136 s / 68.852 s   PCB mg06 (sample 8784): MG06 closed' in result.stdout
    result = run_script('Program/show_program.py', ['--trajectory', trajectory, '--step', '11', '--headless'], tmp_path)
    assert 'step 11/22 test' in result.stdout
    result = run_script('Program/show_program.py', ['--trajectory', trajectory, '--time', '3.276', '--headless'], tmp_path)
    assert '[INFO] step 2/22 pick_input   t = 3.276 s / 68.852 s   PCB input_crate (sample 819)' in result.stdout and os.listdir(tmp_path) == []


@pytest.mark.slow
def test_the_image_is_written(trajectory, tmp_path, run_script):
    """
    Description:
        --image FILE writes the offscreen render of the posed cell (960 x 720).
    """

    path = tmp_path / 'moment.png'
    result = run_script('Program/show_program.py', ['--trajectory', trajectory, '--step', 'handover', '--image', str(path)], tmp_path)
    assert result.returncode == 0 and mpimg.imread(str(path)).shape[:2] == (720, 960)


@pytest.mark.slow
def test_bad_requests_are_error_lines(trajectory, tmp_path, run_script):
    """
    Description:
        An unknown step, a missing trajectory, a trajectory built from other inputs (its input hashes changed) and the
        optimized version (F9) are [ERROR] lines, exit 1.
    """

    result = run_script('Program/show_program.py', ['--trajectory', trajectory, '--step', 'dance', '--headless'], tmp_path)
    assert result.returncode == 1 and "[ERROR] There is no step 'dance'; the steps are 1-22:" in result.stdout
    result = run_script('Program/show_program.py', ['--trajectory', str(tmp_path / 'none.npz'), '--headless'], tmp_path)
    assert result.returncode == 1 and 'cannot be read' in result.stdout
    data = dict(np.load(trajectory))
    data['meta'] = np.array(str(data['meta']).replace('"cell": "', '"cell": "0'))
    stale = str(tmp_path / 'stale.npz')
    np.savez(stale, **data)
    result = run_script('Program/show_program.py', ['--trajectory', stale, '--headless'], tmp_path)
    assert result.returncode == 1 and 'was built from other inputs (cell changed since); rebuild it' in result.stdout
    result = run_script('Program/show_program.py', ['--version', 'optimized', '--headless'], tmp_path)
    assert result.returncode == 1 and '[ERROR] The optimized program comes with F9' in result.stdout


@pytest.mark.slow
@pytest.mark.skipif(sys.platform != 'darwin', reason='the mjpython requirement exists on macOS only')
def test_the_viewer_under_python_on_macos_names_mjpython(trajectory, tmp_path, run_script):
    """
    Description:
        The playback needs the passive viewer, which macOS runs only under mjpython: under python the script exits 2 with
        the command to use.
    """

    result = run_script('Program/show_program.py', ['--trajectory', trajectory], tmp_path)
    assert result.returncode == 2
    assert 'On macOS run it under mjpython: .venv/bin/mjpython Example/Program/show_program.py --version default' in result.stdout
