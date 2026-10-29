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
# File Name: ../test/Example/Cell/test_show_program.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Threading (Thread-based parallelism)
import threading
# Types (Dynamic type creation and names for built-in types)
import types
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.:
#   ../Render/Core, ../Program/Check, ../Trajectory/Core, ../Scene/Core, ../Kinematics/Core - the viewer loop and the posing it runs
from Render.Core import Playback_Cls, Run_Viewer_Loop, CONST_FRAME_PERIOD
from Program.Check import Pose_Sample
from Trajectory.Core import Read_Trajectory, CONST_DT
from Scene.Core import Build_Cell_Model
from Kinematics.Core import Robot_Kinematics_Cls


@pytest.fixture(scope='module')
def trajectory(built_trajectory):
    return str(built_trajectory('taught'))


@pytest.mark.slow
def test_headless_posing_at_a_step_and_at_a_time(trajectory, tmp_path, run_script):
    """
    Description:
        --step S11_Test (or 11) poses the cell at the end of the test segment, --time 3.276 at the press (the PCB still in the input
        crate); the step, the time and the command line being executed are printed; exit 0, nothing written to the working directory.
    """

    result = run_script('Cell/show_program.py', ['--trajectory', trajectory, '--step', 'S11_Test', '--headless'], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert ('[INFO] step 11/22 S11_Test   t = 35.640 s / 70.404 s   PCB mg06 (sample 8910): Tester.Run(J_TEST) > Test(Process.Test_Duration)'
            in result.stdout)
    result = run_script('Cell/show_program.py', ['--trajectory', trajectory, '--step', '11', '--headless'], tmp_path)
    assert 'step 11/22 S11_Test' in result.stdout
    result = run_script('Cell/show_program.py', ['--trajectory', trajectory, '--time', '3.276', '--headless'], tmp_path)
    assert ('[INFO] step 2/22 S2_Pick_Input   t = 3.276 s / 70.404 s   PCB input_crate (sample 819): Robot.MoveL(P_INPUT_PICK, v=0.03)'
            in result.stdout)
    assert os.listdir(tmp_path) == []


@pytest.mark.slow
def test_bad_requests_are_error_lines(trajectory, tmp_path, run_script):
    """
    Description:
        An unknown step, a missing trajectory, a trajectory built from other inputs (its input hashes changed; also the
        taught trajectory played as the optimized version) are [ERROR] lines, exit 1.
    """

    result = run_script('Cell/show_program.py', ['--trajectory', trajectory, '--step', 'dance', '--headless'], tmp_path)
    assert result.returncode == 1 and "[ERROR] There is no step 'dance'; the steps are 1-22:" in result.stdout
    result = run_script('Cell/show_program.py', ['--trajectory', str(tmp_path / 'none.npz'), '--headless'], tmp_path)
    assert result.returncode == 1 and 'cannot be read' in result.stdout
    data = dict(np.load(trajectory))
    data['meta'] = np.array(str(data['meta']).replace('"cell": "', '"cell": "0'))
    stale = str(tmp_path / 'stale.npz')
    np.savez(stale, **data)
    result = run_script('Cell/show_program.py', ['--trajectory', stale, '--headless'], tmp_path)
    assert result.returncode == 1 and 'was built from other inputs (cell changed since); rebuild it' in result.stdout
    result = run_script('Cell/show_program.py', ['--version', 'optimized', '--trajectory', trajectory, '--headless'], tmp_path)
    assert result.returncode == 1 and 'was built from other inputs (program changed since); rebuild it' in result.stdout


@pytest.mark.slow
@pytest.mark.skipif(sys.platform != 'darwin', reason='the mjpython requirement exists on macOS only')
def test_the_viewer_under_python_on_macos_names_mjpython(trajectory, tmp_path, run_script):
    """
    Description:
        The playback needs the passive viewer, which macOS runs only under mjpython: under python the script exits 2 with
        the command to use.
    """

    result = run_script('Cell/show_program.py', ['--trajectory', trajectory], tmp_path)
    assert result.returncode == 2
    assert 'On macOS run it under mjpython: .venv/bin/mjpython Example/Program/show_program.py --version default' in result.stdout


@pytest.mark.slow
def test_a_frame_of_the_playback_fits_the_frame_period(trajectory, cell_configuration):
    """
    Description:
        The work of a playback frame on the real cell (posing every sample of 2 s of the default program at 1/60 s frames,
        the overlay text) stays within the frame period: mean below half of it, the 95th percentile below it (measured
        2026-10-24 on an Apple M2 Pro with the real viewer: 4.22 ms mean). The viewer's own sync is not part of it.
    """

    data = Read_Trajectory(trajectory)
    kinematics = Robot_Kinematics_Cls(Build_Cell_Model(cell_configuration))
    frames = iter(range(121))
    viewer = types.SimpleNamespace(is_running=lambda: next(frames) < 120, lock=threading.Lock, set_texts=lambda texts: None, sync=lambda: None)
    stats = Run_Viewer_Loop(viewer, Playback_Cls(data.Step_Index, CONST_DT), lambda i: Pose_Sample(kinematics, data, i),
                            lambda i: ('t = %.3f s' % data.T[i], data.Segment_Command[int(data.Segment_Id[i])], int(data.Segment_Id[i])))
    work = np.asarray(stats.Work)
    assert len(work) == 120
    assert work.mean() < CONST_FRAME_PERIOD / 2 and np.percentile(work, 95) < CONST_FRAME_PERIOD, stats.Summary()


@pytest.mark.slow
def test_the_optimized_version_is_posed(tmp_path_factory, run_script, root):
    """
    Description:
        --version optimized poses the optimized program's trajectory (built with build_program.py --program): its segment
        S5_To_MG06 ends with the loader's park waited for.
    """

    out = tmp_path_factory.mktemp('optimized')
    optimized = os.path.join(root, 'Data', 'Optimized', 'Program_Optimized.yaml')
    run_script('Cell/build_program.py', ['--program', optimized, '--out', str(out)], out)
    result = run_script('Cell/show_program.py', ['--version', 'optimized', '--trajectory', str(out / 'Program_Optimized.npz'), '--step',
                                                    'S5_To_MG06', '--headless'], out)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '[INFO] step 5/16 S5_To_MG06   t = 18.880 s / 59.148 s   PCB mg06 (sample 4720): Loader.WaitDone(J_PARK)' in result.stdout
