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
# File Name: ../test/Example/Simulation/test_show_physics.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Importlib (Import machinery)
import importlib.util
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    Example/Simulation/show_physics.py: headless (a fresh interpreter, an unrelated working directory) and its physics view
    imported as a module; the viewer itself needs mjpython (checked by hand, FEATURES.md F11).
"""
# The script.
#   Unit [string]
CONST_SCRIPT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', '..', 'Example', 'Simulation', 'show_physics.py')


@pytest.fixture(scope='module')
def taught_trajectory(built_trajectory):
    return built_trajectory('taught')


def test_headless_runs_the_physics_to_a_time_and_renders_it(tmp_path, run_script, taught_trajectory):
    """
    Description:
        --headless --time 3.6: the physics run to 3.6 s (the vacuum switch's wait at the input pick), the state line - step,
        time, 4 of 4 cups, slip, the line -, exit 0; --image writes the render; nothing in the working directory.
    """

    work = tmp_path / 'work'
    work.mkdir()
    image = tmp_path / 'pick.png'
    result = run_script('Simulation/show_physics.py', ['--trajectory', str(taught_trajectory), '--headless', '--time', '3.5', '--image', str(image)],
                        work)
    assert result.returncode == 0, result.stdout + result.stderr
    assert result.stdout.startswith('[INFO] step 2/22 S2_Pick_Input   t = 3.500 s / 70.404 s (physics)   4/4 cups, slip ')
    assert ': Robot.WaitDI(DI_VACUUM_OK, timeout=1.0)\n' in result.stdout and image.stat().st_size > 10000 and os.listdir(work) == []


def test_bad_options_and_the_viewer_without_mjpython_are_refused(tmp_path, run_script, taught_trajectory):
    """
    Description:
        --headless without --time and a speed of 0 exit 1 naming the options; the viewer under python (macOS) exits 2 naming the
        mjpython command.
    """

    result = run_script('Simulation/show_physics.py', ['--trajectory', str(taught_trajectory), '--headless'], tmp_path)
    assert result.returncode == 1 and '[ERROR] --speed must be > 0 and --headless needs --time T.' in result.stdout
    result = run_script('Simulation/show_physics.py', ['--trajectory', str(taught_trajectory), '--speed', '0'], tmp_path)
    assert result.returncode == 1
    if sys.platform == 'darwin':
        result = run_script('Simulation/show_physics.py', ['--trajectory', str(taught_trajectory)], tmp_path)
        assert result.returncode == 2 and '.venv/bin/mjpython Example/Simulation/show_physics.py --program taught' in result.stdout


def test_the_physics_view_steps_within_its_budget_and_copies_the_state(taught_trajectory):
    """
    Description:
        Physics_View_Cls: Prepare steps the run towards a sample's time but no longer than CONST_STEP_BUDGET of wall time (a jump
        to 60 s stops far short of it); Pose copies the run's state into the viewer's data; State names the step and the line.
    """

    spec = importlib.util.spec_from_file_location('show_physics', CONST_SCRIPT)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    run_program = module._Script('Simulation/run_program.py')
    cell, meta, program, trajectory, free, model, _ = run_program.Load_Run('taught', str(taught_trajectory))
    drive = module.Trajectory_Drive_Cls(trajectory, program, free)
    view = module.mujoco.MjData(model)
    physics = module.Physics_View_Cls(module.Physics_Run_Cls(model, drive, meta, cell), drive, view)
    physics.Prepare(int(60.0 / 0.004))
    assert 0.0 < physics.Run.Data.time < 10.0
    physics.Step_To(1.0)
    assert physics.Run.Data.time == pytest.approx(1.0, abs=0.001)
    physics.Pose(0)
    assert np.array_equal(view.qpos, physics.Run.Data.qpos) and view.time == physics.Run.Data.time
    title, comment, _ = physics.State()
    assert title.startswith('step 1/22 S1_Approach_Input   t = 1.000 s') and comment == 'Robot.MoveJ(P_INPUT_ABOVE)'
