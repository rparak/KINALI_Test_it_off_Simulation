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
# File Name: ../test/conftest.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Importlib (Import machinery)
import importlib.util
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    Shared fixtures. src/ is put on sys.path here (there are no __init__.py files in the RPX layout), and
    MuJoCo's log is routed to logger/, so no MUJOCO_LOG.TXT lands in a test's working directory.
"""
# The repository root.
#   Unit [string]
CONST_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if os.path.join(CONST_ROOT, 'src') not in sys.path:
    sys.path.insert(0, os.path.join(CONST_ROOT, 'src'))
from Configuration.Core import Configure_MuJoCo_Log  # noqa: E402
Configure_MuJoCo_Log()


@pytest.fixture(scope='session')
def root():
    """
    Description:
        The repository root.
    """

    return CONST_ROOT


@pytest.fixture(scope='session')
def cell_configuration():
    """
    Description:
        The shipped cell configuration (Configuration/Cell.yaml).
    """

    from Configuration.Core import Load_Cell_Configuration
    return Load_Cell_Configuration()


@pytest.fixture(scope='session')
def constants():
    """
    Description:
        The shipped constants (Configuration/Constants.yaml).
    """

    from Configuration.Program_Parser import Load_Constants
    return Load_Constants()


@pytest.fixture(scope='session')
def simulation(constants):
    """
    Description:
        The shipped simulation data (Configuration/Simulation.yaml).
    """

    from Configuration.Program_Parser import Load_Simulation
    return Load_Simulation(constants)


@pytest.fixture(scope='session')
def program(constants, simulation, cell_configuration):
    """
    Description:
        The shipped program (Configuration/Program.yaml) with its constants and simulation data.
    """

    from Configuration.Program_Parser import Load_Program
    return Load_Program(constants, simulation, cell_configuration)


@pytest.fixture(scope='session')
def cell_model(cell_configuration):
    """
    Description:
        The compiled cell (built once per session).
    """

    from Scene.Core import Build_Cell_Model
    return Build_Cell_Model(cell_configuration)


@pytest.fixture(scope='session')
def program_tools(cell_configuration):
    """
    Description:
        The taught program's build and check tools on the planning model: (kinematics, PCB poses, collision).
    """

    from Configuration.Core import Read_PCB_Meta
    from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
    from Kinematics.Core import Robot_Kinematics_Cls
    model = Build_Planning_Model(cell_configuration)
    collision = Robot_Collision_Cls(model)
    return Robot_Kinematics_Cls(model, collision.Data), Read_PCB_Meta(), collision


@pytest.fixture(scope='session')
def program_built(program, cell_configuration, program_tools):
    """
    Description:
        The taught program built in process (Build_Program): (trajectory, notes).
    """

    from Program.Core import Build_Program
    kinematics, meta, _ = program_tools
    return Build_Program(program, cell_configuration, meta, kinematics, {})


@pytest.fixture(scope='session')
def program_checked(program_built, program, cell_configuration, program_tools):
    """
    Description:
        The taught program's check (Check_Trajectory), its FREE steps as the optimizer reads them.
    """

    from Configuration.Program_Parser import Load_Optimization, Free_Steps
    from Program.Check import Check_Trajectory
    kinematics, meta, collision = program_tools
    free = Free_Steps(program, Load_Optimization(program))
    return Check_Trajectory(program_built[0], program, program_built[1], kinematics, collision, cell_configuration, meta, free)


def _Probe_Program(lines, constants, simulation, cell, start='P_HOME', pcb='input_crate', allow=(), loader_jobs=None, tester_jobs=None):
    """
    Description:
        A one-segment program of the given robot lines, from a start point (loader home, gripper open, vacuum off, no end settle).
    """

    import dataclasses
    from Configuration.Program_Parser import Program_Str, Segment_Str, Parse_Commands
    probe_simulation = dataclasses.replace(simulation, Start_Robot=start, Start_PCB=pcb, End_Settle=0.0)
    commands = Parse_Commands(lines, 'S1_Probe', constants, probe_simulation, cell, loader_jobs, tester_jobs)
    return Program_Str(Name='probe', Segments=[Segment_Str(Key='S1_Probe', Name='Probe', Number=1, Commands=commands, Allow_Contact=list(allow))],
                       Constants=constants, Simulation=probe_simulation)


def _Check_Program(program, cell, program_tools):
    """
    Description:
        Build and check a program.
    """

    from Program.Core import Build_Program
    from Program.Check import Check_Trajectory
    kinematics, meta, collision = program_tools
    trajectory, notes = Build_Program(program, cell, meta, kinematics, {})
    return Check_Trajectory(trajectory, program, notes, kinematics, collision, cell, meta)


@pytest.fixture(scope='session')
def optimized_built(cell_configuration, program, constants, simulation):
    """
    Description:
        Both programs built in process on the planning model, with what the optimizer's checks need: the model, collision,
        kinematics, PCB poses, the taught trajectory ('default'), the optimized program and its trajectory and notes, the
        optimization settings.
    """

    from Configuration.Core import Read_PCB_Meta
    from Configuration.Program_Parser import Load_Program, Load_Optimization, CONST_OPTIMIZED_PROGRAM_YAML
    from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
    from Kinematics.Core import Robot_Kinematics_Cls
    from Program.Core import Build_Program
    model = Build_Planning_Model(cell_configuration)
    collision = Robot_Collision_Cls(model)
    kinematics = Robot_Kinematics_Cls(model, collision.Data)
    meta = Read_PCB_Meta()
    default, _ = Build_Program(program, cell_configuration, meta, kinematics, {})
    optimized = Load_Program(constants, simulation, cell_configuration, CONST_OPTIMIZED_PROGRAM_YAML)
    trajectory, notes = Build_Program(optimized, cell_configuration, meta, kinematics, {})
    return {'model': model, 'collision': collision, 'kinematics': kinematics, 'meta': meta, 'default': default, 'optimized': optimized,
            'trajectory': trajectory, 'notes': notes, 'optimization': Load_Optimization(program)}


CONST_TIMED_TAKE = """    - Loader.Start(J_RETURN_Y)
    - Wait(1.648)
    - Robot.MoveSpline(W_TO_TAKE, v=0.6)
    - Loader.WaitDone(J_RETURN_Y)
    - Loader.Start(J_RETURN_Z)
    - Loader.WaitDone(J_RETURN_Z)
"""


@pytest.fixture(scope='session')
def timed_take(optimized_built, cell_configuration, constants, simulation, tmp_path_factory):
    """
    Description:
        The forced failure of F15: the optimized program with its take gated by the old timer - W_TO_TAKE after
        Loader.Start(J_RETURN_Y) and a Wait(1.648), as F14's program ran it - built in process: (trajectory, program).
    """

    from Configuration.Program_Parser import Load_Program, CONST_OPTIMIZED_PROGRAM_YAML
    from Program.Core import Build_Program
    text = open(CONST_OPTIMIZED_PROGRAM_YAML, encoding='utf-8').read()
    gated = """    - Loader.Start(J_RETURN_Y)
    - Loader.WaitDone(J_RETURN_Y)
    - Loader.Start(J_RETURN_Z)
    - Robot.MoveSpline(W_TO_TAKE, v=0.6)
    - Loader.WaitDone(J_RETURN_Z)
"""
    assert text.count(gated) == 1
    path = tmp_path_factory.mktemp('timed_take') / 'Program_Optimized.yaml'
    path.write_text(text.replace(gated, CONST_TIMED_TAKE), encoding='utf-8')
    program = Load_Program(constants, simulation, cell_configuration, str(path))
    trajectory, _ = Build_Program(program, cell_configuration, optimized_built['meta'], optimized_built['kinematics'], {})
    return trajectory, program


CONST_EXIT_INSERT = """    - Robot.MoveSpline(W_TO_MG06_EXIT, v=0.6)
    - Loader.Start(J_INSERT)
"""


@pytest.fixture(scope='session')
def prompt_insert(optimized_built, cell_configuration, constants, simulation, tmp_path_factory):
    """
    Description:
        The forced failure of F16: the optimized program with J_INSERT started before the robot leaves the handover - its
        Loader.Start before the leg to the exit point, as F15's program started it with W_TO_MG06 - built in process: (trajectory,
        program).
    """

    from Configuration.Program_Parser import Load_Program, CONST_OPTIMIZED_PROGRAM_YAML
    from Program.Core import Build_Program
    text = open(CONST_OPTIMIZED_PROGRAM_YAML, encoding='utf-8').read()
    assert text.count(CONST_EXIT_INSERT) == 1
    lines = CONST_EXIT_INSERT.split('\n')
    path = tmp_path_factory.mktemp('prompt_insert') / 'Program_Optimized.yaml'
    path.write_text(text.replace(CONST_EXIT_INSERT, '\n'.join([lines[1], lines[0], ''])), encoding='utf-8')
    program = Load_Program(constants, simulation, cell_configuration, str(path))
    trajectory, _ = Build_Program(program, cell_configuration, optimized_built['meta'], optimized_built['kinematics'], {})
    return trajectory, program


def _Failed_Criteria(result):
    """
    Description:
        The names of the SPEC §10.1 criteria a run's result fails (Check_Acceptance on the shipped cell).
    """

    from Configuration.Core import Load_Cell_Configuration
    from Simulation.Report import Check_Acceptance
    return [c.Name for c in Check_Acceptance(result, Load_Cell_Configuration()) if not c.Passed]


@pytest.fixture(scope='session')
def failed_criteria():
    """
    Description:
        The failed criteria of a run's result (_Failed_Criteria).
    """

    return _Failed_Criteria


@pytest.fixture(scope='session')
def probe_program():
    """
    Description:
        The builder of a one-segment probe program (_Probe_Program).
    """

    return _Probe_Program


@pytest.fixture(scope='session')
def check_program():
    """
    Description:
        Build and check a program (_Check_Program).
    """

    return _Check_Program


def _Run_Script(script, arguments, cwd, timeout=600):
    """
    Description:
        Run Example/<Domain>/<script> the way a person does: a fresh interpreter, from cwd (an unrelated directory).
    """

    import subprocess
    return subprocess.run([sys.executable, os.path.join(CONST_ROOT, 'Example', script)] + list(arguments), cwd=str(cwd),
                          capture_output=True, text=True, timeout=timeout)


@pytest.fixture(scope='session')
def built_trajectory(tmp_path_factory):
    """
    Description:
        A program's trajectory built by Example/Program/build_program.py, each program at most once per session (per pytest-xdist
        worker), into one folder: built_trajectory('taught' | 'optimized') -> the .npz path. Read only: a test that needs a
        changed trajectory writes a copy.
    """

    folder, made = tmp_path_factory.mktemp('trajectories'), {}

    def _Build(program):
        if program not in made:
            arguments = ['--out', str(folder)]
            if program == 'optimized':
                arguments += ['--program', os.path.join(CONST_ROOT, 'Data', 'Optimized', 'Program_Optimized.yaml')]
            result = _Run_Script('Program/build_program.py', arguments, folder)
            assert result.returncode == 0, result.stdout[-2000:] + result.stderr
            made[program] = folder / ('Program.npz' if program == 'taught' else 'Program_Optimized.npz')
        return made[program]
    return _Build


@pytest.fixture(scope='session')
def run_script():
    """
    Description:
        The runner of Example/ scripts (_Run_Script).
    """

    return _Run_Script


@pytest.fixture(scope='session')
def fingerprint_tool():
    """
    Description:
        The model fingerprint script imported as a module (a script, not part of src/).
    """

    path = os.path.join(CONST_ROOT, 'Example', 'Maintenance', 'model_fingerprint.py')
    spec = importlib.util.spec_from_file_location('model_fingerprint', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module
