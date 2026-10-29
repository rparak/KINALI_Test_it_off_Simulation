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
# File Name: ../Example/Cell/run_program.py
## =========================================================================== ##

# System (Default)
import sys
# OS (Operating system interfaces)
import os
# The repository root, built from this file's location, so the script runs from any working directory.
#   Unit [string]
CONST_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..'))
#   Add access if it is not in the system path.
if os.path.join(CONST_REPO_ROOT, 'src') not in sys.path:
    sys.path.append(os.path.join(CONST_REPO_ROOT, 'src'))
# Argparse (Command-line parsing)
import argparse
# Time (Time access)
import time
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Configuration/Core, ../Configuration/Const, ../Configuration/Program_Parser - the cell, the programs, the logs, the folders
from Configuration.Core import (Configure_MuJoCo_Log, Read_PCB_Meta, Load_Cell_Configuration, Script_Log_Cls, Configuration_Error_Cls, CONST_REPORTS,
                                CONST_TRAJECTORIES)
from Configuration.Const import CONST_ROOT
from Configuration.Program_Parser import (Load_Constants, Load_Simulation, Load_Program, Load_Optimization, Free_Steps, CONST_PROGRAM_YAML,
                                          CONST_OPTIMIZED_PROGRAM_YAML)
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model, Scene_Error_Cls
#   ../Collision/Core, ../Optimizer/Core, ../Program/Core - a stale optimized program or trajectory
from Collision.Core import Build_Planning_Model
from Optimizer.Core import Stale_Inputs
from Program.Core import Stale_Trajectory
#   ../Trajectory/Core - the trajectory file
from Trajectory.Core import Read_Trajectory, Trajectory_Error_Cls
#   ../Simulation/Core - the physics run of a trajectory
from Simulation.Core import Trajectory_Drive_Cls, Run_Physics, Simulation_Error_Cls
#   ../Simulation/Report - a run's acceptance and report
from Simulation.Report import Format_Program_Report, Check_Program_Acceptance

"""
Description:
    Initialization of constants.
"""
# The programs by version; the exit codes (0 pass, 1 an acceptance failure, 2 the run cannot run).
#   Unit [Dict<string, string>], [int]
CONST_VERSIONS = {'taught': CONST_PROGRAM_YAML, 'optimized': CONST_OPTIMIZED_PROGRAM_YAML}
CONST_EXIT_PASSED, CONST_EXIT_FAILED, CONST_EXIT_ERROR = 0, 1, 2


class Run_Error_Cls(RuntimeError):
    """
    Description:
        A program's physics run cannot start (a missing or stale file, an invalid configuration); the message says what to do.
    """


def Load_Run(version: str, trajectory_path: tp.Optional[str] = None) -> tp.Tuple[tp.Any, ...]:
    """
    Description:
        What a program's physics run needs: the cell, the PCB poses, the program, its trajectory (refused when the program or
        the trajectory is stale), its free transfers and the compiled cell. Example/Cell/show_physics.py and
        Example/Maintenance/physics_baseline.py import it.

    Args:
        (1) version [string]: 'taught' or 'optimized'.
        (2) trajectory_path [string]: The trajectory file (default Data/Generated/Trajectories/<program>.npz), or None.

    Returns:
        (1) parameter [Tuple]: (cell, pcb_meta, program, trajectory, free_steps, model, trajectory_path).
    """

    path = CONST_VERSIONS[version]
    if not os.path.isfile(path):
        raise Run_Error_Cls('There is no %s; make it with .venv/bin/python Example/Program/optimize_program.py' % os.path.relpath(path, CONST_ROOT))
    try:
        cell, meta, constants = Load_Cell_Configuration(), Read_PCB_Meta(), Load_Constants()
        program = Load_Program(constants, Load_Simulation(constants), cell, path)
        trajectory_path = trajectory_path or os.path.join(CONST_TRAJECTORIES, program.Name + '.npz')
        trajectory = Read_Trajectory(trajectory_path)
        model = Build_Cell_Model(cell)
        free = Free_Steps(program, Load_Optimization(program) if program.Optimized is None else None)
    except (Configuration_Error_Cls, Scene_Error_Cls, Trajectory_Error_Cls) as error:
        raise Run_Error_Cls(str(error))
    stale = Stale_Inputs(program, Build_Planning_Model(cell)) if program.Optimized is not None else []
    if stale:
        raise Run_Error_Cls('%s was optimized from other inputs (%s changed since); make it again: .venv/bin/python '
                            'Example/Program/optimize_program.py' % (os.path.relpath(path, CONST_ROOT), ', '.join(stale)))
    stale = Stale_Trajectory(trajectory, path)
    if stale:
        raise Run_Error_Cls('The trajectory %s was built from other inputs (%s changed since); rebuild it: .venv/bin/python '
                            'Example/Program/build_program.py%s' % (trajectory_path, ', '.join(stale), '' if program.Optimized is None else
                                                                    ' --program ' + os.path.relpath(path, CONST_ROOT)))
    return cell, meta, program, trajectory, free, model, trajectory_path


def main(argv: tp.Optional[tp.List[str]] = None, log_name: tp.Optional[str] = None) -> int:
    """
    Description:
        The physics run of a program's trajectory (F11): the taught program (Configuration/Program.yaml) or the optimized one
        (Data/Optimized/Program_Optimized.yaml), its trajectory from cell.py build or optimize (refused when stale). The
        report (Simulation.Format_Program_Report: SPEC §4.6's lines, the vacuum holds, a row per step, the carrying free
        transfers' slip, the warnings, the acceptance of SPEC §10.1) on stdout and in DIR/<program>_physics.txt. Exit 0 when
        every criterion is met, 1 when one fails (a lost PCB, the vacuum switch not OK included), 2 when the run cannot start or
        becomes unstable.

            Called by Example/cell.py run --program default|optimized.

    Args:
        (1) argv [List<string>]: The arguments (default: the command line).
        (2) log_name [string]: The log to append to (cell.py's logger/cell_<subcommand>.log), or None: logger/run_program.log,
            rewritten.

    Returns:
        (1) parameter [int]: The exit code.
    """

    parser = argparse.ArgumentParser(description='Physics run of a program\'s trajectory: report and acceptance.')
    parser.add_argument('--program', default='taught', choices=list(CONST_VERSIONS), help='the program (default: taught)')
    parser.add_argument('--trajectory', help='the trajectory file (default Data/Generated/Trajectories/<program>.npz)')
    parser.add_argument('--out', default=CONST_REPORTS, help='write <program>_physics.txt into this directory (default: %s)'
                        % os.path.relpath(CONST_REPORTS, CONST_ROOT))
    parser.add_argument('--progress', action='store_true', help='print the simulated time and step every 2 s (stderr)')
    args = parser.parse_args(argv)

    Configure_MuJoCo_Log()
    log = Script_Log_Cls(log_name or 'run_program', append=log_name is not None)
    note = lambda line: print(log.Line(line), file=sys.stderr, flush=True)
    began = time.perf_counter()
    try:
        cell, meta, program, trajectory, free, model, path = Load_Run(args.program, args.trajectory)
        progress = (lambda t, step: note('[INFO] t = %5.1f s  %s' % (t, step))) if args.progress else None
        result = Run_Physics(model, Trajectory_Drive_Cls(trajectory, program, free), meta, cell, progress)
    except (Run_Error_Cls, Simulation_Error_Cls) as error:
        note('[ERROR] %s' % error)
        log.Close()
        return CONST_EXIT_ERROR
    lines = Format_Program_Report(result, cell, program.Name)
    for line in lines:
        print(log.Line(line), flush=True)
    os.makedirs(args.out, exist_ok=True)
    report = os.path.join(os.path.abspath(args.out), program.Name + '_physics.txt')
    with open(report, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    failed = [c for c in Check_Program_Acceptance(result, cell) if not c.Passed]
    note('[%s] %s run from %s in %.1f s; report written to %s; %d acceptance check(s) failed. Simulation results only: not validated '
         'for the real cell.' % ('ERROR' if failed else 'INFO', program.Name, os.path.relpath(path, CONST_ROOT), time.perf_counter() - began,
                                 report, len(failed)))
    log.Close()
    return CONST_EXIT_FAILED if failed else CONST_EXIT_PASSED
