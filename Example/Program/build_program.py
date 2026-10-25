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
# File Name: ../Example/Program/build_program.py
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
# Platform (Access to the platform's identifying data)
import platform
# Subprocess (Subprocess management)
import subprocess
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core - the configuration, the program, the logs, the folders
from Configuration.Core import (Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls, Configuration_Error_Cls,
                                CONST_CELL_YAML, CONST_FORK_PATH, CONST_TRAJECTORIES, CONST_REPORTS)
#   ../Configuration/Program_Parser - the program's three files
from Configuration.Program_Parser import (Load_Constants, Load_Simulation, Load_Program, Load_Optimization, Free_Steps, CONST_CONSTANTS_YAML,
                                          CONST_SIMULATION_YAML, CONST_PROGRAM_YAML, CONST_OPTIMIZATION_YAML)
#   ../Scene/Core - the cell's errors
from Scene.Core import Scene_Error_Cls
#   ../Kinematics/Core, ../Collision/Core - FK, IK and exact distances on the planning model (the table included)
from Kinematics.Core import Robot_Kinematics_Cls
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
#   ../Program/Core, ../Program/Check - the build and the check
from Program.Core import Build_Program, File_Hash, Program_Error_Cls
from Program.Check import Check_Trajectory, Format_Check_Report
#   ../Trajectory/Core - the trajectory file
from Trajectory.Core import Write_Trajectory
#   ../Optimizer/Core - an optimized program's inputs
from Optimizer.Core import Stale_Inputs
#   ../Optimizer/Measure - the schedule's differences
from Optimizer.Measure import Schedule_Differences


def Git_State() -> tp.Tuple[str, bool]:
    """
    Description:
        The repository's commit and whether its working tree has changes (for the trajectory's metadata).

    Returns:
        (1) parameter [string]: The commit hash, or 'unknown' outside git.
        (2) parameter [bool]: True when the working tree differs from the commit.
    """

    try:
        commit = subprocess.run(['git', '-C', CONST_REPO_ROOT, 'rev-parse', 'HEAD'], capture_output=True, text=True, check=True).stdout.strip()
        status = subprocess.run(['git', '-C', CONST_REPO_ROOT, 'status', '--porcelain'], capture_output=True, text=True, check=True)
        dirty = bool(status.stdout.strip())
        return commit, dirty
    except (OSError, subprocess.CalledProcessError):
        return 'unknown', True


def Build_Meta(program_path: str, optimized: bool = False) -> tp.Dict[str, tp.Any]:
    """
    Description:
        The trajectory's metadata: the sha256 of every input file (an optimized program's also Configuration/
        Optimization.yaml's), the git commit, the platform and the versions (B5: byte-identical outputs are promised on one
        machine and platform only).

    Args:
        (1) program_path [string]: The program file.
        (2) optimized [bool]: Whether it is an optimized program.

    Returns:
        (1) parameter [Dict<string, Any>]: The metadata.
    """

    commit, dirty = Git_State()
    inputs = {name: File_Hash(path) for name, path in (('program', program_path), ('constants', CONST_CONSTANTS_YAML),
                                                        ('simulation', CONST_SIMULATION_YAML), ('cell', CONST_CELL_YAML), ('fork_path',
                                                                                                                           CONST_FORK_PATH))}
    if optimized:
        inputs['optimization'] = File_Hash(CONST_OPTIMIZATION_YAML)
    return {'inputs_sha256': inputs, 'git_commit': commit, 'git_dirty': dirty,
            'platform': {'system': platform.system(), 'machine': platform.machine(), 'python': platform.python_version(),
                         'numpy': np.__version__, 'mujoco': mujoco.__version__}}


def main():
    """
    Description:
        Build a program into its trajectory (Data/Generated/Trajectories/<program>.npz and .csv) and check it (the report:
        stdout and Data/Generated/Reports/<program>_check.txt); exit 0 when the check passes, 1 when it fails or the
        program cannot be built. (The diagnostic against the legacy plan, --compare-legacy, was retired in step 6: commit
        7a8560c8c16806739a02da8e989c2a2f65d1e41d.)

            .venv/bin/python Example/Program/build_program.py [--program FILE] [--out DIR]
    """

    parser = argparse.ArgumentParser(description='Build and check a program (default: Configuration/Program.yaml).')
    parser.add_argument('--program', default=CONST_PROGRAM_YAML, help='the program file (with Configuration/Constants.yaml and Simulation.yaml)')
    parser.add_argument('--out', help='the folder for the trajectory and the report (default: Data/Generated/Trajectories and Reports)')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('build_program')
    try:
        cell, meta = Load_Cell_Configuration(), Read_PCB_Meta()
        constants = Load_Constants()
        program = Load_Program(constants, Load_Simulation(constants), cell, args.program)
        for level, message in cell.Notes + program.Notes:
            print(log.Line('[%s] %s' % (level, message)))
        model = Build_Planning_Model(cell)
        collision = Robot_Collision_Cls(model)
        kinematics = Robot_Kinematics_Cls(model, collision.Data)
        stale = Stale_Inputs(program, model)
        if stale:
            print(log.Line('[ERROR] %s was optimized from other inputs (%s changed since); make it again: .venv/bin/python '
                           'Example/Program/optimize_program.py' % (args.program, ', '.join(stale))))
            log.Close()
            return 1
        trajectory, notes = Build_Program(program, cell, meta, kinematics, Build_Meta(args.program, program.Optimized is not None))
        # The free transfers (Clearance.Loader_Free): the optimized program's splines, the taught one's Optimization.yaml list.
        free = Free_Steps(program, Load_Optimization(program) if program.Optimized is None else None)
        differences = Schedule_Differences(program, trajectory)
        if differences:
            print(log.Line('[ERROR] The build of %s does not match its recorded Schedule (another platform, or the file was edited): %s; make '
                           'it again: .venv/bin/python Example/Program/optimize_program.py' % (args.program, '; '.join(differences))))
            log.Close()
            return 1
    except (Configuration_Error_Cls, Scene_Error_Cls, Program_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    npz, csv = Write_Trajectory(trajectory, os.path.join(args.out or CONST_TRAJECTORIES, program.Name + '.npz'))
    result = Check_Trajectory(trajectory, program, notes, kinematics, collision, cell, meta, free)
    lines = Format_Check_Report(result, trajectory, cell)
    report = os.path.join(args.out or CONST_REPORTS, program.Name + '_check.txt')
    os.makedirs(os.path.dirname(report), exist_ok=True)
    with open(report, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    for line in lines:
        print(log.Line(line))
    print(log.Line('[INFO] Wrote %s, %s and %s.' % (npz, csv, report)))
    print(log.Line('[%s] %s: the check %s.' % ('ERROR' if result.Verdict == 'FAIL' else 'INFO', program.Name,
                                               'failed' if result.Verdict == 'FAIL' else 'passed')))
    log.Close()
    return 1 if result.Verdict == 'FAIL' else 0


if __name__ == '__main__':
    sys.exit(main())
