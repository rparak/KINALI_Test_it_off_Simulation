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
# File Name: ../Example/Cell/check_programs.py
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
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Configuration/Core, ../Configuration/Const, ../Configuration/Program_Parser - the cell, the programs, the logs, the folders
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Script_Log_Cls, Configuration_Error_Cls, CONST_TRAJECTORIES
from Configuration.Const import CONST_ROOT
from Configuration.Program_Parser import (Load_Constants, Load_Simulation, Load_Program, Load_Optimization, CONST_PROGRAM_YAML,
                                          CONST_OPTIMIZED_PROGRAM_YAML)
#   ../Scene/Core - the cell's errors
from Scene.Core import Scene_Error_Cls
#   ../Collision/Core, ../Optimizer/Core, ../Program/Core - a stale optimized program or trajectory
from Collision.Core import Build_Planning_Model
from Optimizer.Core import Stale_Inputs
from Program.Core import Stale_Trajectory
#   ../Trajectory/Core - the trajectory file
from Trajectory.Core import Read_Trajectory, Trajectory_Error_Cls


def Optimized_Stale() -> tp.List[str]:
    """
    Description:
        The inputs of the Optimized program that changed since it was made (cell.py all runs the optimizer only then; owner,
        2026-10-29); every one when it does not load (it must be made again).

    Returns:
        (1) parameter [List<string>]: The changed inputs' names; empty when it is current.
    """

    try:
        cell, constants = Load_Cell_Configuration(), Load_Constants()
        optimized = Load_Program(constants, Load_Simulation(constants), cell, CONST_OPTIMIZED_PROGRAM_YAML)
        return Stale_Inputs(optimized, Build_Planning_Model(cell))
    except (Configuration_Error_Cls, Scene_Error_Cls) as error:
        return [str(error)]


def main(argv: tp.Optional[tp.List[str]] = None, log_name: tp.Optional[str] = None) -> int:
    """
    Description:
        The configuration and programs check of Example/cell.py check (owner, 2026-10-29: fast, no build): the cell, both
        programs and the optimizer's settings load (their notes printed); whether the Optimized program and each built
        trajectory are current, each stale or missing one named with the subcommand that makes it again. It writes nothing;
        the kinematic check of a program is its build (cell.py build, cell.py optimize). Exit 1 when a file does not load,
        0 otherwise (a stale or missing output is a warning).

    Args:
        (1) argv [List<string>]: Unused (the check has no options).
        (2) log_name [string]: The log to append to (cell.py's logger/cell_<subcommand>.log), or None: logger/check_programs.log,
            rewritten.

    Returns:
        (1) parameter [int]: The exit code.
    """

    Configure_MuJoCo_Log()
    log = Script_Log_Cls(log_name or 'check_programs', append=log_name is not None)
    try:
        cell, constants = Load_Cell_Configuration(), Load_Constants()
        simulation = Load_Simulation(constants)
        taught = Load_Program(constants, simulation, cell, CONST_PROGRAM_YAML)
        Load_Optimization(taught)
        optimized = Load_Program(constants, simulation, cell, CONST_OPTIMIZED_PROGRAM_YAML)
        model = Build_Planning_Model(cell)
    except (Configuration_Error_Cls, Scene_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    for level, message in cell.Notes + taught.Notes + optimized.Notes:
        print(log.Line('[%s] %s' % (level, message)))
    warnings = 0
    stale = Stale_Inputs(optimized, model)
    if stale:
        warnings += 1
        print(log.Line('[WARNING] %s was optimized from other inputs (%s changed since): run Example/cell.py optimize'
                       % (os.path.relpath(CONST_OPTIMIZED_PROGRAM_YAML, CONST_ROOT), ', '.join(stale))))
    else:
        print(log.Line('[INFO] %s is current (its inputs unchanged).' % os.path.relpath(CONST_OPTIMIZED_PROGRAM_YAML, CONST_ROOT)))
    for program, command in ((taught, 'build'), (optimized, 'optimize')):
        path = os.path.join(CONST_TRAJECTORIES, program.Name + '.npz')
        shown = os.path.relpath(path, CONST_ROOT)
        if not os.path.isfile(path):
            warnings += 1
            print(log.Line('[WARNING] %s is not built yet: run Example/cell.py %s' % (shown, command)))
            continue
        try:
            changed = Stale_Trajectory(Read_Trajectory(path), program.Path)
        except Trajectory_Error_Cls as error:
            changed = [str(error)]
        if changed:
            warnings += 1
            print(log.Line('[WARNING] %s was built from other inputs (%s changed since): run Example/cell.py %s'
                           % (shown, ', '.join(changed), command)))
        else:
            print(log.Line('[INFO] %s is current.' % shown))
    print(log.Line('[INFO] Configuration and programs: they load; %d warning(s). Simulation results only: not validated for the real cell.'
                   % warnings))
    log.Close()
    return 0
