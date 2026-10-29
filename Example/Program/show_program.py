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
# File Name: ../Example/Program/show_program.py
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
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Configuration/Core - the configuration, the program, the logs, the folders
from Configuration.Core import (Configure_MuJoCo_Log, Load_Cell_Configuration, Script_Log_Cls, Configuration_Error_Cls, CONST_TRAJECTORIES)
#   ../Configuration/Program_Parser - the program's three files, the PCB locations
from Configuration.Program_Parser import (CONST_PCB_LOCATIONS, Load_Constants, Load_Simulation, Load_Program, CONST_PROGRAM_YAML,
                                          CONST_OPTIMIZED_PROGRAM_YAML)
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model, Scene_Error_Cls
#   ../Kinematics/Core - the model's robot joints
from Kinematics.Core import Robot_Kinematics_Cls
#   ../Program/Core, ../Program/Check - a stale trajectory, posing at a sample
from Program.Core import Stale_Trajectory
from Program.Check import Pose_Sample
#   ../Trajectory/Core - the trajectory file
from Trajectory.Core import Read_Trajectory, Trajectory_Error_Cls, Trajectory_Str, CONST_DT
#   ../Collision/Core, ../Optimizer/Core - an optimized program's inputs (its planning model's hash)
from Collision.Core import Build_Planning_Model
from Optimizer.Core import Stale_Inputs as Stale_Optimized_Inputs
#   ../Render/Core - the viewer
from Render.Core import (Launch_Passive_Viewer, Render_Error_Cls, Playback_Cls, Run_Viewer_Loop,
                         Watchdog_Cls)

"""
Description:
    Initialization of constants.
"""
# The program file of each version: the taught one and the optimized one (F9, written by Example/Program/optimize_program.py).
#   Unit [Dict<string, string>]
CONST_VERSIONS = {'default': CONST_PROGRAM_YAML, 'optimized': CONST_OPTIMIZED_PROGRAM_YAML}
# The keys (GLFW codes) and their playback actions: Home pause / resume, Page Down or '.' the next step's start, Page Up the
# current (or previous) step's start, End the current step's end. Every letter and , / ; ' \\ ` toggles a MuJoCo flag and the
# viewer binds Space, the arrows, Tab, Esc, [ ], - =, F1-F5, 0-5; these are in none of them (FEATURES.md F6).
#   Unit [Dict<int, string>]
CONST_KEY_ACTIONS = {268: 'pause', 267: 'next', 46: 'next', 266: 'previous', 269: 'end'}


def Sample_Of(trajectory: Trajectory_Str, step: tp.Optional[str], at: tp.Optional[float]) -> int:
    """
    Description:
        The sample to pose: the end of a step (its number from 1, or its name), or the one nearest a time, or the first.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) step [string]: The step's number or name, or None.
        (3) at [float]: The time (s), or None.

    Returns:
        (1) parameter [int]: The sample.
    """

    if step is not None:
        index = int(step) - 1 if step.isdigit() else trajectory.Step_Names.index(step)
        if not 0 <= index < len(trajectory.Step_Names):
            raise ValueError
        return int(np.flatnonzero(trajectory.Step_Index == index)[-1])
    if at is not None:
        return int(np.clip(round(at / CONST_DT), 0, len(trajectory.T) - 1))
    return 0


def main():
    """
    Description:
        Play a program's trajectory in MuJoCo's viewer: the whole cell posed from the trajectory sample by sample in real
        time (kinematic, no physics), the segment's name and the command line being executed shown; Home pauses / resumes,
        Page Down or '.' jumps to the
        next step, Page Up back to the current step's start, End to its end. --step N / --time T pose one moment; --headless
        prints it. Reads the trajectory Example/Program/build_program.py wrote and refuses
        one whose inputs changed since. The viewer needs mjpython on macOS (the passive viewer); it follows the viewer rules
        of Render.Run_Viewer_Loop (the lock only for posing, the overlay outside it) and prints its frame times when closed;
        --watchdog S dumps every thread's traceback and exits 3 when the loop stalls for S seconds.

            .venv/bin/mjpython Example/Program/show_program.py [--version default] [--step N | --time T] [--watchdog S]
            .venv/bin/python Example/Program/show_program.py [--step N | --time T] --headless
    """

    parser = argparse.ArgumentParser(description='Play or pose a program trajectory (kinematic).')
    parser.add_argument('--version', default='default', choices=['default', 'optimized'], help='the program version (optimized: F9)')
    parser.add_argument('--trajectory', help='the trajectory file (default Data/Generated/Trajectories/<program>.npz)')
    parser.add_argument('--step', help='pose the cell at the end of this step (its number from 1 or its name)')
    parser.add_argument('--time', type=float, help='pose the cell at this time [s]')
    parser.add_argument('--headless', action='store_true', help='pose and print, no window')
    parser.add_argument('--watchdog', type=float, metavar='S', help='debug: when the viewer loop stalls for S seconds, print every '
                        "thread's traceback and exit 3")
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('show_program')
    program_path = CONST_VERSIONS[args.version]
    if not os.path.isfile(program_path):
        print(log.Line('[ERROR] There is no %s; make it with .venv/bin/python Example/Program/optimize_program.py' % program_path))
        log.Close()
        return 1
    try:
        cell, constants = Load_Cell_Configuration(), Load_Constants()
        program = Load_Program(constants, Load_Simulation(constants), cell, program_path)
        path = args.trajectory or os.path.join(CONST_TRAJECTORIES, program.Name + '.npz')
        trajectory = Read_Trajectory(path)
        model = Build_Cell_Model(cell)
    except (Configuration_Error_Cls, Scene_Error_Cls, Trajectory_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    stale = Stale_Trajectory(trajectory, program_path)
    optimized_stale = Stale_Optimized_Inputs(program, Build_Planning_Model(cell)) if program.Optimized is not None else []
    if optimized_stale:
        print(log.Line('[ERROR] %s was optimized from other inputs (%s changed since); make it again: .venv/bin/python '
                       'Example/Program/optimize_program.py' % (program_path, ', '.join(optimized_stale))))
        log.Close()
        return 1
    if stale:
        print(log.Line('[ERROR] The trajectory %s was built from other inputs (%s changed since); rebuild it: '
                       '.venv/bin/python Example/Program/build_program.py%s' % (path, ', '.join(stale), '' if program.Optimized is None
                                                                                 else ' --program ' + os.path.relpath(program_path,
                                                                                                                      CONST_REPO_ROOT))))
        log.Close()
        return 1
    try:
        sample = Sample_Of(trajectory, args.step, args.time)
    except ValueError:
        print(log.Line("[ERROR] There is no step '%s'; the steps are 1-%d: %s."
                       % (args.step, len(trajectory.Step_Names), ', '.join(trajectory.Step_Names))))
        log.Close()
        return 1
    kinematics = Robot_Kinematics_Cls(model)

    def Describe(i: int) -> tp.Tuple[str, str]:
        """
        Description:
            The segment and the command line of sample i, as the viewer's two text columns and the printed line.

        Args:
            (1) i [int]: The sample.

        Returns:
            (1) parameter [string]: 'step k/n S<k>_<Name>', the time and the PCB's location.
            (2) parameter [string]: The command line being executed.
        """

        k = int(trajectory.Step_Index[i])
        return ('step %d/%d %s   t = %.3f s / %.3f s   PCB %s' % (k + 1, len(trajectory.Step_Names), trajectory.Step_Names[k], trajectory.T[i],
                                                                trajectory.T[-1], CONST_PCB_LOCATIONS[trajectory.PCB_Location[i]]),
                trajectory.Segment_Command[int(trajectory.Segment_Id[i])])

    Pose_Sample(kinematics, trajectory, sample)
    title, comment = Describe(sample)
    print(log.Line('[INFO] %s (sample %d): %s' % (title, sample, comment)))
    print(log.Line('[INFO] Simulation result only: not validated for the real cell.'))
    if args.headless:
        log.Close()
        return 0
    playback = Playback_Cls(trajectory.Step_Index, CONST_DT, sample, paused=args.step is not None or args.time is not None)

    def On_Key(key: int) -> None:
        """
        Description:
            Queue the key's playback action (the viewer's thread; the loop applies it at its next frame).

        Args:
            (1) key [int]: The GLFW key code.
        """

        if key in CONST_KEY_ACTIONS:
            playback.Request(CONST_KEY_ACTIONS[key])

    def Overlay(i: int) -> tp.Tuple[str, str, int]:
        """
        Description:
            The overlay's text and the segment of sample i.

        Args:
            (1) i [int]: The sample.

        Returns:
            (1 - 2) parameter [string]: The two text columns (Describe).
            (3) parameter [int]: The segment.
        """

        return Describe(i) + (int(trajectory.Segment_Id[i]),)

    try:
        viewer = Launch_Passive_Viewer(model, kinematics.Data, On_Key, 'Example/Program/show_program.py --version %s' % args.version)
    except Render_Error_Cls as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 2
    print(log.Line('[INFO] Home = pause / resume, Page Down or . = next step, Page Up = step start / previous step, End = step end.'))
    watchdog = Watchdog_Cls(args.watchdog) if args.watchdog else None
    with viewer:
        stats = Run_Viewer_Loop(viewer, playback, lambda i: Pose_Sample(kinematics, trajectory, i), Overlay,
                                lambda title, comment: print(log.Line('[INFO] %s: %s' % (title, comment)), flush=True), watchdog)
    if watchdog is not None:
        watchdog.Stop()
    print(log.Line('[INFO] Viewer: %s.' % stats.Summary()))
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
