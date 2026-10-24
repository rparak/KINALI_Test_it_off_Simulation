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
# Time (Time access)
import time
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core - the configuration, the program, the logs, the folders
from Configuration.Core import (Configure_MuJoCo_Log, Load_Cell_Configuration, Load_Keypoints, Load_Program, Script_Log_Cls, Configuration_Error_Cls,
                                CONST_PROGRAM_DEFAULT_YAML, CONST_KEYPOINTS_YAML, CONST_CELL_YAML, CONST_FORK_PATH, CONST_TRAJECTORIES, CONST_IMAGES,
                                CONST_PCB_LOCATIONS)
#   ../Scene/Core - the cell
from Scene.Core import Build_Cell_Model, Scene_Error_Cls
#   ../Kinematics/Core - the model's robot joints
from Kinematics.Core import Robot_Kinematics_Cls
#   ../Program/Core - posing at a sample, the inputs' hashes
from Program.Core import Pose_Sample, File_Hash
#   ../Trajectory/Core - the trajectory file
from Trajectory.Core import Read_Trajectory, Trajectory_Error_Cls, Trajectory_Str, CONST_DT
#   ../Render/Core - the viewer and the image
from Render.Core import Launch_Passive_Viewer, Render_Free_Image, Save_Image, Render_Error_Cls

"""
Description:
    Initialization of constants.
"""
# The program file of each version (the optimized one comes with F9).
#   Unit [Dict<string, string>]
CONST_VERSIONS = {'default': CONST_PROGRAM_DEFAULT_YAML}
# The keys (GLFW codes): Home pause / resume, Page Down or '.' the next step's start, Page Up the current (or previous) step's
# start, End the current step's end. Every letter and , / ; ' \\ ` toggles a MuJoCo flag and the viewer binds Space, the
# arrows, Tab, Esc, [ ], - =, F1-F5, 0-5; these are in none of them (FEATURES.md F6).
#   Unit [int]
CONST_KEY_PAUSE, CONST_KEY_NEXT, CONST_KEY_DOT, CONST_KEY_PREVIOUS, CONST_KEY_END = 268, 267, 46, 266, 269
# The image camera: the cell's centre (the studio look's statistic, SPEC §4.10) seen from the front right.
#   Unit [Vector<float> 1x3] (m), [float] (m), [float] (deg), [float] (deg)
CONST_IMAGE_LOOKAT, CONST_IMAGE_DISTANCE, CONST_IMAGE_AZIMUTH, CONST_IMAGE_ELEVATION = [0.15, -0.40, 0.25], 1.9, 135.0, -28.0


def Stale_Inputs(trajectory: Trajectory_Str, program_path: str) -> tp.List[str]:
    """
    Description:
        The input files that changed since the trajectory was built (their sha256 against its metadata).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) program_path [string]: Its program file.

    Returns:
        (1) parameter [List<string>]: The changed inputs' names; empty when it is current.
    """

    built = trajectory.Meta.get('inputs_sha256', {})
    now = {'program': program_path, 'keypoints': CONST_KEYPOINTS_YAML, 'cell': CONST_CELL_YAML, 'fork_path': CONST_FORK_PATH}
    return [name for name, path in now.items() if built.get(name) != File_Hash(path)]


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
        time (kinematic, no physics), the step's name and comment shown; Home pauses / resumes, Page Down or '.' jumps to the
        next step, Page Up back to the current step's start, End to its end. --step N / --time T pose one moment; --headless
        prints it; --image writes an offscreen render. Reads the trajectory Example/Program/build_program.py wrote and refuses
        one whose inputs changed since. The viewer needs mjpython on macOS (the passive viewer).

            .venv/bin/mjpython Example/Program/show_program.py [--version default] [--step N | --time T]
            .venv/bin/python Example/Program/show_program.py [--step N | --time T] --headless [--image [FILE]]
    """

    parser = argparse.ArgumentParser(description='Play or pose a program trajectory (kinematic).')
    parser.add_argument('--version', default='default', choices=['default', 'optimized'], help='the program version (optimized: F9)')
    parser.add_argument('--trajectory', help='the trajectory file (default Data/Generated/Trajectories/<program>.npz)')
    parser.add_argument('--step', help='pose the cell at the end of this step (its number from 1 or its name)')
    parser.add_argument('--time', type=float, help='pose the cell at this time [s]')
    parser.add_argument('--headless', action='store_true', help='pose and print, no window')
    parser.add_argument('--image', nargs='?', const='', help='write an offscreen render (default Data/Generated/Images/<program>_<sample>.png)')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('show_program')
    if args.version not in CONST_VERSIONS:
        print(log.Line('[ERROR] The optimized program comes with F9; only --version default exists now.'))
        log.Close()
        return 1
    program_path = CONST_VERSIONS[args.version]
    try:
        cell, keypoints = Load_Cell_Configuration(), Load_Keypoints()
        program = Load_Program(keypoints, cell, program_path)
        path = args.trajectory or os.path.join(CONST_TRAJECTORIES, program.Name + '.npz')
        trajectory = Read_Trajectory(path)
        model = Build_Cell_Model(cell)
    except (Configuration_Error_Cls, Scene_Error_Cls, Trajectory_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    stale = Stale_Inputs(trajectory, program_path)
    if stale:
        print(log.Line('[ERROR] The trajectory %s was built from other inputs (%s changed since); rebuild it: '
                       '.venv/bin/python Example/Program/build_program.py' % (path, ', '.join(stale))))
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
    comments = [s.Comment for s in program.Steps]

    def Describe(i: int) -> tp.Tuple[str, str]:
        """
        Description:
            The step of sample i, as the viewer's two text columns and the printed line.

        Args:
            (1) i [int]: The sample.

        Returns:
            (1) parameter [string]: 'step k/n name' and the time.
            (2) parameter [string]: The step's comment.
        """

        k = int(trajectory.Step_Index[i])
        return ('step %d/%d %s   t = %.3f s / %.3f s   PCB %s' % (k + 1, len(trajectory.Step_Names), trajectory.Step_Names[k], trajectory.T[i],
                                                                trajectory.T[-1], CONST_PCB_LOCATIONS[trajectory.PCB_Location[i]]), comments[k])

    Pose_Sample(kinematics, trajectory, sample)
    title, comment = Describe(sample)
    print(log.Line('[INFO] %s (sample %d): %s' % (title, sample, comment)))
    print(log.Line('[INFO] Simulation result only: not validated for the real cell.'))
    if args.image is not None:
        image = args.image or os.path.join(CONST_IMAGES, '%s_%05d.png' % (program.Name, sample))
        image = Save_Image(Render_Free_Image(model, kinematics.Data, CONST_IMAGE_LOOKAT, CONST_IMAGE_DISTANCE, CONST_IMAGE_AZIMUTH,
                                             CONST_IMAGE_ELEVATION), image)
        print(log.Line('[INFO] Wrote %s.' % image))
    if args.headless or args.image is not None:
        log.Close()
        return 0
    starts = [int(np.flatnonzero(trajectory.Step_Index == k)[0]) for k in range(len(trajectory.Step_Names))]
    state = {'sample': sample, 'paused': args.step is not None or args.time is not None}

    def On_Key(key: int) -> None:
        """
        Description:
            Pause / resume, or jump between steps (the main loop poses the new sample).

        Args:
            (1) key [int]: The GLFW key code.
        """

        k = int(trajectory.Step_Index[state['sample']])
        if key == CONST_KEY_PAUSE:
            state['paused'] = not state['paused']
        elif key in (CONST_KEY_NEXT, CONST_KEY_DOT):
            state['sample'] = starts[(k + 1) % len(starts)]
        elif key == CONST_KEY_PREVIOUS:
            state['sample'] = starts[k] if state['sample'] > starts[k] else starts[(k - 1) % len(starts)]
        elif key == CONST_KEY_END:
            state['sample'] = int(np.flatnonzero(trajectory.Step_Index == k)[-1])

    try:
        viewer = Launch_Passive_Viewer(model, kinematics.Data, On_Key, 'Example/Program/show_program.py --version %s' % args.version)
    except Render_Error_Cls as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 2
    print(log.Line('[INFO] Home = pause / resume, Page Down or . = next step, Page Up = step start / previous step, End = step end.'))
    shown, clock = -1, time.perf_counter()
    with viewer:
        while viewer.is_running():
            now = time.perf_counter()
            if not state['paused']:
                state['sample'] = min(state['sample'] + int(round((now - clock) / CONST_DT)), len(trajectory.T) - 1)
                state['paused'] = state['sample'] == len(trajectory.T) - 1
            clock = now if not state['paused'] or int(round((now - clock) / CONST_DT)) else clock
            if state['sample'] != shown:
                with viewer.lock():
                    Pose_Sample(kinematics, trajectory, state['sample'])
                    title, comment = Describe(state['sample'])
                    viewer.set_texts((mujoco.mjtFontScale.mjFONTSCALE_150, mujoco.mjtGridPos.mjGRID_TOPLEFT, title, comment))
                if int(trajectory.Step_Index[state['sample']]) != int(trajectory.Step_Index[max(shown, 0)]) or shown < 0:
                    print(log.Line('[INFO] %s: %s' % (title, comment)), flush=True)
                shown = state['sample']
            viewer.sync()
            time.sleep(CONST_DT)
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
