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
# File Name: ../Example/Cell/show_physics.py
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
# Importlib (Import machinery)
import importlib.util
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core - the logs, the folders
from Configuration.Core import Configure_MuJoCo_Log, Script_Log_Cls
#   ../Collision/Core - the cups touching the PCB
from Collision.Core import Cup_Contacts
#   ../Trajectory/Core - the sample time
from Trajectory.Core import CONST_DT
#   ../Simulation/Core - the physics run of a trajectory
from Simulation.Core import Trajectory_Drive_Cls, Physics_Run_Cls, Simulation_Error_Cls
#   ../Render/Core - the viewer
from Render.Core import (Launch_Passive_Viewer, Render_Error_Cls, Playback_Cls, Run_Viewer_Loop,
                         Watchdog_Cls)


def _Script(name: str) -> tp.Any:
    """
    Description:
        Another Example script as a module (the program loading of run_program.py, the camera of show_program.py).

    Args:
        (1) name [string]: The script's path below Example/.

    Returns:
        (1) parameter [module]: The script's module.
    """

    spec = importlib.util.spec_from_file_location(os.path.splitext(os.path.basename(name))[0], os.path.join(CONST_REPO_ROOT, 'Example', name))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


"""
Description:
    Initialization of constants.
"""
# The keys: Home pause / resume, Page Down the next step's start, '.' one sample on (paused), End the step's end; Page Up
# would go back, which a physics run cannot (it is refused).
#   Unit [Dict<int, string>]
CONST_KEY_ACTIONS = {268: 'pause', 267: 'next', 46: 'frame', 269: 'end'}
CONST_KEY_BACK = 266
# The physics stepped per frame at most this long (s of wall time): a jump ahead catches up over frames, the window still
# answers.
#   Unit [float] (s)
CONST_STEP_BUDGET = 0.05


class Physics_View_Cls(object):
    """
    Description:
        The physics run behind the viewer (F11): a private run (Physics_Run_Cls) stepped towards the playback's sample within
        CONST_STEP_BUDGET per frame (Prepare, outside the viewer's lock), its state copied into the viewer's data (Pose, under
        it), the overlay from the run's own time (Describe). An instability stops the stepping and shows its message.

    Initialization of the Class:
        Args:
            (1) run [Physics_Run_Cls(object)]: The run.
            (2) drive [Trajectory_Drive_Cls(object)]: Its drive.
            (3) view [mujoco.MjData]: The viewer's data.

        Example:
            Initialization:
                # Assignment of the variables.
                physics = Physics_View_Cls(run, drive, view)

            Features:
                # Functions of the class (Run_Viewer_Loop calls them).
                physics.Prepare(i); physics.Pose(i); physics.Describe(i)
    """

    def __init__(self, run: Physics_Run_Cls, drive: Trajectory_Drive_Cls, view: mujoco.MjData) -> None:
        self.Run, self.Drive, self.View = run, drive, view
        self.Error: tp.Optional[str] = None

    def Step_To(self, t: float, budget: float = float('inf')) -> None:
        """
        Description:
            Step the run until its time reaches t, it ends, or budget seconds of wall time pass.

        Args:
            (1) t [float]: The time to reach (s).
            (2) budget [float]: The wall time allowed (s).
        """

        began = time.perf_counter()
        try:
            while self.Error is None and not self.Run.Done and self.Run.Data.time < t and time.perf_counter() - began < budget:
                self.Run.Step()
        except Simulation_Error_Cls as error:
            self.Error = str(error)

    def Prepare(self, sample: int) -> None:
        """
        Description:
            The frame's stepping (outside the lock): towards the sample's time within CONST_STEP_BUDGET.

        Args:
            (1) sample [int]: The playback's sample.
        """

        self.Step_To(sample * CONST_DT, CONST_STEP_BUDGET)

    def Pose(self, sample: int) -> None:
        """
        Description:
            The run's state into the viewer's data (under the lock).

        Args:
            (1) sample [int]: The playback's sample (the run shows its own time).
        """

        mujoco.mj_copyData(self.View, self.Run.Model, self.Run.Data)

    def State(self) -> tp.Tuple[str, str, int]:
        """
        Description:
            The run's state as two text columns and its segment: the step, the time, the cups and the slip; the line, or the
            fault or the instability.

        Returns:
            (1 - 2) parameter [string]: The columns.
            (3) parameter [int]: The segment.
        """

        run, tr = self.Run, self.Drive.Trajectory
        t = run.Data.time
        i = min(int(round(t / CONST_DT)), len(tr.T) - 1)
        k = int(tr.Step_Index[i])
        vacuum = run.Data.ctrl[run.A_VAC[0]] > 0.5
        cups = '%d/4 cups' % Cup_Contacts(run.Model, run.Data, run.PCB, run.CUPS, run.Cell.Cup_Touch) if vacuum else 'vacuum off'
        title = 'step %d/%d %s   t = %.3f s / %.3f s (physics)   %s, slip %.2f mm' % (k + 1, len(tr.Step_Names), tr.Step_Names[k], t, tr.T[-1], cups,
                                                                                    1000 * run.slip)
        comment = self.Error or run.Fault or tr.Segment_Command[int(tr.Segment_Id[i])]
        return title, comment, int(tr.Segment_Id[i])

    def Describe(self, sample: int) -> tp.Tuple[str, str, int]:
        """
        Description:
            The overlay at a frame (State).

        Args:
            (1) sample [int]: The playback's sample (the run shows its own time).

        Returns:
            (1) parameter [Tuple]: As State.
        """

        return self.State()


def main(argv: tp.Optional[tp.List[str]] = None, log_name: tp.Optional[str] = None) -> int:
    """
    Description:
        A program's physics run in the MuJoCo viewer (F11): the taught or the optimized program's trajectory (refused when stale,
        as Example/Cell/run_program.py), stepped at the model's 0.5 ms in real time - the frames skipped by the wall clock
        - or at --speed (0.25: four times slower); keys Home pause, Page Down next step, '.' one sample, End the step's end (a
        physics run cannot go back); the overlay: the step, the time, the cups, the slip, the line (or the fault). The viewer's
        rules (F10.4): the lock only for the state copy. --headless --time T runs to T, prints the state and exits.
        The viewer needs .venv/bin/mjpython on macOS: under python it exits 2 naming it.

            Called by Example/cell.py show --physics [--program default|optimized] [--time T] [--speed F] [--headless] [--watchdog S].

    Args:
        (1) argv [List<string>]: The arguments (default: the command line).
        (2) log_name [string]: The log to append to (cell.py's logger/cell_<subcommand>.log), or None: logger/show_physics.log,
            rewritten.

    Returns:
        (1) parameter [int]: The exit code.
    """

    run_program = _Script('Cell/run_program.py')
    parser = argparse.ArgumentParser(description='Play a program\'s physics run in the viewer.')
    parser.add_argument('--program', default='taught', choices=list(run_program.CONST_VERSIONS), help='the program (default: taught)')
    parser.add_argument('--trajectory', help='the trajectory file (default Data/Generated/Trajectories/<program>.npz)')
    parser.add_argument('--speed', type=float, default=1.0, help='the playback speed against real time (default 1; 0.25: slow motion)')
    parser.add_argument('--time', type=float, help='start (paused) at this time [s]; with --headless: run to it and print the state')
    parser.add_argument('--headless', action='store_true', help='run to --time, print, no window')
    parser.add_argument('--watchdog', type=float, metavar='S', help='debug: when the viewer loop stalls for S seconds, print every '
                        'thread\'s traceback and exit 3')
    args = parser.parse_args(argv)

    Configure_MuJoCo_Log()
    log = Script_Log_Cls(log_name or 'show_physics', append=log_name is not None)
    if not args.speed > 0 or (args.headless and args.time is None):
        print(log.Line('[ERROR] --speed must be > 0 and --headless needs --time T.'))
        log.Close()
        return 1
    try:
        cell, meta, program, trajectory, free, model, _ = run_program.Load_Run(args.program, args.trajectory)
    except run_program.Run_Error_Cls as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    drive = Trajectory_Drive_Cls(trajectory, program, free)
    view = mujoco.MjData(model)
    physics = Physics_View_Cls(Physics_Run_Cls(model, drive, meta, cell), drive, view)
    start = min(max(args.time or 0.0, 0.0), float(trajectory.T[-1]))
    physics.Step_To(start)
    physics.Pose(0)
    title, comment, _ = physics.State()
    print(log.Line('[INFO] %s: %s' % (title, comment)))
    if args.headless:
        print(log.Line('[INFO] Simulation result only: not validated for the real cell.'))
        log.Close()
        return 1 if physics.Error or physics.Run.Fault else 0
    playback = Playback_Cls(trajectory.Step_Index, CONST_DT, int(round(start / CONST_DT)), paused=args.time is not None, speed=args.speed)

    def On_Key(key: int) -> None:
        """
        Description:
            Queue the key's playback action (the viewer's thread); Page Up is refused (a physics run goes forward only).

        Args:
            (1) key [int]: The GLFW key code.
        """

        if key in CONST_KEY_ACTIONS:
            playback.Request(CONST_KEY_ACTIONS[key])
        elif key == CONST_KEY_BACK:
            print(log.Line('[INFO] A physics run goes forward only: Page Up is not available here (Example/Program/show_program.py '
                           'plays the trajectory both ways).'), flush=True)

    try:
        viewer = Launch_Passive_Viewer(model, view, On_Key, 'Example/Simulation/show_physics.py --program %s' % args.program)
    except Render_Error_Cls as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 2
    print(log.Line('[INFO] Home = pause / resume, Page Down = next step, . = one sample (paused), End = step end; speed %g.' % args.speed))
    watchdog = Watchdog_Cls(args.watchdog) if args.watchdog else None
    with viewer:
        stats = Run_Viewer_Loop(viewer, playback, physics.Pose, physics.Describe,
                                lambda title, comment: print(log.Line('[INFO] %s: %s' % (title, comment)), flush=True), watchdog,
                                prepare=physics.Prepare)
    if watchdog is not None:
        watchdog.Stop()
    run = physics.Run
    stopped = physics.Error or run.Fault
    print(log.Line('[INFO] Viewer: %s; physics reached %.3f s of %.3f s%s.' % (stats.Summary(), run.Data.time, trajectory.T[-1],
                                                                            '; ' + stopped if stopped else '')))
    print(log.Line('[INFO] Simulation result only: not validated for the real cell.'))
    log.Close()
    return 0
