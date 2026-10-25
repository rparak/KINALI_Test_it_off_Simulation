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
# File Name: ../Example/Program/compare_programs.py
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
#   ../Configuration/Core - the configuration, the logs, the folders
from Configuration.Core import (Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls, Configuration_Error_Cls, CONST_REPORTS,
                                CONST_IMAGES)
#   ../Configuration/Program_Parser - the two programs and the optimizer's settings
from Configuration.Program_Parser import Load_Constants, Load_Simulation, Load_Program, Load_Optimization, CONST_OPTIMIZED_PROGRAM_YAML
#   ../Scene/Core - the cell's errors
from Scene.Core import Scene_Error_Cls
#   ../Kinematics/Core, ../Collision/Core - FK on the planning model
from Kinematics.Core import Robot_Kinematics_Cls
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
#   ../Program/Core - the builds
from Program.Core import Build_Program, Program_Error_Cls
#   ../Optimizer/Core - the FREE transfers, the stale check
from Optimizer.Core import Free_Transfers, Stale_Inputs
#   ../Optimizer/Measure - the metrics
from Optimizer.Measure import Measure, Program_Metrics_Str
#   ../Trajectory/Core - the trajectory format
from Trajectory.Core import Trajectory_Str, CONST_DT

"""
Description:
    Initialization of constants.
"""
# The figure's colours (the dataviz reference palette, light mode): the surface, the ink, the grid; categorical slot 1
# (the robot moving) and slot 2 (the loader moving).
#   Unit [string] (hex)
CONST_SURFACE, CONST_INK, CONST_INK_2, CONST_GRID = '#fcfcfb', '#0b0b0b', '#52514e', '#e4e3df'
CONST_ROBOT, CONST_LOADER = '#2a78d6', '#eb6834'


def Moving_Intervals(values: np.ndarray, dt: float) -> tp.List[tp.Tuple[float, float]]:
    """
    Description:
        The time intervals in which a sampled channel changes (some component by more than 1e-9 between samples).

    Args:
        (1) values [Matrix<float> nxk]: The samples.
        (2) dt [float]: The sample time (s).

    Returns:
        (1) parameter [List<Tuple<float, float>>]: (start, length) in s, for matplotlib's broken_barh.
    """

    moving = np.max(np.abs(np.diff(values.reshape(len(values), -1), axis=0)), axis=1) > 1e-9
    out, start = [], None
    for i, m in enumerate(moving):
        if m and start is None:
            start = i
        elif not m and start is not None:
            out.append((start * dt, (i - start) * dt))
            start = None
    if start is not None:
        out.append((start * dt, (len(moving) - start) * dt))
    return out


def Part_Rows(default: Trajectory_Str, optimized: Trajectory_Str, taught_names: tp.List[str],
              transfers: tp.List[tp.Any]) -> tp.List[tp.Tuple[str, float, float]]:
    """
    Description:
        Per part of the process, its duration in both programs: a FREE transfer against the taught segments it replaces
        (its FREE ones and the loader-only ones between them), every other segment against its namesake.

    Args:
        (1 - 2) default, optimized [Trajectory_Str(object)]: The trajectories.
        (3) taught_names [List<string>]: The taught program's segment keys and End_Settle.
        (4) transfers [List<Free_Transfer_Str>]: The FREE transfers.

    Returns:
        (1) parameter [List<Tuple<string, float, float>>]: (part, taught s, optimized s).
    """

    def Duration(trajectory: Trajectory_Str, name: str) -> float:
        """
        Description:
            A step's duration by its name without its number.

        Args:
            (1) trajectory [Trajectory_Str(object)]: The trajectory.
            (2) name [string]: The name.

        Returns:
            (1) parameter [float]: The duration (s).
        """

        index = next(k for k, n in enumerate(trajectory.Step_Names) if n.split('_', 1)[-1] == name or n == name)
        return float(np.sum(trajectory.Step_Index == index)) * CONST_DT

    by_first = {t.Keys[0]: t for t in transfers}
    merged = {k for t in transfers for k in t.Keys[1:] + t.Job_Keys}
    rows = []
    for key in taught_names:
        if key in merged:
            continue
        if key in by_first:
            t = by_first[key]
            taught = sum(Duration(default, k.split('_', 1)[1]) for k in t.Keys + t.Job_Keys)
            rows.append(('%s (FREE: %s)' % (t.Name, ', '.join(k.split('_', 1)[0] for k in sorted(t.Keys + t.Job_Keys, key=taught_names.index))),
                         taught, Duration(optimized, t.Name)))
        else:
            name = key.split('_', 1)[1] if key.startswith('S') else key
            rows.append((name, Duration(default, name), Duration(optimized, name)))
    return rows


def Figure(default: Trajectory_Str, optimized: Trajectory_Str, metrics: tp.Dict[str, Program_Metrics_Str], path: str) -> str:
    """
    Description:
        The comparison figure: the robot's and the loader's motion over time in both programs (lanes: a bar where it moves),
        and each metric of the optimized program as a percentage of the taught one (one axis, 100 % marked).

    Args:
        (1 - 2) default, optimized [Trajectory_Str(object)]: The trajectories.
        (3) metrics [Dict<string, Program_Metrics_Str>]: 'taught' and 'optimized'.
        (4) path [string]: The PNG file.

    Returns:
        (1) parameter [string]: The absolute path written.
    """

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    figure, (lanes, bars) = plt.subplots(2, 1, figsize=(10.0, 6.4), gridspec_kw={'height_ratios': [1.0, 1.15]}, facecolor=CONST_SURFACE)
    rows = [('taught: robot', default.Robot_Q, CONST_ROBOT), ('taught: loader', np.c_[default.Loader_Y, default.Loader_Z, default.Gripper],
                                                              CONST_LOADER),
            ('optimized: robot', optimized.Robot_Q, CONST_ROBOT), ('optimized: loader', np.c_[optimized.Loader_Y, optimized.Loader_Z,
                                                                                              optimized.Gripper],
                                                                   CONST_LOADER)]
    for k, (label, values, colour) in enumerate(rows):
        y = len(rows) - 1 - k
        lanes.broken_barh(Moving_Intervals(values, CONST_DT), (y - 0.32, 0.64), facecolors=colour, edgecolor=CONST_SURFACE, linewidth=0.5)
    for k, trajectory in ((0, default), (2, optimized)):
        lanes.text(trajectory.T[-1] + 0.6, len(rows) - 1 - k, 'end %.1f s' % trajectory.T[-1], color=CONST_INK, va='center', fontsize=9)
    lanes.set_yticks(range(len(rows)))
    lanes.set_yticklabels([r[0] for r in rows][::-1], color=CONST_INK_2, fontsize=9)
    lanes.set_xlim(0, max(default.T[-1], optimized.T[-1]) + 8)
    lanes.set_xlabel('time [s]  (a bar: moving; blue the robot, orange the loader)', color=CONST_INK_2, fontsize=9)
    lanes.set_title('Robot and loader motion, taught vs optimized program (simulation)', color=CONST_INK, fontsize=11, loc='left')
    a, b = metrics['taught'], metrics['optimized']
    items = [('cycle time', a.Cycle_Time, b.Cycle_Time, 's'), ('robot joint travel (sum)', float(a.Joint_Travel.sum()),
                                                               float(b.Joint_Travel.sum()), 'rad'),
             ('TCP path', a.Tcp_Path, b.Tcp_Path, 'm'), ('PCB path', a.Pcb_Path, b.Pcb_Path, 'm'), ('loader travel', a.Loader_Travel,
                                                                                                    b.Loader_Travel, 'm'),
             ('robot at rest (stops >= 0.1 s)', a.Rest_Time, b.Rest_Time, 's')]
    names = [i[0] for i in items][::-1]
    percent = [100.0 * i[2] / i[1] for i in items][::-1]
    bars.barh(range(len(items)), percent, height=0.56, color=CONST_ROBOT, edgecolor=CONST_SURFACE, linewidth=2)
    bars.axvline(100.0, color=CONST_INK_2, linewidth=1.0, linestyle=(0, (3, 3)))
    for k, (name, x, y, unit) in enumerate(items[::-1]):
        text = ('%.3g -> %.3g %s' % (x, y, unit)).strip() if unit else '%d -> %d' % (x, y)
        bars.text(max(100.0, 100.0 * y / x) + 2.0, k, '%.0f %%  (%s)' % (100.0 * y / x, text), va='center', color=CONST_INK, fontsize=9)
    bars.set_yticks(range(len(items)))
    bars.set_yticklabels(names, color=CONST_INK_2, fontsize=9)
    bars.set_xlim(0, 160)
    bars.set_xlabel('optimized as a percentage of the taught program (the dashed line: 100 %)', color=CONST_INK_2, fontsize=9)
    bars.set_title('Optimized program relative to the taught one', color=CONST_INK, fontsize=11, loc='left')
    for axis in (lanes, bars):
        axis.set_facecolor(CONST_SURFACE)
        axis.grid(axis='x', color=CONST_GRID, linewidth=0.6)
        axis.set_axisbelow(True)
        axis.tick_params(colors=CONST_INK_2, labelsize=9, length=0)
        for side in ('top', 'right', 'left'):
            axis.spines[side].set_visible(False)
        axis.spines['bottom'].set_color(CONST_GRID)
    figure.text(0.01, 0.005, 'Simulation results only: not validated for the real cell. Example/Program/compare_programs.py', color=CONST_INK_2,
                fontsize=8)
    figure.tight_layout(rect=(0, 0.02, 1, 1))
    path = os.path.abspath(path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    figure.savefig(path, dpi=110, facecolor=CONST_SURFACE)
    plt.close(figure)
    return path


def main():
    """
    Description:
        Compare the taught program and the optimized one (FEATURES.md F9; the legacy reference plan's column until step 6:
        commit 7a8560c8c16806739a02da8e989c2a2f65d1e41d): both programs built afresh (the optimized one refused when stale); the cycle time,
        the robot's joint travel per joint and the TCP's path, the PCB's path, the loader's travel, the robot's stops; per part
        of the process both durations. A text table (stdout and Data/Generated/Reports/compare_programs.txt) and a figure
        (Data/Generated/Images/compare_programs.png). Exit 0, 1 when a program cannot be built.

            .venv/bin/python Example/Program/compare_programs.py [--out DIR]
    """

    parser = argparse.ArgumentParser(description='Compare the taught and the optimized program.')
    parser.add_argument('--out', help='the folder for the table and the figure (default: Data/Generated/Reports and Images)')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('compare_programs')
    try:
        cell, meta = Load_Cell_Configuration(), Read_PCB_Meta()
        constants = Load_Constants()
        simulation = Load_Simulation(constants)
        taught = Load_Program(constants, simulation, cell)
        optimized = Load_Program(constants, simulation, cell, CONST_OPTIMIZED_PROGRAM_YAML)
        model = Build_Planning_Model(cell)
        stale = Stale_Inputs(optimized, model)
        if stale:
            raise Program_Error_Cls('%s was optimized from other inputs (%s changed since); make it again: .venv/bin/python '
                                    'Example/Program/optimize_program.py' % (CONST_OPTIMIZED_PROGRAM_YAML, ', '.join(stale)))
        kinematics = Robot_Kinematics_Cls(model, Robot_Collision_Cls(model).Data)
        default, _ = Build_Program(taught, cell, meta, kinematics, {})
        fast, _ = Build_Program(optimized, cell, meta, kinematics, {})
        transfers = Free_Transfers(taught, Load_Optimization(taught), default)
    except (Configuration_Error_Cls, Scene_Error_Cls, Program_Error_Cls, OSError) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    metrics = {'taught': Measure(default.Robot_Q, CONST_DT, kinematics, default.Loader_Y, default.Loader_Z, default.PCB_Pose[:, :3]),
               'optimized': Measure(fast.Robot_Q, CONST_DT, kinematics, fast.Loader_Y, fast.Loader_Z, fast.PCB_Pose[:, :3])}
    columns = (('taught', metrics['taught']), ('optimized', metrics['optimized']))
    lines = ['compare: the taught program (Configuration/Program.yaml) and the optimized one (Data/Optimized/Program_Optimized.yaml)',
             '%-28s %12s %12s %12s' % ('metric', 'taught', 'optimized', 'opt / taught')]

    def Row(name: str, values: tp.List[float], form: str) -> str:
        """
        Description:
            One metric row: the two values and the optimized one's ratio to the taught one.

        Args:
            (1) name [string]: The metric.
            (2) values [List<float>]: Taught, optimized.
            (3) form [string]: The number format.

        Returns:
            (1) parameter [string]: The row.
        """

        cells = ['%12s' % (form % v) for v in values]
        return '%-28s %s %11.1f%%' % (name, ' '.join(cells), 100.0 * values[1] / values[0])

    lines.append(Row('cycle time [s]', [c.Cycle_Time for _, c in columns], '%.3f'))
    lines.append(Row('robot joint travel [rad]', [float(c.Joint_Travel.sum()) for _, c in columns], '%.3f'))
    for j in range(6):
        lines.append(Row('  joint %d [rad]' % (j + 1), [float(c.Joint_Travel[j]) for _, c in columns], '%.3f'))
    lines.append(Row('TCP path [m]', [c.Tcp_Path for _, c in columns], '%.3f'))
    lines.append(Row('PCB path [m]', [c.Pcb_Path for _, c in columns], '%.3f'))
    lines.append(Row('loader travel Y + Z [m]', [c.Loader_Travel for _, c in columns], '%.3f'))
    lines.append(Row('robot stops (>= 0.1 s)', [float(c.Stops) for _, c in columns], '%.0f'))
    lines.append(Row('robot at rest in them [s]', [c.Rest_Time for _, c in columns], '%.3f'))
    lines.append('per part of the process (s): taught, optimized, difference')
    for name, a, b in Part_Rows(default, fast, default.Step_Names, transfers):
        lines.append('  %-58s %8.3f %8.3f %+8.3f' % (name, a, b, b - a))
    lines.append('a stop: every joint speed below 1e-3 rad/s (joint change per sample over 4 ms) for at least 0.1 s, after a motion. '
                 'Simulation results only: not validated for the real cell.')
    table = os.path.join(args.out or CONST_REPORTS, 'compare_programs.txt')
    os.makedirs(os.path.dirname(table), exist_ok=True)
    with open(table, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    image = Figure(default, fast, metrics, os.path.join(args.out or CONST_IMAGES, 'compare_programs.png'))
    for line in lines:
        print(log.Line(line))
    print(log.Line('[INFO] Wrote %s and %s.' % (table, image)))
    log.Close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
