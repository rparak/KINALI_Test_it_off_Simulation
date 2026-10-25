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
# File Name: ../Example/Program/optimize_program.py
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
# Math (Mathematical functions)
import math
# Time (Time access)
import time
# Custom Lib.:
#   ../Configuration/Core - the configuration, the logs, the folders
from Configuration.Core import Configure_MuJoCo_Log, Load_Cell_Configuration, Read_PCB_Meta, Script_Log_Cls, Configuration_Error_Cls, CONST_REPORTS
#   ../Configuration/Program_Parser - the taught program, the optimizer's settings, the optimized program's file
from Configuration.Program_Parser import Load_Constants, Load_Simulation, Load_Program, Load_Optimization, CONST_OPTIMIZED_PROGRAM_YAML
#   ../Scene/Core - the cell's errors
from Scene.Core import Scene_Error_Cls
#   ../Kinematics/Core, ../Collision/Core - FK, IK and exact distances on the planning model (the table included)
from Kinematics.Core import Robot_Kinematics_Cls
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
#   ../Program/Core, ../Program/Check - the taught program's build, the check report
from Program.Core import Build_Program, Program_Error_Cls
from Program.Check import Format_Check_Report
#   ../Planner/Core - the planner's errors
from Planner.Core import Planner_Error_Cls
#   ../Optimizer/Core - the optimization and its verification
from Optimizer.Core import (Optimize_Program, Free_Transfers, Compare_Locked, Interlocks, Carried_Tilt, Step_Bounds, Optimizer_Error_Cls)


def main():
    """
    Description:
        Make the optimized program (FEATURES.md F9): the FREE transfers of Configuration/Program.yaml (Configuration/
        Optimization.yaml) planned, the loader's jobs between them run in parallel, every LOCKED segment copied; the program
        written to Data/Optimized/Program_Optimized.yaml and checked like the taught one. The report (stdout and
        Data/Generated/Reports/optimize_program.txt): per transfer the planner's effort, durations, peaks and the carried PCB's
        largest tilt; the schedule; the interlocks I1-I8 with their times; each LOCKED part against the taught program; the
        check. Exit 0 when the check passes, every interlock holds and every LOCKED part is byte-identical, 1 otherwise or when it
        cannot be made.

            .venv/bin/python Example/Program/optimize_program.py [--out DIR]
    """

    parser = argparse.ArgumentParser(description='Make the optimized program (Data/Optimized/Program_Optimized.yaml) and check it.')
    parser.add_argument('--out', help='the folder for the program and the report (default: Data/Optimized and Data/Generated/Reports)')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('optimize_program')
    began = time.perf_counter()
    path = os.path.join(args.out, os.path.basename(CONST_OPTIMIZED_PROGRAM_YAML)) if args.out else CONST_OPTIMIZED_PROGRAM_YAML
    try:
        cell, meta = Load_Cell_Configuration(), Read_PCB_Meta()
        constants = Load_Constants()
        taught = Load_Program(constants, Load_Simulation(constants), cell)
        optimization = Load_Optimization(taught)
        for level, message in cell.Notes + taught.Notes + optimization.Notes:
            print(log.Line('[%s] %s' % (level, message)))
        model = Build_Planning_Model(cell)
        collision = Robot_Collision_Cls(model)
        kinematics = Robot_Kinematics_Cls(model, collision.Data)
        default, _ = Build_Program(taught, cell, meta, kinematics, {})
        os.makedirs(os.path.dirname(path), exist_ok=True)
        result = Optimize_Program(taught, optimization, cell, meta, kinematics, collision, default, path,
                                  lambda line: print(log.Line('[INFO] ' + line),
                                                                                                                        flush=True))
    except (Configuration_Error_Cls, Scene_Error_Cls, Program_Error_Cls, Planner_Error_Cls, Optimizer_Error_Cls) as error:
        print(log.Line('[ERROR] %s' % error))
        log.Close()
        return 1
    tr = result.Trajectory
    lines = ['optimized program %s: cycle %.3f s (taught %.3f s, %+.3f s); made in %.1f s' % (os.path.relpath(path, CONST_REPO_ROOT)
                                                                                           if path.startswith(CONST_REPO_ROOT) else path, tr.T[-1],
                                                                                           default.T[-1], tr.T[-1] - default.T[-1],
                                                                                           time.perf_counter() - began),
             'FREE transfers (planner margin; RRT iterations / nodes; vertices RRT -> shortcut and prune -> spline waypoints, refits; '
             'validity checks; time-optimal / sampled duration; peak speed / acceleration of the limits; carried PCB tilt max):']
    for p in result.Planned:
        t = p.Transfer
        first, last = Step_Bounds(tr, t.Name)
        tilt = '%.2f deg (limit %.1f)' % (math.degrees(Carried_Tilt(tr, first, last)),
                                          math.degrees(optimization.Carry_Max_Tilt)) if t.Carrying else '-'
        slot, wait = result.Delays.get(t.Name, (1, 0.0))
        lines.append('  %-20s %s -> %s: margin %.1f mm; %d / %d; %d -> %d -> %d, %d refits; %d checks; %.3f / %.3f s; %.3f / %.3f; tilt %s'
                     % (p.Set, p.Start, p.End, 1000 * p.Margin, p.Iterations, p.Nodes, p.Rrt_Points, p.Short_Points, len(p.Waypoints), p.Refits,
                        p.Checks, p.Optimal_Time, p.Duration, p.Peak_Speed, p.Peak_Acceleration, tilt))
        if t.Jobs and p.End == t.End:
            later = ', %.3f s later' % wait if wait else ''
            waits = '; the robot waits at %s' % t.Wait if t.Wait else ''
            lines.append('  %-20s %s replaces %s; parallel jobs %s%s; the last spline after the start of %s%s'
                         % ('', t.Name, ', '.join(t.Keys + t.Job_Keys), ', '.join(t.Jobs), waits, t.Jobs[slot - 1], later))
    lines += ['  attempt: %s' % a for a in result.Attempts]
    lines.append('schedule: %s' % ', '.join('%s %.3f s' % (k, v) for k, v in result.Program.Optimized.Schedule.items()))
    interlocks = Interlocks(tr, kinematics, collision, cell, result.Program)
    lines += ['interlock %s %s: %s' % (name, 'holds' if ok else 'FAILS', text) for name, ok, text in interlocks]
    transfers = Free_Transfers(taught, optimization, default)
    compared = Compare_Locked(default, tr, taught, transfers)
    lines.append('LOCKED parts against the taught program, from each part\'s start (byte-identical: the same bytes on the compared channels):')
    lines += ['  %-22s %-6s %5d samples, start %.3f s -> %.3f s: %s' % (c.Name, c.Channels, c.Samples, c.Start_Default, c.Start_Optimized,
                                                                     'byte-identical' if c.Identical else 'DIFFERS') for c in compared]
    if not all(c.Identical for c in compared):
        lines.append('[ERROR] LOCKED parts that are not byte-identical: %s' % ', '.join(c.Name for c in compared if not c.Identical))
    lines += Format_Check_Report(result.Check, tr, cell)
    ok = result.Check.Verdict == 'PASS' and all(ok for _, ok, _ in interlocks) and all(c.Identical for c in compared)
    report = os.path.join(args.out or CONST_REPORTS, 'optimize_program.txt')
    os.makedirs(os.path.dirname(report), exist_ok=True)
    with open(report, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(lines) + '\n')
    for line in lines:
        print(log.Line(line))
    print(log.Line('[INFO] Wrote %s and %s; build it with .venv/bin/python Example/Program/build_program.py --program %s' % (path, report, path)))
    print(log.Line('[%s] The optimized program %s. Simulation results only: not validated for the real cell.'
                   % ('INFO' if ok else 'ERROR', 'passes its check' if ok else 'FAILS')))
    log.Close()
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
