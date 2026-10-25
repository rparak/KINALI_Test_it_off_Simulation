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
# File Name: ../Example/Simulation/physics_baseline.py
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
# Difflib (Helpers for computing deltas)
import difflib
# Functools (Higher-order functions)
import functools
# Importlib (Import machinery)
import importlib.util
# JSON (JSON encoder and decoder)
import json
# Subprocess (Subprocess management)
import subprocess
# Time (Time access)
import time
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Configuration/Core - the logs
from Configuration.Core import Configure_MuJoCo_Log, Script_Log_Cls
#   ../Optimizer/Core - the platform record
from Optimizer.Core import Platform_Record
#   ../Simulation/Core - the physics run, its report and acceptance
from Simulation.Core import (Trajectory_Drive_Cls, Run_Physics, Format_Program_Report, Check_Program_Acceptance, Free_Loader_Distance,
                             Simulation_Error_Cls)

"""
Description:
    Initialization of constants.

    The physics baselines of the two programs (step 6, owner 2026-10-25): per program the full report of
    Example/Simulation/run_program.py (baseline_report_<program>.txt) and its metrics (baseline_metrics_<program>.json), with
    the environment they were captured in (baseline_environment.json), in test/Simulation/data/. A run FAILS the regression on
    SPEC §10.1 (the acceptance bounds) or when a metric drifts from its baseline beyond CONST_TOLERANCES; the report's diff is
    printed for information only (contact physics changes with sub-micrometre differences, design-decisions). This script is the
    one reader and writer of the format; test/Simulation/test_baselines.py imports it.
"""
# The baseline folder and the programs.
#   Unit [string], [List<string>]
CONST_BASELINES = os.path.join(CONST_REPO_ROOT, 'test', 'Simulation', 'data')
CONST_PROGRAMS = ['taught', 'optimized']
# The drift tolerances per metric (owner, 2026-10-25): (kind, value) - 'exact', 'absolute' (the metric's unit), 'relative'
# (a fraction of the baseline); a vector metric's every element.
#   Unit [Dict<string, Tuple<string, float>>]
CONST_TOLERANCES = {'cycle_time': ('exact', 0.0), 'fault': ('exact', 0.0), 'vacuum_times': ('absolute', 0.004), 'vacuum_cups': ('exact', 0.0),
                    'hold_times': ('absolute', 0.004), 'hold_min_cups': ('exact', 0.0), 'placed_error': ('absolute', 0.00005),
                    'placed_tilt_deg': ('absolute', 0.02), 'final_position': ('absolute', 0.00005), 'final_tilt_deg': ('absolute', 0.02),
                    'closed_deg': ('absolute', 0.1), 'end_deg': ('absolute', 0.1), 'penetration': ('absolute', 0.00005), 'slip': ('absolute', 0.0002),
                    'tracking': ('relative', 0.10), 'loader_lag': ('relative', 0.10), 'fork_net_force': ('relative', 0.10),
                    'handle_load': ('relative', 0.10), 'joint_torque': ('relative', 0.10), 'free_loader_distance': ('absolute', 0.0005)}


@functools.lru_cache(maxsize=None)
def _Script(name: str) -> tp.Any:
    """
    Description:
        Another Example script as a module, loaded once (run_program.py: the program loading and its stale checks).

    Args:
        (1) name [string]: The script's path below Example/.

    Returns:
        (1) parameter [module]: The script's module.
    """

    spec = importlib.util.spec_from_file_location(os.path.splitext(os.path.basename(name))[0], os.path.join(CONST_REPO_ROOT, 'Example', name))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def Metrics(result: tp.Any) -> tp.Dict[str, tp.Any]:
    """
    Description:
        The compared metrics of a program's physics run (CONST_TOLERANCES' keys; SI units, angles in degrees as named).

    Args:
        (1) result [Cycle_Result_Str(object)]: The run's values.

    Returns:
        (1) parameter [Dict<string, Any>]: The metrics (JSON-ready).
    """

    r = result
    closest = Free_Loader_Distance(r)
    groups = sorted(r.Penetration)
    return {'cycle_time': round(float(r.Cycle_Time), 6), 'fault': r.Fault or '', 'vacuum_times': [float(v.T) for v in r.Vacuum],
            'vacuum_cups': [v.Cups if v.Cups is not None else -1 for v in r.Vacuum],
            'hold_times': [x for h in r.Holds for x in (float(h.T_On), float(h.T_Off))],
            'hold_min_cups': [int(h.Min_Cups) for h in r.Holds], 'placed_error': [float(x) for x in r.Placed_Error],
            'placed_tilt_deg': float(np.degrees(r.Placed_Tilt)), 'final_position': [float(x) for x in r.Final_PCB[:3]],
            'final_tilt_deg': float(np.degrees(r.Final_Tilt)),
            'closed_deg': [float(np.degrees(r.Closed[0])), float(np.degrees(r.Closed[1]))] if r.Closed else [],
            'end_deg': [float(np.degrees(r.Lid_End)), float(np.degrees(r.Handle_End))], 'penetration_groups': groups,
            'penetration': [float(r.Penetration[g][0]) for g in groups], 'slip': float(r.Slip), 'tracking': float(r.Track_Error),
            'loader_lag': float(r.Loader_Lag), 'fork_net_force': float(r.Fork_Net_Force), 'fork_summed_force': float(r.Fork_Force),
            'handle_load': float(r.Handle_Load), 'joint_torque': [float(x) for x in r.Joint_Torque],
            'free_loader_distance': float(closest[0]) if closest else -1.0}


def Capture(program: str, trajectories: tp.Optional[str] = None) -> tp.Dict[str, tp.Any]:
    """
    Description:
        A program's physics run now (its trajectory must be current, as run_program.py's): the report, the metrics, the
        acceptance.

    Args:
        (1) program [string]: 'taught' or 'optimized'.
        (2) trajectories [string]: The folder of the trajectory files (default Data/Generated/Trajectories), or None.

    Returns:
        (1) parameter [Dict<string, Any>]: 'report' (List<string>), 'metrics' (Dict), 'acceptance' (List<Tuple<string, bool, string>>).
    """

    run_program = _Script('Simulation/run_program.py')
    name = os.path.basename(run_program.CONST_VERSIONS[program]).replace('.yaml', '.npz')
    cell, meta, prog, trajectory, free, model, _ = run_program.Load_Run(program, os.path.join(trajectories, name) if trajectories else None)
    result = Run_Physics(model, Trajectory_Drive_Cls(trajectory, prog, free), meta, cell)
    return {'report': Format_Program_Report(result, cell, prog.Name), 'metrics': Metrics(result),
            'acceptance': [(c.Name, c.Passed, c.Detail) for c in Check_Program_Acceptance(result, cell)]}


def Environment() -> tp.Dict[str, str]:
    """
    Description:
        The environment of a capture: the platform and the versions, the git commit, the date.

    Returns:
        (1) parameter [Dict<string, string>]: The record.
    """

    try:
        commit = subprocess.run(['git', 'rev-parse', '--short', 'HEAD'], cwd=CONST_REPO_ROOT, capture_output=True, text=True).stdout.strip()
    except OSError:
        commit = ''
    return dict(Platform_Record(), commit=commit, date=time.strftime('%Y-%m-%d'))


def Read_Baseline(program: str, folder: str = CONST_BASELINES) -> tp.Dict[str, tp.Any]:
    """
    Description:
        A program's baseline (Write_Baseline's files).

    Args:
        (1) program [string]: 'taught' or 'optimized'.
        (2) folder [string]: The folder.

    Returns:
        (1) parameter [Dict<string, Any>]: 'report' and 'metrics'.
    """

    with open(os.path.join(folder, 'baseline_report_%s.txt' % program), encoding='utf-8') as handle:
        report = handle.read().splitlines()
    with open(os.path.join(folder, 'baseline_metrics_%s.json' % program), encoding='utf-8') as handle:
        return {'report': report, 'metrics': json.load(handle)}


def Write_Baseline(program: str, captured: tp.Dict[str, tp.Any], folder: str) -> None:
    """
    Description:
        Write a program's baseline (the report and the metrics) and the environment record into a folder.

    Args:
        (1) program [string]: 'taught' or 'optimized'.
        (2) captured [Dict<string, Any>]: As Capture.
        (3) folder [string]: The folder.
    """

    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, 'baseline_report_%s.txt' % program), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(captured['report']) + '\n')
    with open(os.path.join(folder, 'baseline_metrics_%s.json' % program), 'w', encoding='utf-8') as handle:
        handle.write(json.dumps(captured['metrics'], indent=1) + '\n')
    with open(os.path.join(folder, 'baseline_environment.json'), 'w', encoding='utf-8') as handle:
        handle.write(json.dumps(Environment(), indent=1, sort_keys=True) + '\n')


def Drift(now: tp.Dict[str, tp.Any], baseline: tp.Dict[str, tp.Any]) -> tp.List[str]:
    """
    Description:
        The metrics that drift from the baseline beyond CONST_TOLERANCES, one line each (a changed length counts as a drift).

    Args:
        (1 - 2) now, baseline [Dict<string, Any>]: Metrics (Metrics).

    Returns:
        (1) parameter [List<string>]: The drifts; empty when every metric is within its tolerance.
    """

    out = []
    for key, (kind, value) in CONST_TOLERANCES.items():
        a, b = now[key], baseline[key]
        if kind == 'exact' or isinstance(a, str):
            if a != b:
                out.append('%s: %s -> %s (exact)' % (key, b, a))
            continue
        a, b = np.atleast_1d(np.asarray(a, float)), np.atleast_1d(np.asarray(b, float))
        if a.shape != b.shape:
            out.append('%s: %d -> %d values' % (key, b.size, a.size))
            continue
        allowed = value if kind == 'absolute' else value * np.abs(b)
        bad = np.abs(a - b) > allowed + 1e-12
        if np.any(bad):
            out.append('%s: %s -> %s (%s %g)' % (key, np.round(b, 6).tolist(), np.round(a, 6).tolist(), kind, value))
    return out


def Compare(now: tp.Dict[str, tp.Any], baseline: tp.Dict[str, tp.Any]) -> tp.Tuple[tp.List[str], tp.List[str]]:
    """
    Description:
        A capture against its baseline: the failures (every failed SPEC §10.1 criterion, every drift) and, for information, the
        report's unified diff.

    Args:
        (1) now [Dict<string, Any>]: As Capture.
        (2) baseline [Dict<string, Any>]: As Read_Baseline.

    Returns:
        (1) parameter [List<string>]: The failures; empty when the run passes.
        (2) parameter [List<string>]: The report diff (informational).
    """

    failures = ['acceptance: %s (%s)' % (name, detail) for name, ok, detail in now['acceptance'] if not ok]
    failures += ['drift: ' + line for line in Drift(now['metrics'], baseline['metrics'])]
    diff = list(difflib.unified_diff(baseline['report'], now['report'], 'baseline', 'now', lineterm='', n=0))
    return failures, diff


def main():
    """
    Description:
        Run each program's physics and compare it with its baseline in test/Simulation/data/ (default): exit 1 when a SPEC §10.1
        criterion fails or a metric drifts beyond CONST_TOLERANCES; the report's diff is printed for information. --out DIR writes
        the baselines (a re-capture, in its own commit with the reason).

            .venv/bin/python Example/Simulation/physics_baseline.py [--program taught|optimized] [--trajectories DIR] [--compare DIR | --out DIR]
    """

    parser = argparse.ArgumentParser(description='The programs\' physics baselines: compare (bounds and drift) or write.')
    parser.add_argument('--program', choices=CONST_PROGRAMS, help='one program (default: both)')
    parser.add_argument('--trajectories', help='the folder of the built trajectories (default Data/Generated/Trajectories)')
    group = parser.add_mutually_exclusive_group()
    group.add_argument('--compare', default=CONST_BASELINES, help='compare with this folder (default: test/Simulation/data/)')
    group.add_argument('--out', default=None, help='write the baselines into this folder instead of comparing')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('physics_baseline')
    failed, run_program = False, _Script('Simulation/run_program.py')
    for program in [args.program] if args.program else CONST_PROGRAMS:
        try:
            now = Capture(program, args.trajectories)
        except (run_program.Run_Error_Cls, Simulation_Error_Cls) as error:
            print(log.Line('[ERROR] %s: the physics run cannot run: %s' % (program, error)))
            log.Close()
            return 2
        if args.out is not None:
            Write_Baseline(program, now, args.out)
            print(log.Line('[INFO] %s: wrote its baseline into %s (cycle %.3f s; %d of %d criteria pass).' % (
                program, args.out, now['metrics']['cycle_time'], sum(ok for _, ok, _ in now['acceptance']), len(now['acceptance']))))
            continue
        failures, diff = Compare(now, Read_Baseline(program, args.compare))
        failed = failed or bool(failures)
        print(log.Line('[%s] %s: %d failure(s) (bounds and drift), %d report line(s) differ (information)' % (
            'ERROR' if failures else 'INFO', program, len(failures), sum(1 for line in diff if line[:1] in '+-' and line[:3] not in ('---', '+++')))))
        for line in failures:
            print(log.Line('  FAIL ' + line))
        for line in diff:
            print(log.Line('  diff ' + line))
    print(log.Line('[INFO] Simulation results only: not validated for the real cell.'))
    log.Close()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
