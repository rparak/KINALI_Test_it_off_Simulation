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
# Multiprocessing (Process-based parallelism)
import multiprocessing
# Subprocess (Subprocess management)
import subprocess
# Tempfile (Temporary files)
import tempfile
# Time (Time access)
import time
# Typing (Support for type hints)
import typing as tp
# MuJoCo (Physics engine) [pip3 install mujoco]
import mujoco
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Configuration/Core, ../Configuration/Program_Parser - the logs, the cell, the program (a settle-wait variant)
from Configuration.Core import Configure_MuJoCo_Log, Script_Log_Cls, Load_Cell_Configuration, Read_PCB_Meta
from Configuration.Program_Parser import Load_Constants, Load_Simulation, Load_Program
#   ../Collision/Core, ../Kinematics/Core, ../Program/Core, ../Scene/Core - a settle-wait variant's build
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
from Kinematics.Core import Robot_Kinematics_Cls
from Program.Core import Build_Program
from Scene.Core import Build_Cell_Model
#   ../Optimizer/Core - the platform record
from Optimizer.Core import Platform_Record
#   ../Simulation/Core - the physics run, its report and acceptance
from Simulation.Core import (Trajectory_Drive_Cls, Physics_Run_Cls, Format_Program_Report, Check_Program_Acceptance, Free_Loader_Distance,
                             Simulation_Error_Cls, Place_Physics_Str)

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
                    'handle_load': ('relative', 0.10), 'joint_torque': ('relative', 0.10), 'free_loader_distance': ('absolute', 0.0005),
                    'place_drop': ('absolute', 0.00002), 'place_transient': ('absolute', 0.00001), 'seat_supports': ('exact', 0.0),
                    'seat_tilt_deg': ('absolute', 0.02), 'place_guide_min': ('absolute', 0.00005), 'handover_centring': ('absolute', 0.00002),
                    'regrip_gap': ('absolute', 0.00002)}
# The fork's contact with the handle bar is chaotic: sub-micrometre differences change its contact points and its forces, and
# its outcomes do not fall into well-defined modes (2026-10-25: 6 contact points gave 247.3 N / 11.45 N m in the taught
# program, 222.2 N / 10.01 N m in the optimized one). So these metrics are compared with an envelope instead of the baseline
# run (owner, 2026-10-25): the min and max over CONST_PERTURBATIONS, stored with the baseline; a run passes within the
# envelope +- CONST_ENVELOPE_MARGIN of it, and always within SPEC §10.1 (the fork's bounds, the actuators). A vector metric's
# listed elements only (the J2 / J3 torques); its others keep their tolerance. The contact count is reported only.
#   Unit [Dict<string, Tuple<int>>], [float] (-)
CONST_ENVELOPE = {'fork_net_force': None, 'handle_load': None, 'joint_torque': (1, 2)}
CONST_ENVELOPE_MARGIN = 0.10
# The envelope's perturbation set (owner, 2026-10-25): (the PCB's start along x [m], a settle wait before the closing path [s],
# the MG06 handle hinge's start [rad]) - the PCB -5 / -1 / 0 / +1 / +5 um without a wait, -5 / 0 / +5 um with a 0.1 s wait,
# and the handle +-1 urad: 10 runs, the first the baseline run (+-1 um added to the owner's set: in the taught program they
# give 6 contact points at 247.3 N / 11.45 N m, which the other runs miss, 2026-10-25). The wait goes after the line
# CONST_WAIT_AFTER names, in a copy of the program.
#   Unit [List<Tuple<float>>] (m, s, rad), [Dict<string, string>]
CONST_PERTURBATIONS = [(0.0, 0.0, 0.0), (-5e-6, 0.0, 0.0), (-1e-6, 0.0, 0.0), (1e-6, 0.0, 0.0), (5e-6, 0.0, 0.0), (-5e-6, 0.1, 0.0),
                       (0.0, 0.1, 0.0), (5e-6, 0.1, 0.0), (0.0, 0.0, 1e-6), (0.0, 0.0, -1e-6)]
CONST_WAIT_AFTER = {'taught': '- Robot.MoveJ(P_MG06_FORK_START)', 'optimized': '- Loader.WaitDone(J_PARK)'}
# The physics runs are independent, so they run in a pool of processes, at most this many at once; the results come back in
# the runs' order, so every report, metric and printed line is the sequential run's byte for byte (2026-10-25, Apple M2 Pro:
# the 20 runs of --out 369.5 s -> 63.0 s, the compare of both programs 37.6 s -> 19.8 s; identical sha256).
#   Unit [int]
CONST_WORKERS = os.cpu_count() or 1

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
    p = r.Place if r.Place is not None else Place_Physics_Str()
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
            'fork_points': int(r.Fork_Points), 'handle_load': float(r.Handle_Load), 'joint_torque': [float(x) for x in r.Joint_Torque],
            'free_loader_distance': float(closest[0]) if closest else -1.0,
            'place_drop': float(p.Drop), 'place_transient': float(p.Transient), 'seat_supports': list(p.Supports),
            'seat_tilt_deg': float(np.degrees(p.Tilt)) if np.isfinite(p.Tilt) else -1.0, 'place_guide_min': float(min(p.Guide_Min, 1.0)),
            'handover_centring': float(np.nan_to_num(p.Centring, nan=-1.0)), 'regrip_gap': float(np.nan_to_num(p.Regrip_Gap, nan=-1.0))}


def _Trajectory_Path(program: str, trajectories: tp.Optional[str]) -> tp.Optional[str]:
    """
    Description:
        A program's trajectory file in the folder trajectories, or None for run_program.py's default
        (Data/Generated/Trajectories).

    Args:
        (1) program [string]: 'taught' or 'optimized'.
        (2) trajectories [string]: The folder of the trajectory files, or None.

    Returns:
        (1) parameter [string]: The file, or None.
    """

    if not trajectories:
        return None
    return os.path.join(trajectories, os.path.basename(_Script('Simulation/run_program.py').CONST_VERSIONS[program]).replace('.yaml', '.npz'))


def _Wait_Variant(program: str, wait: float, trajectories: tp.Optional[str] = None) -> tp.Tuple[tp.Any, ...]:
    """
    Description:
        A program with a settle wait before the closing path (CONST_WAIT_AFTER), built now: what Capture runs. The variant's
        file lives in a temporary folder removed once it is built.

    Args:
        (1) program [string]: 'taught' or 'optimized'.
        (2) wait [float]: The wait (s).
        (3) trajectories [string]: The folder of the trajectory files (default Data/Generated/Trajectories), or None.

    Returns:
        (1) parameter [Tuple]: (cell, pcb_meta, program, trajectory, free_steps, model).
    """

    run_program = _Script('Simulation/run_program.py')
    with open(run_program.CONST_VERSIONS[program], encoding='utf-8') as handle:
        lines = handle.read().splitlines(True)
    at = [i for i, line in enumerate(lines) if line.strip().startswith(CONST_WAIT_AFTER[program])]
    if len(at) != 1:
        raise run_program.Run_Error_Cls('%s: %d lines start with %r, not 1' % (program, len(at), CONST_WAIT_AFTER[program]))
    lines.insert(at[0] + 1, lines[at[0]][:len(lines[at[0]]) - len(lines[at[0]].lstrip())] + '- Wait(%g)\n' % wait)
    with tempfile.TemporaryDirectory() as folder:
        path = os.path.join(folder, os.path.basename(run_program.CONST_VERSIONS[program]))
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(''.join(lines))
        cell, meta, constants = Load_Cell_Configuration(), Read_PCB_Meta(), Load_Constants()
        variant = Load_Program(constants, Load_Simulation(constants), cell, path)
        planning = Build_Planning_Model(cell)
        trajectory, _ = Build_Program(variant, cell, meta, Robot_Kinematics_Cls(planning, Robot_Collision_Cls(planning).Data), {})
    shipped = run_program.Load_Run(program, _Trajectory_Path(program, trajectories))
    return cell, meta, variant, trajectory, shipped[4], Build_Cell_Model(cell)


def Capture(program: str, trajectories: tp.Optional[str] = None,
            perturbation: tp.Tuple[float, float, float] = (0.0, 0.0, 0.0)) -> tp.Dict[str, tp.Any]:
    """
    Description:
        A program's physics run now (its trajectory must be current, as run_program.py's), unperturbed or with one of
        CONST_PERTURBATIONS: the report, the metrics, the acceptance.

    Args:
        (1) program [string]: 'taught' or 'optimized'.
        (2) trajectories [string]: The folder of the trajectory files (default Data/Generated/Trajectories), or None.
        (3) perturbation [Tuple<float>]: The PCB's start along x (m), a settle wait before the closing path (s; the program
            built again with it), the handle hinge's start (rad).

    Returns:
        (1) parameter [Dict<string, Any>]: 'report' (List<string>), 'metrics' (Dict), 'perturbation', 'acceptance'
            (List<Tuple<string, bool, string>>).
    """

    pcb, wait, handle = perturbation
    if wait:
        cell, meta, prog, trajectory, free, model = _Wait_Variant(program, wait, trajectories)
    else:
        cell, meta, prog, trajectory, free, model, _ = _Script('Simulation/run_program.py').Load_Run(program, _Trajectory_Path(program, trajectories))
    run = Physics_Run_Cls(model, Trajectory_Drive_Cls(trajectory, prog, free), meta, cell)
    if pcb or handle:
        run.Data.qpos[run.qpcb] += pcb
        run.Data.qpos[run.qh] += handle
        mujoco.mj_forward(model, run.Data)
    while not run.Done:
        run.Step()
    result = run.Result()
    return {'report': Format_Program_Report(result, cell, prog.Name), 'metrics': Metrics(result), 'perturbation': list(perturbation),
            'acceptance': [(c.Name, c.Passed, c.Detail) for c in Check_Program_Acceptance(result, cell)]}


def _Capture_Job(job: tp.Tuple[str, tp.Optional[str], tp.Tuple[float, float, float]]) -> tp.Tuple[bool, tp.Any]:
    """
    Description:
        One Capture in a pool process. A run that cannot run comes back as its sentence: the scripts' error classes are loaded
        from a file path, so they cannot be unpickled in the parent.

    Args:
        (1) job [Tuple]: (program, trajectories, perturbation), as Capture takes them.

    Returns:
        (1) parameter [Tuple<bool, Any>]: (True, the capture) or (False, the error's sentence).
    """

    try:
        return True, Capture(*job)
    except (_Script('Simulation/run_program.py').Run_Error_Cls, Simulation_Error_Cls) as error:
        return False, str(error)


def Capture_All(jobs: tp.List[tp.Tuple[str, tp.Optional[str], tp.Tuple[float, float, float]]]) -> tp.List[tp.Tuple[bool, tp.Any]]:
    """
    Description:
        Capture every job, in a pool of up to CONST_WORKERS processes (spawned: each a fresh interpreter, MuJoCo's log routed
        to logger/), the results in the jobs' order. For this script run as a program (a spawned process imports it by path).

    Args:
        (1) jobs [List<Tuple>]: (program, trajectories, perturbation) each.

    Returns:
        (1) parameter [List<Tuple<bool, Any>>]: _Capture_Job's result per job.
    """

    if len(jobs) == 1:
        return [_Capture_Job(jobs[0])]
    with multiprocessing.get_context('spawn').Pool(min(len(jobs), CONST_WORKERS), initializer=Configure_MuJoCo_Log) as pool:
        return pool.map(_Capture_Job, jobs, chunksize=1)


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


def Write_Baseline(program: str, captures: tp.List[tp.Dict[str, tp.Any]], folder: str) -> None:
    """
    Description:
        Write a program's baseline (the report and the metrics of its run, with the envelope over the perturbation runs) and the
        environment record into a folder.

    Args:
        (1) program [string]: 'taught' or 'optimized'.
        (2) captures [List<Dict<string, Any>>]: As Capture, over CONST_PERTURBATIONS (the baseline run first).
        (3) folder [string]: The folder.
    """

    captured = captures[0]
    runs = [c['perturbation'] + [c['metrics']['fork_points'], c['metrics']['fork_net_force'], c['metrics']['handle_load']] for c in captures]
    os.makedirs(folder, exist_ok=True)
    with open(os.path.join(folder, 'baseline_report_%s.txt' % program), 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(captured['report']) + '\n')
    with open(os.path.join(folder, 'baseline_metrics_%s.json' % program), 'w', encoding='utf-8') as handle:
        handle.write(json.dumps(dict(captured['metrics'], envelope=Envelope(captures), envelope_runs=runs), indent=1) + '\n')
    with open(os.path.join(folder, 'baseline_environment.json'), 'w', encoding='utf-8') as handle:
        handle.write(json.dumps(Environment(), indent=1, sort_keys=True) + '\n')


def Envelope(captures: tp.List[tp.Dict[str, tp.Any]]) -> tp.Dict[str, tp.List[tp.Any]]:
    """
    Description:
        The envelope of CONST_ENVELOPE's metrics over captures: per metric [min, max] (a vector's per element).

    Args:
        (1) captures [List<Dict<string, Any>>]: As Capture.

    Returns:
        (1) parameter [Dict<string, List>]: Metric -> [min, max].
    """

    out = {}
    for key in CONST_ENVELOPE:
        values = np.array([c['metrics'][key] for c in captures], float)
        out[key] = [values.min(axis=0).tolist(), values.max(axis=0).tolist()]
    return out


def Drift(now: tp.Dict[str, tp.Any], baseline: tp.Dict[str, tp.Any]) -> tp.List[str]:
    """
    Description:
        The metrics that drift from the baseline beyond CONST_TOLERANCES, one line each (a changed length counts as a drift);
        CONST_ENVELOPE's against the baseline's envelope +- CONST_ENVELOPE_MARGIN (their other elements against the baseline run).

    Args:
        (1 - 2) now, baseline [Dict<string, Any>]: Metrics (Metrics; the baseline's with 'envelope').

    Returns:
        (1) parameter [List<string>]: The drifts; empty when every metric is within its tolerance.
    """

    out = []
    for key, (kind, value) in CONST_TOLERANCES.items():
        if key not in baseline or (key in CONST_ENVELOPE and key not in baseline.get('envelope', {})):
            out.append('%s: not in the baseline (re-capture it)' % key)
            continue
        a, b = now[key], baseline[key]
        if key in CONST_ENVELOPE:
            low, high = (np.atleast_1d(np.asarray(x, float)) for x in baseline['envelope'][key])
            v = np.atleast_1d(np.asarray(a, float))
            picked = np.arange(v.size) if CONST_ENVELOPE[key] is None else np.array(CONST_ENVELOPE[key])
            margin = CONST_ENVELOPE_MARGIN
            outside = [int(i) for i in picked if not low[i] * (1 - margin) - 1e-12 <= v[i] <= high[i] * (1 + margin) + 1e-12]
            if outside:
                out.append('%s: %s outside the envelope %s .. %s +- %g' % (key, np.round(v[outside], 6).tolist(), np.round(low[outside], 6).tolist(),
                                                                         np.round(high[outside], 6).tolist(), CONST_ENVELOPE_MARGIN))
            if CONST_ENVELOPE[key] is None:
                continue
            rest = [i for i in range(v.size) if i not in CONST_ENVELOPE[key]]
            a, b = [a[i] for i in rest], [b[i] for i in rest]
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
        A capture against its baseline: the failures (every failed SPEC §10.1 criterion - whatever the envelope -, every drift)
        and, for information, the report's unified diff.

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
        criterion fails or a metric drifts beyond CONST_TOLERANCES (CONST_ENVELOPE's beyond the envelope); the report's diff is
        printed for information. --out DIR writes the baselines (a re-capture, in its own commit with the reason): each program run
        over CONST_PERTURBATIONS, refused when one fails SPEC §10.1. Every run first, in parallel (Capture_All); then each
        program's in order, as one after the other would print and write them.

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
    failed, programs = False, [args.program] if args.program else CONST_PROGRAMS
    perturbations = CONST_PERTURBATIONS if args.out is not None else CONST_PERTURBATIONS[:1]
    results = iter(Capture_All([(program, args.trajectories, x) for program in programs for x in perturbations]))
    for program in programs:
        runs = [next(results) for _ in perturbations]
        ran, now = runs[0]
        if not ran:
            print(log.Line('[ERROR] %s: the physics run cannot run: %s' % (program, now)))
            log.Close()
            return 2
        if args.out is not None:
            if not all(ran for ran, _ in runs):
                raise Simulation_Error_Cls('; '.join(message for ran, message in runs if not ran))
            captures = [capture for _, capture in runs]
            failing = [(c['perturbation'], [name for name, ok, _ in c['acceptance'] if not ok]) for c in captures
                       if not all(ok for _, ok, _ in c['acceptance'])]
            if failing:
                print(log.Line('[ERROR] %s: perturbation runs fail SPEC §10.1 (PCB m, wait s, handle rad: items): %s; no baseline written.' % (
                    program, '; '.join('%s: %s' % (p, ', '.join(names)) for p, names in failing))))
                log.Close()
                return 2
            Write_Baseline(program, captures, args.out)
            envelope = Envelope(captures)
            print(log.Line('[INFO] %s: wrote its baseline into %s (cycle %.3f s; %d of %d criteria pass; the envelope over %d runs: net force '
                           '%.1f-%.1f N, hinge load %.2f-%.2f N m; contact points %s).' % (
                               program, args.out, now['metrics']['cycle_time'], sum(ok for _, ok, _ in now['acceptance']), len(now['acceptance']),
                               len(captures), *envelope['fork_net_force'], *envelope['handle_load'],
                               '/'.join(str(c['metrics']['fork_points']) for c in captures))))
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
