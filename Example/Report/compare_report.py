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
# File Name: ../Example/Report/compare_report.py
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
# Functools (Higher-order functions: the scripts loaded once)
import functools
# Importlib (Import another script as a module)
import importlib.util
# Math (Mathematical functions)
import math
# Multiprocessing (The programs analysed in parallel)
import multiprocessing
# Time (Wall-clock timing)
import time
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Configuration/Core - the logs, the folders
from Configuration.Core import Configure_MuJoCo_Log, Script_Log_Cls, CONST_REPORTS
#   ../Configuration/Const - the repository root
from Configuration.Const import CONST_ROOT
#   ../Configuration/Program_Parser - the PCB locations, the optimizer's settings, the free transfers
from Configuration.Program_Parser import CONST_PCB_LOCATIONS, Load_Optimization
#   ../Kinematics/Core, ../Collision/Core - the planning model
from Kinematics.Core import Robot_Kinematics_Cls
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
#   ../Keypoints/Core - the IK family
from Keypoints.Core import Family
#   ../Program/Check - the loader where its actuators put it
from Program.Check import Gating_Faults, Swept_Pairs, Loader_Actual_Trajectory, CONST_GATING_STRIDE
#   ../Simulation/Core, ../Simulation/Report - the drive, the report and its acceptance
from Simulation.Core import Trajectory_Drive_Cls, Simulation_Error_Cls
from Simulation.Report import Format_Program_Report, Check_Program_Acceptance, Free_Loader_Distance
#   ../Optimizer/Core, ../Optimizer/Measure - the transfers, the LOCKED comparison, the interlocks, the metrics
from Optimizer.Core import Free_Transfers
from Optimizer.Measure import Compare_Locked, Interlocks, Measure, Step_Bounds
#   ../Trajectory/Core - the sample time
from Trajectory.Core import CONST_DT
#   ../Report/Style - the figures' themes, the programs' display names
from Report.Style import (Theme_Str, Parameters, Panel_Letter, Save_Figure, Label, Contrast, CONST_THEMES, CONST_PROGRAM_NAMES, CONST_DOUBLE,
                          CONST_BOUND)
#   ../Report/Core - the comparison
from Report.Core import (CONST_CUT, Joint_Arc, CONST_MG06_LOADER, Swept_Zone, Record_Physics, Cell_Geometry_Cls, Geometry_Series, By_Design,
                         Handovers,
                         Custody_Gaps, Location_Spans, Overhang,
                         Critical_Path, Robot_Waits, Signal_Order, Wrist_Approach, Stretched_Distance, Clear_Height, Job_Window, Moving, Spans,
                         Object_Channels, Travel, Channel_Peaks, Tilt, Holder_Matrix, CONST_PAIRS, CONST_HOLDERS, CONST_LOCATION_HOLDER,
                         CONST_ROBOT_LINKS, CONST_ZONE_BODIES)

"""
Description:
    Initialization of constants.
"""
# The programs compared, in the report's column order.
#   Unit [Tuple<string>]
CONST_VERSIONS = ('taught', 'optimized')
# The trajectory files of the programs in a --trajectories folder (build_program.py's names).
#   Unit [Dict<string, string>]
CONST_TRAJECTORY_FILES = {'taught': 'Program.npz', 'optimized': 'Program_Optimized.npz'}
# The shared zones (owner, 2026-10-25): the volume the loader's jaws (and the PCB in them, Report.Core CONST_ZONE_BODIES) sweep
# in its jobs, split by its Y (owner, F16): the jaw zone at the handover and take position (L_HOME's Y: from home to the travel
# height, the jaws open and closed) and the MG06 corridor (L_TRAVEL's Y: from the travel height down to the holder); each with
# its title and the loader position whose Y it keeps. The carriages are left out: they reach far above and below the jaws (the
# Z carriage over z -0.49..0.25 m), so their volume at the MG06 Y covers the handover too; the robot against them is the
# gating rule's and I8's.
#   Unit [List<string>], [Dict<string, Tuple<string, string>>]
CONST_SWEPT_JOBS = ['J_CLAMP', 'J_INSERT', 'J_PARK', 'J_FETCH', 'J_RETURN_Y', 'J_RETURN_Z', 'J_RELEASE']
CONST_SWEPT_PARTS = {'jaw': ('jaw zone', 'L_HOME'), 'corridor': ('MG06 corridor', CONST_MG06_LOADER)}
# The summary chart's name in --media (the README's).
#   Unit [string]
CONST_MEDIA_SUMMARY = 'Default_vs_Optimized'
# The default folder of the report and its figures.
#   Unit [string]
CONST_OUT = os.path.join(CONST_REPORTS, 'Comparison')
# The loader slow-downs tried against I8 (owner, 2026-10-25: +10 / +25 / +50 %; faster ones and a finer step around the
# first failure too), and the bisection's resolution for each bound's first failure.
#   Unit [List<float>] (-), [float] (-)
CONST_FACTORS = [0.5, 0.75, 0.9, 1.0, 1.02, 1.05, 1.1, 1.25, 1.5, 2.0]
CONST_FACTOR_RESOLUTION = 0.001
# The robot's speed overrides tried against I8, the loader as programmed (owner, F16: 0.5 / 0.75 / 0.9; 0.25 and 0.1 too, the
# overrides a first run on a real cell uses).
#   Unit [List<float>] (-)
CONST_OVERRIDES = [0.1, 0.25, 0.5, 0.75, 0.9, 1.0]
# The PCB's acceleration is also given smoothed over this window (contact transients are single samples).
#   Unit [int] (samples)
CONST_SMOOTH = 5
# The robot's waits in the timeline: a hatch per kind.
#   Unit [Dict<string, string>]
CONST_WAIT_HATCH = {'loader': '////', 'loader (timed wait)': 'xxxx', 'tester': '....', 'vacuum signal': '\\\\\\\\', 'robot dwell': '----'}
# The project's sentence on every result (feature-manager project reference).
#   Unit [string]
CONST_DISCLAIMER = 'Outputs (keypoints, programs, trajectories) are simulation results and are NOT validated for driving the real cell.'


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


def Load(version: str, trajectories: tp.Optional[str]) -> tp.Tuple[tp.Any, ...]:
    """
    Description:
        A program's run inputs (run_program.py's Load_Run: refused when stale), its trajectory from a folder or the default.

    Args:
        (1) version [string]: 'taught' or 'optimized'.
        (2) trajectories [string]: The folder of Program.npz and Program_Optimized.npz, or None (Data/Generated/Trajectories).

    Returns:
        (1) parameter [Tuple]: Load_Run's (cell, pcb_meta, program, trajectory, free_steps, model, trajectory_path).
    """

    path = os.path.join(trajectories, CONST_TRAJECTORY_FILES[version]) if trajectories else None
    return _Script('Simulation/run_program.py').Load_Run(version, path)


def Planning_Tools(cell: tp.Any) -> tp.Tuple[tp.Any, Robot_Kinematics_Cls, Robot_Collision_Cls]:
    """
    Description:
        The planning model (the check model) with its kinematics and collision on one data.

    Args:
        (1) cell [Cell_Configuration_Str(object)]: The cell.

    Returns:
        (1 - 3) parameter [Tuple]: model, kinematics, collision.
    """

    model = Build_Planning_Model(cell)
    collision = Robot_Collision_Cls(model)
    return model, Robot_Kinematics_Cls(model, collision.Data), collision


def Zones(cell: tp.Any, trajectories: tp.List[tp.Any]) -> tp.Dict[str, tp.Tuple[np.ndarray, np.ndarray]]:
    """
    Description:
        The jaw zone (Report.Core CONST_ZONE_BODIES, owner 2026-10-25) over every sample of both trajectories with the loader at
        its start position (home, the jaws open and clamped), grown by Loader_Free: the jaws' box, a cross-check in physics. The
        shared zones are swept volumes (Swept_Analysis).

    Args:
        (1) cell [Cell_Configuration_Str(object)]: The cell.
        (2) trajectories [List<Trajectory_Str>]: Both programs' trajectories.

    Returns:
        (1) parameter [Dict<string, Tuple<Vector<float> 1x3>>]: Per zone its lower and upper corner (m).
    """

    _, kinematics, collision = Planning_Tools(cell)
    geometry = Cell_Geometry_Cls(kinematics, collision, cell.Loader_Free)
    for tr in trajectories:
        home = np.flatnonzero((tr.Loader_Y == tr.Loader_Y[0]) & (tr.Loader_Z == tr.Loader_Z[0]))
        for i in np.unique(np.r_[home[::25], home[np.flatnonzero(Moving(tr.Jaw[home]))]]):
            geometry.Pose_Sample(tr, int(i))
            geometry.Grow_Zone('jaw zone')
    return geometry.Zones


def Analyse_Physics(job: tp.Tuple[str, tp.Dict[str, tp.Tuple[np.ndarray, np.ndarray]]]) -> tp.Dict[str, tp.Any]:
    """
    Description:
        One program's physics analysis (a pool job): its physics run recorded every 4 ms (Report.Core.Record_Physics: the
        run's own report and acceptance), the cell's geometry along the recording (every 20 ms, minima refined), the PCB's
        handovers and custody gaps.

    Args:
        (1) job [Tuple]: (program 'taught' / 'optimized', the zones, the trajectories' folder or None).

    Returns:
        (1) parameter [Dict<string, Any>]: The results by name.
    """

    version, zones, trajectories = job
    began = time.perf_counter()
    cell, meta, program, tr, free, model, path = Load(version, trajectories)
    result, record = Record_Physics(model, Trajectory_Drive_Cls(tr, program, free), meta, cell)
    physics_time = time.perf_counter() - began
    planning, kinematics, collision = Planning_Tools(cell)
    geometry = Cell_Geometry_Cls(kinematics, collision, cell.Loader_Free)
    geometry.Zones = dict(zones)
    location = lambda i: CONST_PCB_LOCATIONS[tr.PCB_Location[i]]
    n = min(len(tr.T), len(record.T))
    physics = Geometry_Series(geometry, lambda i: geometry.Pose_Record(record.Qpos[i]), n, location, By_Design(tr, program), record.T)
    pcb_address = int(planning.jnt_qposadr[planning.joint('pcb_free').id])
    return {'version': version, 'name': program.Name, 'path': os.path.relpath(path, CONST_ROOT),
            'report': Format_Program_Report(result, cell, program.Name),
            'acceptance': [(c.Name, c.Passed, c.Detail) for c in Check_Program_Acceptance(result, cell)], 'result': result, 'record': record,
            'physics': physics, 'handovers': Handovers(tr, record, pcb_address), 'gaps': Custody_Gaps(record, pcb_address),
            'physics_time': physics_time, 'time': time.perf_counter() - began}


def Analyse_Planned(job: tp.Tuple[str, tp.Dict[str, tp.Tuple[np.ndarray, np.ndarray]]]) -> tp.Dict[str, tp.Any]:
    """
    Description:
        One program's kinematic analysis (a pool job): the cell's geometry along the trajectory (every 20 ms, minima
        refined), the overhang of the PCB beyond the cups and the jaws, the comparison's metrics, the interlocks.

    Args:
        (1) job [Tuple]: (program 'taught' / 'optimized', the zones, the trajectories' folder or None).

    Returns:
        (1) parameter [Dict<string, Any>]: The results by name.
    """

    version, zones, trajectories = job
    began = time.perf_counter()
    cell, _, program, tr, _, _, _ = Load(version, trajectories)
    planning, kinematics, collision = Planning_Tools(cell)
    geometry = Cell_Geometry_Cls(kinematics, collision, cell.Loader_Free)
    geometry.Zones = dict(zones)
    location = lambda i: CONST_PCB_LOCATIONS[tr.PCB_Location[i]]
    planned = Geometry_Series(geometry, lambda i: geometry.Pose_Sample(tr, i), len(tr.T), location, By_Design(tr, program), tr.T)
    overhang = {}
    for name, a, b in Location_Spans(tr):
        if name in ('cups', 'jaws') and name not in overhang:
            geometry.Pose_Sample(tr, (a + b) // 2)
            supports = set(CONST_ROBOT_LINKS[-1][1:]) if name == 'cups' else {'jaw_L', 'jaw_R'}
            overhang[name] = Overhang(planning, kinematics.Data, supports)
    metrics = Measure(tr.Robot_Q, CONST_DT, kinematics, tr.Loader_Y, tr.Loader_Z, tr.PCB_Pose[:, :3])
    return {'planned': planned, 'overhang': overhang, 'metrics': metrics, 'interlocks': Interlocks(tr, kinematics, collision, cell, program),
            'gating': Gating_Faults(tr, kinematics, collision, cell),
            'planned_time': time.perf_counter() - began}


def Stretch_Analysis(job: tp.Tuple[str, tp.Optional[str]]) -> tp.Dict[str, tp.Any]:
    """
    Description:
        The interlock I8 with the loader slowed and with the robot slowed (a pool job): for every parallel loader job of the
        program (Loader.Start) the closest robot - loader distance (Report.Core.Stretched_Distance) at every factor of
        CONST_FACTORS (the job slowed) and every speed override of CONST_OVERRIDES (the robot's lines in the job's window slowed,
        F16); for the job that comes closest in each, the first slow-down at which it falls below each bound (Loader_Free +
        Tracking_Margin, Loader_Free, the acceptance's hard floor, contact), bisected to CONST_FACTOR_RESOLUTION; the insert's gate
        (F16): the exit leg before J_INSERT starts, the job's first move, the exit point's distance to all J_INSERT sweeps
        (Program.Check.Swept_Pairs); and the take's gate (F15): the loader heights
        at home Y that keep the take's spline clear (Report.Core.Clear_Height), the spline's start against J_RETURN_Y's end.

    Args:
        (1) job [Tuple<string, string>]: (the program, the trajectories' folder or None).

    Returns:
        (1) parameter [Dict<string, Any>]: The results by name.
    """

    version, trajectories = job
    cell, _, program, tr, _, _, _ = Load(version, trajectories)
    model, kinematics, collision = Planning_Tools(cell)
    jobs = [text[len('Loader.Start('):-1] for text in tr.Segment_Command if text.startswith('Loader.Start(')]
    out: tp.Dict[str, tp.Any] = {'jobs': jobs, 'rows': {}, 'first': {}, 'bounds': {}, 'robot': {'rows': {}, 'first': {}}}
    if not jobs:
        return out
    out['bounds'] = {'Loader_Free + Tracking_Margin (I8)': cell.Loader_Free + cell.Tracking_Margin, 'Loader_Free (item 16)': cell.Loader_Free,
                     'Acceptance.Loader_Distance (item 11)': cell.Loader_Distance, 'contact': 0.0}
    for lane, factors, into in (('loader', CONST_FACTORS, out), ('robot', [1.0 / o for o in CONST_OVERRIDES], out['robot'])):
        for job in jobs:
            into['rows'][job] = [(f,) + Stretched_Distance(tr, job, f, kinematics, collision, lane) for f in factors]
        worst = min(jobs, key=lambda j: min(r[1] for r in into['rows'][j]))
        into['worst'] = worst
        for name, bound in out['bounds'].items():
            rows = sorted(r for r in into['rows'][worst] if r[0] >= 1.0)
            fail = next((r[0] for r in rows if r[1] < bound), None)
            if fail is None:
                into['first'][name] = None
                continue
            lo, hi = max([r[0] for r in rows if r[0] < fail] + [1.0]), fail
            while hi - lo > CONST_FACTOR_RESOLUTION * hi:
                mid = 0.5 * (lo + hi)
                lo, hi = (mid, hi) if Stretched_Distance(tr, worst, mid, kinematics, collision, lane)[0] >= bound else (lo, mid)
            into['first'][name] = (hi, Stretched_Distance(tr, worst, hi, kinematics, collision, lane))
    texts = tr.Segment_Command
    if 'Loader.Start(J_INSERT)' in texts:
        k = texts.index('Loader.Start(J_INSERT)')
        leg = max(m for m in range(k) if texts[m].startswith('Robot.Move'))
        command = next(c for segment in program.Segments for c in segment.Commands if c.Text == texts[leg])
        a, _, last = Job_Window(tr, 'J_INSERT')
        moved = next(i for i in range(a, len(tr.T)) if tr.Loader_Y[i] != tr.Loader_Y[i - 1] or tr.Loader_Z[i] != tr.Loader_Z[i - 1])
        loader = sorted(set(range(a, last + 1, CONST_GATING_STRIDE)) | {last})
        distances, _ = Swept_Pairs(Loader_Actual_Trajectory(tr, model), kinematics, collision, [a], loader, 0.05)
        out['insert'] = {'set': command.Target if command.Kind == 'MoveSpline' else command.Text, 'point': command.Path_End or command.Target,
                         'duration': float(tr.Segment_End[leg] - tr.Segment_Start[leg]) * CONST_DT, 'arrived': float(tr.T[tr.Segment_End[leg] - 1]),
                         'moved': float(tr.T[moved]), 'distance': float(distances.min())}
    take = next((t for t in tr.Segment_Command if t.startswith('Robot.MoveSpline(W_TO_TAKE,')), None)
    if take is not None:
        k = tr.Segment_Command.index(take)
        start, _, last = Job_Window(tr, 'J_RETURN_Z')
        travel = float(np.max(tr.Loader_Z[Job_Window(tr, 'J_RETURN_Y')[0]:last + 1]))
        out['clear'] = Clear_Height(tr, take, kinematics, collision, np.linspace(0.0, travel, 9), cell.Loader_Free + cell.Tracking_Margin)
        out['take'] = {'spline_start': float(tr.T[tr.Segment_Start[k]]), 'spline': float(tr.Segment_End[k] - tr.Segment_Start[k]) * CONST_DT,
                       'y_done': float(tr.T[Job_Window(tr, 'J_RETURN_Y')[2]]), 'z_done': float(tr.T[last])}
    return out


def Swept_Analysis(job: tp.Tuple[str, str, tp.Optional[str]]) -> tp.Tuple[str, str, tp.Any]:
    """
    Description:
        A shared zone of a program (a pool job; owner 2026-10-25, F16): the volume the loader sweeps in its jobs
        (CONST_SWEPT_JOBS) where its actuators put it, grown by Loader_Free (Report.Core.Swept_Zone), at one Y of
        CONST_SWEPT_PARTS - the jaw zone at L_HOME's, the MG06 corridor at L_TRAVEL's - and who is in it when (the trajectory,
        every 20 ms).

    Args:
        (1) job [Tuple<string>]: (the program, 'jaw' or 'corridor', the trajectories' folder or None).

    Returns:
        (1) parameter [Tuple]: (the program, the part, the Swept_Zone_Str).
    """

    version, part, trajectories = job
    cell, _, program, tr, _, _, _ = Load(version, trajectories)
    _, kinematics, collision = Planning_Tools(cell)
    at_y = program.Constants.Loader[CONST_SWEPT_PARTS[part][1]].Y
    return version, part, Swept_Zone(tr, kinematics, collision, CONST_SWEPT_JOBS, cell.Loader_Free, at_y, bodies=CONST_ZONE_BODIES['jaw zone'])


def _Job(job: tp.Tuple[str, tp.Any]) -> tp.Any:
    """
    Description:
        A pool job by its kind: ('physics' or 'planned', (program, zones, folder)), ('stretch', (program, folder)) or ('swept',
        (program, part, folder)).

    Args:
        (1) job [Tuple<string, Any>]: The kind and its argument.

    Returns:
        (1) parameter [Any]: Its result.
    """

    kind, argument = job
    return {'physics': Analyse_Physics, 'planned': Analyse_Planned, 'stretch': Stretch_Analysis, 'swept': Swept_Analysis}[kind](argument)


def _Fmt(x: float, scale: float = 1.0, digits: int = 2) -> str:
    """
    Description:
        A number for the report: scaled, fixed digits, '-' for NaN or None.

    Args:
        (1) x [float]: The value.
        (2) scale [float]: Its factor (1000 for m -> mm).
        (3) digits [int]: Its decimals.

    Returns:
        (1) parameter [string]: The text.
    """

    return '-' if x is None or (isinstance(x, float) and math.isnan(x)) else '%.*f' % (digits, x * scale)


def _Table(header: tp.List[str], rows: tp.List[tp.List[str]]) -> tp.List[str]:
    """
    Description:
        A Markdown table.

    Args:
        (1) header [List<string>]: The column titles.
        (2) rows [List<List<string>>]: The cells.

    Returns:
        (1) parameter [List<string>]: Its lines.
    """

    return ['| ' + ' | '.join(header) + ' |', '|' + '|'.join('---' for _ in header) + '|'] + ['| ' + ' | '.join(r) + ' |' for r in rows]


def _Distance_Text(found: tp.Tuple[float, float, str], cut: float) -> str:
    """
    Description:
        A pair's minimum for the report: 'x mm at t s (pair)', or '> cut mm'.

    Args:
        (1) found [Tuple<float, float, string>]: (distance, time, pair) (m, s, -).
        (2) cut [float]: The measurement's cut (m).

    Returns:
        (1) parameter [string]: The text.
    """

    if found is None:
        return 'always by design'
    distance, t, pair = found
    if distance >= cut:
        return '> %.0f mm' % (1000 * cut)
    return '%.2f mm at %.2f s (%s)' % (1000 * distance, t, pair or '-')


class Context_Cls(object):
    """
    Description:
        Everything the report and the figures read, by program: the programs, trajectories and cell (loaded in this process)
        and the pool's analyses.

    Initialization of the Class:
        Args:
            (1) analyses [Dict<string, Dict>]: Analyse_Program's result per program.
            (2) stretch [Dict<string, Any>]: Stretch_Analysis's result for the optimized program.
            (3) zones [Dict<string, Tuple>]: The zones.
            (4) trajectories [string]: The trajectories' folder, or None.

        Example:
            Initialization:
                Cls = Context_Cls(analyses, stretch, zones, None)

            Features:
                # Properties of the class.
                Cls.Cell; Cls.Programs; Cls.Trajectories
    """

    def __init__(self, analyses: tp.Dict[str, tp.Dict[str, tp.Any]], stretch: tp.Dict[str, tp.Any],
                 zones: tp.Dict[str, tp.Tuple[np.ndarray, np.ndarray]], trajectories: tp.Optional[str]) -> None:
        self.Analyses, self.Stretch, self.Zones = analyses, stretch, zones
        loaded = {v: Load(v, trajectories) for v in CONST_VERSIONS}
        self.Cell = loaded['taught'][0]
        self.Programs = {v: loaded[v][2] for v in CONST_VERSIONS}
        self.Trajectories = {v: loaded[v][3] for v in CONST_VERSIONS}
        self.Free = {v: loaded[v][4] for v in CONST_VERSIONS}
        self.Paths = {v: os.path.relpath(loaded[v][6], CONST_ROOT) for v in CONST_VERSIONS}
        self.Model, self.Kinematics, self.Collision = Planning_Tools(self.Cell)
        self.Carry_Max_Tilt = Load_Optimization(self.Programs['taught']).Carry_Max_Tilt
        m = self.Model
        self.Robot_Dofs = np.array([m.jnt_dofadr[m.joint('aubo/' + j).id] for j in ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                                                                    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')])
        self.Robot_Q = np.array([m.jnt_qposadr[m.joint('aubo/' + j).id] for j in ('shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                                                                                  'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint')])
        self.Address = {n: (int(m.jnt_qposadr[m.joint(n).id]), int(m.jnt_dofadr[m.joint(n).id])) for n in
                        ('y_axis', 'z_axis', 'jaw_L', 'lid_hinge', 'handle_hinge', 'pcb_free')}


def Section_Same_Task(c: Context_Cls) -> tp.List[str]:
    """
    Description:
        Item 1: the same task - the start and end state, the LOCKED parts, the task points and their order, the points that
        differ, the IK families, each FREE transfer's joint turns, what a real cell would need re-taught.

    Args:
        (1) c [Context_Cls(object)]: The context.

    Returns:
        (1) parameter [List<string>]: The section's lines.
    """

    a, b = c.Trajectories['taught'], c.Trajectories['optimized']
    taught, optimized = c.Programs['taught'], c.Programs['optimized']
    lines = ['## 1. Same task', '']
    rows = []
    for label, i, j in (('start', 0, 0), ('end', -1, -1)):
        dq = float(np.max(np.abs(a.Robot_Q[i] - b.Robot_Q[j])))
        dl = max(abs(a.Loader_Y[i] - b.Loader_Y[j]), abs(a.Loader_Z[i] - b.Loader_Z[j]), abs(a.Gripper[i] - b.Gripper[j]))
        dp = float(np.linalg.norm(a.PCB_Pose[i, :3] - b.PCB_Pose[j, :3]))
        dm = max(abs(a.MG06_Lid[i] - b.MG06_Lid[j]), abs(a.MG06_Handle[i] - b.MG06_Handle[j]))
        same = CONST_PCB_LOCATIONS[a.PCB_Location[i]] == CONST_PCB_LOCATIONS[b.PCB_Location[j]] and a.Vacuum[i] == b.Vacuum[j]
        rows.append([label, '%.1e' % dq, '%.1e' % dl, '%.1e' % dp, '%.1e' % dm, '%s / %s%s' % (CONST_PCB_LOCATIONS[a.PCB_Location[i]],
                     CONST_PCB_LOCATIONS[b.PCB_Location[j]], '' if same else ' DIFFERENT')])
    lines += ['The cell\'s state at the first and the last sample, taught against optimized [K]:', '']
    lines += _Table(['sample', 'robot max abs dq (rad)', 'loader / gripper max abs d (m)', 'PCB position d (m)', 'MG06 max abs d (rad)',
                     'PCB location (taught / optimized)'], rows)
    transfers = Free_Transfers(taught, Load_Optimization(taught), a)
    locked = Compare_Locked(a, b, taught, transfers)
    lines += ['', 'Each LOCKED part (`Optimizer.Measure.Compare_Locked`: its samples relative to its own start, byte by byte) [K]:', '']
    lines += _Table(['LOCKED part', 'Default start (s)', 'Optimized start (s)', 'samples', 'identical'],
                    [[p.Name, '%.3f' % p.Start_Default, '%.3f' % p.Start_Optimized, str(p.Samples), 'yes' if p.Identical else 'NO (%s)' % p.Channels]
                     for p in locked])

    def Targets(program: tp.Any, free: tp.Set[str]) -> tp.Tuple[tp.List[str], tp.List[str]]:
        """
        Description:
            A program's robot lines outside its FREE transfers and loader-only segments (the task), and every point its
            motions use.

        Args:
            (1) program [Program_Str(object)]: The program.
            (2) free [Set<string>]: Its FREE segment keys.

        Returns:
            (1 - 2) parameter [List<string>]: The task lines in order; the points in order of first use.
        """

        task, points = [], []
        for segment in program.Segments:
            for command in segment.Commands:
                if command.Station == 'Robot' and segment.Key not in free and command.Kind not in ('Start', 'WaitDone'):
                    task.append(command.Text)
                for name in (command.Target, command.Path_Start, command.Path_End):
                    if name and name in program.Constants.Points and name not in points:
                        points.append(name)
        return task, points

    free_taught = c.Free['taught'] | {k for t in transfers for k in t.Job_Keys}
    task_a, points_a = Targets(taught, free_taught)
    task_b, points_b = Targets(optimized, c.Free['optimized'])
    lines += ['', 'The task - the robot\'s lines outside the FREE transfers, in order (the program files [C]): Default %d, Optimized %d '
              'lines, %s.' % (len(task_a), len(task_b), 'identical in text and order' if task_a == task_b else 'DIFFERENT'), '']
    jobs = {v: [t.split('(', 1)[1].split(')', 1)[0] for t in c.Trajectories[v].Segment_Command if t.startswith(('Loader.Run(', 'Loader.Start('))]
            for v in CONST_VERSIONS}
    jobs = {v: [j for k, j in enumerate(js) if j not in js[:k]] for v, js in jobs.items()}
    same_jobs = all([x.Text for x in taught.Loader_Jobs[j]] == [x.Text for x in optimized.Loader_Jobs.get(j, [])] for j in taught.Loader_Jobs)
    lines += ['The loader jobs in order [K]: taught %s; optimized %s; their lines %s [C].' % (', '.join(jobs['taught']), ', '.join(jobs['optimized']),
              'identical' if same_jobs else 'DIFFERENT'), '']
    only_a = [p for p in points_a if p not in points_b]
    only_b = [p for p in points_b if p not in points_a]
    rows = []
    for name in points_a + only_b:
        q = taught.Constants.Points[name].Joints
        rows.append([name, Family(q), '%.1f' % math.degrees(q[4]), 'both' if name in points_a and name in points_b else
                     ('taught only' if name in points_a else 'optimized only')])
    lines += ['The points the motions use, with their IK family (wrist_2 sign) [C]:', '']
    lines += _Table(['point', 'family', 'wrist_2 (deg)', 'used by'], rows)
    lines += ['', 'Each FREE transfer: the joint path in both programs - net change and travel per joint (deg) [K]. A travel larger than '
              'the net change is a turn back; a different sign of the net change would be a turn the other way.', '']
    rows = []
    for t in transfers:
        ka = [Step_Bounds(a, k.split('_', 1)[1]) for k in t.Keys + t.Job_Keys]
        a0, a1 = min(x[0] for x in ka), max(x[1] for x in ka)
        b0, b1 = Step_Bounds(b, t.Name)
        qa, qb = a.Robot_Q[a0:a1 + 1], b.Robot_Q[b0:b1 + 1]
        fam = lambda q: '%s -> %s' % (Family(q[0]), Family(q[-1]))
        for j in range(6):
            na, nb = math.degrees(qa[-1, j] - qa[0, j]), math.degrees(qb[-1, j] - qb[0, j])
            ta, tb = math.degrees(Travel(qa[:, j])[0]), math.degrees(Travel(qb[:, j])[0])
            rows.append([t.Name if j == 0 else '', fam(qa) if j == 0 else '', 'J%d' % (j + 1), '%+.1f / %.1f' % (na, ta), '%+.1f / %.1f' % (nb, tb),
                         'same' if abs(na - nb) < 0.1 else 'END DIFFERS'])
    lines += _Table(['transfer', 'family', 'joint', 'Default net / travel', 'Optimized net / travel', 'net'], rows)
    waypoints = [(cmd.Target, len(cmd.Waypoints) - 2) for s in optimized.Segments for cmd in s.Commands if cmd.Kind == 'MoveSpline']
    waits = [cmd.Text for s in optimized.Segments if s.Key in c.Free['optimized'] for cmd in s.Commands if cmd.Kind == 'Wait']
    lines += ['', 'What a real cell would need re-taught or verified for the optimized program (from the program files [C]):', '',
              '- Points only the optimized program uses: %s; points only the taught one uses (its vias, dropped by the transfers): %s.'
              % (', '.join(only_b) or 'none', ', '.join(only_a) or 'none'),
              '- %d MoveSpline transfers through %d joint-space via points (%s): generated, not taught; the AUBO controller\'s MoveSpline '
              'execution is not verified (KNOWN_ISSUES.md). Each would be imported as joint waypoints and run at reduced speed first.'
              % (len(waypoints), sum(n for _, n in waypoints), ', '.join('%s %d' % w for w in waypoints)),
              '- The parallel loader handshake: Loader.Start / Loader.WaitDone instead of Loader.Run - the loader PLC must accept a start '
              'while the robot moves and report done.',
              '- Timed waits inside the FREE transfers: %s - they stand for the loader\'s timing (section 4).' % (', '.join(waits) or 'none'),
              '- Every LOCKED part (the picks, places, handover, take and fork paths) is the taught program\'s, unchanged: nothing there to '
              're-teach beyond what the taught program needs.']
    return lines


def Section_Conditions(c: Context_Cls) -> tp.List[str]:
    """
    Description:
        Item 2: the same conditions - model, limits, speeds, blends, tools, payload, timing method, signals, interlocks.

    Args:
        (1) c [Context_Cls(object)]: The context.

    Returns:
        (1) parameter [List<string>]: The section's lines.
    """

    lines = ['## 2. Same conditions', '']
    rows = []
    kinds = {}
    for v in CONST_VERSIONS:
        tr, program = c.Trajectories[v], c.Programs[v]
        commands = [cmd for s in program.Segments for cmd in s.Commands]
        count = lambda kind: sum(1 for cmd in commands if cmd.Kind == kind)
        speeds = sorted({cmd.Speed for cmd in commands if cmd.Kind in ('MoveJ', 'MoveSpline')})
        tools = sorted({cmd.Site for cmd in commands if cmd.Site})
        ends = [k for k, text in enumerate(tr.Segment_Command) if text.startswith('Robot.Move')]
        blend = max(float(np.max(np.abs(tr.Robot_QD[tr.Segment_End[k] - 1]))) for k in ends)
        kinds[v] = {'model': tr.Meta['inputs_sha256']['cell'][:12], 'motions': 'MoveJ %d, MoveL %d, MoveSpline %d, MovePath %d' % (
            count('MoveJ'), count('MoveL'), count('MoveSpline'), count('MovePath')), 'speeds': ', '.join('%.2f' % s for s in speeds) or '-',
            'blend': '%.1e rad/s' % blend, 'tools': ', '.join(tools), 'signals': 'SetDO %d, WaitDI %d' % (count('SetDO'), count('WaitDI')),
            'loader': 'Run %d, Start %d, WaitDone %d' % (count('Run'), count('Start'), count('WaitDone')),
            'waits': ', '.join('%g s' % cmd.Duration for cmd in commands if cmd.Kind == 'Wait')}
    cell = c.Cell
    rows.append(['cell model and Cell.yaml (sha256 of Cell.yaml in the trajectory) [K]', kinds['taught']['model'], kinds['optimized']['model']])
    rows.append(['joint speed limit: datasheet x fraction (deg/s) [C]', ', '.join('%.0f' % math.degrees(s) for s in cell.Datasheet_Speeds) +
                 ' x %.2f' % cell.Speed_Fraction, 'the same; MoveSpline v= below'])
    rows.append(['joint acceleration limits (rad/s^2, an assumption) [C]', ', '.join('%.0f' % a for a in cell.Acceleration_Limits), 'the same'])
    rows.append(['MoveJ / MoveSpline speed fractions [C]', kinds['taught']['speeds'], kinds['optimized']['speeds']])
    rows.append(['motions [C]', kinds['taught']['motions'], kinds['optimized']['motions']])
    rows.append(['timing', 'MoveJ min-jerk (SPEC §7.9), MoveL min-jerk on a line, MovePath recorded', 'MoveSpline: RRT-Connect, shortcut, '
                 'cubic spline, TOPP (§7.10); MoveL and MovePath as taught'])
    rows.append(['blends: largest joint speed at a motion\'s end [K]', kinds['taught']['blend'], kinds['optimized']['blend']])
    rows.append(['tool sites [C]', kinds['taught']['tools'], kinds['optimized']['tools']])
    rows.append(['payload: the PCB (one model) [M]', '60 g, 120 x 85 x 1.8 mm', 'the same'])
    rows.append(['robot signals [C]', kinds['taught']['signals'], kinds['optimized']['signals']])
    rows.append(['loader handshake [C]', kinds['taught']['loader'], kinds['optimized']['loader']])
    rows.append(['robot Wait lines [C]', kinds['taught']['waits'], kinds['optimized']['waits']])
    lines += _Table(['condition', CONST_PROGRAM_NAMES['taught'], CONST_PROGRAM_NAMES['optimized']], rows)
    lines += ['', 'The interlocks I1-I10 (SPEC §7.11), verified on each built trajectory (`Optimizer.Measure.Interlocks`) [K]:', '']
    rows = []
    for (name, ok_a, text_a), (_, ok_b, text_b) in zip(c.Analyses['taught']['interlocks'], c.Analyses['optimized']['interlocks']):
        rows.append([name, ('holds' if ok_a else 'FAILS') + ': ' + text_a, ('holds' if ok_b else 'FAILS') + ': ' + text_b])
    lines += _Table(['interlock', CONST_PROGRAM_NAMES['taught'], CONST_PROGRAM_NAMES['optimized']], rows)
    lines += ['', 'What differs: the timing method of the FREE transfers (MoveJ through taught vias against planned splines), the loader '
              'handshake (sequential Run against parallel Start / WaitDone; every parallel part gated by the program\'s order, F15, F16). '
              'The model, '
              'the limits, the tools, the '
              'payload, the LOCKED motions and the signals are the same.']
    return lines


def Object_Rows(c: Context_Cls, v: str) -> tp.Dict[str, tp.Dict[str, str]]:
    """
    Description:
        Item 3's cells for one program: per moving object its metrics, each with its source.

    Args:
        (1) c [Context_Cls(object)]: The context.
        (2) v [string]: The program.

    Returns:
        (1) parameter [Dict<string, Dict<string, string>>]: Per object, per metric its text.
    """

    tr, an, cell, m = c.Trajectories[v], c.Analyses[v], c.Cell, c.Model
    record, physics, planned = an['record'], an['physics'], an['planned']
    channels = Object_Channels(tr)
    cycle = float(tr.T[-1])
    out: tp.Dict[str, tp.Dict[str, str]] = {}
    waits: tp.Dict[str, float] = {}
    for _, duration, kind, _ in Robot_Waits(tr):
        waits[kind] = waits.get(kind, 0.0) + duration
    for name, values in channels.items():
        moving = Moving(values)
        spans = Spans(moving, tr.T)
        speed, acceleration = Channel_Peaks(values, CONST_DT)
        travel = Travel(values)
        row = {'active (moving) [K]': '%.2f s' % (np.sum(moving) * CONST_DT), 'still [K]': '%.2f s' % (cycle - np.sum(moving) * CONST_DT),
               'moves (moving spans) [K]': str(len(spans))}
        if name == 'robot':
            row['robot waits by kind [K]'] = ', '.join('%s %.2f s' % (k, w) for k, w in sorted(waits.items()))
            splines = [cmd for s in c.Programs[v].Segments for cmd in s.Commands if cmd.Kind == 'MoveSpline']
            row['via points [C]'] = '%d (in %d MoveSpline)' % (sum(len(s.Waypoints) - 2 for s in splines), len(splines)) if splines else \
                '%d MoveJ to vias' % sum(1 for p in ('P_VIA_INPUT_LOADER', 'P_VIA_HOME_MG06', 'P_VIA_LOADER_OUTPUT') for t in tr.Segment_Command
                                         if t == 'Robot.MoveJ(%s)' % p)
            row['joint travel J1..J6 (deg) [K]'] = ', '.join('%.0f' % math.degrees(x) for x in travel)
            row['TCP path (m) [K]'] = '%.3f' % an['metrics'].Tcp_Path
            box = physics.Boxes['robot all']
            row['robot geoms: lowest / highest z (m) [P]'] = '%.3f / %.3f' % (box[:, 0, 2].min(), box[:, 1, 2].max())
            row['robot geoms: extent x, y (m) [P]'] = '%.3f..%.3f, %.3f..%.3f' % (box[:, 0, 0].min(), box[:, 1, 0].max(), box[:, 0, 1].min(),
                                                                                   box[:, 1, 1].max())
            lo, hi = c.Kinematics.Limits
            margin = np.minimum(np.min(values, axis=0) - lo, hi - np.max(values, axis=0))
            row['joint range used J1..J6 (deg) [K]'] = ', '.join('%.0f..%.0f' % (math.degrees(a), math.degrees(b)) for a, b in
                                                                 zip(np.min(values, axis=0), np.max(values, axis=0)))
            row['closest to a joint limit (deg) [K, M]'] = '%.1f (J%d; limits +-%.1f)' % (math.degrees(margin.min()), int(np.argmin(margin)) + 1,
                                                                                          math.degrees(hi[0]))
            limit = np.array(cell.Datasheet_Speeds)
            qd = np.max(np.abs(tr.Robot_QD), axis=0)
            row['peak joint speed J1..J6 (deg/s; % of datasheet) [K]'] = ', '.join('%.0f (%.0f%%)' % (math.degrees(s), 100 * s / l) for s, l in
                                                                                    zip(qd, limit))
            actual = np.max(np.abs(record.Qvel[:, c.Robot_Dofs]), axis=0)
            row['peak joint speed J1..J6 (deg/s) [P]'] = ', '.join('%.0f' % math.degrees(s) for s in actual)
            dqd = np.abs(np.diff(tr.Robot_QD, axis=0)) / CONST_DT
            paths = np.zeros(len(dqd), bool)
            for text, a, b in zip(tr.Segment_Command, tr.Segment_Start, tr.Segment_End):
                if text.startswith('Robot.MovePath'):
                    paths[max(a - 1, 0):b] = True
            for label, mask in (('', np.ones(len(dqd), bool)), (' outside the recorded fork paths', ~paths)):
                acc = np.max(dqd[mask], axis=0)
                row['peak joint acceleration%s J1..J6 (rad/s^2; %% of the assumed limit) [K]' % label] = ', '.join(
                    '%.1f (%.0f%%)' % (x, 100 * x / l) for x, l in zip(acc, cell.Acceleration_Limits))
            for scope, (w, t, line) in Wrist_Approach(tr).items():
                row['closest |wrist_2| %s [K]' % scope] = '%.2f deg at %.2f s (%s)' % (math.degrees(w), t, line)
            for pair in ('robot - environment', 'robot - own links', 'robot - MG06 lid/handle/press', 'robot - loader'):
                row['min clearance %s [P] / [K]' % pair] = '%s / %s' % (_Distance_Text(physics.Minimum.get(pair), CONST_CUT),
                                                                         _Distance_Text(planned.Minimum.get(pair), CONST_CUT))
        else:
            unit, scale = ('deg', math.degrees(1.0)) if name.startswith('MG06') else ('mm', 1000.0)
            row['travel (%s) [K]' % unit] = ', '.join('%.1f' % (x * scale) for x in travel)
            row['range used (%s) [K]' % unit] = ', '.join('%.1f..%.1f' % (a * scale, b * scale)
                                                          for a, b in zip(np.min(values, axis=0), np.max(values, axis=0)))
            row['peak speed (%s/s) [K]' % unit] = ', '.join('%.1f' % (x * scale) for x in speed)
            row['peak acceleration (%s/s^2) [K]' % unit] = ', '.join('%.0f' % (x * scale) for x in acceleration)
            joint = {'loader Y': 'y_axis', 'loader Z': 'z_axis', 'loader jaws': 'jaw_L'}.get(name)
            if joint is not None:
                q, d = c.Address[joint]
                rng = m.jnt_range[m.joint(joint).id]
                row['axis range of the model (mm) [M]'] = '%.1f..%.1f' % (1000 * rng[0], 1000 * rng[1])
                row['peak speed (mm/s) [P]'] = '%.1f' % (1000 * np.max(np.abs(record.Qvel[:, d])))
                posed = {'loader Y': tr.Loader_Y, 'loader Z': tr.Loader_Z, 'loader jaws': tr.Jaw}[name]
                k = min(len(posed), len(record.T))
                label = 'lag behind the command' if name != 'loader jaws' else 'off the posed jaws (Jaw_Actual)'
                row['%s max (mm) [P]' % label] = '%.1f' % (1000 * np.max(np.abs(record.Qpos[:k, q] - posed[:k])))
            if name == 'loader Y':
                box = physics.Boxes['loader']
                row['loader (Z carriage, jaws): lowest / highest z (m) [P]'] = '%.3f / %.3f' % (box[:, 0, 2].min(), box[:, 1, 2].max())
                for pair in ('loader - environment', 'loader - MG06 lid/handle/press'):
                    row['min clearance %s [P] / [K]' % pair] = '%s / %s' % (_Distance_Text(physics.Minimum.get(pair), CONST_CUT),
                                                                             _Distance_Text(planned.Minimum.get(pair), CONST_CUT))
            if name == 'loader jaws':
                row['jaw speed limit (mm/s) [C]'] = '%.0f (EGI 040)' % (1000 * cell.Jaw_Speed_Max)
            if name.startswith('MG06'):
                dofs = [c.Address[j][1] for j in ('lid_hinge', 'handle_hinge')]
                row['peak speed (deg/s) [P]'] = ', '.join('%.1f' % math.degrees(np.max(np.abs(record.Qvel[:, d]))) for d in dofs)
        out[name] = row
    return out


def Section_Objects(c: Context_Cls) -> tp.List[str]:
    """
    Description:
        Item 3: per moving object and program, its metrics (Object_Rows).

    Args:
        (1) c [Context_Cls(object)]: The context.

    Returns:
        (1) parameter [List<string>]: The section's lines.
    """

    rows = {v: Object_Rows(c, v) for v in CONST_VERSIONS}
    lines = ['## 3. Each moving object', '', 'The loader Y and Z axes, the jaws and MG06 are listed as channels; MG06\'s press plate follows its '
             'handle (the model\'s coupling). A "move" is a span of samples in which the channel changes. Clearances: the smallest '
             'exact distance outside the contacts by design (the cups on the PCB, the fork on the handle bar in the closing and opening, '
             'the PCB on its crate or holder, the PCB between the jaws and the MG06 holder while the loader is down in MG06), '
             'physics [P] and planned [K], every 20 ms and refined to 4 ms around the minimum; cut at '
             '%.0f mm. Two pairs are constant by construction: the robot\'s base and upper arm at the shoulder, and the loader\'s Y '
             'carriage over the table on its rail.' % (1000 * CONST_CUT), '']
    for name in rows['taught']:
        lines += ['### %s' % name, '']
        lines += _Table(['metric', CONST_PROGRAM_NAMES['taught'], CONST_PROGRAM_NAMES['optimized']],
                        [[k, rows['taught'][name][k], rows['optimized'][name].get(k, '-')]
                                                           for k in rows['taught'][name]])
        lines.append('')
    return lines


def Section_Interactions(c: Context_Cls) -> tp.List[str]:
    """
    Description:
        Item 4: between the moving objects - each pair's minimum, the shared zones, the synchronization, the loader slowed.

    Args:
        (1) c [Context_Cls(object)]: The context.

    Returns:
        (1) parameter [List<string>]: The section's lines.
    """

    lines = ['## 4. Between the moving objects', '', '### Clearance per pair', '']
    rows = []
    for name, _, _ in CONST_PAIRS:
        rows.append([name] + [_Distance_Text(c.Analyses[v][k].Minimum.get(name), CONST_CUT) for v in CONST_VERSIONS for k in ('physics', 'planned')])
    lines += _Table(['pair', 'Default [P]', 'Default [K]', 'Optimized [P]', 'Optimized [K]'], rows)
    lines += ['', 'Planning clearance `Clearance.Robot` %.0f mm, free transfers `Loader_Free` %.0f mm + `Tracking_Margin` %.0f mm [C]. A '
              'pair always in contact by design reads "always by design"; the robot\'s mount is not measured.' % (1000 * c.Cell.Clearance,
              1000 * c.Cell.Loader_Free, 1000 * c.Cell.Tracking_Margin), '', '### Shared zones', '']
    zone = c.Analyses['optimized']['swept']
    lines.append('The shared zones are the volume the loader\'s jaws and the PCB in them sweep in its jobs (%s), where its '
                 'actuators put it, grown by Loader_Free %.0f mm (`Report.Core.Swept_Zone`: one pose per %.0f mm cell of (Y, Z, jaw); the '
                 'robot is in a zone when an exact distance to one of its poses is below Loader_Free) [K, M], split by the loader\'s Y '
                 '(owner, 2026-10-25; the carriages, which reach far above and below the jaws, are the gating rule\'s and I8\'s):'
                 % (', '.join(CONST_SWEPT_JOBS), 1000 * c.Cell.Loader_Free, 500 * c.Cell.Loader_Free))
    lines += ['- **%s** (the loader at %s\'s Y): %d poses, bounding box x %.3f..%.3f, y %.3f..%.3f, z %.3f..%.3f m.'
              % (title, at, len(zone[part].Poses), *zone[part].Box[:, 0], *zone[part].Box[:, 1], *zone[part].Box[:, 2])
              for part, (title, at) in CONST_SWEPT_PARTS.items()]
    lo, hi = c.Zones['jaw zone']
    lines.append('- the **jaws\' box** (a cross-check in physics): x %.3f..%.3f, y %.3f..%.3f, z %.3f..%.3f m, the box of the loader\'s jaws '
                 'at home, open and clamped, grown by Loader_Free (Report.Core CONST_ZONE_BODIES) [K, M].' % (lo[0], hi[0], lo[1], hi[1],
                                                                                                             lo[2], hi[2]))
    lines += ['', 'Who is in a zone when [K for the swept zones, every 20 ms; P for the jaws\' box: a point of an object\'s collision geoms '
              'inside the box]. An overlap is the robot and the loader in the zone together; a moving overlap, either of them moving then:', '']
    rows = []
    for v in CONST_VERSIONS:
        series, tr = c.Analyses[v]['physics'], c.Trajectories[v]
        robot_moving, loader_moving = Moving(tr.Robot_Q), Moving(np.c_[tr.Loader_Y, tr.Loader_Z, tr.Jaw])
        robot, loader = series.Occupied['robot in jaw zone'], series.Occupied['loader in jaw zone']
        for part, (title, _) in CONST_SWEPT_PARTS.items():
            z = c.Analyses[v]['swept'][part]
            t = tr.T[z.Samples]
            moving = robot_moving[z.Samples] | loader_moving[z.Samples]
            rows.append([CONST_PROGRAM_NAMES[v], title + ' [K]', _Spans_Text(Spans(z.Robot, t)), _Spans_Text(Spans(z.Loader, t)),
                         _Spans_Text(Spans(z.Robot & z.Loader, t)), _Spans_Text(Spans(z.Robot & z.Loader & moving, t))])
        moving = robot_moving[series.Samples] | loader_moving[series.Samples]
        rows.append([CONST_PROGRAM_NAMES[v], 'jaws\' box [P]', _Spans_Text(Spans(robot, series.T)), _Spans_Text(Spans(loader, series.T)),
                     _Spans_Text(Spans(robot & loader, series.T)), _Spans_Text(Spans(robot & loader & moving, series.T))])
    lines += _Table(['program', 'zone', 'robot', 'loader', 'robot and loader', 'moving overlap'], rows)
    lines += ['', 'The interlocks that keep the overlaps safe (SPEC §7.11): in the jaw zone I1 (the handover starts with the loader home and '
              'open), I2 (the loader moves only after the handover ends), I10 (J_INSERT starts only after the robot\'s motion out of the '
              'handover), I7 (the take starts only after the loader is home) and I9 (the robot moves to the take only after J_RETURN_Y); in '
              'the MG06 corridor I3 (the fork closes MG06 only after the park), I6 (the loader fetches only after MG06 is open and the fork '
              'gone), I4 and I5 (nothing moves at MG06 during the test); everywhere the gating rule (in a parallel part neither object comes '
              'where the other has been, below). Their times on each trajectory: section 2. A moving overlap in the jaw zone is the handover '
              'or the take, by design: the loader\'s jaws close on the PCB the robot holds, or open under it, while the robot is at the jaws '
              'only in those LOCKED steps. The loader is in a zone whenever its Y and Z are on one of the zone\'s poses, so "robot and loader" '
              'is the robot\'s own time in the zone while the loader is there.', '', '### Synchronization', '']
    for v in CONST_VERSIONS:
        tr = c.Trajectories[v]
        lines += ['**%s** - the robot\'s waits (its segments that are not motions) and the handshake in order [K]:' % CONST_PROGRAM_NAMES[v], '']
        lines += _Table(['start (s)', 'duration (s)', 'waits for', 'line'], [['%.3f' % t, '%.3f' % d, k, '`%s`' % text] for t, d, k, text in
                                                                              Robot_Waits(tr)])
        lines.append('')
        lines.append('Handshake: ' + '; '.join('%.2f s `%s`' % (t, text) for t, text in Signal_Order(tr)) + '.')
        lines.append('')
    loader_lines = {cmd.Kind for job in c.Programs['optimized'].Loader_Jobs.values() for cmd in job}
    lines += ['**Deadlock.** The loader jobs contain only %s lines [C]: the loader never waits for the robot, so the waits form no cycle '
              'and no timing change can deadlock the cell; a slower loader makes the robot wait longer at its WaitDone or Run.'
              % ', '.join(sorted(loader_lines)), '']
    lines += Section_Stretch(c)
    return lines


def _Spans_Text(spans: tp.List[tp.Tuple[float, float]]) -> str:
    """
    Description:
        Time spans for the report: 'a-b s' each, '-' when none.

    Args:
        (1) spans [List<Tuple<float, float>>]: The spans (s).

    Returns:
        (1) parameter [string]: The text.
    """

    return ', '.join('%.1f-%.1f' % s for s in spans) or '-'


def Section_Stretch(c: Context_Cls) -> tp.List[str]:
    """
    Description:
        The loader slowed and the robot slowed against I8 (Stretch_Analysis) and the gates (the gating rule; F15: the take, I9, the
        clear height; F16: the insert, I10).

    Args:
        (1) c [Context_Cls(object)]: The context.

    Returns:
        (1) parameter [List<string>]: The lines.
    """

    s = c.Stretch
    cell = lambda r: _Distance_Text((r[1], r[2], r[3]), 0.05) if r[1] < 0.05 else '> 50 mm'
    lines = ['### A slower (or faster) loader, a slower robot', '', 'The Default program runs every loader job with Loader.Run: the robot does '
             'not move while the loader does, so either\'s speed changes only the cycle time. In the Optimized program the robot moves while '
             'the jobs %s run. Each job slowed by a factor (every move and hold of the job takes factor times as long; the robot\'s lines run '
             'as programmed and the robot holds at its WaitDone until the job ends); the closest robot - loader distance over the '
             'job, the loader posed where its actuators put it, exact every 4 ms (`Report.Core.Stretched_Distance`) [K]:'
             % ', '.join(s['jobs']), '']
    rows = []
    for k, f in enumerate(CONST_FACTORS):
        rows.append(['%.2f (%+.0f %%)' % (f, 100 * (f - 1))] + [cell(s['rows'][j][k]) for j in s['jobs']])
    lines += _Table(['factor'] + s['jobs'], rows)
    lines += ['', 'The robot slowed by a speed override (owner, F16): its lines from each job\'s Loader.Start to its WaitDone take 1 / '
              'override as long, the job runs as programmed and the loader holds at its end; the same distance [K]:', '']
    rows = []
    for k, o in enumerate(CONST_OVERRIDES):
        rows.append(['%.0f %%' % (100 * o)] + [cell(s['robot']['rows'][j][k]) for j in s['jobs']])
    lines += _Table(['override'] + s['jobs'], rows)
    lines += ['', 'The first slow-down at which the job that comes closest falls below each bound (the loader up to %.0f x, the robot down to '
              'a %.0f %% override; bisected to %.1f %%) [K, C]:' % (CONST_FACTORS[-1], 100 * CONST_OVERRIDES[0], 100 * CONST_FACTOR_RESOLUTION), '']
    rows = []
    for name, bound in s['bounds'].items():
        first, robot = s['first'].get(name), s['robot']['first'].get(name)
        rows.append([name, '%.1f mm' % (1000 * bound), 'none' if first is None else '%s %+.1f %%: %.2f mm (%s)' % (
            s['worst'], 100 * (first[0] - 1), 1000 * first[1][0], first[1][2]), 'none' if robot is None else '%s below %.1f %%: %.2f mm (%s)' % (
            s['robot']['worst'], 100 / robot[0], 1000 * robot[1][0], robot[1][2])])
    lines += _Table(['bound', 'value', 'loader slowed: first failure', 'robot override: first failure'], rows)
    lines += ['', '### The gates (F15, F16)', '', 'The gating rule (`Program.Check.Gating_Faults`, symmetric): while a parallel loader job may run '
              '(from its Loader.Start to the end of its Loader.WaitDone) the robot, moving or holding, stays at least Loader_Free from every '
              'pose the job sweeps (the loader where its actuators put it) - otherwise one object comes where the other has been and their '
              'separation rests on timing: on the loader\'s speed when the robot comes later ("robot enters"), on the robot\'s when the '
              'loader does ("loader enters"). The order must come from the program: the robot out of the way before the job\'s start, or '
              'the job\'s WaitDone before the robot\'s motion in [K]:', '']
    rows = [[CONST_PROGRAM_NAMES[v], f.Job, f.Kind, '`%s`' % f.Motion, '%.2f-%.2f s' % f.Robot_Span, '%.2f mm (%s)' % (1000 * f.Distance, f.Pair),
             '%.2f s' % f.Lead] for v in CONST_VERSIONS for f in c.Analyses[v]['gating']]
    lines += _Table(['program', 'job', 'kind', 'the robot\'s line', 'the robot there', 'closest', 'apart by'], rows) if rows else \
        ['No fault in either program: no parallel part rests on timing.']
    insert = s.get('insert')
    if insert is not None:
        lines += ['', 'The insert (F16): `Robot.MoveSpline(%s)` to %s (%.3f s), then `Loader.Start(J_INSERT)`: the job\'s first loader move at '
                  '%.3f s follows the robot\'s arrival at %.3f s (I10, section 2); %s lies %s from everything J_INSERT sweeps [K].' % (
                      insert['set'], insert['point'], insert['duration'], insert['moved'], insert['arrived'], insert['point'],
                      '%.2f mm' % (1000 * insert['distance']) if insert['distance'] < 0.05 else 'more than 50 mm')]
    t = s.get('take')
    if t is not None:
        best, found = s['clear']
        lines += ['', 'The take (F15): `Loader.Start(J_RETURN_Y)`, `Loader.WaitDone(J_RETURN_Y)`, `Loader.Start(J_RETURN_Z)`, '
                  '`Robot.MoveSpline(W_TO_TAKE)` (%.3f s), `Loader.WaitDone(J_RETURN_Z)`: the spline starts at %.3f s, after J_RETURN_Y\'s '
                  'last loader move at %.3f s (I9, section 2), and ends before J_RETURN_Z\'s at %.3f s [K]. With the loader at its home Y '
                  'the whole spline keeps %s from it at every Z from home to the travel height (`Report.Core.Clear_Height`, a static '
                  'loader, exact) [K], so the loader\'s Y at home is all it waits for; the robot waits at P_LOADER_WAIT meanwhile, %s '
                  'from the passing loader at any slow-down (the J_RETURN_Y column).' % (
                      t['spline'], t['spline_start'], t['y_done'], t['z_done'], 'more than 50 mm' if min(d for _, d in found) >= 0.05 else
                      '%.1f mm' % (1000 * min(d for _, d in found)),
                      _Distance_Text((min(r[1] for r in s['rows']['J_RETURN_Y']), 0, ''), 0.05).split(' at')[0])]
    return lines


def Section_PCB(c: Context_Cls) -> tp.List[str]:
    """
    Description:
        Item 5: the PCB from input to output - stages, handovers, custody, clearance, overhang, peaks, slip, times.

    Args:
        (1) c [Context_Cls(object)]: The context.

    Returns:
        (1) parameter [List<string>]: The section's lines.
    """

    lines = ['## 5. The PCB, input to output', '', '### Stages (the trajectory\'s PCB location) [K]', '']
    rows = []
    stage_lists = {v: Location_Spans(c.Trajectories[v]) for v in CONST_VERSIONS}
    for (la, a0, a1), (lb, b0, b1) in zip(stage_lists['taught'], stage_lists['optimized']):
        ta, tb = c.Trajectories['taught'].T, c.Trajectories['optimized'].T
        rows.append([la, CONST_LOCATION_HOLDER[la], '%.3f-%.3f (%.3f)' % (ta[a0], ta[a1], ta[a1] - ta[a0] + CONST_DT),
                     '%.3f-%.3f (%.3f)' % (tb[b0], tb[b1], tb[b1] - tb[b0] + CONST_DT), '%+.3f' % ((tb[b1] - tb[b0]) - (ta[a1] - ta[a0]))])
    lines += _Table(['location', 'holder', 'Default (s)', 'Optimized (s)', 'difference (s)'], rows)
    lines += ['', '### Handovers [P against K]', '', 'At each change of location in the trajectory: the PCB\'s pose in physics against the '
              'trajectory\'s; around it the new holder\'s first touch and the old one\'s release in physics (a contact closer than '
              '`Cup_Touch` %.1f mm), how long both held it, how long none did.' % (1000 * c.Cell.Cup_Touch), '']
    rows = []
    for v in CONST_VERSIONS:
        for h in c.Analyses[v]['handovers']:
            error = '%.2f mm, %.3f deg' % (1000 * h.Position_Error, math.degrees(h.Angle_Error))
            rows.append([CONST_PROGRAM_NAMES[v], '%.3f' % h.T, '%s -> %s' % (h.Before, h.After), error,
                         _Fmt(h.New_Touch, 1, 3), _Fmt(h.Old_Release, 1, 3), '%.3f' % h.Both, '%.3f' % h.Free])
    lines += _Table(['program', 't (s)', 'holder before -> after', 'pose error', 'new first touch (s)', 'old release (s)', 'both (s)',
                     'none (s)'], rows)
    lines += ['', '### Custody [P]', '']
    for v in CONST_VERSIONS:
        an = c.Analyses[v]
        holders = Holder_Matrix(an['record'])
        held = np.mean(np.any(holders, axis=1)) * 100
        gaps = '; '.join('%.3f-%.3f s (%.0f ms), fell %.2f mm' % (a, b, 1000 * (b - a), 1000 * dz) for a, b, dz in an['gaps'])
        lines.append('- **%s**: a holder touches the PCB in %.2f %% of the samples; %d handovers (regrasps); the spans with no holder: %s.'
                     % (v, held, len(an['handovers']), gaps or 'none'))
    lines += ['', 'The spans without a holder are by design: the PCB starts %.1f mm above the input crate floor (`Simulation.Core` '
              'CONST_PCB_DROP), the loader releases it 0.15 mm above its MG06 seat (SPEC §4.4) and the robot releases it 1 mm above the '
              'output crate floor (SPEC §4.8). Between them something holds or supports it at every 4 ms sample.' % 0.3, '',
              '### Clearance, overhang, dynamics [P, K]', '']
    rows = []
    for name in ('PCB - environment', 'PCB - loader', 'PCB - MG06 lid/handle/press', 'robot - PCB'):
        rows.append([name] + [_Distance_Text(c.Analyses[v]['physics'].Minimum.get(name), CONST_CUT) for v in CONST_VERSIONS])
    ov = c.Analyses['taught']['overhang']
    for holder, sides in ov.items():
        rows.append(['overhang beyond the %s, +x / -x / +y / -y (mm; negative: the support reaches past the edge) [K, M]' % holder] +
                     ['%s' % ' / '.join('%.1f' % (1000 * c.Analyses[v]['overhang'][holder][k]) for k in ('+x', '-x', '+y', '-y'))
                      for v in CONST_VERSIONS])
    peak = {}
    for v in CONST_VERSIONS:
        rec, tr = c.Analyses[v]['record'], c.Trajectories[v]
        q0, d0 = c.Address['pcb_free']
        vel = rec.Qvel[:, d0:d0 + 3]
        speed = np.linalg.norm(vel, axis=1)
        acc = np.linalg.norm(np.diff(vel, axis=0), axis=1) / CONST_DT
        kernel = np.ones(CONST_SMOOTH) / CONST_SMOOTH
        smooth = np.c_[[np.convolve(vel[:, k], kernel, mode='same') for k in range(3)]].T
        acc_s = np.linalg.norm(np.diff(smooth, axis=0), axis=1) / CONST_DT
        planned_v = np.linalg.norm(np.gradient(tr.PCB_Pose[:, :3], CONST_DT, axis=0), axis=1)
        planned_a = np.linalg.norm(np.gradient(np.gradient(tr.PCB_Pose[:, :3], CONST_DT, axis=0), CONST_DT, axis=0), axis=1)
        tilt_p, tilt_k = Tilt(rec.Qpos[:, q0 + 3:q0 + 7]), Tilt(tr.PCB_Pose[:, 3:])
        stages = []
        for name, a, b in Location_Spans(tr):
            stages.append('%s %.2f..%.2f (%.2f..%.2f)' % (name, math.degrees(tilt_p[a:b + 1].min()), math.degrees(tilt_p[a:b + 1].max()),
                                                          math.degrees(tilt_k[a:b + 1].min()), math.degrees(tilt_k[a:b + 1].max())))
        slips = [(s.Name, s.Slip) for s in c.Analyses[v]['result'].Steps if s.Slip > 0]
        name, slip = max(slips, key=lambda x: x[1]) if slips else ('-', 0.0)
        peak[v] = ['%.3f m/s at %.2f s [P]; %.3f m/s [K]' % (speed.max(), rec.T[int(np.argmax(speed))], planned_v.max()),
                   '%.1f m/s^2 at %.2f s raw, %.1f m/s^2 over %d ms [P]; %.1f m/s^2 [K]' % (acc.max(), rec.T[int(np.argmax(acc)) + 1], acc_s.max(),
                                                                                           CONST_SMOOTH * 4, planned_a.max()),
                   '; '.join(stages), '%.2f mm in %s [P]' % (1000 * slip, name)]
    for k, label in enumerate(['peak speed', 'peak acceleration', 'tilt from horizontal per stage, min..max deg [P] (planned [K])', 'largest slip in '
                               'the cups']):
        rows.append([label, peak['taught'][k], peak['optimized'][k]])
    lines += _Table(['', CONST_PROGRAM_NAMES['taught'], CONST_PROGRAM_NAMES['optimized']], rows)
    lines += ['', 'A PCB in the jaws or MG06 is tilted 8.94 deg by design (the cassette plane, SPEC §4.2); the carried tilt limit is %.0f deg '
              '(Optimization.yaml Carry_Max_Tilt, an assumption, KNOWN_ISSUES.md) [C].' % math.degrees(c.Carry_Max_Tilt), '',
              '### Time from input to output [P]', '']
    rows = []
    for v in CONST_VERSIONS:
        ev = c.Analyses[v]['result'].Vacuum
        on, off = ev[0].T, ev[-1].T
        rows.append([CONST_PROGRAM_NAMES[v], '%.3f' % on, '%.3f' % off, '%.3f' % (off - on), '%.3f' % c.Analyses[v]['result'].Cycle_Time])
    lines += _Table(['program', 'picked: vacuum on (s)', 'placed: vacuum off (s)', 'input to output (s)', 'cycle (s)'], rows)
    return lines


def Section_Cycle(c: Context_Cls) -> tp.List[str]:
    """
    Description:
        Item 6: the cycle time and its critical path.

    Args:
        (1) c [Context_Cls(object)]: The context.

    Returns:
        (1) parameter [List<string>]: The section's lines.
    """

    paths = {v: Critical_Path(c.Trajectories[v]) for v in CONST_VERSIONS}
    kinds = [k for k in ('robot motion', 'loader', 'loader (timed wait)', 'tester', 'vacuum signal', 'robot dwell')
             if any(k in p for p in paths.values())]
    lines = ['## 6. Cycle time and critical path', '', 'The robot\'s program is the cell\'s timeline (the loader and the tester start from its '
             'lines), so each of its segments is spent on its own motion or on what it waits for (`Report.Core.Critical_Path`) [K]:', '']
    rows = [[k] + ['%.3f' % paths[v].get(k, 0.0) for v in CONST_VERSIONS] + ['%+.3f' % (paths['optimized'].get(k, 0.0) - paths['taught'].get(k, 0.0))]
            for k in kinds]
    rows.append(['**cycle**'] + ['**%.3f**' % c.Trajectories[v].T[-1] for v in CONST_VERSIONS] + ['**%+.3f**' % (c.Trajectories['optimized'].T[-1] -
                                                                                                           c.Trajectories['taught'].T[-1])])
    lines += _Table(['spent on', 'Default (s)', 'Optimized (s)', 'difference (s)'], rows)
    lines += ['', 'Physics follows the trajectory\'s timeline: the physics runs end at %s s [P].' % ' / '.join(
        '%.3f' % c.Analyses[v]['result'].Cycle_Time for v in CONST_VERSIONS)]
    return lines


def Section_Figures(c: Context_Cls, figures: tp.Dict[str, tp.List[str]]) -> tp.List[str]:
    """
    Description:
        Item 7: the figures (their light PNGs; the dark ones beside them) and the summary chart's values with their sources.

    Args:
        (1) c [Context_Cls(object)]: The context.
        (2) figures [Dict<string, List<string>>]: Per figure its files.

    Returns:
        (1) parameter [List<string>]: The section's lines.
    """

    lines = ['## 7. Figures', '', '**Default vs Optimized** - the values of the summary chart:', '']
    rows = [[name.replace(r'$\sum\Vert\Delta q\Vert$', 'sum of |dq|'), unit, '%.*f' % (digits, d), '%.*f' % (digits, o),
             '%+.1f %%' % (100 * (o / d - 1)),
             '[%s]' % source] for name, unit, d, o, source, digits in Summary_Values(c)]
    lines += _Table(['metric', 'unit', CONST_PROGRAM_NAMES['taught'], CONST_PROGRAM_NAMES['optimized'], 'change', 'source'], rows)
    lines += ['', 'Throughput = 3600 s / cycle time; the joint-space arc length is the sum over the 4 ms samples of the Euclidean norm of '
              'the six joints\' change (`Report.Core.Joint_Arc`; section 3 gives the per-joint travel sum of |dq|).', '']
    titles = {'summary': 'Default vs Optimized', 'timeline': 'the timeline of every moving object, waits and shared zones',
              'joints': 'the joint angles', 'paths': 'the TCP and PCB paths, top and side', 'clearance': 'the clearance per pair over time',
              'critical_path': 'the cycle by its critical path', 'stretch': 'I8 against the loader\'s and the robot\'s speed'}
    for name in figures:
        lines += ['![%s](%s_light.png)' % (titles.get(name, name), name), '']
    return lines


def Section_Not_Modelled(c: Context_Cls) -> tp.List[str]:
    """
    Description:
        Item 8: what is not modelled and how it could change the ranking (the model's limits; the numbers from the runs).

    Args:
        (1) c [Context_Cls(object)]: The context.

    Returns:
        (1) parameter [List<string>]: The section's lines.
    """

    ma, mb = c.Analyses['taught']['metrics'], c.Analyses['optimized']['metrics']
    fa, fb = (Free_Loader_Distance(c.Analyses[v]['result']) for v in CONST_VERSIONS)
    ta, tb = c.Trajectories['taught'].T[-1], c.Trajectories['optimized'].T[-1]
    free = lambda f: '-' if f is None else '%.1f mm in %s' % (1000 * f[0], f[3])
    return ['## 8. Not modelled, and how it could change the ranking', '',
            '| not modelled | effect on the comparison |', '|---|---|',
            '| blends (every motion ends at rest in both) | the Default program rests %.1f s in %d stops, the Optimized one %.1f s in %d '
            '[K]; blending the taught vias would recover part of the optimized program\'s gain on a real controller |' % (
                ma.Rest_Time, ma.Stops, mb.Rest_Time, mb.Stops),
            '| tool and machine times (vacuum build-up beyond the WaitDI response, gripper and PLC cycle times, the real test time; the '
            'test is a %.1f s placeholder, KNOWN_ISSUES.md) | add to both programs alike where sequential; where the optimized program '
            'runs in parallel they move the meeting point of robot and loader (section 4) |' % c.Cell.Test_Duration,
            '| cables and hoses on the robot | could limit the splines\' wrist turns; not checked |',
            '| signal latency (loader PLC, I/O) | adds to every handshake in both; the optimized program waits for the loader\'s done '
            'everywhere (F15), so a latency costs time, not clearance - except the robot leaving the loader\'s way at a job\'s start '
            '(section 4) |',
            '| model error (CAD deviations compensated, estimated masses and torque limits, the loader\'s actuator model) | the closest '
            'robot - loader distance in a free transfer is %s taught, %s optimized [P]: a model error eats the smaller margin first |' % (
                free(fa), free(fb)),
            '| part tolerance (PCB size, crate and holder positions) | the LOCKED parts are identical, so both programs share this risk |',
            '| the real controller\'s in-position wait before a LOCKED operation (KNOWN_ISSUES.md) | adds a settle per stop to both |',
            '| contact stiffness (fork on the handle, PCB on the ledges) | identical LOCKED parts: the same in both |', '',
            'None of these is likely to reverse the cycle-time ranking (%.1f s, %.0f %% of the taught cycle [K]); several shrink the optimized '
            'program\'s margins, which is why its risks rank first in the verdict.' % (ta - tb, 100 * (1 - tb / ta))]


def Section_Verdict(c: Context_Cls) -> tp.List[str]:
    """
    Description:
        Item 9: the verdict - which program is better on which metric, by how much, the risks, what to check on the real cell.

    Args:
        (1) c [Context_Cls(object)]: The context.

    Returns:
        (1) parameter [List<string>]: The section's lines.
    """

    ta, tb = c.Trajectories['taught'], c.Trajectories['optimized']
    ma, mb = c.Analyses['taught']['metrics'], c.Analyses['optimized']['metrics']
    ra = c.Analyses['taught']['physics'].Minimum.get('robot - loader')
    rb = c.Analyses['optimized']['physics'].Minimum.get('robot - loader')
    s = c.Stretch
    fa, fb = (Free_Loader_Distance(c.Analyses[v]['result']) for v in CONST_VERSIONS)
    free = lambda f: '-' if f is None else '%.1f mm (%s)' % (1000 * f[0], f[3])
    tilt = {}
    for v in CONST_VERSIONS:
        rec, q0 = c.Analyses[v]['record'], c.Address['pcb_free'][0]
        cups = c.Trajectories[v].PCB_Location[:len(rec.T)] == CONST_PCB_LOCATIONS.index('cups')
        tilt[v] = math.degrees(float(np.max(Tilt(rec.Qpos[:, q0 + 3:q0 + 7])[cups])))
    i8 = s['first'].get('Loader_Free + Tracking_Margin (I8)')
    hit = s['first'].get('contact')
    faults = c.Analyses['optimized']['gating']
    timed = '; '.join('%s: the %s at `%s`, %.2f s apart [K]: it rests on timing' % (f.Job, f.Kind, f.Motion, f.Lead) for f in faults)
    r8, rhit = s['robot']['first'].get('Loader_Free + Tracking_Margin (I8)'), s['robot']['first'].get('contact')
    rows = [['cycle time [K]', '%.3f s' % ta.T[-1], '%.3f s' % tb.T[-1], 'Optimized, %.3f s (%.1f %%) shorter' % (ta.T[-1] - tb.T[-1],
                                                                                                             100 * (1 - tb.T[-1] / ta.T[-1]))],
            ['throughput = 3600 s / cycle [K]', '%.1f PCB/h' % (3600 / ta.T[-1]), '%.1f PCB/h' % (3600 / tb.T[-1]), 'Optimized, %+.1f %%' % (
                100 * (ta.T[-1] / tb.T[-1] - 1))],
            ['robot joint travel [K]', '%.1f rad' % ma.Joint_Travel.sum(), '%.1f rad' % mb.Joint_Travel.sum(), 'Optimized, %.0f %% less' % (
                100 * (1 - mb.Joint_Travel.sum() / ma.Joint_Travel.sum()))],
            ['TCP path [K]', '%.2f m' % ma.Tcp_Path, '%.2f m' % mb.Tcp_Path, 'Optimized, %.0f %% shorter' % (100 * (1 - mb.Tcp_Path / ma.Tcp_Path))],
            ['PCB path [K]', '%.2f m' % ma.Pcb_Path, '%.2f m' % mb.Pcb_Path, 'Default, %.0f %% shorter' % (100 * (mb.Pcb_Path / ma.Pcb_Path - 1))],
            ['closest robot - loader in a free transfer [P]', free(fa), free(fb), 'Default' if (fa or (1,))[0] > (fb or (1,))[0] else 'Optimized'],
            ['closest robot - loader overall (the take, LOCKED) [P]', _Distance_Text(ra, CONST_CUT), _Distance_Text(rb, CONST_CUT), 'equal'],
            ['largest carried PCB tilt [P]', '%.1f deg' % tilt['taught'], '%.1f deg' % tilt['optimized'], 'Default' if tilt['taught'] <
             tilt['optimized'] else 'Optimized'],
            ['tolerance to a slower loader [K]', 'any (sequential)', ('any up to %.0f x (gated, F15)' % CONST_FACTORS[-1]) if i8 is None else
             '%+.1f %% before I8, %s before contact' % (100 * (i8[0] - 1), '-' if hit is None else '%+.1f %%' % (100 * (hit[0] - 1))),
             'equal' if i8 is None else 'Default'],
            ['tolerance to a slower robot (speed override) [K]', 'any (sequential)', ('any down to %.0f %% (gated, F15, F16)' % (
                100 * CONST_OVERRIDES[0])) if r8 is None else 'below %.1f %% breaks I8, below %s a contact' % (
                100 / r8[0], '-' if rhit is None else '%.1f %%' % (100 / rhit[0])), 'equal' if r8 is None else 'Default'],
            ['acceptance checks (SPEC §10.1) [P]', '%d/%d pass' % (sum(1 for x in c.Analyses['taught']['acceptance'] if x[1]),
                                                                            len(c.Analyses['taught']['acceptance'])),
             '%d/%d pass' % (sum(1 for x in c.Analyses['optimized']['acceptance'] if x[1]), len(c.Analyses['optimized']['acceptance'])), 'equal'],
            ['re-teaching on a real cell [C]', 'the taught points only', '+ P_LOADER_WAIT (waiting and exit point), spline via points, '
             'MoveSpline on the AUBO', 'Default']]
    lines = ['## 9. Verdict', '']
    lines += _Table(['metric', CONST_PROGRAM_NAMES['taught'], CONST_PROGRAM_NAMES['optimized'], 'better'], rows)
    lines += ['', '**Gain.** The optimized program is %.1f s (%.1f %%) shorter per cycle; the whole gain is in the FREE transfers and the '
              'parallel loader (section 6); the LOCKED process parts are identical.' % (ta.T[-1] - tb.T[-1], 100 * (1 - tb.T[-1] / ta.T[-1])), '',
              '**Risks, highest first.**', '',
              '1. **%s**' % (timed if timed else 'No parallel part rests on timing (the gating rule holds both ways, section 4): a slower '
                             'loader or a speed override only lengthens the cycle.'),
              '2. **A smaller margin to the loader in the free transfers**: %s against %s in the Default program [P]; a model error or a '
              'tracking lag on the real cell eats it first.' % (free(fb), free(fa)),
              '3. **A larger carried tilt**: the PCB on the cups tilts up to %.1f deg in the optimized program against %.1f deg [P] (the '
              'assumed limit %.0f deg [C], KNOWN_ISSUES.md); the slip stays at %.2f mm against %.2f mm [P].' % (
                  tilt['optimized'], tilt['taught'], math.degrees(c.Carry_Max_Tilt), 1000 * c.Analyses['optimized']['result'].Slip,
                  1000 * c.Analyses['taught']['result'].Slip),
              '4. **MoveSpline on the AUBO controller** is not verified (KNOWN_ISSUES.md): the splines\' joint-space paths and timing must '
              'be reproduced by the controller, or converted into MoveJ through their via points (slower).',
              '5. **The family switch inside a spline**: wrist_2 crosses zero (%.2f deg closest) in a joint-space move - harmless in joint '
              'space, but a controller that blends or replans it in Cartesian space would meet the wrist singularity.' % math.degrees(
                  Wrist_Approach(tb)['whole cycle'][0]), '',
              '**On the real cell.** %s Simulation is not proof of safety. Before any run: the real-cell validation procedure (not defined '
              'yet, KNOWN_ISSUES.md); reduced speed first with the controller\'s own safety limits active; one moving object at a time where '
              'possible - the loader jobs alone, then the robot\'s transfers alone with the loader parked, then the LOCKED parts, then the '
              'parallel parts last, with the loader\'s Loader.Start / WaitDone handshake tested on its own first.'
              % CONST_DISCLAIMER]
    return lines


def _Figure(theme: Theme_Str, size: tp.Tuple[float, float], rows: int = 1, columns: int = 1, **options: tp.Any) -> tp.Tuple[tp.Any, tp.Any]:
    """
    Description:
        A new figure in a theme (Report.Style.Parameters): its axes, in a grid.

    Args:
        (1) theme [Theme_Str(object)]: The theme.
        (2) size [Tuple<float>]: Width, height (in).
        (3 - 4) rows, columns [int]: The grid.
        (5) options [Dict]: For plt.subplots (sharex, gridspec_kw ...).

    Returns:
        (1 - 2) parameter [Tuple]: The figure and its axes.
    """

    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    with plt.rc_context(Parameters(theme)):
        return plt.subplots(rows, columns, figsize=size, constrained_layout=True, **options)


def _Themed(make: tp.Callable[[Context_Cls, Theme_Str], tp.Any], c: Context_Cls, folder: str, stem: str) -> tp.List[str]:
    """
    Description:
        A figure made and saved in both themes (Report.Style.CONST_THEMES: <stem>_dark / <stem>_light, PNG and SVG), each made
        inside its theme's settings.

    Args:
        (1) make [function]: (context, theme) -> the figure.
        (2) c [Context_Cls(object)]: The context.
        (3) folder [string]: The folder.
        (4) stem [string]: The figure's name.

    Returns:
        (1) parameter [List<string>]: The files written.
    """

    import matplotlib.pyplot as plt
    out = []
    for theme in CONST_THEMES.values():
        with plt.rc_context(Parameters(theme)):
            out += Save_Figure(make(c, theme), folder, stem, theme)
    return out


def Summary_Values(c: Context_Cls) -> tp.List[tp.Tuple[str, str, float, float, str, int]]:
    """
    Description:
        The summary chart's values (owner, 2026-10-25): the cycle time, the throughput (3600 / the cycle time), the TCP's arc length,
        the joint-space arc length (the sum over the samples of |dq|, the Euclidean norm of the six joints' change), the PCB's
        arc length - each from the trajectory [K] (Optimizer.Measure.Measure, Report.Core.Joint_Arc).

    Args:
        (1) c [Context_Cls(object)]: The context.

    Returns:
        (1) parameter [List<Tuple>]: (metric, the unit, Default, Optimized, the source, the decimals), in the chart's order.
    """

    a, b = c.Analyses['taught']['metrics'], c.Analyses['optimized']['metrics']
    ta, tb = c.Trajectories['taught'], c.Trajectories['optimized']
    return [('process (cycle) time', 's', float(ta.T[-1]), float(tb.T[-1]), 'K', 2),
            ('throughput', 'PCB/h', 3600.0 / float(ta.T[-1]), 3600.0 / float(tb.T[-1]), 'K', 1),
            ('TCP arc length', 'm', a.Tcp_Path, b.Tcp_Path, 'K', 2),
            (r'joint-space arc length $\sum\Vert\Delta q\Vert$', 'rad', Joint_Arc(ta.Robot_Q), Joint_Arc(tb.Robot_Q), 'K', 1),
            ('PCB arc length', 'm', a.Pcb_Path, b.Pcb_Path, 'K', 2)]


def Figure_Summary(c: Context_Cls, theme: Theme_Str) -> tp.Any:
    """
    Description:
        "Default vs Optimized" (owner, 2026-10-25): grouped bars per metric (Summary_Values), the Default program 100 %, the
        Optimized program relative to it, each bar's absolute value with its unit above it.

    Args:
        (1) c [Context_Cls(object)]: The context.
        (2) theme [Theme_Str(object)]: The theme.

    Returns:
        (1) parameter [matplotlib Figure]: The figure.
    """

    values = Summary_Values(c)
    figure, axis = _Figure(theme, (CONST_DOUBLE, 3.0))
    x, w = np.arange(len(values)), 0.36
    for k, v in enumerate(CONST_VERSIONS):
        percent = [100.0 if v == 'taught' else 100.0 * o / d for _, _, d, o, _, _ in values]
        bars = axis.bar(x + (k - 0.5) * w, percent, w, color=theme.Program[v], label=CONST_PROGRAM_NAMES[v], edgecolor=theme.Background,
                        linewidth=0.6)
        for bar, (name, unit, d, o, _, digits) in zip(bars, values):
            value = d if v == 'taught' else o
            ink = max(('#111111', '#FFFFFF'), key=lambda c: Contrast(c, theme.Program[v]))
            axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height() - 3.0, '%.*f\n%s' % (digits, value, unit), ha='center', va='top',
                      fontsize=6.5, color=ink)
            if v == 'optimized':
                axis.text(bar.get_x() + bar.get_width() / 2, bar.get_height() + 2.0, '%+.1f %%' % (100.0 * (o / d - 1.0)), ha='center',
                          va='bottom', fontsize=7, color=theme.Program[v])
    axis.axhline(100.0, color=theme.Bound, linewidth=CONST_BOUND, linestyle=(0, (4, 3)), zorder=0)
    axis.set_xticks(x)
    axis.set_xticklabels(['%s\n[%s]' % (name, unit) for name, unit, _, _, _, _ in values])
    axis.set_ylabel(r'relative to the Default program [%]')
    axis.set_ylim(0, 135)
    axis.grid(axis='x', visible=False)
    axis.legend(loc='upper right', ncol=2)
    axis.set_title('Default vs Optimized (all values from the trajectories [K]; throughput = 3600 s / cycle time)', loc='left')
    return figure


def Figure_Timeline(c: Context_Cls, theme: Theme_Str) -> tp.Any:
    """
    Description:
        The timeline of every moving object in both programs, a panel each on one time axis: a bar where it moves, the robot's
        waits hatched by kind, the PCB's holder, the robot in the jaw zone and in the MG06 corridor, the loader there [K]; the
        cycle time annotated.

    Args:
        (1) c [Context_Cls(object)]: The context.
        (2) theme [Theme_Str(object)]: The theme.

    Returns:
        (1) parameter [matplotlib Figure]: The figure.
    """

    from matplotlib.patches import Patch
    figure, axes = _Figure(theme, (CONST_DOUBLE, 5.6), 2, 1, sharex=True)
    end = max(c.Trajectories[v].T[-1] for v in CONST_VERSIONS)
    for p, (axis, v) in enumerate(zip(axes, CONST_VERSIONS)):
        tr, zone, jaw = c.Trajectories[v], c.Analyses[v]['swept']['corridor'], c.Analyses[v]['swept']['jaw']
        lanes = []
        lanes.append(('loader in MG06 corridor', Spans(zone.Loader, tr.T[zone.Samples]), theme.Object['loader Y'], 0.45))
        lanes.append(('robot in MG06 corridor', Spans(zone.Robot, tr.T[zone.Samples]), theme.Object['robot'], 0.45))
        lanes.append(('robot in jaw zone', Spans(jaw.Robot, tr.T[jaw.Samples]), theme.Object['robot'], 0.45))
        for name, values in reversed(list(Object_Channels(tr).items())):
            lanes.append((name, Spans(Moving(values), tr.T), theme.Object[name], 1.0))
        for y, (name, spans, colour, alpha) in enumerate(lanes):
            axis.broken_barh([(a, b - a) for a, b in spans], (y - 0.32, 0.64), facecolors=colour, alpha=alpha, edgecolor=theme.Ink_2,
                             linewidth=0.3)
            if name == 'robot':
                for t0, duration, kind, _ in Robot_Waits(tr):
                    axis.broken_barh([(t0, duration)], (y - 0.32, 0.64), facecolors='none', edgecolor=theme.Ink_2,
                                     hatch=CONST_WAIT_HATCH.get(kind, ''), linewidth=0.0)
        y = len(lanes)
        for name, a, b in Location_Spans(tr):
            colour = {'cups': theme.Object['robot'], 'jaws': theme.Object['loader jaws'], 'mg06': theme.Object['MG06 lid/handle']}.get(name,
                                                                                                                              theme.Object['PCB'])
            axis.broken_barh([(tr.T[a], tr.T[b] - tr.T[a] + CONST_DT)], (y - 0.32, 0.64), facecolors=colour, alpha=0.55, edgecolor=theme.Ink_2,
                             linewidth=0.3)
            axis.text(0.5 * (tr.T[a] + tr.T[b]), y, {'input_crate': 'in', 'output_crate': 'out', 'mg06': 'MG06'}.get(name, name),
                      ha='center', va='center', fontsize=6, color=theme.Ink)
        labels = [l[0] for l in lanes] + ['PCB holder']
        axis.set_yticks(range(len(labels)))
        axis.set_yticklabels(labels)
        axis.grid(axis='y', visible=False)
        axis.axvline(tr.T[-1], color=theme.Program[v], linewidth=CONST_BOUND)
        axis.annotate('cycle %.3f s' % tr.T[-1], (tr.T[-1], len(labels) - 0.5), xytext=(4, 0), textcoords='offset points', fontsize=7,
                      color=theme.Program[v], va='center')
        axis.set_title(CONST_PROGRAM_NAMES[v], loc='left', color=theme.Program[v])
        axis.set_xlim(0, end + 8)
        Panel_Letter(axis, p, theme)
    axes[-1].set_xlabel('$t$ [s]')
    handles = [Patch(facecolor='none', edgecolor=theme.Ink_2, hatch=h, label='robot waits: %s' % k) for k, h in CONST_WAIT_HATCH.items()
               if k != 'loader (timed wait)']
    figure.legend(handles=handles, loc='outside lower center', ncol=4, fontsize=6.5)
    return figure


def Figure_Joints(c: Context_Cls, theme: Theme_Str) -> tp.Any:
    """
    Description:
        The robot's joint angles over time, a panel per joint, both programs on one time axis; wrist_2 = 0 (the wrist
        singularity) marked.

    Args:
        (1) c [Context_Cls(object)]: The context.
        (2) theme [Theme_Str(object)]: The theme.

    Returns:
        (1) parameter [matplotlib Figure]: The figure.
    """

    figure, axes = _Figure(theme, (CONST_DOUBLE, 6.2), 3, 2, sharex=True)
    names = ['shoulder pan', 'shoulder lift', 'elbow', 'wrist 1', 'wrist 2', 'wrist 3']
    for j, axis in enumerate(axes.T.flat):
        for v in CONST_VERSIONS:
            tr = c.Trajectories[v]
            axis.plot(tr.T, np.degrees(tr.Robot_Q[:, j]), color=theme.Program[v], label=CONST_PROGRAM_NAMES[v])
        if j == 4:
            axis.axhline(0.0, color=theme.Bound, linewidth=CONST_BOUND, linestyle=(0, (2, 2)))
            axis.annotate('wrist singularity', (1.0, 0.0), xytext=(0, 3), textcoords='offset points', fontsize=6.5, color=theme.Ink_2)
        axis.set_ylabel(r'$q_%d$ (%s) [deg]' % (j + 1, names[j]))
        Panel_Letter(axis, j, theme)
    for axis in axes[-1]:
        axis.set_xlabel('$t$ [s]')
    axes[0, 0].legend(loc='lower left', fontsize=6.5)
    return figure


def Figure_Paths(c: Context_Cls, theme: Theme_Str) -> tp.Any:
    """
    Description:
        The robot TCP's and the PCB's paths from the top (x-y) and from the side (x-z) in physics [P], both programs; the jaw
        zone and the MG06 corridor (their bounding boxes) dashed.

    Args:
        (1) c [Context_Cls(object)]: The context.
        (2) theme [Theme_Str(object)]: The theme.

    Returns:
        (1) parameter [matplotlib Figure]: The figure.
    """

    from matplotlib.patches import Rectangle
    figure, axes = _Figure(theme, (CONST_DOUBLE, 6.4), 2, 2)
    q0 = c.Address['pcb_free'][0]
    boxes = {title: tuple(c.Analyses['optimized']['swept'][part].Box) for part, (title, _) in CONST_SWEPT_PARTS.items()}
    for row, (name, get) in enumerate((('TCP', lambda r: r.Tcp), ('PCB', lambda r: r.Qpos[:, q0:q0 + 3]))):
        for col, (a, b) in enumerate(((0, 1), (0, 2))):
            axis = axes[row, col]
            for v in CONST_VERSIONS:
                p = get(c.Analyses[v]['record'])
                axis.plot(p[:, a], p[:, b], color=theme.Program[v], linewidth=1.0, label=CONST_PROGRAM_NAMES[v])
            for zone, (lo, hi) in boxes.items():
                axis.add_patch(Rectangle((lo[a], lo[b]), hi[a] - lo[a], hi[b] - lo[b], fill=False, edgecolor=theme.Bound,
                                         linestyle=(0, (3, 2)), linewidth=CONST_BOUND))
            data = np.vstack([get(c.Analyses[v]['record'])[:, [a, b]] for v in CONST_VERSIONS])
            pad = 0.05 * np.ptp(data, axis=0).max()
            axis.set_xlim(data[:, 0].min() - pad, data[:, 0].max() + pad)
            axis.set_ylim(data[:, 1].min() - pad, data[:, 1].max() + pad)
            axis.set_aspect('equal', adjustable='box')
            axis.set_xlabel('$x$ [m]')
            axis.set_ylabel('$y$ [m]' if b == 1 else '$z$ [m]')
            axis.set_title('%s, %s' % (name, 'top view' if b == 1 else 'side view'), loc='left')
            Panel_Letter(axis, 2 * row + col, theme)
    from matplotlib.lines import Line2D
    handles = axes[0, 0].get_legend_handles_labels()[0] + [Line2D([], [], color=theme.Bound, linewidth=CONST_BOUND, linestyle=(0, (3, 2)))]
    axes[0, 0].legend(handles, [CONST_PROGRAM_NAMES[v] for v in CONST_VERSIONS] + ['zones: jaw zone, MG06 corridor (boxes)'], loc='lower left',
                      fontsize=6.5)
    return figure


def Figure_Clearance(c: Context_Cls, theme: Theme_Str) -> tp.Any:
    """
    Description:
        The clearance over time per pair in physics (every 20 ms; the contacts by design left out) [P], both programs, the
        planning clearance and the loader's free-transfer distance as bound lines, each program's minimum marked and annotated.

    Args:
        (1) c [Context_Cls(object)]: The context.
        (2) theme [Theme_Str(object)]: The theme.

    Returns:
        (1) parameter [matplotlib Figure]: The figure.
    """

    figure, axes = _Figure(theme, (CONST_DOUBLE, 8.6), 5, 2, sharex=True)
    for p, (axis, (name, _, _)) in enumerate(zip(axes.T.flat, CONST_PAIRS)):
        for v in CONST_VERSIONS:
            series, tr, program = c.Analyses[v]['physics'], c.Trajectories[v], c.Programs[v]
            design = By_Design(tr, program)
            d = series.Distance[:, p].copy()
            d[[design(name, int(i)) for i in series.Samples]] = np.nan
            axis.plot(series.T, 1000 * d, color=theme.Program[v], linewidth=1.0, label=CONST_PROGRAM_NAMES[v])
            found = series.Minimum.get(name)
            if found is not None and found[0] < CONST_CUT:
                axis.plot([found[1]], [1000 * found[0]], marker='o', markersize=3, color=theme.Program[v])
                axis.text(0.99, 0.97 - 0.11 * (v == 'optimized'), r'min %.1f mm at %.1f s' % (1000 * found[0], found[1]), transform=axis.transAxes,
                          ha='right', va='top', fontsize=6, color=theme.Program[v])
        axis.axhline(1000 * c.Cell.Clearance, color=theme.Bound, linewidth=CONST_BOUND, linestyle=(0, (1, 2)))
        if 'loader' in name:
            axis.axhline(1000 * (c.Cell.Loader_Free + c.Cell.Tracking_Margin), color=theme.Bound, linewidth=CONST_BOUND, linestyle=(0, (5, 3)))
        axis.set_ylim(-5, 1000 * CONST_CUT + 5)
        axis.set_title(name, loc='left', fontsize=8)
        axis.set_ylabel('$d$ [mm]')
        Panel_Letter(axis, p, theme)
    for axis in axes[-1]:
        axis.set_xlabel('$t$ [s]')
    axes[0, 0].legend(loc='upper right', fontsize=6)
    figure.suptitle('Clearance per pair [P] (cut at %.0f mm; dotted: %.0f mm planning clearance; dashed: %.0f mm loader free-transfer '
                    'distance; gaps: contact by design)' % (1000 * CONST_CUT, 1000 * c.Cell.Clearance, 1000 * (c.Cell.Loader_Free +
                                                                                                      c.Cell.Tracking_Margin)),
                    fontsize=8, x=0.01, ha='left')
    return figure


def Figure_Critical_Path(c: Context_Cls, theme: Theme_Str) -> tp.Any:
    """
    Description:
        The cycle by its critical path in both programs [K]: stacked bars of what the robot's timeline spends its time on.

    Args:
        (1) c [Context_Cls(object)]: The context.
        (2) theme [Theme_Str(object)]: The theme.

    Returns:
        (1) parameter [matplotlib Figure]: The figure.
    """

    figure, axis = _Figure(theme, (CONST_DOUBLE, 2.2))
    paths = {v: Critical_Path(c.Trajectories[v]) for v in CONST_VERSIONS}
    kinds = [('robot motion', theme.Object['robot']), ('loader', theme.Object['loader Y']), ('tester', theme.Object['MG06 lid/handle']),
             ('vacuum signal', theme.Object['loader jaws']), ('robot dwell', theme.Object['PCB']), ('loader (timed wait)', theme.Object['loader Z'])]
    for k, v in enumerate(CONST_VERSIONS):
        left = 0.0
        for kind, colour in kinds:
            w = paths[v].get(kind, 0.0)
            if w <= 0:
                continue
            axis.barh(1 - k, w, left=left, color=colour, height=0.55, edgecolor=theme.Background, linewidth=0.8, label=kind if k == 0 else None)
            if w > 2.5:
                axis.text(left + w / 2, 1 - k, '%.1f' % w, ha='center', va='center', fontsize=6.5, color='#111111')
            left += w
        axis.annotate('%.3f s' % left, (left, 1 - k), xytext=(4, 0), textcoords='offset points', va='center', fontsize=7.5,
                      color=theme.Program[v])
    axis.set_yticks([1, 0])
    axis.set_yticklabels([CONST_PROGRAM_NAMES[v] for v in CONST_VERSIONS])
    axis.set_xlabel('$t$ [s] (each segment of the robot\'s timeline: its motion or what it waits for)')
    axis.set_xlim(0, 80)
    axis.grid(axis='y', visible=False)
    figure.legend(loc='outside right upper', fontsize=6.5)
    return figure


def Figure_Stretch(c: Context_Cls, theme: Theme_Str) -> tp.Any:
    """
    Description:
        The I8 check against a slower or faster loader and a slower robot [K]: per parallel job of the Optimized program the
        closest robot - loader distance (Report.Core.Stretched_Distance) against the job's slow-down (a) and the robot's speed
        override (b), the bounds as lines.

    Args:
        (1) c [Context_Cls(object)]: The context.
        (2) theme [Theme_Str(object)]: The theme.

    Returns:
        (1) parameter [matplotlib Figure]: The figure.
    """

    s = c.Stretch
    figure, axes = _Figure(theme, (CONST_DOUBLE, 2.8), 1, 2, sharey=True)
    colours = [theme.Object[k] for k in ('robot', 'loader Y', 'loader jaws', 'MG06 lid/handle', 'PCB')]
    markers = ['o', 's', '^', 'D', 'v', 'P']
    panels = [(s['rows'], lambda f: 100 * (f - 1), 'job slowed by [%]', 'the loader\'s speed', (-55, 105)),
              (s['robot']['rows'], lambda f: 100 / f, 'robot speed override [%]', 'the robot\'s speed', (105, 5))]
    for p, (axis, (table, x, xlabel, title, limits)) in enumerate(zip(axes, panels)):
        for k, job in enumerate(s['jobs']):
            f = x(np.array([r[0] for r in table[job]]))
            far = np.array([r[1] for r in table[job]]) >= 0.05
            d = np.minimum([1000 * r[1] for r in table[job]], 50.0) + 0.6 * (k - len(s['jobs']) / 2) * far
            axis.plot(f, d, marker=markers[k % len(markers)], markersize=3, linewidth=1.2, color=colours[k % len(colours)],
                      label=Label(job) if p == 0 else None)
        for k, ((name, bound), style) in enumerate(zip(s['bounds'].items(), [(0, (5, 3)), (0, (3, 2)), (0, (1, 2)), '-'])):
            axis.axhline(1000 * bound, color=theme.Bound, linewidth=CONST_BOUND, linestyle=style)
            if p == 0:
                axis.annotate(Label(name), (100, 1000 * bound), xytext=(0, 1.5 if k < 2 else (-7 if k == 3 else 1.5)),
                              textcoords='offset points', ha='right', fontsize=5.5, color=theme.Ink_2)
        axis.set_xlabel(xlabel)
        axis.set_xlim(*limits)
        axis.set_title('%s: I8 against %s' % (CONST_PROGRAM_NAMES['optimized'], title), loc='left')
        Panel_Letter(axis, p, theme)
    axes[0].set_ylabel(r'closest robot$-$loader $d$ [mm]')
    axes[0].set_ylim(-3, 53)
    figure.legend(loc='outside right center', fontsize=6, title='at 50 mm: farther\n(the cut; spread\napart to be seen)', title_fontsize=5.5)
    return figure


def main():
    """
    Description:
        The comparison of the taught program (Configuration/Program.yaml) and the optimized one (Data/Optimized/
        Program_Optimized.yaml) (FEATURES.md F14): both trajectories (refused when stale; build them with build_program.py)
        run with physics and recorded every 4 ms (the runs' reports written beside: byte-identical to run_program.py's), the
        cell's geometry along them, the loader slowed against I8; Comparison.md and its figures in DIR. Exit 0, 1 when a run
        fails its acceptance, 2 when it cannot run.

            .venv/bin/python Example/Report/compare_report.py [--out DIR] [--trajectories DIR] [--media DIR]
    """

    parser = argparse.ArgumentParser(description='Compare the taught and the optimized program: Comparison.md and its figures.')
    parser.add_argument('--out', default=CONST_OUT, help='the folder (default: %s)' % os.path.relpath(CONST_OUT, CONST_ROOT))
    parser.add_argument('--trajectories', help='the folder of Program.npz and Program_Optimized.npz (default: Data/Generated/Trajectories)')
    parser.add_argument('--media', help='also copy the summary chart (both themes, PNG and SVG) into this folder for the README (e.g. Media)')
    args = parser.parse_args()

    Configure_MuJoCo_Log()
    log = Script_Log_Cls('compare_report')
    note = lambda line: print(log.Line(line), flush=True)
    began = time.perf_counter()
    run_program = _Script('Simulation/run_program.py')
    try:
        loaded = [Load(v, args.trajectories) for v in CONST_VERSIONS]
        zones = Zones(loaded[0][0], [x[3] for x in loaded])
        jobs = [(kind, (v, zones, args.trajectories)) for v in CONST_VERSIONS for kind in ('physics', 'planned')]
        jobs.append(('stretch', ('optimized', args.trajectories)))
        jobs += [('swept', (v, part, args.trajectories)) for v in CONST_VERSIONS for part in CONST_SWEPT_PARTS]
        with multiprocessing.get_context('spawn').Pool(len(jobs), initializer=Configure_MuJoCo_Log) as pool:
            results = pool.map(_Job, jobs, chunksize=1)
    except (run_program.Run_Error_Cls, Simulation_Error_Cls) as error:
        note('[ERROR] %s' % error)
        log.Close()
        return 2
    analyses = {v: dict(results[2 * k], **results[2 * k + 1]) for k, v in enumerate(CONST_VERSIONS)}
    for version, part, zone in results[2 * len(CONST_VERSIONS) + 1:]:
        analyses[version].setdefault('swept', {})[part] = zone
    context = Context_Cls(analyses, results[2 * len(CONST_VERSIONS)], zones, args.trajectories)
    os.makedirs(args.out, exist_ok=True)
    for v in CONST_VERSIONS:
        with open(os.path.join(args.out, analyses[v]['name'] + '_physics.txt'), 'w', encoding='utf-8') as handle:
            handle.write('\n'.join(analyses[v]['report']) + '\n')
    if args.media:
        for path in _Themed(Figure_Summary, context, args.media, CONST_MEDIA_SUMMARY):
            note('[INFO] %s (%.0f kB)' % (path, os.path.getsize(path) / 1e3))
    figures = {stem: _Themed(make, context, args.out, stem) for stem, make in (
        ('summary', Figure_Summary), ('timeline', Figure_Timeline), ('joints', Figure_Joints), ('paths', Figure_Paths),
        ('clearance', Figure_Clearance), ('critical_path', Figure_Critical_Path), ('stretch', Figure_Stretch))}
    sources = ['# Default vs Optimized program: comparison', '',
               '> %s' % CONST_DISCLAIMER, '',
               'Written by `Example/Report/compare_report.py` (commit %s%s). Sources of every number: **[K]** the kinematic trajectory '
               '(%s, %s: `Program.Core.Build_Program`, 4 ms); **[P]** the physics run of that trajectory, recorded every 4 ms '
               '(`Report.Core.Record_Physics`; its report `%s_physics.txt` and `%s_physics.txt` here, byte-identical to `run_program.py`\'s); '
               '**[C]** the configuration and program files; **[M]** the compiled model. Planning-model distances are exact '
               '(`mj_geomDistance`).' % (context.Trajectories['taught'].Meta['git_commit'][:7], ' + uncommitted changes' if
                                         context.Trajectories['taught'].Meta['git_dirty'] else '', 'Data/Generated/Trajectories/Program.npz',
                                         'Program_Optimized.npz', analyses['taught']['name'], analyses['optimized']['name']), '',
               'Figures (each `<name>_light` and `<name>_dark`, PNG at 300 dpi and SVG): %s.' % ', '.join(
                   '[%s](%s_light.png)' % (k, k) for k in figures), '',
               'The programs: **Default program** - the taught program, `Configuration/Program.yaml`; **Optimized program** - '
               '`Data/Optimized/Program_Optimized.yaml` (in the code and the files: taught / optimized).', '']
    summary = ['## Summary', '', '- Cycle: Default %.3f s, Optimized %.3f s [K] (%.1f %% shorter); throughput %.1f and %.1f PCB/h.' % (
        context.Trajectories['taught'].T[-1], context.Trajectories['optimized'].T[-1],
        100 * (1 - context.Trajectories['optimized'].T[-1] / context.Trajectories['taught'].T[-1]), 3600 / context.Trajectories['taught'].T[-1],
        3600 / context.Trajectories['optimized'].T[-1])]
    first = context.Stretch['first'].get('Loader_Free + Tracking_Margin (I8)')
    hit = context.Stretch['first'].get('contact')
    summary.append('- Acceptance: Default %d/%d, Optimized %d/%d checks pass [P].' % (
        sum(1 for x in analyses['taught']['acceptance'] if x[1]), len(analyses['taught']['acceptance']),
        sum(1 for x in analyses['optimized']['acceptance'] if x[1]), len(analyses['optimized']['acceptance'])))
    summary.append('- A slower loader: %s [K].' % ('no failure of I8 for any job up to %.0f x as long (the take gated by '
                                                   'Loader.WaitDone(J_RETURN_Y), F15)' % CONST_FACTORS[-1] if first is None else
                                                   '`%s` %.1f %% slower breaks I8, %s slower is a contact' % (
                                                       context.Stretch.get('worst', '-'), 100 * (first[0] - 1),
                                                       '-' if hit is None else '%.1f %%' % (100 * (hit[0] - 1)))))
    robot = context.Stretch['robot']['first'].get('Loader_Free + Tracking_Margin (I8)')
    summary.append('- A slower robot: %s [K].' % ('no failure of I8 for any job down to a %.0f %% speed override (the insert started after '
                                                  'the robot leaves, F16)' % (100 * CONST_OVERRIDES[0]) if robot is None else
                                                  'below a %.1f %% override `%s` breaks I8' % (100 / robot[0], context.Stretch['robot']['worst'])))
    summary.append('')
    body = (sources + summary + Section_Same_Task(context) + [''] + Section_Conditions(context) + [''] + Section_Objects(context) +
            Section_Interactions(context) + [''] + Section_PCB(context) + [''] + Section_Cycle(context) + [''] +
            Section_Figures(context, figures) + [''] + Section_Not_Modelled(context) + [''] +
            Section_Verdict(context) + ['', '## 10. Media', '', 'Written by `Example/Report/make_media.py` (videos, previews, the key points, the '
                                         'collision structure); listed in its own output.', ''])
    report = os.path.join(os.path.abspath(args.out), 'Comparison.md')
    with open(report, 'w', encoding='utf-8') as handle:
        handle.write('\n'.join(body) + '\n')
    failed = [(v, n) for v in CONST_VERSIONS for n, ok, _ in analyses[v]['acceptance'] if not ok]
    for v in CONST_VERSIONS:
        note('[INFO] %s: physics run %.1f s, its analysis %.1f s, the planned analysis %.1f s (in parallel)' % (
            v, analyses[v]['physics_time'], analyses[v]['time'], analyses[v]['planned_time']))
    note('[%s] Wrote %s and %d figures in %.1f s; %d acceptance check(s) failed. %s' % ('ERROR' if failed else 'INFO', report, len(figures),
                                                                                         time.perf_counter() - began, len(failed), CONST_DISCLAIMER))
    log.Close()
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
