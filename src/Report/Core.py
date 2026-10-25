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
# File Name: ../Report/Core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Dataclasses (Data Classes)
import dataclasses
from dataclasses import dataclass, field
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core - the cell, the PCB poses
from Configuration.Core import Cell_Configuration_Str, PCB_Meta_Str
#   ../Configuration/Program_Parser - the program, the PCB locations
from Configuration.Program_Parser import CONST_PCB_LOCATIONS, Program_Str
#   ../Kinematics/Core - FK on the planning model
from Kinematics.Core import Robot_Kinematics_Cls
#   ../Collision/Core - exact distances, the loader's bodies, the cups touching the PCB
from Collision.Core import Robot_Collision_Cls, Cup_Contacts, CONST_LOADER_BODIES, CONST_MG06_UPPER_BODIES
#   ../Simulation/Core - the physics run
from Simulation.Core import Physics_Run_Cls, Cycle_Result_Str
#   ../Program/Check - posing at a sample, the loader where its actuators put it, its distance to the robot
from Program.Check import Pose_Sample, Loader_Actual_Trajectory, Robot_Loader_Distance
#   ../Optimizer/Measure - a loader job's samples
from Optimizer.Measure import Job_Bounds
#   ../Trajectory/Core - the trajectory format
from Trajectory.Core import Trajectory_Str, CONST_DT

"""
Description:
    Initialization of constants.

    The comparison of the taught and the optimized program (FEATURES.md F14, SPEC §7.12): a physics run recorded every 4 ms
    without changing it, the cell's geometry along a trajectory or a recording (exact clearances per pair, extents, the
    shared zones), the PCB's custody and handovers, the robot's waits and the cycle's critical path, the loader's timing
    stretched against the interlock I8. It prints nothing; Example/Report/compare_report.py writes the report.
"""
# A channel moves between two samples when some component changes by more than this (the plans hold exactly).
#   Unit [float] (rad or m)
CONST_MOVING_TOLERANCE = 1e-9
# The clearance pass: every CONST_STRIDE-th sample (20 ms), distances up to CONST_CUT (100 mm: every near approach, at a
# third of the exact distances a 200 mm cut needs); each pair's minimum then refined on every sample within CONST_REFINE of it.
#   Unit [int] (-), [float] (m), [int] (-)
CONST_STRIDE, CONST_CUT, CONST_REFINE = 5, 0.1, 5
# The robot's links for its self-clearance, base to tool (the flange's tool0 with wrist 3, the cups with the tool block); a
# pair is measured when the links are not neighbours and the model does not exclude it.
#   Unit [List<Tuple<string>>]
CONST_ROBOT_LINKS = [('aubo/base_link',), ('aubo/shoulder_link',), ('aubo/upper_arm_link',), ('aubo/forearm_link',), ('aubo/wrist_1_link',),
                     ('aubo/wrist_2_link',), ('aubo/wrist_3_link', 'aubo/tool0_link'),
                     ('aubo/tool/ee_tool', 'aubo/tool/vgc10_cup0', 'aubo/tool/vgc10_cup1', 'aubo/tool/vgc10_cup2', 'aubo/tool/vgc10_cup3')]
# The static environment: what never moves; the robot's mount (its base on the pedestal plate, distance 0 by construction,
# as in the keypoint check) is not measured against it.
#   Unit [Set<string>], [string]
CONST_STATIC_BODIES = {'world', 'workstation', 'mg06_base', 'loader_base'}
CONST_MOUNT = 'aubo/base_link'
# The jaw zone (owner, 2026-10-25): the bounding box of the loader's jaws' collision geoms at L_HOME (open and clamped), grown by
# Cell.yaml Clearance.Loader_Free. The MG06 shared zone is the volume the loader sweeps (Swept_Zone), not a box.
#   Unit [Dict<string, Set<string>>]
CONST_ZONE_BODIES = {'jaw zone': {'jaw_L', 'jaw_R'}}
# Who may occupy a zone: the robot's moving links (its base and shoulder turn in place), the loader's Z carriage and jaws
# (the Y carriage runs on its rail beside MG06), the PCB.
#   Unit [Dict<string, Set<string>>]
CONST_OCCUPANTS = {'robot': {'aubo/upper_arm_link', 'aubo/forearm_link', 'aubo/wrist_1_link', 'aubo/wrist_2_link', 'aubo/wrist_3_link',
                             'aubo/tool0_link', 'aubo/tool/ee_tool', 'aubo/tool/vgc10_cup0', 'aubo/tool/vgc10_cup1', 'aubo/tool/vgc10_cup2',
                             'aubo/tool/vgc10_cup3'},
                   'loader': {'z_carriage', 'jaw_L', 'jaw_R'}, 'PCB': {'pcb'}}
# The pairs measured over time: (name, side 1, side 2) by group (robot, PCB, loader, MG06 = its lid, handle and press plate,
# environment = CONST_STATIC_BODIES).
#   Unit [List<Tuple<string>>]
CONST_PAIRS = [('robot - environment', 'robot', 'environment'), ('robot - MG06 lid/handle/press', 'robot', 'MG06'),
               ('robot - loader', 'robot', 'loader'), ('robot - PCB', 'robot', 'PCB'), ('robot - own links', 'robot', 'self'),
               ('PCB - environment', 'PCB', 'environment'), ('PCB - loader', 'PCB', 'loader'), ('PCB - MG06 lid/handle/press', 'PCB', 'MG06'),
               ('loader - environment', 'loader', 'environment'), ('loader - MG06 lid/handle/press', 'loader', 'MG06')]
# The contacts by design: a pair touches by design in a step whose Simulation.yaml Allow_Contact lists its group, and the PCB
# rests on the environment by design where it lies (a crate, MG06).
#   Unit [Dict<string, string>], [Set<string>]
CONST_PAIR_GROUP = {'robot - PCB': 'cups_pcb', 'robot - MG06 lid/handle/press': 'handle_bar', 'PCB - environment': 'pcb_crate',
                    'PCB - loader': 'pcb_jaws'}
CONST_RESTING = {'input_crate', 'output_crate', 'mg06'}
# The loader's position above MG06 (Constants.yaml): below its Z at its Y, the jaws place and re-grip the PCB on the holder.
#   Unit [string]
CONST_MG06_LOADER = 'L_TRAVEL'
# The PCB's holders: a physics contact closer than Cell.yaml Simulation.Cup_Touch with these bodies; each PCB location's.
#   Unit [Dict<string, string>]
CONST_HOLDERS = ('cups', 'jaws', 'MG06 holder', 'crate')
CONST_LOCATION_HOLDER = {'input_crate': 'crate', 'cups': 'cups', 'jaws': 'jaws', 'mg06': 'MG06 holder', 'output_crate': 'crate'}
# The robot's waits by their line (the critical path): the line's start and what the robot waits for.
#   Unit [List<Tuple<string, string>>]
CONST_WAIT_KINDS = [('Loader.WaitDone', 'loader'), ('Loader.Run', 'loader'), ('Tester.Run', 'tester'), ('Robot.WaitDI', 'vacuum signal'),
                    ('Robot.SetDO', 'robot dwell'), ('Wait(', 'robot dwell'), ('End_Settle', 'robot dwell')]
# A holding geom of the PCB's overhang: closer to the PCB than this.
#   Unit [float] (m)
CONST_SUPPORT_GAP = 0.002
# The joint whose zero is the wrist singularity (wrist_2), its index.
#   Unit [int]
CONST_WRIST_2 = 4


@dataclass
class Physics_Record_Str:
    """
    Description:
        A physics run's state at every trajectory sample (4 ms): what Record_Physics saw, read only.
    """

    # The time of each record.
    #   Unit [Vector<float> 1xN] (s)
    T: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # The model's positions and velocities.
    #   Unit [Matrix<float> Nxnq] (m, rad, -), [Matrix<float> Nxnv] (m/s, rad/s)
    Qpos: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))
    Qvel: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))
    # The PCB's holders: the cups touching it, and whether the loader's jaws, the MG06 holder or a crate touch it.
    #   Unit [Vector<int> 1xN], [Matrix<bool> Nx3] (jaws, MG06 holder, crate)
    Cups: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int8))
    Holders: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), bool))
    # The vacuum command.
    #   Unit [Vector<int> 1xN] (-)
    Vacuum: np.ndarray = field(default_factory=lambda: np.zeros(0, np.int8))
    # The suction TCP's position.
    #   Unit [Matrix<float> Nx3] (m)
    Tcp: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))


def Record_Physics(model: mujoco.MjModel, drive: tp.Any, pcb_meta: PCB_Meta_Str,
                   cell: Cell_Configuration_Str) -> tp.Tuple[Cycle_Result_Str, Physics_Record_Str]:
    """
    Description:
        A program's physics run (Physics_Run_Cls, stepped as Run_Physics steps it) with its state recorded at every trajectory
        sample (4 ms): it reads the run's data after a step and never writes it, so the run's result is the one Run_Physics
        gives (its report byte-identical, test/Report/test_core.py).

    Args:
        (1) model [mujoco.MjModel]: The compiled cell.
        (2) drive [Trajectory_Drive_Cls(object)]: What drives it.
        (3) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (4) cell [Cell_Configuration_Str(object)]: The cell configuration.

    Returns:
        (1) parameter [Cycle_Result_Str(object)]: The run's measured values.
        (2) parameter [Physics_Record_Str(object)]: Its states.
    """

    run = Physics_Run_Cls(model, drive, pcb_meta, cell)
    m, d = model, run.Data
    sub = max(1, int(round(CONST_DT / m.opt.timestep)))
    pcb, cups = m.body('pcb').id, {m.body(f'aubo/tool/vgc10_cup{i}').id for i in range(4)}
    others = [{m.body(n).id for n in ('jaw_L', 'jaw_R')}, {m.body('mg06_base').id}, {m.body('workstation').id}]
    tcp, vacuum = m.site('aubo/tool/suction_tcp').id, m.actuator('vacuum0').id
    rows: tp.List[tp.Tuple[tp.Any, ...]] = []

    def Row() -> None:
        """
        Description:
            Record the run's current state.
        """

        touching = [False, False, False]
        for c in d.contact[:d.ncon]:
            bodies = {m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]}
            if pcb in bodies and c.dist < cell.Cup_Touch:
                for k, group in enumerate(others):
                    touching[k] = touching[k] or bool(bodies & group)
        rows.append((d.time, d.qpos.copy(), d.qvel.copy(), Cup_Contacts(m, d, pcb, cups, cell.Cup_Touch), touching, d.ctrl[vacuum] > 0.5,
                     d.site_xpos[tcp].copy()))

    Row()
    while not run.Done:
        run.Step()
        if int(round(d.time / m.opt.timestep)) % sub == 0:
            Row()
    record = Physics_Record_Str(T=np.array([r[0] for r in rows]), Qpos=np.array([r[1] for r in rows]), Qvel=np.array([r[2] for r in rows]),
                                Cups=np.array([r[3] for r in rows], np.int8), Holders=np.array([r[4] for r in rows], bool),
                                Vacuum=np.array([r[5] for r in rows], np.int8), Tcp=np.array([r[6] for r in rows]))
    return run.Result(), record


def Moving(values: np.ndarray, tolerance: float = CONST_MOVING_TOLERANCE) -> np.ndarray:
    """
    Description:
        Per sample, whether a channel moved into it from the sample before (sample 0 never moves).

    Args:
        (1) values [Matrix<float> Nxk or Vector<float> N]: The channel.
        (2) tolerance [float]: The change that counts (its unit).

    Returns:
        (1) parameter [Vector<bool> N]: Moving.
    """

    v = values.reshape(len(values), -1)
    out = np.zeros(len(v), bool)
    out[1:] = np.max(np.abs(np.diff(v, axis=0)), axis=1) > tolerance
    return out


def Spans(mask: np.ndarray, t: np.ndarray) -> tp.List[tp.Tuple[float, float]]:
    """
    Description:
        The time spans where a per-sample mask is true; a span of samples i..j covers t[i] - dt .. t[j] (sample i is
        reached from the one before it).

    Args:
        (1) mask [Vector<bool> N]: The mask.
        (2) t [Vector<float> N]: The sample times (s).

    Returns:
        (1) parameter [List<Tuple<float, float>>]: (start, end) per span (s).
    """

    dt = float(t[1] - t[0]) if len(t) > 1 else CONST_DT
    edges = np.flatnonzero(np.diff(np.r_[0, mask.astype(np.int8), 0]))
    return [(max(0.0, float(t[a]) - dt) if a > 0 else float(t[0]), float(t[b - 1])) for a, b in zip(edges[::2], edges[1::2])]


def Object_Channels(trajectory: Trajectory_Str) -> tp.Dict[str, np.ndarray]:
    """
    Description:
        The moving objects' channels in a trajectory: the robot's joints, the loader's Y and Z axis commands, the gripper's
        jaw command (the posed jaws stop at the PCB: Jaw_Actual), MG06's lid and handle (the press plate follows the handle).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.

    Returns:
        (1) parameter [Dict<string, Matrix<float>>]: Per object its samples (rad or m).
    """

    tr = trajectory
    return {'robot': tr.Robot_Q, 'loader Y': tr.Loader_Y[:, None], 'loader Z': tr.Loader_Z[:, None], 'loader jaws': tr.Gripper[:, None],
            'MG06 lid/handle': np.c_[tr.MG06_Lid, tr.MG06_Handle]}


def Segment_Kind(text: str, parallel: bool = False) -> str:
    """
    Description:
        What the robot does in a segment of its timeline (the critical path): 'robot motion' for a motion line, else what it
        waits for (CONST_WAIT_KINDS); a Wait in a step that runs a loader job in parallel waits for the loader by its time
        ('loader (timed wait)').

    Args:
        (1) text [string]: The segment's command text.
        (2) parallel [bool]: Whether the segment's step starts a loader job (Loader.Start).

    Returns:
        (1) parameter [string]: The kind.
    """

    if text.startswith('Robot.Move'):
        return 'robot motion'
    if parallel and text.startswith('Wait('):
        return 'loader (timed wait)'
    for prefix, kind in CONST_WAIT_KINDS:
        if text.startswith(prefix):
            return kind
    return 'loader start' if text.startswith('Loader.Start') else 'other'


def Segment_Kinds(trajectory: Trajectory_Str) -> tp.List[str]:
    """
    Description:
        Segment_Kind of every segment of a trajectory.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.

    Returns:
        (1) parameter [List<string>]: The kinds, per segment.
    """

    tr = trajectory
    parallel = {int(tr.Segment_Step[k]) for k, text in enumerate(tr.Segment_Command) if text.startswith('Loader.Start')}
    return [Segment_Kind(text, int(step) in parallel) for text, step in zip(tr.Segment_Command, tr.Segment_Step)]


def Critical_Path(trajectory: Trajectory_Str) -> tp.Dict[str, float]:
    """
    Description:
        The cycle's critical path: the robot's program is the cell's timeline (every other object starts from its lines),
        so each of its segments is spent on what it does - its motion, or the loader, the tester, the vacuum signal or a
        dwell it waits for (Segment_Kinds). A sample i >= 1 stands for the time from sample i - 1 to it (sample 0 is the start
        state), so the sums add up to the cycle time.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.

    Returns:
        (1) parameter [Dict<string, float>]: Seconds per kind.
    """

    tr = trajectory
    out: tp.Dict[str, float] = {}
    for kind, a, b in zip(Segment_Kinds(tr), tr.Segment_Start, tr.Segment_End):
        out[kind] = out.get(kind, 0.0) + float(b - max(a, 1)) * CONST_DT
    return out


def Robot_Waits(trajectory: Trajectory_Str) -> tp.List[tp.Tuple[float, float, str, str]]:
    """
    Description:
        Every segment in which the robot waits for something (not its motion, not a zero-length line): its first sample's
        time (as the interlocks and the per-part tables give a start) and its duration (Critical_Path's).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.

    Returns:
        (1) parameter [List<Tuple<float, float, string, string>>]: (start, duration, kind, line) (s, s, -, -).
    """

    tr = trajectory
    out = []
    for kind, text, a, b in zip(Segment_Kinds(tr), tr.Segment_Command, tr.Segment_Start, tr.Segment_End):
        if kind != 'robot motion' and b > max(a, 1):
            out.append((float(tr.T[a]), float(b - max(a, 1)) * CONST_DT, kind, text))
    return out


def Signal_Order(trajectory: Trajectory_Str) -> tp.List[tp.Tuple[float, str]]:
    """
    Description:
        The cell's handshake in time: every loader start, wait and run, tester run, digital output and input line of the robot's
        program, with the time its segment starts.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.

    Returns:
        (1) parameter [List<Tuple<float, string>>]: (time, line) (s, -).
    """

    tr = trajectory
    keys = ('Loader.', 'Tester.', 'Robot.SetDO', 'Robot.WaitDI')
    return [(float(tr.T[a]), text) for text, a in zip(tr.Segment_Command, tr.Segment_Start) if text.startswith(keys)]


def Wrist_Approach(trajectory: Trajectory_Str) -> tp.Dict[str, tp.Tuple[float, float, str]]:
    """
    Description:
        The robot's closest approach to the wrist singularity (wrist_2 = 0): over the whole cycle; outside the segments where
        wrist_2 changes sign (the IK family's switch, a joint-space move by design, SPEC §4.7); within the Cartesian moves
        (MoveL, MovePath), where it would matter.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.

    Returns:
        (1) parameter [Dict<string, Tuple<float, float, string>>]: Per scope (|wrist_2| (rad), time (s), the line).
    """

    tr = trajectory
    w = np.abs(tr.Robot_Q[:, CONST_WRIST_2])
    scopes = {'whole cycle': np.ones(len(w), bool), 'outside the family switch': np.ones(len(w), bool), 'Cartesian moves': np.zeros(len(w), bool)}
    for text, a, b in zip(tr.Segment_Command, tr.Segment_Start, tr.Segment_End):
        q = tr.Robot_Q[max(a - 1, 0):b, CONST_WRIST_2]
        if len(q) and np.min(q) < 0 < np.max(q):
            scopes['outside the family switch'][a:b] = False
        if text.startswith(('Robot.MoveL', 'Robot.MovePath')):
            scopes['Cartesian moves'][a:b] = True
    out = {}
    for scope, mask in scopes.items():
        i = int(np.flatnonzero(mask)[np.argmin(w[mask])])
        segment = int(np.searchsorted(tr.Segment_End, i, side='right'))
        out[scope] = (float(w[i]), float(tr.T[i]), tr.Segment_Command[min(segment, len(tr.Segment_Command) - 1)])
    return out


def Job_Window(trajectory: Trajectory_Str, job: str) -> tp.Tuple[int, int, int]:
    """
    Description:
        A loader job run in parallel (Loader.Start ... Loader.WaitDone): its first sample, the sample where the robot's
        WaitDone starts (its own lines before it run while the loader moves) and the job's last loader change.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) job [string]: The job.

    Returns:
        (1 - 3) parameter [int]: Start, the WaitDone's start, the last change.
    """

    tr = trajectory
    texts = tr.Segment_Command
    start = int(tr.Segment_Start[texts.index('Loader.Start(%s)' % job)])
    wait = int(tr.Segment_Start[texts.index('Loader.WaitDone(%s)' % job)])
    return start, wait, Job_Bounds(tr, job)[1]


def Stretch_Job(trajectory: Trajectory_Str, job: str, factor: float) -> tp.Tuple[Trajectory_Str, range]:
    """
    Description:
        The trajectory up to the end of a parallel loader job with the job's loader lane slowed by a factor (every move and
        hold of the job takes factor times as long; the robot's lines run as programmed, and the robot holds at its
        WaitDone until the job ends - the program's own rule). The PCB in the jaws moves with the loader. Nothing after
        the job is kept. At factor 1 the samples are the trajectory's own (test/Report/test_core.py).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) job [string]: The job (started with Loader.Start).
        (3) factor [float]: The slow-down (1 = as programmed; 1.25 = 25 % slower).

    Returns:
        (1) parameter [Trajectory_Str(object)]: The trajectory from sample 0 to the slowed job's end.
        (2) parameter [range]: The samples of the job's window in it.
    """

    tr = trajectory
    a, w, last = Job_Window(tr, job)
    n = max(w - a, int(math.ceil(factor * (last - a) - 1e-9))) + 1
    k = np.arange(n)
    u = a + np.minimum(k / factor, last - a)
    lo = np.floor(u).astype(int)
    hi = np.minimum(lo + 1, len(tr.T) - 1)
    f = u - lo
    r = a + np.minimum(k, w - a)

    def Lane(x: np.ndarray) -> np.ndarray:
        """
        Description:
            A loader channel at the slowed times (linear between samples).

        Args:
            (1) x [Vector<float> N]: The channel.

        Returns:
            (1) parameter [Vector<float> n]: Its window.
        """

        return x[lo] + (x[hi] - x[lo]) * f

    location = tr.PCB_Location[np.where(f < 0.5, lo, hi)]
    pose = tr.PCB_Pose[r].copy()
    jaws = location == CONST_PCB_LOCATIONS.index('jaws')
    pose[jaws] = tr.PCB_Pose[lo][jaws] + (tr.PCB_Pose[hi][jaws] - tr.PCB_Pose[lo][jaws]) * f[jaws][:, None]
    pose[jaws, 3:] = tr.PCB_Pose[lo][jaws, 3:]
    times = tr.T[np.minimum(a + k, len(tr.T) - 1)] + np.maximum(0, a + k - (len(tr.T) - 1)) * CONST_DT
    window = {'T': times, 'Robot_Q': tr.Robot_Q[r], 'Robot_QD': tr.Robot_QD[r] * (k <= w - a)[:, None],
              'Loader_Y': Lane(tr.Loader_Y), 'Loader_Z': Lane(tr.Loader_Z), 'Gripper': Lane(tr.Gripper), 'Jaw': Lane(tr.Jaw),
              'Vacuum': tr.Vacuum[r], 'Step_Index': tr.Step_Index[r], 'Segment_Id': tr.Segment_Id[r], 'PCB_Location': location,
              'PCB_Pose': pose, 'MG06_Lid': tr.MG06_Lid[r], 'MG06_Handle': tr.MG06_Handle[r]}
    joined = {name: np.concatenate([getattr(tr, name)[:a], value]) for name, value in window.items()}
    return dataclasses.replace(tr, **joined), range(a, a + n)


def Stretched_Distance(trajectory: Trajectory_Str, job: str, factor: float, kinematics: Robot_Kinematics_Cls,
                       collision: Robot_Collision_Cls) -> tp.Tuple[float, float, str]:
    """
    Description:
        The interlock I8's distance with a parallel loader job slowed (Stretch_Job): the closest robot (with a carried PCB) -
        loader (with the PCB in its jaws) distance over the job's window, the loader posed where its actuators put it
        (Loader_Actual_Trajectory), exact (Robot_Loader_Distance), every 4 ms.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) job [string]: The job.
        (3) factor [float]: The slow-down.
        (4) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (5) collision [Robot_Collision_Cls(object)]: On the same model and data.

    Returns:
        (1) parameter [float]: The distance (m; Program.Check's cut when nothing is closer).
        (2) parameter [float]: Its time from the job's start (s; NaN when nothing is closer).
        (3) parameter [string]: The pair.
    """

    stretched, window = Stretch_Job(trajectory, job, factor)
    actual = Loader_Actual_Trajectory(stretched, kinematics.Model)
    distance, sample, pair = Robot_Loader_Distance(actual, kinematics, collision, window)
    return distance, (sample - window.start) * CONST_DT if sample >= 0 else math.nan, pair


@dataclass
class Handover_Str:
    """
    Description:
        One handover of the PCB from one holder to the next (a change of its location in the trajectory), checked in physics.
    """

    # The trajectory's location change: its time, the holders before and after.
    #   Unit [float] (s), [string], [string]
    T: float = 0.0
    Before: str = ''
    After: str = ''
    # The PCB's pose in physics against the trajectory's then: the position error and the angle between them.
    #   Unit [float] (m), [float] (rad)
    Position_Error: float = 0.0
    Angle_Error: float = 0.0
    # In physics: the new holder's first touch and the old one's release around the change (_Run_Edge), how long both touched
    # it between them, and how long no holder touched it between the release and a later first touch.
    #   Unit [float] (s)
    Both: float = 0.0
    New_Touch: float = math.nan
    Old_Release: float = math.nan
    Free: float = 0.0


def Holder_Matrix(record: Physics_Record_Str) -> np.ndarray:
    """
    Description:
        Per record, which holders touch the PCB (the CONST_HOLDERS order): the cups (any of them), the jaws, the MG06 holder,
        a crate.

    Args:
        (1) record [Physics_Record_Str(object)]: The recording.

    Returns:
        (1) parameter [Matrix<bool> Nx4]: The holders.
    """

    return np.c_[record.Cups > 0, record.Holders]


def Custody_Gaps(record: Physics_Record_Str, pcb_address: int) -> tp.List[tp.Tuple[float, float, float]]:
    """
    Description:
        The spans in which nothing holds or supports the PCB in physics (no holder touches it), with how far it fell.

    Args:
        (1) record [Physics_Record_Str(object)]: The recording.
        (2) pcb_address [int]: The PCB free joint's qpos address.

    Returns:
        (1) parameter [List<Tuple<float, float, float>>]: (start, end, drop) (s, s, m).
    """

    held = np.any(Holder_Matrix(record), axis=1)
    out = []
    edges = np.flatnonzero(np.diff(np.r_[0, (~held).astype(np.int8), 0]))
    for a, b in zip(edges[::2], edges[1::2]):
        z = record.Qpos[max(a - 1, 0):min(b + 1, len(held)), pcb_address + 2]
        out.append((float(record.T[max(a - 1, 0)]), float(record.T[min(b, len(held) - 1)]), float(z[0] - np.min(z))))
    return out


def _Run_Edge(mask: np.ndarray, i: int, direction: int) -> int:
    """
    Description:
        A touch's edge around a handover: forwards (+1), the start of the run of touches that is on at sample i or begins
        first after it (the new holder's first touch); backwards (-1), the end of the run that is on at i or ends last before
        it (the old holder's release).

    Args:
        (1) mask [Vector<bool> N]: The touches.
        (2) i [int]: The handover's sample.
        (3) direction [int]: +1 or -1.

    Returns:
        (1) parameter [int]: The sample, or -1 when there is none.
    """

    i = min(max(i, 0), len(mask) - 1)
    if direction > 0:
        after = np.flatnonzero(mask[i:])
        if not len(after):
            return -1
        j = i + int(after[0])
        while j > 0 and mask[j - 1]:
            j -= 1
        return j
    before = np.flatnonzero(mask[:i + 1])
    if not len(before):
        return -1
    j = int(before[-1])
    while j + 1 < len(mask) and mask[j + 1]:
        j += 1
    return j


def Handovers(trajectory: Trajectory_Str, record: Physics_Record_Str, pcb_address: int) -> tp.List[Handover_Str]:
    """
    Description:
        Every change of the PCB's location in the trajectory, checked in physics: the pose error against the trajectory at
        the change, and around it (from the previous change to the next) how long both holders touched the PCB, when the new
        one first touched it, when the old one last did, and how long neither did.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) record [Physics_Record_Str(object)]: The physics recording on its samples.
        (3) pcb_address [int]: The PCB free joint's qpos address.

    Returns:
        (1) parameter [List<Handover_Str(object)>]: The handovers in order.
    """

    tr = trajectory
    n = min(len(tr.T), len(record.T))
    loc = tr.PCB_Location[:n]
    changes = list(np.flatnonzero(np.diff(loc)) + 1)
    holders = Holder_Matrix(record)[:n]
    out = []
    for k, i in enumerate(changes):
        before, after = CONST_LOCATION_HOLDER[CONST_PCB_LOCATIONS[loc[i - 1]]], CONST_LOCATION_HOLDER[CONST_PCB_LOCATIONS[loc[i]]]
        lo, hi = changes[k - 1] if k else 0, changes[k + 1] if k + 1 < len(changes) else n
        old, new = holders[lo:hi, CONST_HOLDERS.index(before)], holders[lo:hi, CONST_HOLDERS.index(after)]
        none = ~np.any(holders[lo:hi], axis=1)
        p, q = record.Qpos[i, pcb_address:pcb_address + 7], tr.PCB_Pose[i]
        dot = abs(float(np.dot(p[3:] / np.linalg.norm(p[3:]), q[3:] / np.linalg.norm(q[3:]))))
        new_touch, old_release = _Run_Edge(new, i - lo, +1), _Run_Edge(old, i - lo, -1)
        both = 0.0
        if new_touch >= 0 and old_release >= 0 and new_touch <= old_release:
            both = float(np.sum(old[new_touch:old_release + 1] & new[new_touch:old_release + 1])) * CONST_DT
        free = float(np.sum(none[old_release + 1:new_touch])) * CONST_DT if 0 <= old_release < new_touch else 0.0
        out.append(Handover_Str(T=float(tr.T[i]), Before=before, After=after, Position_Error=float(np.linalg.norm(p[:3] - q[:3])),
                                Angle_Error=2.0 * math.acos(min(1.0, dot)), Both=both,
                                New_Touch=float(record.T[lo + new_touch]) if new_touch >= 0 else math.nan,
                                Old_Release=float(record.T[lo + old_release]) if old_release >= 0 else math.nan, Free=free))
    return out


def Tilt(quaternions: np.ndarray) -> np.ndarray:
    """
    Description:
        The tilt of a body's z axis from the vertical, per quaternion.

    Args:
        (1) quaternions [Matrix<float> Nx4]: w, x, y, z.

    Returns:
        (1) parameter [Vector<float> N]: The tilt (rad).
    """

    q = quaternions / np.linalg.norm(quaternions, axis=1)[:, None]
    return np.arccos(np.clip(1.0 - 2.0 * (q[:, 1] ** 2 + q[:, 2] ** 2), -1.0, 1.0))


def Location_Spans(trajectory: Trajectory_Str) -> tp.List[tp.Tuple[str, int, int]]:
    """
    Description:
        The PCB's stages: each run of one location in the trajectory.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.

    Returns:
        (1) parameter [List<Tuple<string, int, int>>]: (location, first sample, last sample).
    """

    loc = trajectory.PCB_Location
    starts = np.r_[0, np.flatnonzero(np.diff(loc)) + 1]
    ends = np.r_[starts[1:] - 1, len(loc) - 1]
    return [(CONST_PCB_LOCATIONS[loc[a]], int(a), int(b)) for a, b in zip(starts, ends)]


def Geom_Points(model: mujoco.MjModel, geoms: np.ndarray) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        Points that span geoms, in each geom's frame, grouped by geom in ascending order: a mesh's vertices (the compiled
        mesh is in its geom's frame), a primitive's box corners (geom_aabb).

    Args:
        (1) model [mujoco.MjModel]: The model.
        (2) geoms [Vector<int> k]: The geoms.

    Returns:
        (1) parameter [Vector<int> P]: The geom of each point.
        (2) parameter [Matrix<float> Px3]: The points (m).
    """

    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)], float)
    owners, points = [np.zeros(0, int)], [np.zeros((0, 3))]
    for g in sorted(geoms):
        if model.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH:
            mesh = model.geom_dataid[g]
            local = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
        else:
            box = model.geom_aabb[g]
            local = box[:3] + signs * box[3:]
        owners.append(np.full(len(local), g))
        points.append(np.asarray(local, float))
    return np.concatenate(owners), np.concatenate(points)


class Cell_Geometry_Cls(object):
    """
    Description:
        The cell's geometry at a pose of the planning model (posed from a trajectory sample or a physics record): exact
        clearances per pair of CONST_PAIRS, the moving objects' extents, the shared zones' occupancy.

    Initialization of the Class:
        Args:
            (1) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
            (2) collision [Robot_Collision_Cls(object)]: On the same model and data.
            (3) margin [float]: The zones' growth (Cell.yaml Clearance.Loader_Free, m).

        Example:
            Initialization:
                Cls = Cell_Geometry_Cls(kinematics, collision, cell.Loader_Free)

            Features:
                # Properties of the class.
                Cls.Zones

                # Functions of the class.
                Cls.Pose_Record(qpos); Cls.Measure('cups')
    """

    def __init__(self, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, margin: float) -> None:
        m = kinematics.Model
        self.__kinematics, self.__collision, self.__margin = kinematics, collision, margin
        self.__model, self.__data = m, kinematics.Data
        names = {i: m.body(i).name for i in range(m.nbody)}
        ids = lambda wanted: {i for i, n in names.items() if n in wanted}
        robot = {i for i, n in names.items() if n.startswith('aubo/')}
        self.__groups = {'robot': robot, 'PCB': ids({'pcb'}), 'loader': ids(CONST_LOADER_BODIES), 'MG06': ids(CONST_MG06_UPPER_BODIES),
                         'environment': ids(CONST_STATIC_BODIES)}
        self.__all = set(names)
        self.__links = [ids(set(link)) for link in CONST_ROBOT_LINKS]
        excluded = {frozenset((int(s) >> 16, int(s) & 0xFFFF)) for s in m.exclude_signature}
        self.__self_pairs = []
        for i, a in enumerate(self.__links):
            partners = set()
            for j in range(i + 2, len(self.__links)):
                if not any(frozenset((x, y)) in excluded for x in a for y in self.__links[j]):
                    partners |= self.__links[j]
            if partners:
                self.__self_pairs.append((a, partners))
        collidable = (m.geom_contype > 0) | (m.geom_conaffinity > 0)
        self.__points = {name: Geom_Points(m, np.flatnonzero(collidable & np.isin(m.geom_bodyid, list(ids(bodies)))))
                         for name, bodies in CONST_OCCUPANTS.items()}
        self.__points['robot all'] = Geom_Points(m, np.flatnonzero(collidable & np.isin(m.geom_bodyid, list(robot))))
        self.__zone_points = {name: Geom_Points(m, np.flatnonzero(collidable & np.isin(m.geom_bodyid, list(ids(bodies)))))
                              for name, bodies in CONST_ZONE_BODIES.items()}
        self.Zones: tp.Dict[str, tp.Tuple[np.ndarray, np.ndarray]] = {}

    def Pose_Record(self, qpos: np.ndarray) -> None:
        """
        Description:
            Pose the planning model at a physics record's positions (the same joints as the physics model).

        Args:
            (1) qpos [Vector<float> nq]: The positions.
        """

        self.__data.qpos[:] = qpos
        mujoco.mj_kinematics(self.__model, self.__data)

    def Pose_Sample(self, trajectory: Trajectory_Str, i: int) -> None:
        """
        Description:
            Pose the planning model at a trajectory's sample (Program.Check.Pose_Sample).

        Args:
            (1) trajectory [Trajectory_Str(object)]: The trajectory.
            (2) i [int]: The sample.
        """

        Pose_Sample(self.__kinematics, trajectory, i)

    def Grow_Zone(self, name: str) -> None:
        """
        Description:
            Add the current pose's zone geoms (CONST_ZONE_BODIES, their points: Geom_Points) to the zone's box (grown by the
            margin).

        Args:
            (1) name [string]: The zone.
        """

        geoms, local = self.__zone_points[name]
        world = self.__data.geom_xpos[geoms] + np.einsum('kij,kj->ki', self.__data.geom_xmat[geoms].reshape(-1, 3, 3), local)
        lo, hi = world.min(axis=0) - self.__margin, world.max(axis=0) + self.__margin
        if name in self.Zones:
            lo, hi = np.minimum(lo, self.Zones[name][0]), np.maximum(hi, self.Zones[name][1])
        self.Zones[name] = (lo, hi)

    def Measure(self, location: str) -> tp.Tuple[np.ndarray, tp.Dict[str, np.ndarray], tp.Dict[str, bool]]:
        """
        Description:
            The current pose's clearances (CONST_PAIRS; NaN where a pair is not separate: the robot and the PCB on its cups,
            the loader and the PCB in its jaws), the objects' extents and the zones' occupancy from their collision geoms'
            points (Geom_Points: a mesh's vertices, a primitive's box corners).

        Args:
            (1) location [string]: The PCB's location then (CONST_PCB_LOCATIONS).

        Returns:
            (1) parameter [Vector<float> len(CONST_PAIRS)]: The distances (m, CONST_CUT when nothing is closer).
            (2) parameter [Dict<string, Matrix<float> 2x3>]: Per object (CONST_OCCUPANTS and 'robot all') its box (m).
            (3) parameter [Dict<string, bool>]: Per 'object in zone' whether a point of it is inside the zone.
        """

        distances = np.array([self.Pair_Distance(name, location)[0] for name, _, _ in CONST_PAIRS])
        d, m = self.__data, self.__model
        boxes, occupied = {}, {}
        for name, (owners, local) in self.__points.items():
            # Each geom's box first (cheap); its points only where the box can decide: a box beyond every first point's
            # extreme, or a box that meets a zone.
            geoms = np.unique(owners)
            R = d.geom_xmat[geoms].reshape(-1, 3, 3)
            centre = d.geom_xpos[geoms] + np.einsum('kij,kj->ki', R, m.geom_aabb[geoms, :3])
            half = np.einsum('kij,kj->ki', np.abs(R), m.geom_aabb[geoms, 3:])
            lo, hi = centre - half, centre + half
            first = d.geom_xpos[geoms] + np.einsum('kij,kj->ki', R, local[np.searchsorted(owners, geoms)])
            wanted = np.any(hi >= first.max(axis=0), axis=1) | np.any(lo <= first.min(axis=0), axis=1)
            for zlo, zhi in self.Zones.values():
                wanted |= np.all((lo <= zhi) & (hi >= zlo), axis=1)
            keep = np.isin(owners, geoms[wanted])
            world = d.geom_xpos[owners[keep]] + np.einsum('kij,kj->ki', d.geom_xmat[owners[keep]].reshape(-1, 3, 3), local[keep])
            boxes[name] = np.array([world.min(axis=0), world.max(axis=0)])
            for zone, (zlo, zhi) in self.Zones.items():
                if name != 'robot all':
                    occupied['%s in %s' % (name, zone)] = bool(np.any(np.all((world >= zlo) & (world <= zhi), axis=1)))
        return distances, boxes, occupied

    def Pair_Distance(self, name: str, location: str) -> tp.Tuple[float, str]:
        """
        Description:
            One pair's exact distance at the current pose (Collision.Close_Pairs), and its closest geom pair.

        Args:
            (1) name [string]: The pair (CONST_PAIRS).
            (2) location [string]: The PCB's location.

        Returns:
            (1) parameter [float]: The distance (m; CONST_CUT when nothing is closer, NaN when not separate).
            (2) parameter [string]: 'body / body' of the closest pair, or ''.
        """

        _, a, b = next(p for p in CONST_PAIRS if p[0] == name)
        if {a, b} == {'robot', 'PCB'} and location == 'cups' or {a, b} == {'loader', 'PCB'} and location == 'jaws':
            return math.nan, ''
        if b == 'self':
            found = [p for mine, theirs in self.__self_pairs
                     for p in self.__collision.Close_Pairs(mine, CONST_CUT, self.__data, exclude=self.__all_ids() - theirs)[:1]]
        else:
            mine, theirs = self.__groups[a], self.__groups[b]
            if a == 'robot' and b == 'environment':
                mine = mine - {self.__model.body(CONST_MOUNT).id}
            found = self.__collision.Close_Pairs(mine, CONST_CUT, self.__data, exclude=self.__all_ids() - theirs)[:1]
        if not found:
            return CONST_CUT, ''
        best = min(found)
        return float(best[0]), '%s / %s' % (best[1], best[2])

    def __all_ids(self) -> tp.Set[int]:
        """
        Description:
            Every body id of the model.

        Returns:
            (1) parameter [Set<int>]: The ids.
        """

        return set(self.__all)


def _Box_Gaps(boxes: np.ndarray, others: np.ndarray) -> np.ndarray:
    """
    Description:
        The smallest gap between any box of a set and any box of each other set: a lower bound of the exact distance (0 when
        two boxes overlap).

    Args:
        (1) boxes [Matrix<float> Ax2x3]: The boxes (lower, upper corner).
        (2) others [Matrix<float> PxBx2x3]: Per set its boxes.

    Returns:
        (1) parameter [Vector<float> P]: Per set the smallest gap (m).
    """

    lo, hi = boxes[:, 0], boxes[:, 1]
    olo, ohi = others[:, :, 0], others[:, :, 1]
    apart = np.maximum(0.0, np.maximum(olo[:, None, :, :] - hi[None, :, None, :], lo[None, :, None, :] - ohi[:, None, :, :]))
    return np.sqrt(np.sum(apart ** 2, axis=3)).min(axis=(1, 2))


@dataclass
class Swept_Zone_Str:
    """
    Description:
        A shared zone as the volume a loader's jobs sweep (Swept_Zone): the loader's poses, its corridor of axis positions,
        and who is in it when.
    """

    # The jobs and their distinct poses (y, z, jaw), the bounding box of the swept collision geoms (for a figure).
    #   Unit [List<string>], [Matrix<float> Kx3] (m), [Matrix<float> 2x3] (m)
    Jobs: tp.List[str] = field(default_factory=list)
    Poses: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    Box: np.ndarray = field(default_factory=lambda: np.zeros((2, 3)))
    # The samples tested and, per sample, whether the robot (with a carried PCB) comes within the margin of the volume, and
    # whether the loader is on the corridor (its Y and Z within the margin of a swept pose's).
    #   Unit [Vector<int> 1xM], [Vector<bool> 1xM], [Vector<bool> 1xM]
    Samples: np.ndarray = field(default_factory=lambda: np.zeros(0, int))
    Robot: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    Loader: np.ndarray = field(default_factory=lambda: np.zeros(0, bool))
    # The exact distances computed (how many), for the record.
    #   Unit [int]
    Exact: int = 0


def Swept_Zone(trajectory: Trajectory_Str, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, jobs: tp.Sequence[str],
               margin: float, at_y: tp.Optional[float] = None, stride: int = CONST_STRIDE) -> Swept_Zone_Str:
    """
    Description:
        A shared zone as the volume the loader (its Z carriage, jaws and the PCB in them) sweeps in jobs, where its actuators put
        it (Loader_Actual_Trajectory), grown by a margin (owner, 2026-10-25: the MG06 shared zone, J_INSERT and J_FETCH); and who
        is in it, every stride samples: the robot when an exact distance to one of the loader's poses is below the margin (the
        poses with the smallest box gap first; a pose whose box gap, a lower bound, is not below the margin is skipped), the
        loader when its Y and Z are within the margin of a swept pose's (its shape depends on its axes only). The poses are the
        jobs' samples one per cell of a grid of half the margin in (Y, Z, jaw): a pose between two kept ones is at most a quarter
        of the margin from one of them along each axis.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (3) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (4) jobs [List<string>]: The jobs whose motion makes the zone.
        (5) margin [float]: The growth (Cell.yaml Clearance.Loader_Free, m).
        (6) at_y [float]: Only the poses with the loader's Y at this command (its part at MG06), or None for all (m).
        (7) stride [int]: The samples tested (every stride-th and the last).

    Returns:
        (1) parameter [Swept_Zone_Str(object)]: The zone.
    """

    tr = Loader_Actual_Trajectory(trajectory, kinematics.Model)
    model, data = kinematics.Model, kinematics.Data
    address = kinematics.Scene_Address
    pcb = model.body('pcb').id
    robot, loader = collision.Robot_Bodies, {model.body(name).id for name in CONST_LOADER_BODIES}
    collidable = (model.geom_contype > 0) | (model.geom_conaffinity > 0)
    geoms = lambda bodies: np.flatnonzero(collidable & np.isin(model.geom_bodyid, list(bodies)))
    jaws, cups = CONST_PCB_LOCATIONS.index('jaws'), CONST_PCB_LOCATIONS.index('cups')
    samples = sorted({i for job in jobs for i in range(Job_Bounds(tr, job)[0], Job_Bounds(tr, job)[1] + 1)
                      if at_y is None or trajectory.Loader_Y[i] == at_y})
    seen, poses = set(), []
    for i in samples:
        key = tuple(np.round(np.array([tr.Loader_Y[i], tr.Loader_Z[i], tr.Jaw[i]]) / (margin / 2))) + (tr.PCB_Location[i] == jaws,)
        if key not in seen:
            seen.add(key)
            poses.append(i)
    loader_geoms = geoms(loader | {pcb})
    boxes, owners = [], []
    for i in poses:
        Pose_Sample(kinematics, tr, i)
        mine = geoms(loader | ({pcb} if tr.PCB_Location[i] == jaws else set()))
        b = Geom_Points(model, mine)
        world = data.geom_xpos[b[0]] + np.einsum('kij,kj->ki', data.geom_xmat[b[0]].reshape(-1, 3, 3), b[1])
        lo = np.array([world[b[0] == g].min(axis=0) for g in mine])
        hi = np.array([world[b[0] == g].max(axis=0) for g in mine])
        pad = np.full((len(loader_geoms) - len(mine), 2, 3), np.nan)
        boxes.append(np.concatenate([np.stack([lo, hi], axis=1), np.where(np.isnan(pad), 1e9, pad)]))
        owners.append(np.vstack([lo, hi]))
    boxes = np.array(boxes)
    whole = np.array([[b[:, 0][b[:, 0, 0] < 1e8].min(axis=0), b[:, 1][b[:, 0, 0] < 1e8].max(axis=0)] for b in boxes])
    corners = np.vstack(owners)
    yz = np.c_[tr.Loader_Y[poses], tr.Loader_Z[poses]]
    tested = np.unique(np.r_[np.arange(0, len(tr.T), stride), len(tr.T) - 1])
    robot_in, loader_in, exact = np.zeros(len(tested), bool), np.zeros(len(tested), bool), 0
    for k, i in enumerate(tested):
        loader_in[k] = bool(np.min(np.hypot(yz[:, 0] - tr.Loader_Y[i], yz[:, 1] - tr.Loader_Z[i])) <= margin)
        Pose_Sample(kinematics, tr, int(i))
        mine = robot | ({pcb} if tr.PCB_Location[i] == cups else set())
        rb = geoms(mine)
        R = data.geom_xmat[rb].reshape(-1, 3, 3)
        centre = data.geom_xpos[rb] + np.einsum('kij,kj->ki', R, model.geom_aabb[rb, :3])
        half = np.einsum('kij,kj->ki', np.abs(R), model.geom_aabb[rb, 3:])
        mine_boxes = np.stack([centre - half, centre + half], axis=1)
        overall = np.array([mine_boxes[:, 0].min(axis=0), mine_boxes[:, 1].max(axis=0)])
        near = np.flatnonzero(_Box_Gaps(overall[None], whole[:, None]) < margin)
        if not len(near):
            continue
        gaps = np.full(len(poses), np.inf)
        gaps[near] = _Box_Gaps(mine_boxes, boxes[near])
        for p in np.argsort(gaps):
            if gaps[p] >= margin:
                break
            j = poses[int(p)]
            Pose_Sample(kinematics, tr, int(i))
            data.qpos[address['y_axis']], data.qpos[address['z_axis']] = tr.Loader_Y[j], tr.Loader_Z[j]
            data.qpos[address['jaw_L']] = data.qpos[address['jaw_R']] = tr.Jaw[j]
            if tr.PCB_Location[j] == jaws:
                data.qpos[address['pcb_free']:address['pcb_free'] + 7] = tr.PCB_Pose[j]
            mujoco.mj_kinematics(model, data)
            theirs = loader | ({pcb} if tr.PCB_Location[j] == jaws else set())
            exact += 1
            if collision.Close_Pairs(mine, margin, data, exclude=set(range(model.nbody)) - theirs):
                robot_in[k] = True
                break
    return Swept_Zone_Str(Jobs=list(jobs), Poses=np.c_[tr.Loader_Y[poses], tr.Loader_Z[poses], tr.Jaw[poses]],
                          Box=np.array([corners.min(axis=0) - margin, corners.max(axis=0) + margin]), Samples=tested, Robot=robot_in,
                          Loader=loader_in, Exact=exact)


def By_Design(trajectory: Trajectory_Str, program: Program_Str) -> tp.Callable[[str, int], bool]:
    """
    Description:
        Whether a pair (CONST_PAIRS) touches by design at a sample: its group (CONST_PAIR_GROUP) allowed in the sample's step
        (Simulation.yaml Allow_Contact, by the step's name without its S<n>_ prefix), the PCB resting where it lies, or the
        PCB between the loader's jaws and the MG06 holder while the loader is below its travel height above MG06 (the place
        and the re-grip).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) program [Program_Str(object)]: Its program.

    Returns:
        (1) parameter [function]: (pair, sample) -> bool.
    """

    tr = trajectory
    allowed = [set(program.Simulation.Allow_Contact.get(name.split('_', 1)[-1], [])) for name in tr.Step_Names]
    above = program.Constants.Loader[CONST_MG06_LOADER]
    in_mg06 = (tr.Loader_Y == above.Y) & (tr.Loader_Z < above.Z)

    def Check(pair: str, i: int) -> bool:
        """
        Description:
            The rule at one sample.

        Args:
            (1) pair [string]: The pair.
            (2) i [int]: The sample.

        Returns:
            (1) parameter [bool]: True when the pair touches by design.
        """

        if pair == 'PCB - environment' and CONST_PCB_LOCATIONS[tr.PCB_Location[i]] in CONST_RESTING:
            return True
        if pair in ('PCB - environment', 'PCB - loader') and in_mg06[i]:
            return True
        return CONST_PAIR_GROUP.get(pair) in allowed[tr.Step_Index[i]]

    return Check


@dataclass
class Geometry_Series_Str:
    """
    Description:
        The cell's geometry along a trajectory or a recording (Geometry_Series): every CONST_STRIDE-th sample, and each pair's
        minimum refined on every sample around it.
    """

    # The samples measured and their times.
    #   Unit [Vector<int> 1xM], [Vector<float> 1xM] (s)
    Samples: np.ndarray = field(default_factory=lambda: np.zeros(0, int))
    T: np.ndarray = field(default_factory=lambda: np.zeros(0))
    # Per CONST_PAIRS pair its distance per measured sample (CONST_CUT when farther; NaN when not separate).
    #   Unit [Matrix<float> MxP] (m)
    Distance: np.ndarray = field(default_factory=lambda: np.zeros((0, 0)))
    # Per pair its minimum outside the contacts by design: (distance, time, the closest pair), refined.
    #   Unit [Dict<string, Tuple<float, float, string>>] (m, s, -)
    Minimum: tp.Dict[str, tp.Tuple[float, float, str]] = field(default_factory=dict)
    # Per object its box per measured sample.
    #   Unit [Dict<string, Matrix<float> Mx2x3>] (m)
    Boxes: tp.Dict[str, np.ndarray] = field(default_factory=dict)
    # Per 'object in zone' its occupancy per measured sample.
    #   Unit [Dict<string, Vector<bool> 1xM>]
    Occupied: tp.Dict[str, np.ndarray] = field(default_factory=dict)


def Geometry_Series(geometry: Cell_Geometry_Cls, pose: tp.Callable[[int], None], n: int, location: tp.Callable[[int], str],
                    by_design: tp.Callable[[str, int], bool], t: np.ndarray) -> Geometry_Series_Str:
    """
    Description:
        The cell's geometry along n samples: every CONST_STRIDE-th sample (and the last) measured (Cell_Geometry_Cls.Measure);
        each pair's smallest distance outside its contacts by design refined on every sample within CONST_REFINE of it.

    Args:
        (1) geometry [Cell_Geometry_Cls(object)]: The geometry (its zones set).
        (2) pose [function]: Poses sample i.
        (3) n [int]: The number of samples.
        (4) location [function]: The PCB's location at sample i.
        (5) by_design [function]: Whether pair p touches by design at sample i (its minimum is taken elsewhere).
        (6) t [Vector<float> N]: The sample times (s).

    Returns:
        (1) parameter [Geometry_Series_Str(object)]: The series.
    """

    samples = np.unique(np.r_[np.arange(0, n, CONST_STRIDE), n - 1])
    rows, boxes, occupied = [], {}, {}
    for i in samples:
        pose(int(i))
        d, b, o = geometry.Measure(location(int(i)))
        rows.append(d)
        for k, v in b.items():
            boxes.setdefault(k, []).append(v)
        for k, v in o.items():
            occupied.setdefault(k, []).append(v)
    distance = np.array(rows)
    out = Geometry_Series_Str(Samples=samples, T=t[samples], Distance=distance, Boxes={k: np.array(v) for k, v in boxes.items()},
                              Occupied={k: np.array(v) for k, v in occupied.items()})
    for p, (name, _, _) in enumerate(CONST_PAIRS):
        allowed = np.array([not by_design(name, int(i)) for i in samples]) & ~np.isnan(distance[:, p])
        if not allowed.any():
            continue
        k = int(np.flatnonzero(allowed)[np.argmin(distance[allowed, p])])
        best = (float(distance[k, p]), float(t[samples[k]]), '')
        lo, hi = max(0, int(samples[k]) - CONST_REFINE), min(n - 1, int(samples[k]) + CONST_REFINE)
        for i in range(lo, hi + 1):
            if by_design(name, i):
                continue
            pose(i)
            value, pair = geometry.Pair_Distance(name, location(i))
            if not math.isnan(value) and (value < best[0] or not best[2] and value == best[0]):
                best = (value, float(t[i]), pair)
        out.Minimum[name] = best
    return out


def Travel(values: np.ndarray) -> np.ndarray:
    """
    Description:
        A channel's travel: the sum of its absolute changes, per component.

    Args:
        (1) values [Matrix<float> Nxk]: The channel.

    Returns:
        (1) parameter [Vector<float> k]: The travel (its unit).
    """

    return np.sum(np.abs(np.diff(values.reshape(len(values), -1), axis=0)), axis=0)


def Joint_Arc(q: np.ndarray) -> float:
    """
    Description:
        The joint-space arc length of a sampled path: the sum over the samples of the Euclidean norm of the joints' change
        (owner, 2026-10-25: the summary chart's sum of ||dq||; Travel gives the per-joint sum of |dq|).

    Args:
        (1) q [Matrix<float> Nxk]: The joints per sample (rad).

    Returns:
        (1) parameter [float]: The arc length (rad).
    """

    return float(np.sum(np.linalg.norm(np.diff(q, axis=0), axis=1)))


def Channel_Peaks(values: np.ndarray, dt: float) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        A sampled channel's largest speed and acceleration per component, by central differences.

    Args:
        (1) values [Matrix<float> Nxk]: The channel.
        (2) dt [float]: The sample time (s).

    Returns:
        (1 - 2) parameter [Vector<float> k]: The peak speed (its unit/s) and acceleration (its unit/s^2).
    """

    v = np.gradient(values.reshape(len(values), -1), dt, axis=0)
    a = np.gradient(v, dt, axis=0)
    return np.max(np.abs(v), axis=0), np.max(np.abs(a), axis=0)


def Overhang(model: mujoco.MjModel, data: mujoco.MjData, supports: tp.Set[str]) -> tp.Dict[str, float]:
    """
    Description:
        How far the PCB's board reaches beyond what holds it, in the board's frame: per side (+x, -x, +y, -y) the board's
        half size less the farthest extent that way of the holding geoms within CONST_SUPPORT_GAP of the PCB (from the geoms'
        own boxes; negative = the support reaches past the edge). Posed in the data's current kinematics.

    Args:
        (1) model [mujoco.MjModel]: The planning model.
        (2) data [mujoco.MjData]: Its data, posed.
        (3) supports [Set<string>]: The holding bodies (the cups, or the jaws).

    Returns:
        (1) parameter [Dict<string, float>]: Per side the overhang (m).
    """

    pcb = model.body('pcb').id
    boxes = [g for g in range(model.ngeom) if model.geom_bodyid[g] == pcb and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX
             and model.geom_contype[g]]
    board = max(boxes, key=lambda g: model.geom_size[g][0] * model.geom_size[g][1])
    R, p, half = data.geom_xmat[board].reshape(3, 3), data.geom_xpos[board], model.geom_size[board]
    ids = {model.body(n).id for n in supports}
    pcb_geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] == pcb and model.geom_contype[g]]
    ft = np.zeros(6)
    geoms = [g for g in range(model.ngeom) if model.geom_bodyid[g] in ids and (model.geom_contype[g] or model.geom_conaffinity[g])
             and min(mujoco.mj_geomDistance(model, data, g, h, CONST_SUPPORT_GAP, ft) for h in pcb_geoms) < CONST_SUPPORT_GAP]
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    for g in geoms:
        box = model.geom_aabb[g]
        corners = data.geom_xpos[g] + (data.geom_xmat[g].reshape(3, 3) @ (box[:3] + signs * box[3:]).T).T
        local = (corners - p) @ R
        lo, hi = np.minimum(lo, local.min(axis=0)), np.maximum(hi, local.max(axis=0))
    return {'+x': float(half[0] - hi[0]), '-x': float(half[0] + lo[0]), '+y': float(half[1] - hi[1]), '-y': float(half[1] + lo[1])}


def Clear_Height(trajectory: Trajectory_Str, segment: str, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls,
                 heights: np.ndarray, need: float) -> tp.Tuple[float, tp.List[tp.Tuple[float, float]]]:
    """
    Description:
        For a robot segment that ends above the loader's home (the take's approach): the highest loader Z, with the loader at
        its home Y and the PCB in its jaws moved with it, at which the robot's whole segment keeps a distance - so a condition
        "Z at or below it" (the loader then only moves down, away) frees the segment whatever the loader's speed. The loader
        stands still at each height (no actuator model); exact distances every 4 ms (Robot_Loader_Distance).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) segment [string]: The robot segment's text (its first occurrence).
        (3) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (4) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (5) heights [Vector<float> k]: The loader Z values tried, ascending (m).
        (6) need [float]: The distance to keep (m).

    Returns:
        (1) parameter [float]: The highest Z up to which every height tried keeps it (m; NaN when the lowest does not).
        (2) parameter [List<Tuple<float, float>>]: (Z, the closest distance) per height (m, m).
    """

    tr = trajectory
    k = tr.Segment_Command.index(segment)
    a, b = int(tr.Segment_Start[k]), int(tr.Segment_End[k])
    end = int(tr.Segment_End[tr.Segment_Command.index(next(t for t in tr.Segment_Command[k:] if t.startswith('Loader.WaitDone')))]) - 1
    found, best, clear = [], math.nan, True
    for z in heights:
        rows = np.r_[np.arange(a, b), end]
        pcb = np.repeat(tr.PCB_Pose[end][None, :], len(rows), axis=0)
        pcb[:, 2] += z - tr.Loader_Z[end]
        size = len(rows)
        window = dataclasses.replace(tr, T=tr.T[rows], Robot_Q=tr.Robot_Q[np.r_[np.arange(a, b), b - 1]], Loader_Y=np.full(size, tr.Loader_Y[end]),
                                     Loader_Z=np.full(size, z), Jaw=np.full(size, tr.Jaw[end]), PCB_Location=np.full(size, tr.PCB_Location[end]),
                                     PCB_Pose=pcb, MG06_Lid=tr.MG06_Lid[rows], MG06_Handle=tr.MG06_Handle[rows])
        distance = Robot_Loader_Distance(window, kinematics, collision, range(len(rows)))[0]
        found.append((float(z), float(distance)))
        clear = clear and distance >= need
        if clear:
            best = float(z)
    return best, found
