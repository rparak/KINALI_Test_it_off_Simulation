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
# File Name: ../Optimizer/Core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Platform (Access to underlying platform's identifying data)
import platform
# Hashlib (Secure hashes)
import hashlib
# Textwrap (Text wrapping and filling)
import textwrap
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# SciPy (Scientific computing) [pip3 install scipy]
import scipy
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core - the cell, the PCB poses, the files
from Configuration.Core import Cell_Configuration_Str, PCB_Meta_Str, CONST_CELL_YAML, CONST_FORK_PATH
#   ../Configuration/Program_Parser - the programs and the optimizer's settings, the PCB locations
from Configuration.Program_Parser import (CONST_PCB_LOCATIONS, Program_Str, Optimization_Str, Load_Program, Free_Steps, CONST_PROGRAM_YAML,
                                          CONST_CONSTANTS_YAML, CONST_SIMULATION_YAML, CONST_OPTIMIZATION_YAML)
#   ../Kinematics/Core - FK, the PCB held by the cups
from Kinematics.Core import Robot_Kinematics_Cls, PCB_For_TCP
#   ../Collision/Core - exact distances
from Collision.Core import Robot_Collision_Cls, CONST_LOADER_BODIES
#   ../Keypoints/Core - the robot's mount, left out of the clearance
from Keypoints.Core import CONST_MOUNT_BODY, CONST_MOUNT_GEOM_PREFIX
#   ../Planner/Core - RRT-Connect, the shortcut, the spline, its timing
from Planner.Core import Rrt_Connect, Shortcut, Prune, Smooth, Time_Path, Planner_Error_Cls
#   ../Program/Core - the build, its notes, the file hashes
from Program.Core import Input_Hash, Build_Program, Build_Note_Str
#   ../Program/Check - the build's check, posing at a sample, the loader's actual position and its distance to the robot
from Program.Check import Pose_Sample, Check_Trajectory, Program_Check_Str, Robot_Loader_Distance, Loader_Actual_Trajectory, Gating_Faults
#   ../Trajectory/Core - the trajectory format
from Trajectory.Core import Trajectory_Str, CONST_DT

"""
Description:
    Initialization of constants.

    The optimized program at work (FEATURES.md F9): the FREE transfers of the taught program (Configuration/Optimization.yaml)
    planned on the planning model - RRT-Connect from a fixed seed, the shortcut, the spline checked and refitted, the
    time-optimal timing - with the loader's jobs between them run in parallel (Loader.Start / Loader.WaitDone); every
    LOCKED segment copied verbatim; the program written as text (Data/Optimized/Program_Optimized.yaml) with the inputs'
    hashes, the platform and the schedule. What it is measured by (the LOCKED parts, the interlocks, the tilt, the metrics):
    Optimizer/Measure.py. It prints nothing.
"""
# The inputs an optimized program is planned from, by name (its Optimized.Inputs); the planning model's hash is added.
#   Unit [Dict<string, string>]
CONST_INPUTS = {'program': CONST_PROGRAM_YAML, 'constants': CONST_CONSTANTS_YAML, 'simulation': CONST_SIMULATION_YAML, 'cell': CONST_CELL_YAML,
                'optimization': CONST_OPTIMIZATION_YAML, 'fork_path': CONST_FORK_PATH}
# A transfer whose spline meets the loader (I8) or comes where a parallel job has been (the gating rule, F15) runs its spline
# after the next job's start - after the WaitDone of the job it met, never after a Wait (owner, 2026-10-25); a job that comes
# where the robot has been needs the robot out of its way before its start (an exit point, F16); any other failure takes
# the next planner margin.


class Optimizer_Error_Cls(ValueError):
    """
    Description:
        The optimized program cannot be made: a FREE transfer that cannot be planned or does not pass the exact check at any
        margin, a stale optimized file. The message names the transfer or the input and what to do.
    """


def Planning_Model_Hash(model: mujoco.MjModel) -> str:
    """
    Description:
        The sha256 of a compiled model's binary form (mj_saveModel): the planning model an optimized program was checked on.

    Args:
        (1) model [mujoco.MjModel]: The model.

    Returns:
        (1) parameter [string]: The hex digest.
    """

    buffer = np.zeros(mujoco.mj_sizeModel(model), np.uint8)
    mujoco.mj_saveModel(model, None, buffer)
    return hashlib.sha256(buffer.tobytes()).hexdigest()


def Input_Hashes(model: mujoco.MjModel) -> tp.Dict[str, str]:
    """
    Description:
        The sha256 of every input of an optimized program (CONST_INPUTS, by Input_Hash: a YAML file's of its content) and of
        the planning model.

    Args:
        (1) model [mujoco.MjModel]: The planning model.

    Returns:
        (1) parameter [Dict<string, string>]: Name -> hex digest.
    """

    return dict({name: Input_Hash(path) for name, path in CONST_INPUTS.items()}, planning_model=Planning_Model_Hash(model))


def Platform_Record() -> tp.Dict[str, str]:
    """
    Description:
        The platform and the versions an optimized program was made with (B5: byte-identical on one machine and platform).

    Returns:
        (1) parameter [Dict<string, string>]: The record.
    """

    return {'system': platform.system(), 'machine': platform.machine(), 'python': platform.python_version(), 'numpy': np.__version__,
            'scipy': scipy.__version__, 'mujoco': mujoco.__version__}


def Stale_Inputs(program: Program_Str, model: mujoco.MjModel) -> tp.List[str]:
    """
    Description:
        The inputs that changed since an optimized program was made (their hashes against its Optimized.Inputs).

    Args:
        (1) program [Program_Str(object)]: The optimized program.
        (2) model [mujoco.MjModel]: The planning model now.

    Returns:
        (1) parameter [List<string>]: The changed inputs' names; empty when it is current.
    """

    if program.Optimized is None:
        return []
    now = Input_Hashes(model)
    return [name for name in now if program.Optimized.Inputs.get(name) != now[name]]


@dataclass
class Free_Transfer_Str:
    """
    Description:
        One FREE transfer: consecutive FREE segments of the taught program with only loader jobs between them, replaced by
        one planned spline; the jobs between them run in parallel with it.
    """

    # Its index (the seed's offset), its name (the last FREE segment's) and the taught segments it replaces: the FREE ones
    # and the loader-only ones between them.
    #   Unit [int], [string], [List<string>], [List<string>]
    Index: int = 0
    Name: str = ''
    Keys: tp.List[str] = field(default_factory=list)
    Job_Keys: tp.List[str] = field(default_factory=list)
    # The points it starts and ends at.
    #   Unit [string]
    Start: str = ''
    End: str = ''
    # The loader jobs that run in parallel, in order.
    #   Unit [List<string>]
    Jobs: tp.List[str] = field(default_factory=list)
    # Whether the cups carry the PCB; the default trajectory's first sample of the transfer.
    #   Unit [bool], [int]
    Carrying: bool = False
    First_Sample: int = 0
    # Where the robot waits while the parallel jobs run (Optimization.yaml Waiting_Points), and where it goes before the first
    # of them starts (Exit_Points, F16), or None.
    #   Unit [string], [string]
    Wait: tp.Optional[str] = None
    Exit: tp.Optional[str] = None


def Free_Transfers(program: Program_Str, optimization: Optimization_Str, default: Trajectory_Str) -> tp.List[Free_Transfer_Str]:
    """
    Description:
        The FREE transfers of the taught program: its FREE segments (Optimization.yaml), consecutive ones merged across
        segments that only run loader jobs (Loader.Run lines), each from the point the robot stands at before its first
        segment to its last segment's last point; whether the cups carry the PCB (the default trajectory at its start); its
        waiting point and its exit point (Optimization.yaml Waiting_Points, Exit_Points).

    Args:
        (1) program [Program_Str(object)]: The taught program.
        (2) optimization [Optimization_Str(object)]: The FREE segments.
        (3) default [Trajectory_Str(object)]: The taught program's trajectory.

    Returns:
        (1) parameter [List<Free_Transfer_Str>]: The transfers, in order.
    """

    here, transfers, current, pending = program.Simulation.Start_Robot, [], None, []
    for index, segment in enumerate(program.Segments):
        if segment.Key in optimization.Free_Segments:
            if current is None:
                first = int(np.flatnonzero(default.Step_Index == index)[0])
                current = Free_Transfer_Str(Index=len(transfers), Start=here, First_Sample=first,
                                            Carrying=CONST_PCB_LOCATIONS[default.PCB_Location[first]] == 'cups')
                transfers.append(current)
            else:
                current.Job_Keys += [key for key, _ in pending]
                current.Jobs += [job for _, jobs in pending for job in jobs]
            pending = []
            current.Keys.append(segment.Key)
            current.Name, current.End = segment.Name, segment.Commands[-1].Target
            here = current.End
        elif current is not None and all(c.Kind == 'Run' and c.Station == 'Loader' for c in segment.Commands):
            pending.append((segment.Key, [c.Target for c in segment.Commands]))
        else:
            current, pending = None, []
            for command in segment.Commands:
                if command.Kind in ('MoveJ', 'MoveL'):
                    here = command.Target
                elif command.Kind == 'MovePath':
                    here = command.Path_End
    for transfer in transfers:
        transfer.Wait = optimization.Waiting_Points.get(transfer.Name)
        transfer.Exit = optimization.Exit_Points.get(transfer.Name)
    return transfers


class Free_Space_Cls(object):
    """
    Description:
        A FREE transfer's configuration validity on the planning model: the scene as the taught program has it at the
        transfer's start (loader, jaws, PCB, MG06); the robot's geoms - and the carried PCB's, hung at the cup face - at
        least the clearance plus a margin from every other collidable geom (exact distances, Collision.Close_Pairs; the
        cups on the PCB they carry and the robot's mount left out) and at least the loader clearance plus the margin from the
        loader and the PCB in its jaws (F11: Clearance.Loader_Free + Tracking_Margin); the carried PCB tilted at most
        Carry_Max_Tilt (B4). The loader moves during a transfer with parallel jobs: the robot-loader distance over time is
        the exact check's.

    Initialization of the Class:
        Args:
            (1) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
            (2) collision [Robot_Collision_Cls(object)]: On the same model and data.
            (3) default [Trajectory_Str(object)]: The taught program's trajectory (the scene).
            (4) transfer [Free_Transfer_Str(object)]: The transfer.
            (5) clearance [float]: Cell.yaml Clearance.Robot (m).
            (6) max_tilt [float]: The carried PCB's largest tilt (rad).
            (7) loader [float]: The clearance to the loader (m; Loader_Free + Tracking_Margin), 0 for clearance only.

        Example:
            Initialization:
                # Assignment of the variables.
                space = Free_Space_Cls(kinematics, collision, default, transfer, cell.Clearance, optimization.Carry_Max_Tilt,
                                       cell.Loader_Free + cell.Tracking_Margin)

            Features:
                # Functions of the class.
                space.Valid(q, margin), space.Tilt(q), space.Distance(q)
    """

    def __init__(self, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, default: Trajectory_Str, transfer: Free_Transfer_Str,
                 clearance: float, max_tilt: float, loader: float = 0.0) -> None:
        self.Kinematics, self.Collision, self.Transfer = kinematics, collision, transfer
        self.Clearance, self.Max_Tilt, self.Loader = clearance, max_tilt, max(loader, clearance)
        # The loader's bodies and the PCB when it is in the jaws in the transfer's scene (F11).
        in_jaws = CONST_PCB_LOCATIONS[default.PCB_Location[transfer.First_Sample]] == 'jaws'
        self.__loader = set(CONST_LOADER_BODIES) | ({'pcb'} if in_jaws else set())
        self.Calls = 0
        model = kinematics.Model
        self.__pcb, self.__robot = model.body('pcb').id, collision.Robot_Bodies
        self.__pcb_address = kinematics.Scene_Address['pcb_free']
        Pose_Sample(kinematics, default, transfer.First_Sample)
        self.__scene = kinematics.Data.qpos.copy()

    def Pose(self, q: np.ndarray) -> None:
        """
        Description:
            Pose the data at q in the transfer's scene, the carried PCB hung at the cup face.

        Args:
            (1) q [Vector<float> 1x6]: The joints (rad).
        """

        d = self.Kinematics.Data
        d.qpos[:] = self.__scene
        d.qpos[self.Kinematics.Q_Address] = q
        if self.Transfer.Carrying:
            p, R = self.Kinematics.Site_Pose(q)
            d.qpos[self.__pcb_address:self.__pcb_address + 7] = np.r_[PCB_For_TCP(p, R)]
        mujoco.mj_kinematics(self.Kinematics.Model, d)

    def Tilt(self, q: np.ndarray) -> float:
        """
        Description:
            The carried PCB's tilt from horizontal at q (the angle of its normal from the vertical).

        Args:
            (1) q [Vector<float> 1x6]: The joints (rad).

        Returns:
            (1) parameter [float]: The tilt (rad); 0 when the cups carry nothing.
        """

        if not self.Transfer.Carrying:
            return 0.0
        _, quaternion = PCB_For_TCP(*self.Kinematics.Site_Pose(q))
        w, x, y, z = quaternion
        return math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y))))

    def Pairs(self, q: np.ndarray, cut: float) -> tp.List[tp.Tuple[float, str, str, str, str]]:
        """
        Description:
            The pairs closer than cut at q (exact): the robot's geoms to everything but the PCB it carries, the carried PCB
            to everything but the robot; the robot's mount left out.

        Args:
            (1) q [Vector<float> 1x6]: The joints (rad).
            (2) cut [float]: The distance bound (m).

        Returns:
            (1) parameter [List<Tuple>]: (distance, body, other body, geom, other geom), closest first.
        """

        self.Pose(q)
        d, carrying = self.Kinematics.Data, self.Transfer.Carrying
        pairs = self.Collision.Close_Pairs(self.__robot, cut, d, exclude={self.__pcb} if carrying else None)
        if carrying:
            pairs += self.Collision.Close_Pairs({self.__pcb}, cut, d, exclude=self.__robot)
        return sorted(p for p in pairs if not (p[1] == CONST_MOUNT_BODY and p[4].startswith(CONST_MOUNT_GEOM_PREFIX)))

    def Valid(self, q: np.ndarray, margin: float) -> bool:
        """
        Description:
            Whether q keeps the clearance plus margin, the loader clearance plus margin from the loader, and the carried PCB's tilt
            limit.

        Args:
            (1) q [Vector<float> 1x6]: The joints (rad).
            (2) margin [float]: The margin beyond the clearance (m).

        Returns:
            (1) parameter [bool]: True when valid.
        """

        self.Calls += 1
        if self.Tilt(q) > self.Max_Tilt:
            return False
        return not [p for p in self.Pairs(q, self.Loader + margin) if p[0] < self.Clearance + margin or p[2] in self.__loader]


@dataclass
class Planned_Transfer_Str:
    """
    Description:
        A planned leg of a FREE transfer (the whole transfer, or its legs to an exit point, to a waiting point and on to its
        end): its spline's waypoints and how they were found.
    """

    # The transfer and the planner margin it was planned with.
    #   Unit [Free_Transfer_Str(object)], [float] (m)
    Transfer: Free_Transfer_Str = field(default_factory=Free_Transfer_Str)
    Margin: float = 0.0
    # The leg (from 0), its end points, its waypoint set's name (W_<TRANSFER>; the leg to an exit point W_<TRANSFER>_EXIT, to a
    # waiting point W_<TRANSFER>_WAIT) and its role: 'exit' (before the first parallel job starts), 'wait' (right after it
    # starts) or 'end' (in its slot).
    #   Unit [int], [string], [string], [string], [string]
    Leg: int = 0
    Start: str = ''
    End: str = ''
    Set: str = ''
    Role: str = 'end'
    # RRT-Connect's iterations and nodes; the path's vertices after RRT, after the shortcut and the prune; the refits.
    #   Unit [int]
    Iterations: int = 0
    Nodes: int = 0
    Rrt_Points: int = 0
    Short_Points: int = 0
    Refits: int = 0
    # The spline's waypoints, the ends included.
    #   Unit [Matrix<float> nx6] (rad)
    Waypoints: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    # The time-optimal and the sampled duration, the peaks (fractions of the limits), the validity checks made.
    #   Unit [float] (s), [float] (s), [float] (-), [float] (-), [int]
    Optimal_Time: float = 0.0
    Duration: float = 0.0
    Peak_Speed: float = 0.0
    Peak_Acceleration: float = 0.0
    Checks: int = 0


def Plan_Transfer(space: Free_Space_Cls, start: np.ndarray, end: np.ndarray, optimization: Optimization_Str, cell: Cell_Configuration_Str,
                  margin: float, lower: np.ndarray, upper: np.ndarray, leg: int = 0, names: tp.Optional[tp.Tuple[str, str]] = None,
                  set_name: str = '', role: str = 'end') -> Planned_Transfer_Str:
    """
    Description:
        One FREE transfer's leg planned: RRT-Connect (seed Optimization.Seed + the transfer's index + 1000 x leg) and the
        shortcut with the
        clearance plus margin; the prune; the spline checked against the clearance plus Spline.Margin and refitted; its
        time-optimal timing within Speed_Fraction of the datasheet speeds and the acceleration limits (the duration only:
        the build times it again from the written waypoints).

    Args:
        (1) space [Free_Space_Cls(object)]: The transfer's validity.
        (2 - 3) start, end [Vector<float> 1x6]: The end points' joints (rad).
        (4) optimization [Optimization_Str(object)]: The settings.
        (5) cell [Cell_Configuration_Str(object)]: The limits.
        (6) margin [float]: The planner margin (m).
        (7 - 8) lower, upper [Vector<float> 1x6]: The joint limits (rad).
        (9) leg [int]: The leg (0: the whole transfer or its leg to the waiting point).
        (10) names [Tuple<string, string>]: The leg's end points (default: the transfer's).
        (11) set_name [string]: Its waypoint set's name (default W_<TRANSFER>).
        (12) role [string]: 'exit', 'wait' or 'end' (Planned_Transfer_Str.Role).

    Returns:
        (1) parameter [Planned_Transfer_Str(object)]: The planned leg.
    """

    o, transfer = optimization, space.Transfer
    names = names or (transfer.Start, transfer.End)
    for name, q in zip(names, (start, end)):
        if not space.Valid(q, margin):
            raise Planner_Error_Cls('%s is within the clearance plus the planner margin %.1f mm (or tilts the PCB beyond the limit)'
                                    % (name, 1000 * margin))
    rng = np.random.default_rng(o.Seed + transfer.Index + 1000 * leg)

    def Plan_Valid(q: np.ndarray) -> bool:
        """
        Description:
            The planner's validity: the clearance plus the planner margin.

        Args:
            (1) q [Vector<float> 1x6]: The joints (rad).

        Returns:
            (1) parameter [bool]: True when valid.
        """

        return space.Valid(q, margin)

    def Spline_Valid(q: np.ndarray) -> bool:
        """
        Description:
            The spline's validity: the clearance plus Spline.Margin.

        Args:
            (1) q [Vector<float> 1x6]: The joints (rad).

        Returns:
            (1) parameter [bool]: True when valid.
        """

        return space.Valid(q, o.Spline_Margin)

    space.Calls = 0
    rrt = Rrt_Connect(start, end, Plan_Valid, lower, upper, rng, o.Rrt_Max_Iterations, o.Rrt_Extend_Step, o.Rrt_Edge_Step, o.Rrt_Goal_Bias)
    short = Prune(Shortcut(rrt.Path, Plan_Valid, rng, o.Shortcut_Iterations, o.Rrt_Edge_Step), Plan_Valid, o.Rrt_Edge_Step)
    smooth = Smooth(short, Spline_Valid, o.Spline_Check_Step, o.Spline_Max_Refits)
    timed = Time_Path(smooth.Path, o.Speed_Fraction * np.array(cell.Datasheet_Speeds), np.array(cell.Acceleration_Limits), o.Topp_Grid_Points,
                      CONST_DT)
    return Planned_Transfer_Str(Transfer=transfer, Margin=margin, Leg=leg, Start=names[0], End=names[1],
                                Set=set_name or 'W_%s' % transfer.Name.upper(), Role=role, Iterations=rrt.Iterations, Nodes=rrt.Nodes,
                                Rrt_Points=len(rrt.Path),
                                Short_Points=len(short), Refits=smooth.Refits, Waypoints=smooth.Waypoints, Optimal_Time=timed.Optimal_Time,
                                Duration=timed.Duration, Peak_Speed=timed.Peak_Speed, Peak_Acceleration=timed.Peak_Acceleration,
                                Checks=space.Calls)


def _Blocks(path: str) -> tp.Tuple[tp.Dict[str, tp.List[str]], tp.List[str]]:
    """
    Description:
        The taught program's text: each segment's block (its comment lines, its key line, its command lines) and the jobs'
        text (from the comment above Loader_Jobs to the end), verbatim.

    Args:
        (1) path [string]: Configuration/Program.yaml.

    Returns:
        (1) parameter [Dict<string, List<string>>]: Segment key -> its lines (key line first after the comments).
        (2) parameter [List<string>]: The jobs' lines.
    """

    lines = open(path, encoding='utf-8').read().split('\n')
    jobs_at = next(i for i, line in enumerate(lines) if line.startswith('Loader_Jobs:'))
    while jobs_at > 0 and lines[jobs_at - 1].startswith('#'):
        jobs_at -= 1
    blocks, current = {}, []
    for line in lines[:jobs_at]:
        stripped = line.strip()
        if line.startswith('  #'):
            current.append(line)
        elif line.startswith('  S') and stripped.endswith(':'):
            key = stripped[:-1]
            blocks[key] = current + [line]
            current = []
        elif line.startswith('    - ') and blocks:
            blocks[list(blocks)[-1]].append(line)
        elif not stripped:
            current = []
    return blocks, [line for line in lines[jobs_at:]]


def _Degrees(q: np.ndarray) -> str:
    """
    Description:
        A joint vector as text in degrees, 9 decimals.

    Args:
        (1) q [Vector<float> 1x6]: The joints (rad).

    Returns:
        (1) parameter [string]: '[a, b, c, d, e, f]'.
    """

    return '[%s]' % ', '.join('%.9f' % math.degrees(v) for v in q)


def Optimized_Text(program: Program_Str, planned: tp.List[tp.List[Planned_Transfer_Str]], optimization: Optimization_Str,
                   slots: tp.Dict[str, int],
                   inputs: tp.Dict[str, str], record: tp.Dict[str, str], schedule: tp.Dict[str, float]) -> str:
    """
    Description:
        The optimized program as text (Data/Optimized/Program_Optimized.yaml, the Program.yaml format): every segment of
        the taught program in order, renumbered - a LOCKED segment copied verbatim, a FREE transfer as one segment named
        after its last FREE segment: a leg to its exit point (W_<NAME>_EXIT) first (F16), then its parallel jobs in order, each
        started when the one before is done (Loader.WaitDone, Loader.Start), a leg to its waiting point (W_<NAME>_WAIT) right
        after the first job's start, its last leg
        Robot.MoveSpline(W_<NAME>, v=...) right after the start of its slot's job - so after the WaitDone of the job before it,
        never after a Wait (F15) - the last job waited for at the end (the interlocks I1-I7, I9 as program order);
        the jobs copied verbatim; the Waypoints; the Optimized record (inputs, platform, schedule, TOPP grid).

    Args:
        (1) program [Program_Str(object)]: The taught program.
        (2) planned [List<List<Planned_Transfer_Str>>]: The planned transfers, each its legs in order.
        (3) optimization [Optimization_Str(object)]: The settings.
        (4) slots [Dict<string, int>]: A transfer's spline slot (it runs after Loader.Start of its slot-th parallel job, from 1),
            by name.
        (5) inputs [Dict<string, string>]: The inputs' hashes.
        (6) record [Dict<string, string>]: The platform.
        (7) schedule [Dict<string, float>]: Each segment's start time, by its new key ({} before the first build).

    Returns:
        (1) parameter [string]: The file's text.
    """

    blocks, jobs = _Blocks(program.Path)
    by_first = {legs[0].Transfer.Keys[0]: legs for legs in planned}
    merged = {key for legs in planned for key in legs[0].Transfer.Keys[1:] + legs[0].Transfer.Job_Keys}
    out = ['## =========================================================================== ##',
           '#   Description:',
           '#     The OPTIMIZED program of the Test-it-off cell (FEATURES.md F9), WRITTEN by Example/Program/optimize_program.py',
           '#     from Configuration/Program.yaml and Configuration/Optimization.yaml - never edit it by hand: change those and',
           '#     run the optimizer again. Read by src/Configuration/Program_Parser.py (Load_Program), the same format as',
           '#     Program.yaml plus Robot.MoveSpline, Loader.Start / Loader.WaitDone, Waypoints and the Optimized record.',
           '#     LOCKED segments are copied verbatim from Program.yaml (renumbered); each FREE transfer is a planned joint',
           '#     spline (RRT-Connect from a fixed seed, shortcut, spline, time-optimal timing) with the loader jobs between',
           '#     its taught segments running in parallel. A build refuses this file when an input changed (Optimized.Inputs).',
           '#     Simulation results only: NOT validated for driving the real cell (KNOWN_ISSUES.md: MoveSpline on the',
           '#     controller).',
           '#   Units: Waypoints [deg], v= the fraction of the datasheet joint speeds, Schedule [s].',
           '## =========================================================================== ##', '',
           '# The robot\'s program: the whole cycle.', 'Program:']
    number, keys = 0, []
    for segment in program.Segments:
        if segment.Key in merged:
            continue
        number += 1
        key = 'S%d_%s' % (number, segment.Name) if segment.Key not in by_first else 'S%d_%s' % (number, by_first[segment.Key][0].Transfer.Name)
        keys.append(key)
        if segment.Key not in by_first:
            block = blocks[segment.Key]
            out += [line.replace('  %s:' % segment.Key, '  %s:' % key, 1) if line.strip() == segment.Key + ':' else line for line in block]
            out.append('')
            continue
        legs = by_first[segment.Key]
        t = legs[0].Transfer
        order = [s.Key for s in program.Segments if s.Key in t.Keys + t.Job_Keys]
        parallel = ' The loader runs %s in parallel.' % ', '.join(t.Jobs) if t.Jobs else ''
        parallel += ' The robot waits at %s meanwhile.' % t.Wait if t.Wait else ''
        parallel += ' The robot goes to %s before %s starts.' % (t.Exit, t.Jobs[0]) if t.Exit and t.Jobs else ''
        comment = 'FREE (planned, F9): %s to %s, replacing the taught %s.%s' % (t.Start, t.End, ', '.join(order), parallel)
        out += textwrap.wrap(comment, width=118, initial_indent='  # ', subsequent_indent='  # ')
        out.append('  %s:' % key)
        slot = slots.get(t.Name, 1)
        move = lambda role: ['Robot.MoveSpline(%s, v=%g)' % (leg.Set, optimization.Speed_Fraction) for leg in legs if leg.Role == role]
        exits, early, spline = move('exit'), move('wait'), move('end')
        lines = exits + ([] if t.Jobs else early + spline)
        for i, job in enumerate(t.Jobs, start=1):
            lines += (['Loader.WaitDone(%s)' % t.Jobs[i - 2]] if i > 1 else []) + ['Loader.Start(%s)' % job] + (early if i == 1 else []) \
                + (spline if i == slot else [])
        lines += ['Loader.WaitDone(%s)' % t.Jobs[-1]] if t.Jobs else []
        out += ['    - %s' % line for line in lines]
        out.append('')
    out += jobs if jobs[-1] == '' else jobs + ['']
    out += ['# The planned splines: From and To the end points, Points the interior waypoints [deg] (the planner\'s output).',
            'Waypoints:']
    for p in (leg for legs in planned for leg in legs):
        out.append('  %s:' % p.Set)
        out.append('    From: %s' % p.Start)
        out.append('    To: %s' % p.End)
        interior = p.Waypoints[1:-1]
        out.append('    Points:%s' % ('' if len(interior) else ' []'))
        out += ['      - %s' % _Degrees(q) for q in interior]
    out += ['', '# The record of this file: the sha256 of its inputs (a build refuses it when one changed), the platform and versions it',
            '# was made with, each segment\'s start time (compared at a build), the TOPP grid.', 'Optimized:', '  Inputs:']
    out += ['    %s: "%s"' % (name, value) for name, value in inputs.items()]
    out.append('  Platform:')
    out += ['    %s: "%s"' % (name, value) for name, value in record.items()]
    out.append('  Schedule:%s' % ('' if schedule else ' {}'))
    out += ['    %s: %.3f' % (key, schedule[key]) for key in keys if key in schedule]
    out.append('  Topp_Grid_Points: %d' % optimization.Topp_Grid_Points)
    return '\n'.join(out) + '\n'


def Schedule(trajectory: Trajectory_Str) -> tp.Dict[str, float]:
    """
    Description:
        Each segment's start time in a built trajectory (End_Settle left out).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.

    Returns:
        (1) parameter [Dict<string, float>]: Segment key -> start (s).
    """

    return {name: round(float(trajectory.T[np.flatnonzero(trajectory.Step_Index == k)[0]]), 3)
            for k, name in enumerate(trajectory.Step_Names) if name.startswith('S')}


@dataclass
class Optimization_Result_Str:
    """
    Description:
        The optimizer's result: the optimized program's text and what it was made and checked with.
    """

    # The file's text and the program loaded from it.
    #   Unit [string], [Program_Str(object)]
    Text: str = ''
    Program: tp.Optional[Program_Str] = None
    # The planned transfers; each spline's slot among the transfer's parallel jobs (from 1; I8 and the gating rule), by name.
    #   Unit [List<Planned_Transfer_Str>], [Dict<string, int>] (-)
    Planned: tp.List[Planned_Transfer_Str] = field(default_factory=list)
    Slots: tp.Dict[str, int] = field(default_factory=dict)
    # Its trajectory, the build's notes and the check.
    #   Unit [Trajectory_Str(object)], [List<Build_Note_Str>], [Program_Check_Str(object)]
    Trajectory: tp.Optional[Trajectory_Str] = None
    Notes: tp.List[Build_Note_Str] = field(default_factory=list)
    Check: tp.Optional[Program_Check_Str] = None
    # The attempts (margins and slots tried), as lines.
    #   Unit [List<string>]
    Attempts: tp.List[str] = field(default_factory=list)


def Optimize_Program(program: Program_Str, optimization: Optimization_Str, cell: Cell_Configuration_Str, pcb_meta: PCB_Meta_Str,
                     kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, default: Trajectory_Str, path: str,
                     progress: tp.Optional[tp.Callable[[str], None]] = None) -> Optimization_Result_Str:
    """
    Description:
        The optimized program of a taught one: each FREE transfer planned at the first planner margin it can be planned with
        (Plan_Transfer); the program written to path (Optimized_Text), loaded, built and checked (the F8 check, every sample
        on the planning model, the loader moving in parallel) and its gating rule (Gating_Faults: neither the robot nor a job
        comes where the other has been while the job may run); a transfer whose check fails on a pair with the loader (I8), or
        whose spline comes where a running job has been, runs its spline after the next job's start (Gate: after the WaitDone
        of the job it met), any other failing transfer is planned again with the next margin; then the schedule is written and
        the file loaded and built once more. Fails, naming the transfer, when the margins or the slots run out.

    Args:
        (1) program [Program_Str(object)]: The taught program.
        (2) optimization [Optimization_Str(object)]: The settings.
        (3) cell [Cell_Configuration_Str(object)]: The limits and the clearance.
        (4) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (5) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (6) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (7) default [Trajectory_Str(object)]: The taught program's trajectory (the transfers' scenes).
        (8) path [string]: The optimized program's file (written).
        (9) progress [function]: Called with a line per step (the script prints it), or None.

    Returns:
        (1) parameter [Optimization_Result_Str(object)]: The result.
    """

    lower, upper = kinematics.Limits
    progress = progress or (lambda line: None)
    transfers = Free_Transfers(program, optimization, default)
    result = Optimization_Result_Str(Slots={t.Name: 1 for t in transfers})
    level = {t.Name: 0 for t in transfers}

    def Plan(transfer: Free_Transfer_Str) -> tp.List[Planned_Transfer_Str]:
        """
        Description:
            Plan a transfer's legs (to its waiting point and on, or the whole) at its current margin level or the next ones
            that work.

        Args:
            (1) transfer [Free_Transfer_Str(object)]: The transfer.

        Returns:
            (1) parameter [List<Planned_Transfer_Str>]: Its legs.
        """

        space = Free_Space_Cls(kinematics, collision, default, transfer, cell.Clearance, optimization.Carry_Max_Tilt,
                               cell.Loader_Free + cell.Tracking_Margin)
        points = [transfer.Start] + ([transfer.Exit] if transfer.Exit else []) + ([transfer.Wait] if transfer.Wait else []) + [transfer.End]
        roles = (['exit'] if transfer.Exit else []) + (['wait'] if transfer.Wait else []) + ['end']
        sets = ['W_%s' % transfer.Name.upper() + {'exit': '_EXIT', 'wait': '_WAIT', 'end': ''}[role] for role in roles]
        while level[transfer.Name] < len(optimization.Margins):
            margin = optimization.Margins[level[transfer.Name]]
            try:
                legs = []
                for leg, (a, b) in enumerate(zip(points[:-1], points[1:])):
                    legs.append(Plan_Transfer(space, np.array(program.Constants.Points[a].Joints), np.array(program.Constants.Points[b].Joints),
                                              optimization, cell, margin, lower, upper, leg, (a, b), sets[leg], roles[leg]))
                    progress('planned %s (%s to %s) at margin %.1f mm: %d RRT iterations, %d waypoints, %.3f s'
                             % (sets[leg], a, b, 1000 * margin, legs[-1].Iterations, len(legs[-1].Waypoints), legs[-1].Duration))
                return legs
            except Planner_Error_Cls as error:
                result.Attempts.append('%s: margin %.1f mm: %s' % (transfer.Name, 1000 * margin, error))
                level[transfer.Name] += 1
        raise Optimizer_Error_Cls('The FREE transfer %s (%s to %s) cannot be planned at any margin of Optimization.yaml Margins: %s'
                                  % (transfer.Name, transfer.Start, transfer.End,
                                     '; '.join(a for a in result.Attempts if a.startswith(transfer.Name))))

    planned = {t.Name: Plan(t) for t in transfers}
    inputs, record = Input_Hashes(kinematics.Model), Platform_Record()
    loader = set(CONST_LOADER_BODIES) | {'pcb'}
    for _ in range(sum(len(t.Jobs) + len(optimization.Margins) for t in transfers) + 1):
        text = Optimized_Text(program, list(planned.values()), optimization, result.Slots, inputs, record, {})
        with open(path, 'w', encoding='utf-8') as handle:
            handle.write(text)
        optimized = Load_Program(program.Constants, program.Simulation, cell, path)
        trajectory, notes = Build_Program(optimized, cell, pcb_meta, kinematics, {})
        check = Check_Trajectory(trajectory, optimized, notes, kinematics, collision, cell, pcb_meta, Free_Steps(optimized))
        failing = [s for s in check.Steps if s.Failures and s.Name.split('_', 1)[1] in planned]
        progress('built and checked: cycle %.3f s, %s; %s' % (trajectory.T[-1], check.Verdict, '; '.join('%s: %s' % (s.Name, f) for s in check.Steps
                                                                                                      for f in s.Failures) or 'no failure'))
        gated = {} if failing else {f.Step: f for f in Gating_Faults(trajectory, kinematics, collision, cell)}
        for name_key, fault in gated.items():
            name = name_key.split('_', 1)[1]
            result.Slots[name] = Gate(trajectory, kinematics, collision, cell, name_key, planned[name][0].Transfer, result.Slots[name], fault)
            result.Attempts.append('%s: %s comes where %s has been (%.2f mm, %s; the gating rule): the spline after the start of %s'
                                   % (name, fault.Motion, fault.Job, 1000 * fault.Distance, fault.Pair,
                                      planned[name][0].Transfer.Jobs[result.Slots[name] - 1]))
        if not failing and not gated:
            break
        for step in failing:
            name = step.Name.split('_', 1)[1]
            pair = step.Closest.Pair if step.Closest is not None else ''
            near = step.Closest is not None and step.Closest.Distance < cell.Clearance and any(b in loader for b in pair.split(' / '))
            # F11: the free transfer too close to the loader where its actuators put it.
            if step.Loader is not None and step.Loader.Distance < cell.Loader_Free + cell.Tracking_Margin:
                near, pair, step.Closest = True, step.Loader.Pair, step.Loader
            if near and planned[name][0].Transfer.Jobs:
                result.Slots[name] = Gate(trajectory, kinematics, collision, cell, step.Name, planned[name][0].Transfer, result.Slots[name])
                result.Attempts.append('%s: %s %.2f mm at %.3f s (I8): the spline after the start of %s' % (
                    name, pair, 1000 * step.Closest.Distance, step.Closest.Time, planned[name][0].Transfer.Jobs[result.Slots[name] - 1]))
            else:
                result.Attempts.append('%s: margin %.1f mm fails the exact check: %s'
                                       % (name, 1000 * planned[name][0].Margin, '; '.join(step.Failures)))
                level[name] += 1
                if level[name] >= len(optimization.Margins):
                    raise Optimizer_Error_Cls('The FREE transfer %s fails the exact check at every margin of Optimization.yaml Margins: %s'
                                              % (name, '; '.join(step.Failures)))
                planned[name] = Plan(planned[name][0].Transfer)
    else:
        raise Optimizer_Error_Cls('The optimized program does not pass its check after every margin and slot was tried.')
    text = Optimized_Text(program, list(planned.values()), optimization, result.Slots, inputs, record, Schedule(trajectory))
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(text)
    result.Program = Load_Program(program.Constants, program.Simulation, cell, path)
    result.Trajectory, result.Notes = Build_Program(result.Program, cell, pcb_meta, kinematics, {})
    result.Check = Check_Trajectory(result.Trajectory, result.Program, result.Notes, kinematics, collision, cell, pcb_meta,
                                    Free_Steps(result.Program))
    result.Text, result.Planned = text, [leg for legs in planned.values() for leg in legs]
    return result


def Gate(trajectory: Trajectory_Str, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, cell: Cell_Configuration_Str,
         step: str, transfer: Free_Transfer_Str, slot: int, fault: tp.Optional[tp.Any] = None) -> int:
    """
    Description:
        The spline's next slot when it meets the loader (I8: every sample of the step measured, Robot_Loader_Distance, the loader
        where its actuators put it) or comes where a job it does not wait for has been (fault 'robot enters', the gating rule):
        the spline runs after the next job's start, so after the WaitDone of the job it met - never after a Wait, whose
        separation rests on the loader's timing (F15). A meeting before the spline starts - the robot waiting at its start
        point - and a job that comes where the robot has been (fault 'loader enters': the robot must leave before the job
        starts, F16) cannot be solved by a later slot, and a spline already after the last job's start has none: an
        Optimizer_Error_Cls names each with its remedy.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The optimized program's trajectory.
        (2) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (3) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (4) cell [Cell_Configuration_Str(object)]: The clearance.
        (5) step [string]: The transfer's segment key.
        (6) transfer [Free_Transfer_Str(object)]: The transfer (its jobs).
        (7) slot [int]: The spline's slot now.
        (8) fault [Gating_Fault_Str(object)]: The gating rule's fault that moves it, or None for I8.

    Returns:
        (1) parameter [int]: The new slot.
    """

    index = trajectory.Step_Names.index(step)
    # The transfer's last spline (the legs to an exit point and to a waiting point come before it).
    spline = [k for k, text in enumerate(trajectory.Segment_Command)
              if text.startswith('Robot.MoveSpline') and trajectory.Segment_Step[k] == index][-1]
    if fault is not None and fault.Kind == 'loader enters':
        raise Optimizer_Error_Cls('In %s %s comes where the robot has been (%s, %.2f mm; the robot there %.3f-%.3f s, %.2f s ahead): the robot '
                                  'leaves its way only by timing. Declare an exit point for %s in Optimization.yaml Exit_Points (a point out '
                                  'of the job\'s volume the robot goes to before the job starts).'
                                  % (step, fault.Job, fault.Pair, 1000 * fault.Distance, fault.Robot_Span[0], fault.Robot_Span[1], fault.Lead,
                                     transfer.Name))
    if fault is not None and fault.Robot_Span[0] < trajectory.T[int(trajectory.Segment_Start[spline])]:
        raise Optimizer_Error_Cls('In %s the robot comes where %s has been at %.3f s, before its last spline starts (%s): a later slot cannot help; '
                                  'the transfer needs another waiting or exit point.' % (step, fault.Job, fault.Robot_Span[0], fault.Motion))
    if fault is None:
        samples = np.flatnonzero(trajectory.Step_Index == index)
        actual, need = Loader_Actual_Trajectory(trajectory, kinematics.Model), cell.Loader_Free + cell.Tracking_Margin
        meeting = [int(i) for i in samples if Robot_Loader_Distance(actual, kinematics, collision, [int(i)])[0] < need]
        if not meeting:
            return slot
        if meeting[0] < int(trajectory.Segment_Start[spline]):
            raise Optimizer_Error_Cls('In %s the robot meets the loader at %.3f s while it waits at the start of its spline: gating cannot help '
                                      '(the loader comes to the robot); the transfer needs another waiting point.' % (step, trajectory.T[meeting[0]]))
    if slot >= len(transfer.Jobs):
        raise Optimizer_Error_Cls('In %s the spline after the start of the last parallel job %s still meets the loader or its volume: split the '
                                  'job so the robot can wait for the part it crosses (Loader.WaitDone), or gate it on an input from the loader.'
                                  % (step, transfer.Jobs[-1]))
    return slot + 1
