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
# File Name: ../Optimizer/Measure.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.:
#   ../Configuration/Core - the cell
from Configuration.Core import Cell_Configuration_Str
#   ../Configuration/Program_Parser - the program, the PCB locations
from Configuration.Program_Parser import CONST_PCB_LOCATIONS, Program_Str
#   ../Kinematics/Core - FK, the PCB held by the cups
from Kinematics.Core import Robot_Kinematics_Cls
#   ../Collision/Core - exact distances
from Collision.Core import Robot_Collision_Cls
#   ../Program/Check - the build's check, posing at a sample, the loader's actual position and its distance to the robot
from Program.Check import Robot_Loader_Distance, Loader_Actual_Trajectory, CONST_LOADER_CUT
#   ../Trajectory/Core - the trajectory format
from Trajectory.Core import Trajectory_Str, CONST_DT

#   ../Optimizer/Core - the free transfers, the schedule
from Optimizer.Core import Free_Transfer_Str, Schedule

"""
Description:
    Initialization of constants.

    What the optimized program is measured by (SPEC §7.11): each LOCKED part against the taught program byte by byte, the
    interlocks I1-I8 on the built trajectory, the carried PCB's tilt, the schedule's differences, and the comparison's
    metrics. It prints nothing.
"""
# A stop of the robot (the comparison's metric): every joint speed below CONST_STOP_SPEED for at least CONST_STOP_TIME,
# after a motion. The speed is a sample's joint change over the sample time; 1e-3 rad/s is far below any planned motion
# (a min-jerk move reaches it within its first or last few samples) and above the plans' numerical noise.
#   Unit [float] (rad/s), [float] (s)
CONST_STOP_SPEED, CONST_STOP_TIME = 1e-3, 0.1
CONST_ROBOT_CHANNELS = ('Robot_Q', 'Robot_QD', 'Loader_Y', 'Loader_Z', 'Gripper', 'Vacuum', 'PCB_Location', 'PCB_Pose', 'MG06_Lid',
                        'MG06_Handle', 'Jaw')
CONST_LOADER_CHANNELS = ('Loader_Y', 'Loader_Z', 'Gripper', 'Jaw', 'PCB_Location', 'PCB_Pose')


def Schedule_Differences(program: Program_Str, trajectory: Trajectory_Str) -> tp.List[str]:
    """
    Description:
        The segments whose start in a built trajectory differs from the optimized program's recorded Schedule (more than half
        a sample): a build on another platform, or a file edited by hand.

    Args:
        (1) program [Program_Str(object)]: The optimized program.
        (2) trajectory [Trajectory_Str(object)]: Its build.

    Returns:
        (1) parameter [List<string>]: 'key: recorded a s, built b s' per difference; empty when it matches.
    """

    if program.Optimized is None:
        return []
    built = Schedule(trajectory)
    return ['%s: recorded %.3f s, built %.3f s' % (key, value, built.get(key, math.nan)) for key, value in program.Optimized.Schedule.items()
            if not abs(built.get(key, math.inf) - value) < CONST_DT / 2]


@dataclass
class Locked_Comparison_Str:
    """
    Description:
        A LOCKED part compared between the taught and the optimized program, sample by sample from its own start: a robot
        segment on every channel, a loader job on the loader's channels (the robot moves meanwhile).
    """

    # The segment's name or the job, and what was compared.
    #   Unit [string], [string]
    Name: str = ''
    Channels: str = ''
    # The samples, their starts in both, and whether every compared value is the same bytes.
    #   Unit [int], [float] (s), [float] (s), [bool]
    Samples: int = 0
    Start_Default: float = 0.0
    Start_Optimized: float = 0.0
    Identical: bool = False


def Compare_Locked(default: Trajectory_Str, optimized: Trajectory_Str, taught: Program_Str,
                   transfers: tp.List[Free_Transfer_Str]) -> tp.List[Locked_Comparison_Str]:
    """
    Description:
        Every LOCKED part of the taught program against the optimized one, from each part's start (B5's companion: LOCKED is
        byte-identical, only its start time moves): each robot segment that is not FREE nor a loader-only segment of a
        transfer, on every channel (CONST_ROBOT_CHANNELS); each job run in parallel, on the loader's channels
        (CONST_LOADER_CHANNELS) from the sample after its Loader.Start.

    Args:
        (1) default [Trajectory_Str(object)]: The taught program's trajectory.
        (2) optimized [Trajectory_Str(object)]: The optimized one.
        (3) taught [Program_Str(object)]: The taught program.
        (4) transfers [List<Free_Transfer_Str>]: The FREE transfers.

    Returns:
        (1) parameter [List<Locked_Comparison_Str>]: The comparisons, in the taught order.
    """

    replaced = {key for t in transfers for key in t.Keys + t.Job_Keys}
    jobs = {key: t.Jobs[t.Job_Keys.index(key)] for t in transfers for key in t.Job_Keys}
    names = [n.split('_', 1)[1] if n.startswith('S') else n for n in optimized.Step_Names]
    out = []
    for index, name in enumerate(default.Step_Names):
        a = np.flatnonzero(default.Step_Index == index)
        if name in jobs:
            start = next(k for k, text in enumerate(optimized.Segment_Command) if text == 'Loader.Start(%s)' % jobs[name])
            b = np.arange(int(optimized.Segment_Start[start]), int(optimized.Segment_Start[start]) + len(a))
            channels = CONST_LOADER_CHANNELS
        elif name in replaced:
            continue
        else:
            short = name.split('_', 1)[1] if name.startswith('S') else name
            b = np.flatnonzero(optimized.Step_Index == names.index(short))
            channels = CONST_ROBOT_CHANNELS
        same = len(a) == len(b) and all(np.asarray(getattr(default, c))[a].tobytes() == np.asarray(getattr(optimized,
                                                                                                           c))[b].tobytes() for c in channels)
        out.append(Locked_Comparison_Str(Name=jobs.get(name, name), Channels='loader' if channels is CONST_LOADER_CHANNELS else 'all', Samples=len(a),
                                         Start_Default=float(default.T[a[0]]), Start_Optimized=float(optimized.T[b[0]]) if len(b) else math.nan,
                                         Identical=bool(same)))
    return out


def Step_Bounds(trajectory: Trajectory_Str, name: str) -> tp.Tuple[int, int]:
    """
    Description:
        The first and the last sample of the step with this name (S<n>_ prefix ignored).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) name [string]: The name without its number.

    Returns:
        (1 - 2) parameter [int]: First, last.
    """

    index = next(k for k, n in enumerate(trajectory.Step_Names) if n.split('_', 1)[-1] == name or n == name)
    samples = np.flatnonzero(trajectory.Step_Index == index)
    return int(samples[0]), int(samples[-1])


def Job_Bounds(trajectory: Trajectory_Str, job: str) -> tp.Tuple[int, int]:
    """
    Description:
        A job's samples in a trajectory: from the first after its start to the last of its loader motion or hold (its
        Loader.WaitDone, or the Run's lines).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) job [string]: The job.

    Returns:
        (1 - 2) parameter [int]: First, last.
    """

    texts = trajectory.Segment_Command
    starts = [k for k, text in enumerate(texts) if text == 'Loader.Start(%s)' % job or text.startswith('Loader.Run(%s) >' % job)]
    first = int(trajectory.Segment_Start[starts[0]])
    done = [k for k, text in enumerate(texts) if text == 'Loader.WaitDone(%s)' % job]
    last = int(trajectory.Segment_End[done[0]]) - 1 if done else int(trajectory.Segment_End[starts[-1]]) - 1
    if done:
        # The loader's last change before the robot's WaitDone ends (the job's last line may end earlier, waiting for the robot).
        y, z, g = trajectory.Loader_Y, trajectory.Loader_Z, trajectory.Gripper
        moving = [i for i in range(first, last + 1) if i > 0 and (y[i] != y[i - 1] or z[i] != z[i - 1] or g[i] != g[i - 1])]
        last = max(moving) if moving else first
    return first, last


def Interlocks(trajectory: Trajectory_Str, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, cell: Cell_Configuration_Str,
               program: Program_Str) -> tp.List[tp.Tuple[str, bool, str]]:
    """
    Description:
        The interlocks I1-I8 of SPEC §7.11 verified on a built trajectory, each with the times that prove it: I1 the
        handover starts with the loader home and the gripper open; I2 the loader's first move after the handover ends; I3
        MG06 is closed only after the park; I4-I5 the test between the leave and the return; I6 the fetch after MG06 is
        open; I7 the take only after the return; I8 the robot (with a carried PCB) at least Clearance.Loader_Free +
        Tracking_Margin from the loader (with the PCB in its jaws, posed where its actuators put it; F11) at every sample of the
        parts where both may move.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (3) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (4) cell [Cell_Configuration_Str(object)]: The clearance.
        (5) program [Program_Str(object)]: Its program (the loader home, the gripper commands).

    Returns:
        (1) parameter [List<Tuple<string, bool, string>>]: (interlock, holds, the evidence).
    """

    tr, T, c = trajectory, trajectory.T, program.Constants
    home, open_jaw = c.Loader['L_HOME'], c.Gripper['G_OPEN'].Jaw
    out = []
    h0, h1 = Step_Bounds(tr, 'Handover')
    # The loader and gripper state just before the handover's first sample.
    i = max(h0 - 1, 0)
    ok = tr.Loader_Y[i] == home.Y and tr.Loader_Z[i] == home.Z and tr.Gripper[i] == open_jaw
    out.append(('I1', bool(ok), 'Handover starts %.3f s with the loader at y %.4f / z %.4f m (L_HOME %.4f / %.4f) and the gripper at %.4f m (G_OPEN)'
                % (T[h0], tr.Loader_Y[i], tr.Loader_Z[i], home.Y, home.Z, tr.Gripper[i])))
    insert = Job_Bounds(tr, 'J_INSERT')
    out.append(('I2', T[insert[0]] > T[h1], 'Handover ends %.3f s; J_INSERT\'s first sample %.3f s' % (T[h1], T[insert[0]])))
    park = Job_Bounds(tr, 'J_PARK')
    c0, _ = Step_Bounds(tr, 'Close_MG06')
    out.append(('I3', T[c0] > T[park[1]], 'J_PARK\'s last loader move %.3f s (parked, gripper open); Close_MG06 starts %.3f s' % (T[park[1]], T[c0])))
    _, l1 = Step_Bounds(tr, 'Leave_MG06')
    t0, t1 = Step_Bounds(tr, 'Test')
    out.append(('I4', T[t0] > T[l1], 'Leave_MG06 ends %.3f s (MG06 closed, fork out of the bar); Test starts %.3f s' % (T[l1], T[t0])))
    r0, _ = Step_Bounds(tr, 'Return_MG06')
    out.append(('I5', T[r0] > T[t1], 'Test ends %.3f s; Return_MG06 starts %.3f s' % (T[t1], T[r0])))
    _, o1 = Step_Bounds(tr, 'Open_MG06')
    fetch = Job_Bounds(tr, 'J_FETCH')
    out.append(('I6', T[fetch[0]] > T[o1], 'Open_MG06 ends %.3f s (MG06 open, fork off the handle); J_FETCH\'s first sample %.3f s' % (T[o1],
                                                                                                                                       T[fetch[0]])))
    ret = Job_Bounds(tr, 'J_RETURN')
    k0, _ = Step_Bounds(tr, 'Take_From_Loader')
    i = k0 - 1
    ok = T[k0] > T[ret[1]] and tr.Loader_Y[i] == home.Y and tr.Loader_Z[i] == home.Z and CONST_PCB_LOCATIONS[tr.PCB_Location[i]] == 'jaws'
    out.append(('I7', bool(ok), 'J_RETURN\'s last loader move %.3f s (loader at L_HOME, PCB %s); Take_From_Loader starts %.3f s'
                % (T[ret[1]], CONST_PCB_LOCATIONS[tr.PCB_Location[i]], T[k0])))
    windows = [(insert[0], park[1]), (fetch[0], ret[1])]
    actual = Loader_Actual_Trajectory(tr, kinematics.Model)
    found = [Robot_Loader_Distance(actual, kinematics, collision, range(a, b + 1)) for a, b in windows]
    distance, sample, pair = min(found)
    closest = '%.2f mm at %.3f s (%s)' % (1000 * distance, T[sample], pair) if sample >= 0 else '> %g mm' % (1000 * CONST_LOADER_CUT)
    need = cell.Loader_Free + cell.Tracking_Margin
    out.append(('I8', distance >= need,
                'robot (and a carried PCB) to loader (and the PCB in its jaws, at its actuator model\'s position) over %s: closest %s; '
                'Loader_Free %.1f mm + Tracking_Margin %.1f mm' % (', '.join('%.3f-%.3f s' % (T[a], T[b]) for a, b in windows), closest,
                                                                   1000 * cell.Loader_Free, 1000 * cell.Tracking_Margin)))
    return out


def Carried_Tilt(trajectory: Trajectory_Str, first: int, last: int) -> float:
    """
    Description:
        The largest tilt from horizontal of the PCB the cups carry over samples (B4).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2 - 3) first, last [int]: The samples.

    Returns:
        (1) parameter [float]: The tilt (rad); 0 when the cups carry nothing.
    """

    best = 0.0
    for i in range(first, last + 1):
        if CONST_PCB_LOCATIONS[trajectory.PCB_Location[i]] == 'cups':
            w, x, y, z = trajectory.PCB_Pose[i][3:]
            best = max(best, math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))
    return best


@dataclass
class Program_Metrics_Str:
    """
    Description:
        What a program's motion costs (compare_programs.py): the cycle time, the robot's joint travel per joint, its TCP's and
        the PCB's path lengths, the loader's travel, the robot's stops (each time it comes to rest).
    """

    # The cycle time.
    #   Unit [float] (s)
    Cycle_Time: float = 0.0
    # The joint travel, per joint (sum of |dq| over the samples).
    #   Unit [Vector<float> 1x6] (rad)
    Joint_Travel: np.ndarray = field(default_factory=lambda: np.zeros(6))
    # The TCP's and the PCB's path lengths (the PCB: NaN when not known).
    #   Unit [float] (m)
    Tcp_Path: float = 0.0
    Pcb_Path: float = math.nan
    # The loader's travel, Y plus Z.
    #   Unit [float] (m)
    Loader_Travel: float = 0.0
    # The robot's stops: every joint speed below CONST_STOP_SPEED for at least CONST_STOP_TIME, after a motion; their total time.
    #   Unit [int], [float] (s)
    Stops: int = 0
    Rest_Time: float = 0.0


def Measure(q: np.ndarray, dt: float, kinematics: Robot_Kinematics_Cls, loader_y: np.ndarray, loader_z: np.ndarray,
            pcb: tp.Optional[np.ndarray] = None) -> Program_Metrics_Str:
    """
    Description:
        A program's metrics from its samples: joint travel, the suction TCP's path (FK per sample), the PCB's path, the
        loader's travel, the robot's stops (each run of samples with every joint speed - its change over dt - below
        CONST_STOP_SPEED lasting at least CONST_STOP_TIME, once the robot has moved).

    Args:
        (1) q [Matrix<float> nx6]: The joints per sample (rad).
        (2) dt [float]: The sample time (s).
        (3) kinematics [Robot_Kinematics_Cls(object)]: FK.
        (4 - 5) loader_y, loader_z [Vector<float> n]: The loader's axes per sample (m).
        (6) pcb [Matrix<float> nx3]: The PCB's position per sample (m), or None.

    Returns:
        (1) parameter [Program_Metrics_Str(object)]: The metrics.
    """

    dq = np.abs(np.diff(q, axis=0))
    tcp = np.array([kinematics.Site_Pose(qi)[0] for qi in q])
    rest = np.max(dq, axis=1) / dt < CONST_STOP_SPEED
    stops, run, moved, rest_time = 0, 0, False, 0.0
    for still in np.r_[rest, False]:
        if still:
            run += 1
            continue
        if moved and run * dt >= CONST_STOP_TIME - 1e-9:
            stops, rest_time = stops + 1, rest_time + run * dt
        run, moved = 0, True
    return Program_Metrics_Str(Cycle_Time=float(len(q) - 1) * dt, Joint_Travel=dq.sum(axis=0),
                               Tcp_Path=float(np.sum(np.linalg.norm(np.diff(tcp, axis=0), axis=1))),
                               Pcb_Path=float(np.sum(np.linalg.norm(np.diff(pcb, axis=0), axis=1))) if pcb is not None else math.nan,
                               Loader_Travel=float(np.sum(np.abs(np.diff(loader_y))) + np.sum(np.abs(np.diff(loader_z)))),
                               Stops=stops, Rest_Time=round(rest_time, 6))
