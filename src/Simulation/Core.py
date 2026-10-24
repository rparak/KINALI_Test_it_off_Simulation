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
# File Name: ../Simulation/Core.py
## =========================================================================== ##

# JSON (JSON encoder and decoder)
import json
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.: the files, the cell, the contact categories
#   ../Configuration/Core
from Configuration.Core import CONST_REFERENCE_PLAN, PCB_Meta_Str, Cell_Configuration_Str
#   ../Configuration/Program_Parser - a loader position
from Configuration.Program_Parser import Loader_Position_Str, Program_Str
#   ../Scene/Core
from Scene.Core import CONST_ROBOT_JOINTS, CONST_CUP_N, CONST_CRATE_FLOOR, Crate_Center, PCB_In_Crate
#   ../Collision/Core
from Collision.Core import (CONST_COLLISION_CHECKS, CONST_LOADER_BODIES, CONST_FORK_PHASES, Contact_Classifier_Cls, Penetration_Group, Cup_Contacts,
                            Robot_Collision_Cls)
#   ../Trajectory/Core - a program's trajectory (F11)
from Trajectory.Core import Trajectory_Str, CONST_DT

"""
Description:
    Initialization of constants.

    The physics run of a plan (SPEC §7.6): servo set-points with velocity feed-forward, min-jerk loader
    profiles, vacuum events, the instability guard; the measured values, the report of SPEC §4.6 and the
    acceptance checks of SPEC §10.1. Ported from legacy/test_cell_cycle.py (git tag legacy-planner-reference;
    design-decisions): the report and every measured value were identical to that run's. It prints nothing.
"""
# The PCB starts 0.3 mm above the input crate's floor, so it settles onto it.
#   Unit [float] (m)
CONST_PCB_DROP = 0.0003
# The robot servos are actuators 3..8 (SPEC §4.3).
#   Unit [slice]
CONST_ROBOT_ACTUATORS = slice(3, 9)
# The progress callback is called every this many steps (2 s of simulated time).
#   Unit [int]
CONST_PROGRESS_STEPS = 4000
# SPEC §10.1: the picks and the cups that must touch at each (the limits are in Configuration/Cell.yaml).
#   Unit [int], [int]
CONST_PICKS, CONST_CUPS = 2, 4
# The per-step measurements' period (the TCP's acceleration by second differences, the tool's tilt, the robot-loader distance
# in a parallel step), and the distance the robot-loader pairs are reported up to.
#   Unit [float] (s), [float] (m)
CONST_SAMPLE_PERIOD, CONST_LOADER_CUT = 0.004, 0.05
# The PCB is lost (F11) when the vacuum is on and no cup has touched it for this long (a contact can blink for a step), or when it
# is this far below the table top (z = 0; the physics model has no table and its floor does not collide, so a dropped PCB falls
# through; the lowest PCB of a cycle lies on a crate floor, 4 mm).
#   Unit [float] (s), [float] (m)
CONST_LOST_TIME, CONST_PCB_FLOOR = 0.02, -0.02


class Simulation_Error_Cls(RuntimeError):
    """
    Description:
        The simulation cannot run or became unstable (MuJoCo reset the state). The message says when and
        what to check; a script prints it verbatim as an [ERROR] line and exits 2.
    """


@dataclass
class Legacy_Plan_Str:
    """
    Description:
        A plan in the format of the legacy IK planner (cell_plan.npz; the planner is legacy/plan_cell.py at git tag
        legacy-planner-reference). Built by Read_Legacy_Plan.
    """

    # Sample time of the robot trajectory.
    #   Unit [float] (s)
    Dt: float = 0.004
    # The robot joint trajectory, one row per sample.
    #   Unit [Matrix<float> Nx6] (rad)
    Q: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    # The cycle time.
    #   Unit [float] (s)
    T_End: float = 0.0
    # Loader moves per channel ('y', 'z', 'jaw'): [start time, duration, target].
    #   Unit [Dict<string, List<List<float>>>] (s, s, m)
    Loader: tp.Dict[str, tp.List[tp.List[float]]] = field(default_factory=dict)
    # Vacuum events: [time, 'suction_on' or 'suction_off'].
    #   Unit [List<List>] (s, -)
    Events: tp.List[tp.List] = field(default_factory=list)
    # Phase labels: [start time, label].
    #   Unit [List<List>] (s, -)
    Phases: tp.List[tp.List] = field(default_factory=list)
    # The jaw opening at the start.
    #   Unit [float] (m)
    Jaw_Open: float = 0.0


@dataclass
class Vacuum_Event_Str:
    """
    Description:
        One vacuum switch of a run, and how many cups touched the PCB when it went on.
    """

    # When it switched.
    #   Unit [float] (s)
    T: float = 0.0
    # On (True) or off.
    #   Unit [bool]
    On: bool = False
    # Cups touching the PCB at switch-on (None at switch-off).
    #   Unit [int]
    Cups: tp.Optional[int] = None


@dataclass
class Cycle_Result_Str:
    """
    Description:
        What a cycle run measured (SPEC §4.6), in SI units. Built by Run_Cycle; formatted by Format_Report.
    """

    # The cycle time.
    #   Unit [float] (s)
    Cycle_Time: float = 0.0
    # The vacuum switches.
    #   Unit [List<Vacuum_Event_Str>]
    Vacuum: tp.List[Vacuum_Event_Str] = field(default_factory=list)
    # When the PCB was left in MG06, its position error against the place target and its tilt.
    #   Unit [float] (s), [Vector<float> 1x3] (m), [float] (rad)
    Placed_T: float = 0.0
    Placed_Error: np.ndarray = field(default_factory=lambda: np.zeros(3))
    Placed_Tilt: float = 0.0
    # MG06 closed with the robot away: lid and handle angles and the time, or None if not reached.
    #   Unit [Tuple<float>] (rad, rad, s)
    Closed: tp.Optional[tp.Tuple[float, float, float]] = None
    # The PCB's final pose, the output crate's centre, the PCB's final tilt.
    #   Unit [Vector<float> 1x7] (m, -), [Vector<float> 1x2] (m), [float] (rad)
    Final_PCB: np.ndarray = field(default_factory=lambda: np.zeros(7))
    Crate_Centre: np.ndarray = field(default_factory=lambda: np.zeros(2))
    Final_Tilt: float = 0.0
    # Largest robot joint tracking error, PCB slip in the cups, loader axis lag.
    #   Unit [float] (rad), [float] (m), [float] (m)
    Track_Error: float = 0.0
    Slip: float = 0.0
    Loader_Lag: float = 0.0
    # Largest total contact force of the fork on the handle bar.
    #   Unit [float] (N)
    Fork_Force: float = 0.0
    # Deepest PCB penetration per group: (distance, time).
    #   Unit [Dict<string, Tuple<float>>] (m, s)
    Penetration: tp.Dict[str, tp.Tuple[float, float]] = field(default_factory=dict)
    # Deepest contact per category of CONST_COLLISION_CHECKS: (distance, geom 1, geom 2, time), or absent.
    #   Unit [Dict<string, Tuple>] (m, -, -, s)
    Contacts: tp.Dict[str, tp.Tuple[float, str, str, float]] = field(default_factory=dict)
    # The MG06 lid's open stop (the model's lid_hinge lower limit), where the run starts.
    #   Unit [float] (rad)
    Lid_Open: float = 0.0
    # MG06 lid and handle at the end.
    #   Unit [float] (rad)
    Lid_End: float = 0.0
    Handle_End: float = 0.0
    # The run's steps (a program's steps or a legacy plan's phases), in order (F11).
    #   Unit [List<Step_Physics_Str>]
    Steps: tp.List['Step_Physics_Str'] = field(default_factory=list)
    # Why the run stopped before its end ('PCB lost at ...'), or None (F11).
    #   Unit [string]
    Fault: tp.Optional[str] = None


@dataclass
class Acceptance_Check_Str:
    """
    Description:
        One acceptance criterion of SPEC §10.1 and its verdict on a run.
    """

    # The criterion.
    #   Unit [string]
    Name: str = ''
    # Whether the run meets it.
    #   Unit [bool]
    Passed: bool = False
    # The measured value(s), for the report.
    #   Unit [string]
    Detail: str = ''


def Read_Legacy_Plan(path: str = CONST_REFERENCE_PLAN) -> Legacy_Plan_Str:
    """
    Description:
        Read a legacy plan (cell_plan.npz), the only reader of the format in src/.

    Args:
        (1) path [string]: The file (default Data/Reference/cell_plan.npz).

    Returns:
        (1) parameter [Legacy_Plan_Str(object)]: The plan.
    """

    try:
        plan = np.load(path)
        return Legacy_Plan_Str(Dt=float(plan['dt']), Q=plan['q'], T_End=float(plan['T']), Loader=json.loads(str(plan['loader'])),
                               Events=json.loads(str(plan['events'])), Phases=json.loads(str(plan['phases'])),
                               Jaw_Open=float(plan['J_OPEN']))
    except (OSError, KeyError, ValueError) as error:
        raise Simulation_Error_Cls('The plan %s cannot be read (%s). Use Data/Reference/cell_plan.npz or a plan in its format.'
                                   % (path, error)) from error


def Min_Jerk(s: float) -> float:
    """
    Description:
        The min-jerk profile 10 s^3 - 15 s^4 + 6 s^5 of a normalised time, clipped to [0, 1].

    Args:
        (1) s [float]: Normalised time.

    Returns:
        (1) parameter [float]: The fraction of the move done.
    """

    s = np.clip(s, 0, 1)
    return s**3 * (10 - 15 * s + 6 * s * s)


def Loader_Target(plan: Legacy_Plan_Str, start: tp.Dict[str, float], channel: str, t: float) -> float:
    """
    Description:
        The loader command of one channel at time t: every move started so far, min-jerk, in order.

    Args:
        (1) plan [Legacy_Plan_Str(object)]: The plan.
        (2) start [Dict<string, float>]: The start value per channel (m).
        (3) channel [string]: 'y', 'z' or 'jaw'.
        (4) t [float]: The time (s).

    Returns:
        (1) parameter [float]: The command (m).
    """

    v = start[channel]
    for t0, T, x in plan.Loader[channel]:
        if t >= t0:
            v = v + (x - v) * Min_Jerk((t - t0) / T)
    return v


def Robot_Target(plan: Legacy_Plan_Str, v_plan: np.ndarray, kv_kp: np.ndarray, t: float) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        The servo set-point at time t: the planned position, linearly interpolated, plus velocity
        feed-forward kv/kp * v_ref (SPEC §4.3).

    Args:
        (1) plan [Legacy_Plan_Str(object)]: The plan.
        (2) v_plan [Matrix<float> Nx6]: The planned joint velocities (rad/s).
        (3) kv_kp [Vector<float> 1x6]: kv / kp of each servo (s).
        (4) t [float]: The time (s).

    Returns:
        (1) parameter [Vector<float> 1x6]: The servo command (rad).
        (2) parameter [Vector<float> 1x6]: The planned position q_ref (rad).
    """

    n = len(plan.Q)
    x = min(t / plan.Dt, n - 1.001)
    i = int(x)
    w = x - i
    j = min(i + 1, n - 1)
    q = (1 - w) * plan.Q[i] + w * plan.Q[j]
    v = (1 - w) * v_plan[i] + w * v_plan[j]
    return q + kv_kp * v, q


def Phase_At(plan: Legacy_Plan_Str, t: float) -> str:
    """
    Description:
        The label of the last phase started at or before t ('' before the first).

    Args:
        (1) plan [Legacy_Plan_Str(object)]: The plan.
        (2) t [float]: The time (s).

    Returns:
        (1) parameter [string]: The phase label.
    """

    ph = [p for t0, p in plan.Phases if t >= t0]
    return ph[-1] if ph else ''


class Legacy_Drive_Cls(object):
    """
    Description:
        The drive of a physics run from a legacy plan (cell_plan.npz): the servo set-points of the plan's joints with the
        feed-forward of their np.gradient, the loader's min-jerk moves, the plan's vacuum events and phase labels; the fork
        may touch the handle in the closing and opening phases; the PCB is placed when the loader goes up to park, MG06 is
        closed with the robot away in the phase that says so. Run_Cycle's run, unchanged (its report is the old baseline).

    Initialization of the Class:
        Args:
            (1) plan [Legacy_Plan_Str(object)]: The plan.
            (2) loader_home [Loader_Position_Str(object)]: The loader's home (Constants.yaml L_HOME), where it starts.

        Example:
            Initialization:
                # Assignment of the variables.
                drive = Legacy_Drive_Cls(plan, constants.Loader['L_HOME'])

            Features:
                # Functions of the class (Physics_Run_Cls calls them).
                drive.Robot(t), drive.Loader(t), drive.Label(t)
    """

    def __init__(self, plan: Legacy_Plan_Str, loader_home: Loader_Position_Str) -> None:
        self.Plan, self.T_End = plan, plan.T_End
        self.__start = {'y': loader_home.Y, 'z': 0.0, 'jaw': plan.Jaw_Open}
        self.__v_plan = np.gradient(plan.Q, plan.Dt, axis=0)
        self.__kv_kp = np.zeros(6)
        self.Events = [(float(t0), name == 'suction_on') for t0, name in plan.Events]

    def Bind(self, kv_kp: np.ndarray) -> None:
        """
        Description:
            The servos' kv / kp, for the feed-forward.

        Args:
            (1) kv_kp [Vector<float> 1x6]: kv / kp of each servo (s).
        """

        self.__kv_kp = kv_kp

    def Start(self) -> tp.Tuple[np.ndarray, float, float, float]:
        """
        Description:
            The start: the robot joints, the loader's y and z, the jaws.

        Returns:
            (1) parameter [Vector<float> 1x6]: The joints (rad).
            (2 - 4) parameter [float]: y, z, jaw (m).
        """

        return self.Plan.Q[0], self.__start['y'], self.__start['z'], self.__start['jaw']

    def Robot(self, t: float) -> tp.Tuple[np.ndarray, np.ndarray]:
        """
        Description:
            The servo command and the reference joints at t (Robot_Target).

        Args:
            (1) t [float]: The time (s).

        Returns:
            (1 - 2) parameter [Vector<float> 1x6]: The command, q_ref (rad).
        """

        return Robot_Target(self.Plan, self.__v_plan, self.__kv_kp, t)

    def Loader(self, t: float) -> tp.List[float]:
        """
        Description:
            The loader's y, z and jaw commands at t (Loader_Target).

        Args:
            (1) t [float]: The time (s).

        Returns:
            (1) parameter [List<float>]: y, z, jaw (m).
        """

        return [Loader_Target(self.Plan, self.__start, 'y', t), Loader_Target(self.Plan, self.__start, 'z', t),
                Loader_Target(self.Plan, self.__start, 'jaw', t)]

    def Label(self, t: float) -> str:
        """
        Description:
            The phase at t (Phase_At).

        Args:
            (1) t [float]: The time (s).

        Returns:
            (1) parameter [string]: The label.
        """

        return Phase_At(self.Plan, t)

    def Command(self, t: float) -> str:
        """
        Description:
            The line at t: a plan has no lines, so its phase.

        Args:
            (1) t [float]: The time (s).

        Returns:
            (1) parameter [string]: The phase.
        """

        return Phase_At(self.Plan, t)

    def Fork_Allowed(self, label: str) -> bool:
        """
        Description:
            Whether the fork may touch the handle in this phase (the closing and opening ones).

        Args:
            (1) label [string]: The phase.

        Returns:
            (1) parameter [bool]: True when allowed.
        """

        return label.startswith(CONST_FORK_PHASES)

    def Placed(self, label: str, t: float) -> bool:
        """
        Description:
            Whether the PCB is placed in MG06: the loader goes up to park.

        Args:
            (1) label [string]: The phase.
            (2) t [float]: The time (s).

        Returns:
            (1) parameter [bool]: True from then on.
        """

        return label.startswith('loader: up, to park')

    def Closed(self, label: str, t: float) -> bool:
        """
        Description:
            Whether MG06 is closed with the robot away (the phase that says so).

        Args:
            (1) label [string]: The phase.
            (2) t [float]: The time (s).

        Returns:
            (1) parameter [bool]: True in it.
        """

        return 'away, MG06 closed' in label

    def Free(self, label: str) -> bool:
        """
        Description:
            Whether the step is a FREE transfer (a legacy plan has none).

        Args:
            (1) label [string]: The step.

        Returns:
            (1) parameter [bool]: False.
        """

        return False

    def Parallel(self, label: str) -> bool:
        """
        Description:
            Whether the robot and the loader may move together in the step (never in a legacy plan's phases).

        Args:
            (1) label [string]: The step.

        Returns:
            (1) parameter [bool]: False.
        """

        return False


class Trajectory_Drive_Cls(object):
    """
    Description:
        The drive of a physics run from a program's trajectory (F11): the servo set-points of its joints, linear between
        the 4 ms samples, with the feed-forward of its stored joint speeds (analytic in MoveJ and MoveSpline); the loader's y
        and z and the gripper's commands, linear between samples (the jaws stop on the PCB in physics); its vacuum events;
        its steps as labels. The fork may touch the handle in the steps whose Allow_Contact has handle_bar; the PCB is placed
        at J_PARK's first sample, MG06 closed at the Test step's first sample; a step is FREE when it runs a MoveSpline and
        parallel when it starts a loader job.

    Initialization of the Class:
        Args:
            (1) trajectory [Trajectory_Str(object)]: The trajectory.
            (2) program [Program_Str(object)]: Its program (the steps' allowed contacts and lines).

        Example:
            Initialization:
                # Assignment of the variables.
                drive = Trajectory_Drive_Cls(trajectory, program)

            Features:
                # Functions of the class (Physics_Run_Cls calls them).
                drive.Robot(t), drive.Loader(t), drive.Label(t)
    """

    def __init__(self, trajectory: Trajectory_Str, program: Program_Str) -> None:
        tr = trajectory
        self.Trajectory, self.T_End, self.Dt, self.N = tr, float(tr.T[-1]), CONST_DT, len(tr.T)
        self.__kv_kp = np.zeros(6)
        self.Events = [(float(t0), name.endswith('_on')) for t0, name in tr.Events if name.startswith('vacuum')]
        allowed = {s.Key for s in program.Segments if 'handle_bar' in s.Allow_Contact}
        self.__fork = {name for name in tr.Step_Names if name in allowed}
        lines = {name: [tr.Segment_Command[k] for k in range(len(tr.Segment_Command)) if tr.Step_Names[tr.Segment_Step[k]] == name]
                 for name in tr.Step_Names}
        self.__free = {name for name, texts in lines.items() if any(text.startswith('Robot.MoveSpline') for text in texts)}
        self.__parallel = {name for name, texts in lines.items() if any(text.startswith('Loader.Start') for text in texts)}
        park = next(k for k, text in enumerate(tr.Segment_Command) if text == 'Loader.Start(J_PARK)' or text.startswith('Loader.Run(J_PARK) >'))
        test = next(k for k, name in enumerate(tr.Step_Names) if name.endswith('_Test'))
        self.Placed_Time = float(tr.T[min(int(tr.Segment_Start[park]), self.N - 1)])
        self.Closed_Time = float(tr.T[int(np.flatnonzero(tr.Step_Index == test)[0])])

    def Bind(self, kv_kp: np.ndarray) -> None:
        """
        Description:
            The servos' kv / kp, for the feed-forward.

        Args:
            (1) kv_kp [Vector<float> 1x6]: kv / kp of each servo (s).
        """

        self.__kv_kp = kv_kp

    def Start(self) -> tp.Tuple[np.ndarray, float, float, float]:
        """
        Description:
            The start: sample 0's robot joints, loader y and z, jaws.

        Returns:
            (1) parameter [Vector<float> 1x6]: The joints (rad).
            (2 - 4) parameter [float]: y, z, jaw (m).
        """

        tr = self.Trajectory
        return tr.Robot_Q[0], float(tr.Loader_Y[0]), float(tr.Loader_Z[0]), float(tr.Gripper[0])

    def __At(self, t: float) -> tp.Tuple[int, int, float]:
        """
        Description:
            The two samples around t and the weight of the second (as Robot_Target).

        Args:
            (1) t [float]: The time (s).

        Returns:
            (1 - 2) parameter [int]: The samples.
            (3) parameter [float]: The weight.
        """

        x = min(t / self.Dt, self.N - 1.001)
        i = int(x)
        return i, min(i + 1, self.N - 1), x - i

    def Robot(self, t: float) -> tp.Tuple[np.ndarray, np.ndarray]:
        """
        Description:
            The servo command q_ref + kv / kp x v_ref and q_ref at t.

        Args:
            (1) t [float]: The time (s).

        Returns:
            (1 - 2) parameter [Vector<float> 1x6]: The command, q_ref (rad).
        """

        i, j, w = self.__At(t)
        tr = self.Trajectory
        q = (1 - w) * tr.Robot_Q[i] + w * tr.Robot_Q[j]
        v = (1 - w) * tr.Robot_QD[i] + w * tr.Robot_QD[j]
        return q + self.__kv_kp * v, q

    def Loader(self, t: float) -> tp.List[float]:
        """
        Description:
            The loader's y, z and the gripper's commands at t.

        Args:
            (1) t [float]: The time (s).

        Returns:
            (1) parameter [List<float>]: y, z, jaw (m).
        """

        i, j, w = self.__At(t)
        tr = self.Trajectory
        return [float((1 - w) * c[i] + w * c[j]) for c in (tr.Loader_Y, tr.Loader_Z, tr.Gripper)]

    def Label(self, t: float) -> str:
        """
        Description:
            The step at t.

        Args:
            (1) t [float]: The time (s).

        Returns:
            (1) parameter [string]: The step's name.
        """

        return self.Trajectory.Step_Names[int(self.Trajectory.Step_Index[self.__At(t)[0]])]

    def Command(self, t: float) -> str:
        """
        Description:
            The command line at t (the viewer's overlay).

        Args:
            (1) t [float]: The time (s).

        Returns:
            (1) parameter [string]: The line.
        """

        return self.Trajectory.Segment_Command[int(self.Trajectory.Segment_Id[self.__At(t)[0]])]

    def Fork_Allowed(self, label: str) -> bool:
        """
        Description:
            Whether the fork may touch the handle in the step (its Allow_Contact has handle_bar).

        Args:
            (1) label [string]: The step.

        Returns:
            (1) parameter [bool]: True when allowed.
        """

        return label in self.__fork

    def Placed(self, label: str, t: float) -> bool:
        """
        Description:
            Whether the PCB is placed in MG06: from J_PARK's first sample on.

        Args:
            (1) label [string]: The step.
            (2) t [float]: The time (s).

        Returns:
            (1) parameter [bool]: True from then on.
        """

        return t >= self.Placed_Time

    def Closed(self, label: str, t: float) -> bool:
        """
        Description:
            Whether MG06 is closed with the robot away: from the Test step's first sample on.

        Args:
            (1) label [string]: The step.
            (2) t [float]: The time (s).

        Returns:
            (1) parameter [bool]: True from then on.
        """

        return t >= self.Closed_Time

    def Free(self, label: str) -> bool:
        """
        Description:
            Whether the step is a FREE transfer (it runs a MoveSpline).

        Args:
            (1) label [string]: The step.

        Returns:
            (1) parameter [bool]: True for a FREE transfer.
        """

        return label in self.__free

    def Parallel(self, label: str) -> bool:
        """
        Description:
            Whether the robot and the loader may move together in the step (it starts a loader job).

        Args:
            (1) label [string]: The step.

        Returns:
            (1) parameter [bool]: True for a parallel step.
        """

        return label in self.__parallel


@dataclass
class Step_Physics_Str:
    """
    Description:
        One step of a physics run (a program's step or a legacy plan's phase): its time span, the largest tracking error, the
        PCB's slip in the cups with the tool's tilt and the TCP's acceleration where it was largest and their maxima, the
        deepest contact per category, the closest robot-loader distance in a parallel step.
    """

    # The step and its span.
    #   Unit [string], [float] (s), [float] (s)
    Name: str = ''
    Start: float = 0.0
    End: float = 0.0
    # Whether it is a FREE transfer and a parallel step.
    #   Unit [bool]
    Free: bool = False
    Parallel: bool = False
    # The largest robot tracking error.
    #   Unit [float] (rad)
    Track_Error: float = 0.0
    # The largest PCB slip in the cups (since vacuum on), its time, the tool's tilt from the vertical and the TCP's acceleration
    # then; the largest tilt and acceleration over the step while the cups hold the PCB.
    #   Unit [float] (m), [float] (s), [float] (rad), [float] (m/s^2), [float] (rad), [float] (m/s^2)
    Slip: float = 0.0
    Slip_T: float = 0.0
    Slip_Tilt: float = 0.0
    Slip_Acceleration: float = 0.0
    Tilt_Max: float = 0.0
    Acceleration_Max: float = 0.0
    # The deepest contact per CONST_COLLISION_CHECKS category: (distance, geom 1, geom 2, time).
    #   Unit [Dict<string, Tuple>] (m, -, -, s)
    Contacts: tp.Dict[str, tp.Tuple[float, str, str, float]] = field(default_factory=dict)
    # A parallel step's closest robot-loader distance (exact, every 4 ms): (distance, time, pair), or None.
    #   Unit [Tuple<float, float, string>] (m, s, -)
    Robot_Loader: tp.Optional[tp.Tuple[float, float, str]] = None


class Physics_Run_Cls(object):
    """
    Description:
        The physics run of a drive (a legacy plan or a program's trajectory) from the PCB in the input crate to the end of the
        cycle, one step at a time (Step), with the measurements of SPEC §4.6 (Result) and, per step, Step_Physics_Str. MuJoCo
        resets the state on instability; the run detects the time going backwards and raises instead of continuing (SPEC
        §4.7). The physics is the model's: the timestep and the solver as compiled.

    Initialization of the Class:
        Args:
            (1) model [mujoco.MjModel]: The compiled cell (Scene.Core.Build_Cell_Model).
            (2) drive [Legacy_Drive_Cls or Trajectory_Drive_Cls (object)]: What drives it.
            (3) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
            (4) cell [Cell_Configuration_Str(object)]: The cell configuration (the phase period, the cup touch).
            (5) progress [function]: Called with (time [s], label) every CONST_PROGRESS_STEPS steps, or None.

        Example:
            Initialization:
                # Assignment of the variables.
                run = Physics_Run_Cls(model, drive, pcb_meta, cell)

            Features:
                # Functions of the class.
                while not run.Done: run.Step()
                result = run.Result()
    """

    def __init__(self, model: mujoco.MjModel, drive: tp.Any, pcb_meta: PCB_Meta_Str, cell: Cell_Configuration_Str,
                 progress: tp.Optional[tp.Callable[[float, str], None]] = None) -> None:
        m = model
        self.Model, self.Drive, self.Meta, self.Cell, self.Progress = m, drive, pcb_meta, cell, progress
        self.Data = d = mujoco.MjData(m)
        J = lambda n: m.jnt_qposadr[m.joint(n).id]
        self.QR = np.array([J('aubo/' + j) for j in CONST_ROBOT_JOINTS])
        self.qa, self.qh, self.qy, self.qz, self.qjl, self.qjr, self.qpcb = (J(n) for n in ['lid_hinge', 'handle_hinge', 'y_axis', 'z_axis',
                                                                                            'jaw_L', 'jaw_R', 'pcb_free'])
        self.TOOL, self.PCB = m.body('aubo/tool/ee_tool').id, m.body('pcb').id
        self.TCP = m.site('aubo/tool/suction_tcp').id
        self.CUPS = {m.body(f'aubo/tool/vgc10_cup{i}').id for i in range(CONST_CUP_N)}
        self.A_VAC = [m.actuator(f'vacuum{i}').id for i in range(CONST_CUP_N)]
        self.classifier = Contact_Classifier_Cls(m)
        A_ROBOT = CONST_ROBOT_ACTUATORS
        drive.Bind(m.actuator_biasprm[A_ROBOT, 2] / m.actuator_biasprm[A_ROBOT, 1])
        q0, y0, z0, jaw0 = drive.Start()
        # 1\\ Initial state: MG06 open, the loader where the drive starts with its jaws, the PCB just above the input crate floor,
        #    the robot at the drive's first joints; the filtered loader drives start where the loader is.
        mujoco.mj_resetData(m, d)
        self.Lid_Open = m.jnt_range[m.joint('lid_hinge').id][0]
        d.qpos[self.qa] = self.Lid_Open
        d.qpos[self.qy] = y0
        d.qpos[self.qz] = z0
        d.qpos[self.qjl] = d.qpos[self.qjr] = jaw0
        pp, pq = PCB_In_Crate('crate_pick')
        d.qpos[self.qpcb:self.qpcb + 3], d.qpos[self.qpcb + 3:self.qpcb + 7] = pp + [0, 0, CONST_PCB_DROP], pq
        d.qpos[self.QR] = q0
        d.ctrl[:3] = [y0, z0, jaw0]
        d.ctrl[A_ROBOT] = q0
        d.ctrl[self.A_VAC] = 0
        d.act[:] = 0
        d.act[m.actuator_actadr[0]] = y0
        d.act[m.actuator_actadr[1]] = z0
        mujoco.mj_forward(m, d)
        self.ev = list(drive.Events)
        self.worst, self.vacuum, self.pen, self.marks = {}, [], {}, {}
        self.track_err, self.slip, self.fork_force, self.loader_lag = 0.0, 0.0, 0.0, 0.0
        self.rel0, self.ph = None, ''
        # The lost PCB (F11): since when no cup touches it with the vacuum on; the last moment cups held it (time, cups, slip,
        # tilt, acceleration); the PCB's last position (the evidence of an instability, after which MuJoCo resets the state).
        self.Fault: tp.Optional[str] = None
        self.no_cups: tp.Optional[float] = None
        self.held: tp.Optional[tp.Tuple[float, int, float, float, float]] = None
        self.pcb_last = d.qpos[self.qpcb:self.qpcb + 3].copy()
        self.fc = np.zeros(6)
        # The per-step measurements (F11): the steps by label; the TCP's positions every 4 ms for its acceleration.
        self.steps: tp.Dict[str, Step_Physics_Str] = {}
        self.tcp_history: tp.List[np.ndarray] = []
        self.tcp_acceleration, self.tilt = 0.0, 0.0
        self.sub = max(1, int(round(CONST_SAMPLE_PERIOD / m.opt.timestep)))
        robot = {i for i in range(m.nbody) if m.body(i).name.startswith('aubo/')}
        loader = {m.body(n).id for n in CONST_LOADER_BODIES}
        self.robot_bodies, self.not_loader = robot, set(range(m.nbody)) - loader
        self.collision = Robot_Collision_Cls(m, d)

    @property
    def Done(self) -> bool:
        """
        Description:
            Whether the run has reached the drive's end, or stopped on a fault (the PCB lost).

        Returns:
            (1) parameter [bool]: True at the end.
        """

        return self.Data.time >= self.Drive.T_End or self.Fault is not None

    def Step(self) -> None:
        """
        Description:
            One physics step: the vacuum events due, the loader, gripper and servo commands, mj_step, the instability guard,
            the measurements of SPEC §4.6 and the step's (F11).
        """

        m, d, drive, cell = self.Model, self.Data, self.Drive, self.Cell
        A_ROBOT, PCB, TOOL, CUPS = CONST_ROBOT_ACTUATORS, self.PCB, self.TOOL, self.CUPS
        t = d.time
        while self.ev and t >= self.ev[0][0]:
            _, on = self.ev.pop(0)
            d.ctrl[self.A_VAC] = 1.0 if on else 0.0
            self.rel0 = None
            self.vacuum.append(Vacuum_Event_Str(T=t, On=on, Cups=Cup_Contacts(m, d, PCB, CUPS, cell.Cup_Touch) if on else None))
        d.ctrl[:3] = drive.Loader(t)
        d.ctrl[A_ROBOT], q_ref = drive.Robot(t)
        mujoco.mj_step(m, d)
        ph = self.ph
        if d.time < t:
            p = 1000 * self.pcb_last
            raise Simulation_Error_Cls('The simulation became unstable at t = %.3f s (phase: %s): MuJoCo reset the state (line: %s; the '
                                       'PCB was at x %.1f y %.1f z %.1f mm). Check the plan around that time and the last change to the '
                                       'model.' % (t, ph or 'start', drive.Command(t), p[0], p[1], p[2]))
        self.pcb_last = d.qpos[self.qpcb:self.qpcb + 3].copy()
        step = int(round(d.time / m.opt.timestep))
        if self.Progress is not None and step % CONST_PROGRESS_STEPS == 0:
            self.Progress(d.time, ph)
        error = np.max(np.abs(d.qpos[self.QR] - q_ref))
        self.track_err = max(self.track_err, error)
        self.loader_lag = max(self.loader_lag, abs(d.qpos[self.qy] - d.ctrl[0]), abs(d.qpos[self.qz] - d.ctrl[1]))
        # PCB slip in the cups while the vacuum is on: from the first moment a cup touches it, and on when the cups lose it (F11).
        slip_now, cups = None, None
        if d.ctrl[self.A_VAC[0]] > 0.5:
            cups = Cup_Contacts(m, d, PCB, CUPS, cell.Cup_Touch)
            if cups or self.rel0 is not None:
                R = d.xmat[TOOL].reshape(3, 3)
                rel = R.T @ (d.xpos[PCB] - d.xpos[TOOL])
                self.rel0 = rel if self.rel0 is None else self.rel0
                slip_now = np.linalg.norm(rel - self.rel0)
                self.slip = max(self.slip, slip_now)
        ff = 0.0
        record = self.steps.setdefault(ph, Step_Physics_Str(Name=ph, Start=t, Free=drive.Free(ph), Parallel=drive.Parallel(ph)))
        for i, c in enumerate(d.contact[:d.ncon]):
            k = self.classifier.Classify(c.geom1, c.geom2, ph, drive.Fork_Allowed(ph))
            if k and c.dist < self.worst.get(k, (0,))[0]:
                self.worst[k] = (c.dist, self.classifier.Geom_Name(c.geom1), self.classifier.Geom_Name(c.geom2), round(d.time, 2))
            if k and c.dist < record.Contacts.get(k, (0,))[0]:
                record.Contacts[k] = (c.dist, self.classifier.Geom_Name(c.geom1), self.classifier.Geom_Name(c.geom2), round(d.time, 3))
            b1, b2 = self.classifier.Body_Name(c.geom1), self.classifier.Body_Name(c.geom2)
            # The thin board: its penetration into whatever holds it.
            if 'pcb' in (b1, b2) and not (b1.startswith('aubo/') or b2.startswith('aubo/')):
                grp = Penetration_Group(b2 if b1 == 'pcb' else b1)
                if c.dist < self.pen.get(grp, (0,))[0]:
                    self.pen[grp] = (c.dist, round(d.time, 2))
            if {b1, b2} == {'aubo/tool/ee_tool', 'mg06_handle'}:
                mujoco.mj_contactForce(m, d, i, self.fc)
                ff += self.fc[0]
        self.fork_force = max(self.fork_force, ff)
        # The step's measurements (F11): tracking; every 4 ms the TCP's acceleration (second differences) and the tool's tilt
        # from the vertical; the slip with the tilt and the acceleration then; a parallel step's robot-loader distance.
        record.End = d.time
        record.Track_Error = max(record.Track_Error, error)
        if step % self.sub == 0:
            self.tcp_history = (self.tcp_history + [d.site_xpos[self.TCP].copy()])[-3:]
            if len(self.tcp_history) == 3:
                p0, p1, p2 = self.tcp_history
                self.tcp_acceleration = float(np.linalg.norm(p2 - 2 * p1 + p0)) / CONST_SAMPLE_PERIOD ** 2
            self.tilt = float(np.arccos(np.clip(-d.site_xmat[self.TCP].reshape(3, 3)[2, 2], -1.0, 1.0)))
            if record.Parallel:
                pairs = self.collision.Close_Pairs(self.robot_bodies, CONST_LOADER_CUT, d, exclude=self.not_loader)
                found = (pairs[0][0], d.time, '%s / %s' % (pairs[0][1], pairs[0][2])) if pairs else (CONST_LOADER_CUT, d.time, '')
                if record.Robot_Loader is None or found[0] < record.Robot_Loader[0]:
                    record.Robot_Loader = found
        if slip_now is not None:
            record.Tilt_Max = max(record.Tilt_Max, self.tilt)
            record.Acceleration_Max = max(record.Acceleration_Max, self.tcp_acceleration)
            if slip_now > record.Slip:
                record.Slip, record.Slip_T, record.Slip_Tilt, record.Slip_Acceleration = slip_now, d.time, self.tilt, self.tcp_acceleration
        if step % cell.Phase_Steps == 0:
            self.ph = drive.Label(d.time)
        if drive.Placed(self.ph, d.time) and 'placed' not in self.marks:
            self.marks['placed'] = (d.qpos[self.qpcb:self.qpcb + 7].copy(), d.time)
        if drive.Closed(self.ph, d.time) and 'closed' not in self.marks:
            self.marks['closed'] = (d.qpos[self.qa], d.qpos[self.qh], d.time)
        self.__Lost(cups, slip_now)

    def __Lost(self, cups: tp.Optional[int], slip: tp.Optional[float]) -> None:
        """
        Description:
            The lost PCB (F11): with the vacuum on, no cup has touched it for CONST_LOST_TIME, or it fell below CONST_PCB_FLOOR. The
            run stops (Fault) with the time, the step, the line and the last moment the cups held it.

        Args:
            (1) cups [int]: The cups touching it with the vacuum on, or None with the vacuum off.
            (2) slip [float]: Its slip in the cups now (m), or None.
        """

        d, why = self.Data, None
        if cups:
            self.no_cups, self.held = None, (d.time, cups, slip or 0.0, self.tilt, self.tcp_acceleration)
        elif cups is None:
            self.no_cups = None
        elif self.no_cups is None:
            self.no_cups = d.time
        elif d.time - self.no_cups >= CONST_LOST_TIME - 1e-9:
            why = 'no cup has touched it for %.0f ms with the vacuum on' % (1000 * (d.time - self.no_cups))
        if why is None and d.qpos[self.qpcb + 2] < CONST_PCB_FLOOR:
            why = 'it is %.0f mm below the table top' % (-1000 * d.qpos[self.qpcb + 2])
        if why is not None:
            last = ('; last held at %.3f s by %d of %d cups, slip %.2f mm, tool tilt %.1f deg, TCP acceleration %.2f m/s^2'
                    % (self.held[0], self.held[1], CONST_CUPS, 1000 * self.held[2], np.degrees(self.held[3]), self.held[4])
                    if self.held is not None else '')
            self.Fault = 'PCB lost at t = %.3f s, step %s, line "%s": %s%s.' % (d.time, self.ph, self.Drive.Command(d.time), why, last)

    def Result(self) -> 'Cycle_Result_Str':
        """
        Description:
            The values the report and the acceptance need, at the end of the run.

        Returns:
            (1) parameter [Cycle_Result_Str(object)]: The measured values.
        """

        d = self.Data
        if 'placed' not in self.marks and self.Fault is None:
            raise Simulation_Error_Cls("The run never placed the PCB in MG06 (the phase 'loader: up, to park' / J_PARK's first sample).")
        pm, tm = self.marks.get('placed', (np.full(7, np.nan), np.nan))
        qd, qi = np.zeros(4), np.zeros(4)
        mujoco.mju_negQuat(qi, np.array(self.Meta.Quat))
        mujoco.mju_mulQuat(qd, pm[3:], qi)
        pf = d.qpos[self.qpcb:self.qpcb + 7].copy()
        return Cycle_Result_Str(Cycle_Time=self.Drive.T_End, Vacuum=self.vacuum, Placed_T=tm, Placed_Error=pm[:3] - np.array(self.Meta.Place_Pos),
                                Placed_Tilt=2 * np.arccos(min(1, abs(qd[0]))), Closed=self.marks.get('closed'), Final_PCB=pf,
                                Crate_Centre=Crate_Center('crate_place'),
                                Final_Tilt=np.arccos(min(1, abs(1 - 2 * (pf[4]**2 + pf[5]**2)))), Track_Error=self.track_err, Slip=self.slip,
                                Loader_Lag=self.loader_lag, Fork_Force=self.fork_force, Penetration=self.pen, Contacts=self.worst,
                                Lid_Open=self.Lid_Open, Lid_End=d.qpos[self.qa], Handle_End=d.qpos[self.qh], Steps=list(self.steps.values()),
                                Fault=self.Fault)


def Run_Physics(model: mujoco.MjModel, drive: tp.Any, pcb_meta: PCB_Meta_Str, cell: Cell_Configuration_Str,
                progress: tp.Optional[tp.Callable[[float, str], None]] = None) -> 'Cycle_Result_Str':
    """
    Description:
        The physics run of a drive to its end (Physics_Run_Cls).

    Args:
        (1) model [mujoco.MjModel]: The compiled cell.
        (2) drive [Legacy_Drive_Cls or Trajectory_Drive_Cls (object)]: What drives it.
        (3) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (4) cell [Cell_Configuration_Str(object)]: The cell configuration.
        (5) progress [function]: Called with (time [s], label) every CONST_PROGRESS_STEPS steps, or None.

    Returns:
        (1) parameter [Cycle_Result_Str(object)]: The measured values.
    """

    run = Physics_Run_Cls(model, drive, pcb_meta, cell, progress)
    while not run.Done:
        run.Step()
    return run.Result()


def Run_Cycle(model: mujoco.MjModel, plan: Legacy_Plan_Str, pcb_meta: PCB_Meta_Str, cell: Cell_Configuration_Str,
              loader_home: Loader_Position_Str, progress: tp.Optional[tp.Callable[[float, str], None]] = None) -> 'Cycle_Result_Str':
    """
    Description:
        The physics run of a legacy plan (Run_Physics with Legacy_Drive_Cls): the run of the old baseline report.

    Args:
        (1) model [mujoco.MjModel]: The compiled cell (Scene.Core.Build_Cell_Model).
        (2) plan [Legacy_Plan_Str(object)]: The plan.
        (3) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (4) cell [Cell_Configuration_Str(object)]: The cell configuration (the phase period, the cup touch).
        (5) loader_home [Loader_Position_Str(object)]: The loader's home (Constants.yaml L_HOME), where it starts.
        (6) progress [function]: Called with (time [s], phase) every CONST_PROGRESS_STEPS steps, or None.

    Returns:
        (1) parameter [Cycle_Result_Str(object)]: The measured values.
    """

    return Run_Physics(model, Legacy_Drive_Cls(plan, loader_home), pcb_meta, cell, progress)


def Format_Report(result: Cycle_Result_Str) -> tp.List[str]:
    """
    Description:
        The report lines of SPEC §4.6 (mm and deg), identical to the legacy run's (SPEC §10.2).

    Args:
        (1) result [Cycle_Result_Str(object)]: A run's values.

    Returns:
        (1) parameter [List<string>]: The lines, without newlines.
    """

    r = result
    lines = ['cycle time %.1f s' % r.Cycle_Time]
    for v in r.Vacuum:
        lines.append(f'{v.T:6.2f}s vacuum {"ON " if v.On else "OFF"}' + (f' ({v.Cups} of 4 cups touch the PCB)' if v.On else ''))
    e = r.Placed_Error
    lines.append(f'PCB in MG06 (t={r.Placed_T:.1f}s): error {1000*np.linalg.norm(e):.2f} mm (dx {1000*e[0]:.2f} dy {1000*e[1]:.2f} '
                 f'dz {1000*e[2]:.2f}), tilt {np.degrees(r.Placed_Tilt):.2f} deg')
    if r.Closed is not None:
        lines.append('MG06 closed with the robot away (t=%.1fs): lid %.1f deg, handle %.1f deg'
                     % (r.Closed[2], np.degrees(r.Closed[0]), np.degrees(r.Closed[1])))
    pf, cc = r.Final_PCB, r.Crate_Centre
    lines.append(f'PCB at the end: x {1000*pf[0]:.1f} y {1000*pf[1]:.1f} z {1000*pf[2]:.1f} mm (right crate centre {1000*cc[0]:.1f} '
                 f'{1000*cc[1]:.1f}, floor {1000*CONST_CRATE_FLOOR:.1f}); tilt {np.degrees(r.Final_Tilt):.2f} deg')
    lines.append('robot joint tracking error max %.4f rad, PCB slip in the cups max %.2f mm, loader axis lag max %.1f mm'
                 % (r.Track_Error, 1000 * r.Slip, 1000 * r.Loader_Lag))
    lines.append('max contact force fork -> handle bar: %.0f N' % r.Fork_Force)
    lines.append('deepest PCB penetration: ' + ', '.join(f'{k} {1000*v[0]:.2f} mm (t={v[1]})' for k, v in sorted(r.Penetration.items())))
    for k in CONST_COLLISION_CHECKS:
        v = r.Contacts.get(k)
        lines.append(f'{k:50s}: ' + ('no contact' if v is None else f'deepest {1000*v[0]:.2f} mm  {v[1]} / {v[2]}  t={v[3]}'))
    lines.append('MG06 at the end: lid %.1f deg, handle %.1f deg' % (np.degrees(r.Lid_End), np.degrees(r.Handle_End)))
    return lines


def Check_Acceptance(result: Cycle_Result_Str, cell: Cell_Configuration_Str) -> tp.List[Acceptance_Check_Str]:
    """
    Description:
        The acceptance criteria of SPEC §10.1 (items 1-5 and 7; item 6, no instability, is Run_Cycle
        returning at all) on a run's values.

    Args:
        (1) result [Cycle_Result_Str(object)]: A run's values.
        (2) cell [Cell_Configuration_Str(object)]: The limits (Cell.yaml Acceptance).

    Returns:
        (1) parameter [List<Acceptance_Check_Str>]: One verdict per criterion, in SPEC order.
    """

    r, out = result, []
    for k in CONST_COLLISION_CHECKS:
        v = r.Contacts.get(k)
        out.append(Acceptance_Check_Str(k, v is None, 'no contact' if v is None else
                                        'deepest %.2f mm %s / %s t=%s' % (1000 * v[0], v[1], v[2], v[3])))
    picks = [v for v in r.Vacuum if v.On]
    out.append(Acceptance_Check_Str('vacuum ON %dx with %d/%d cups' % (CONST_PICKS, CONST_CUPS, CONST_CUPS),
                                    len(picks) == CONST_PICKS and all(v.Cups == CONST_CUPS for v in picks),
                                    ', '.join('t=%.2fs %s/%d' % (v.T, v.Cups, CONST_CUPS) for v in picks)))
    error = float(np.linalg.norm(r.Placed_Error))
    out.append(Acceptance_Check_Str('PCB in MG06 error < %.1f mm, tilt < %.1f deg' % (1000 * cell.Place_Error, np.degrees(cell.Place_Tilt)),
                                    error < cell.Place_Error and r.Placed_Tilt < cell.Place_Tilt,
                                    '%.2f mm, %.2f deg' % (1000 * error, np.degrees(r.Placed_Tilt))))
    distance = float(np.linalg.norm(r.Final_PCB[:2] - r.Crate_Centre))
    out.append(Acceptance_Check_Str('PCB in the output crate < %.1f mm from the centre' % (1000 * cell.Crate_Distance),
                                    distance < cell.Crate_Distance, '%.2f mm' % (1000 * distance)))
    out.append(Acceptance_Check_Str('MG06 ends open', abs(r.Lid_End - r.Lid_Open) < cell.Open_Tolerance
                                    and abs(r.Handle_End) < cell.Open_Tolerance,
                                    'lid %.1f deg, handle %.1f deg' % (np.degrees(r.Lid_End), np.degrees(r.Handle_End))))
    deepest = max([-v[0] for v in r.Penetration.values()] + [0.0])
    out.append(Acceptance_Check_Str('PCB penetration <= %.1f mm' % (1000 * cell.Penetration), deepest <= cell.Penetration,
                                    ', '.join('%s %.2f mm' % (k, -1000 * v[0]) for k, v in sorted(r.Penetration.items()))))
    return out
