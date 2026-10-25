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

# Regular expressions (Default)
import re
# Math (Mathematical functions)
import math
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
from Configuration.Core import PCB_Meta_Str, Cell_Configuration_Str
#   ../Configuration/Program_Parser - the program, the PCB locations
from Configuration.Program_Parser import CONST_PCB_LOCATIONS, Program_Str
#   ../Scene/Core
from Scene.Core import CONST_ROBOT_JOINTS, CONST_CUP_N, Crate_Center, PCB_In_Crate
#   ../Collision/Core
from Collision.Core import CONST_LOADER_BODIES, Contact_Classifier_Cls, Penetration_Group, Cup_Contacts, Robot_Collision_Cls, Ledge_Gap
#   ../Trajectory/Core - a program's trajectory (F11)
from Trajectory.Core import Trajectory_Str, CONST_DT
#   ../Simulation/Const - the constants the run and its report share
from Simulation.Const import CONST_CUPS, CONST_LOADER_CUT, CONST_VACUUM_OK

"""
Description:
    Initialization of constants.

    The physics run of a program's trajectory (SPEC §7.6): servo set-points with velocity feed-forward, the loader's and the
    gripper's commands, vacuum events, the instability guard; the measured values (their acceptance checks of SPEC §10.1 and
    the report: Simulation/Report.py). Ported from legacy/test_cell_cycle.py (commit f754e2d0379b06ffb43286f4fb7aabf88e25ca5f).
    It prints nothing.
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
# The per-step measurements' period (the TCP's acceleration by second differences, the tool's tilt, the robot-loader distance
# in a parallel step).
#   Unit [float] (s)
CONST_SAMPLE_PERIOD = 0.004
# The place into MG06 (B work, items 7 and 9): the MG06 holder's four supports the PCB is seated on - the holder geoms within
# 0.03 mm of the centred PCB at its static seat (holder_10, the rear-right one, lowered 0.8 mm: the CAD deviation); every
# other holder geom is a side guide or a pin. The settled penetration is the deepest over CONST_SETTLED_WINDOW after the
# seating time (the PCB at rest, MG06 still open); the re-grip's ledge gap is measured CONST_REGRIP_SETTLE after its clamp
# command ends.
#   Unit [Tuple<string>], [float] (s), [float] (s)
CONST_HOLDER_SUPPORTS = ('col_holder_03', 'col_holder_04', 'col_holder_06', 'col_holder_10')
CONST_SETTLED_WINDOW, CONST_REGRIP_SETTLE = 0.5, 0.2
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
class Hold_Str:
    """
    Description:
        One vacuum hold of a run (vacuum on to off): the fewest cups that touched the PCB and when (F11).
    """

    # When the vacuum went on and off (nan while it is on).
    #   Unit [float] (s)
    T_On: float = 0.0
    T_Off: float = float('nan')
    # The fewest cups touching the PCB during the hold, and the first time it was that few.
    #   Unit [int], [float] (s)
    Min_Cups: int = CONST_CUPS
    T_Min: float = 0.0


@dataclass
class Cycle_Result_Str:
    """
    Description:
        What a cycle run measured (SPEC §4.6), in SI units. Built by Physics_Run_Cls; formatted by Format_Report.
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
    # Largest total contact force of the fork on the handle bar (the sum of the contact points' normal forces) and the number
    # of its contact points then: the force's mode (4 or 6, KNOWN_ISSUES.md; step 6 follow-up).
    #   Unit [float] (N), [int] (-)
    Fork_Force: float = 0.0
    Fork_Points: int = 0
    # The fork on the handle bar (F11): the largest net contact force on the bar (the vector sum's magnitude), its direction in
    # the fork's frame then (ee_tool: the bar's axis is its z), its time; the largest squeeze ((the summed normal force - the
    # vector sum of the normal forces) / 2: what opposing slot walls cancel); the largest handle hinge load (its constraint
    # torque).
    #   Unit [float] (N), [Vector<float> 1x3] (-), [float] (s), [float] (N), [float] (N m)
    Fork_Net_Force: float = 0.0
    Fork_Net_Direction: np.ndarray = field(default_factory=lambda: np.zeros(3))
    Fork_Net_T: float = 0.0
    Fork_Squeeze: float = 0.0
    Handle_Load: float = 0.0
    # Each robot joint's largest actuator torque and the model's limit (step 6).
    #   Unit [Vector<float> 1x6] (N m), [Vector<float> 1x6] (N m)
    Joint_Torque: np.ndarray = field(default_factory=lambda: np.zeros(6))
    Torque_Limit: np.ndarray = field(default_factory=lambda: np.full(6, np.inf))
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
    # The run's steps (a program's steps), in order (F11).
    #   Unit [List<Step_Physics_Str>]
    Steps: tp.List['Step_Physics_Str'] = field(default_factory=list)
    # Why the run stopped before its end ('PCB lost at ...', 'vacuum not OK at ...'), or None, and its kind ('lost', 'input')
    # (F11).
    #   Unit [string], [string]
    Fault: tp.Optional[str] = None
    Fault_Kind: tp.Optional[str] = None
    # The vacuum holds (F11).
    #   Unit [List<Hold_Str>]
    Holds: tp.List[Hold_Str] = field(default_factory=list)
    # The place into MG06 (B work), or None in a run without one.
    #   Unit [Place_Physics_Str(object)]
    Place: tp.Optional['Place_Physics_Str'] = None


@dataclass
class Place_Physics_Str:
    """
    Description:
        The loader's place into MG06 and the gripper operations around it, measured in physics (B work, items 7 and 9).
    """

    # When the jaws release the PCB (their last contact with it after the open command), or inf; the PCB's height then and
    # seated (Seat_Time later), and the drop between.
    #   Unit [float] (s), [float] (m), [float] (m), [float] (m)
    Release: float = math.inf
    Release_Z: float = 0.0
    Seated_Z: float = 0.0
    Drop: float = 0.0
    # The descent from the travel height to the release: the PCB's closest distance to a side guide and the guide.
    #   Unit [float] (m), [string]
    Guide_Min: float = math.inf
    Guide_Pair: str = ''
    # From reaching L_PLACE to the open command: the jaw gap's largest decrease, the largest PCB - holder force.
    #   Unit [float] (m), [float] (N)
    Jaw_Closing: float = 0.0
    Holder_Force: float = 0.0
    # The deepest PCB - holder penetration while it drops (to Seat_Time) and settled (CONST_SETTLED_WINDOW after).
    #   Unit [float] (m), [float] (m)
    Transient: float = 0.0
    Settled: float = 0.0
    # Seated: the supports it touches, the four supports' distances, its speed and its tilt from the place pose, its other
    # contacts (side guides, jaws), the time from the release until it stays slower than Seat_Speed.
    #   Unit [List<string>], [Dict<string, float>] (m), [float] (m/s), [float] (rad), [List<string>], [float] (s)
    Supports: tp.List[str] = field(default_factory=list)
    Support_Gaps: tp.Dict[str, float] = field(default_factory=dict)
    Speed: float = math.inf
    Tilt: float = math.inf
    Other_Contacts: tp.List[str] = field(default_factory=list)
    Settle_Time: float = math.inf
    # The handover clamp's centring of the PCB along the jaw axis (signed), and the re-grip's PCB-ledge gap.
    #   Unit [float] (m), [float] (m)
    Centring: float = math.nan
    Regrip_Gap: float = math.nan


def _Gripper_Times(trajectory: Trajectory_Str) -> tp.Tuple[tp.Optional[tp.Tuple[float, float, float]], tp.Optional[float],
                                                        tp.Optional[tp.Tuple[float, float]]]:
    """
    Description:
        The times of a trajectory's gripper operations the place measurements need (B work): the place into MG06 (the open with
        the PCB going from the jaws to MG06) - the loader's descent to L_PLACE starting, L_PLACE reached, the open command; the
        re-grip (the close on the PCB in MG06) - its command's end; the handover (the first close on the PCB on the cups) - its
        start and the vacuum off after it.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.

    Returns:
        (1) parameter [Tuple<float>]: The place's (descent, reached, open) times (s), or None.
        (2) parameter [float]: The re-grip's end (s), or None.
        (3) parameter [Tuple<float>]: The handover's (clamp, vacuum off) times (s), or None.
    """

    tr = trajectory
    command, location, y, z = np.asarray(tr.Gripper), np.asarray(tr.PCB_Location), np.asarray(tr.Loader_Y), np.asarray(tr.Loader_Z)
    jaws, mg06, cups = (CONST_PCB_LOCATIONS.index(n) for n in ('jaws', 'mg06', 'cups'))
    moving = np.flatnonzero(np.abs(np.diff(command)) > 0.0)
    place, regrip, handover = None, None, None
    for operation in np.split(moving, np.flatnonzero(np.diff(moving) > 1) + 1) if moving.size else []:
        first, last = int(operation[0]), int(operation[-1]) + 1
        if command[last] > command[first] and location[first] == jaws and location[last] == mg06 and place is None:
            reached = first
            while reached > 0 and z[reached - 1] == z[reached] and y[reached - 1] == y[reached]:
                reached -= 1
            down = reached
            while down > 0 and z[down - 1] > z[down] and y[down - 1] == y[down]:
                down -= 1
            place = (float(tr.T[down]), float(tr.T[reached]), float(tr.T[first]))
        elif command[last] < command[first] and location[first] == mg06 and regrip is None:
            regrip = float(tr.T[last])
        elif command[last] < command[first] and location[first] == cups and handover is None:
            off = next((float(t) for t, name in tr.Events if name == 'vacuum_off' and t >= tr.T[first]), None)
            handover = (float(tr.T[first]), off) if off is not None else None
    return place, regrip, handover


class Trajectory_Drive_Cls(object):
    """
    Description:
        The drive of a physics run from a program's trajectory (F11): the servo set-points of its joints, linear between
        the 4 ms samples, with the feed-forward of its stored joint speeds (analytic in MoveJ and MoveSpline); the loader's y
        and z and the gripper's commands, linear between samples (the jaws stop on the PCB in physics); its vacuum events;
        its steps as labels. The fork may touch the handle in the steps whose Allow_Contact has handle_bar; the PCB is placed
        at J_PARK's first sample, MG06 closed at the Test step's first sample; a step is FREE when free_steps has it
        (Configuration.Program_Parser.Free_Steps; by default when it runs a MoveSpline) and parallel when it starts a loader
        job.

    Initialization of the Class:
        Args:
            (1) trajectory [Trajectory_Str(object)]: The trajectory.
            (2) program [Program_Str(object)]: Its program (the steps' allowed contacts and lines).
            (3) free_steps [Set<string>]: Its free transfers, or None for its MoveSpline steps.

        Example:
            Initialization:
                # Assignment of the variables.
                drive = Trajectory_Drive_Cls(trajectory, program)

            Features:
                # Functions of the class (Physics_Run_Cls calls them).
                drive.Robot(t), drive.Loader(t), drive.Label(t)
    """

    def __init__(self, trajectory: Trajectory_Str, program: Program_Str, free_steps: tp.Optional[tp.Set[str]] = None) -> None:
        tr = trajectory
        self.Trajectory, self.T_End, self.Dt, self.N = tr, float(tr.T[-1]), CONST_DT, len(tr.T)
        self.__kv_kp = np.zeros(6)
        self.Events = [(float(t0), name.endswith('_on')) for t0, name in tr.Events if name.startswith('vacuum')]
        allowed = {s.Key for s in program.Segments if 'handle_bar' in s.Allow_Contact}
        self.__fork = {name for name in tr.Step_Names if name in allowed}
        lines = {name: [tr.Segment_Command[k] for k in range(len(tr.Segment_Command)) if tr.Step_Names[tr.Segment_Step[k]] == name]
                 for name in tr.Step_Names}
        self.__free = set(free_steps) if free_steps is not None else {name for name, texts in lines.items()
                                                                      if any(text.startswith('Robot.MoveSpline') for text in texts)}
        self.__parallel = {name for name, texts in lines.items() if any(text.startswith('Loader.Start') for text in texts)}
        park = next((k for k, text in enumerate(tr.Segment_Command) if text == 'Loader.Start(J_PARK)' or text.startswith('Loader.Run(J_PARK) >')),
                    None)
        test = next((k for k, name in enumerate(tr.Step_Names) if name.endswith('_Test')), None)
        # A trajectory without J_PARK or a Test step (a short probe) never places the PCB or closes MG06.
        self.Placed_Time = float(tr.T[min(int(tr.Segment_Start[park]), self.N - 1)]) if park is not None else float('inf')
        self.Closed_Time = float(tr.T[int(np.flatnonzero(tr.Step_Index == test)[0])]) if test is not None else float('inf')
        # B work: the place into MG06 (the descent's start, L_PLACE reached, the open command), the re-grip's clamp end, the
        # handover's clamp to its vacuum off; None in a probe without them.
        self.Place, self.Regrip_End, self.Handover = _Gripper_Times(tr)
        # The vacuum switch's checks (F11): each WaitDI on DI_VACUUM_OK - when its wait ends, its step, its line, the input's
        # response time, the cups it needs, the WaitDI's timeout.
        sim = program.Simulation
        self.Inputs = [(float(tr.T[min(int(tr.Segment_End[k]), self.N - 1)]), tr.Step_Names[int(tr.Segment_Step[k])], text,
                        sim.Signal_Response[CONST_VACUUM_OK], sim.Vacuum_OK_Min_Cups, float(re.search(r'timeout=([0-9.]+)', text).group(1)))
                       for k, text in enumerate(tr.Segment_Command) if text.startswith('Robot.WaitDI(%s' % CONST_VACUUM_OK)]

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

    def PCB_Location(self, t: float) -> str:
        """
        Description:
            Where the program has the PCB at t (one of Configuration.Program_Parser.CONST_PCB_LOCATIONS).

        Args:
            (1) t [float]: The time (s).

        Returns:
            (1) parameter [string]: The location.
        """

        return CONST_PCB_LOCATIONS[int(self.Trajectory.PCB_Location[self.__At(t)[0]])]

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
        One step of a physics run (a program's step): its time span, the largest tracking error, the
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
    # The closest robot - loader distance (the carriages and jaws, and in a free transfer the PCB where the program has it in the
    # jaws) and the closest carried PCB - loader distance (exact, every 4 ms, up to CONST_LOADER_CUT): (distance, time, pair), or
    # None when never measured (F11).
    #   Unit [Tuple<float, float, string>] (m, s, -)
    Robot_Loader: tp.Optional[tp.Tuple[float, float, str]] = None
    PCB_Loader: tp.Optional[tp.Tuple[float, float, str]] = None


class Physics_Run_Cls(object):
    """
    Description:
        The physics run of a drive (a program's trajectory) from the PCB in the input crate to the end of the
        cycle, one step at a time (Step), with the measurements of SPEC §4.6 (Result) and, per step, Step_Physics_Str. MuJoCo
        resets the state on instability; the run detects the time going backwards and raises instead of continuing (SPEC
        §4.7). The physics is the model's: the timestep and the solver as compiled.

    Initialization of the Class:
        Args:
            (1) model [mujoco.MjModel]: The compiled cell (Scene.Core.Build_Cell_Model).
            (2) drive [Trajectory_Drive_Cls(object)]: What drives it.
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
        self.track_err, self.slip, self.fork_force, self.loader_lag, self.fork_points = 0.0, 0.0, 0.0, 0.0, 0
        self.rel0, self.ph = None, ''
        # The lost PCB (F11): since when no cup touches it with the vacuum on; the last moment cups held it (time, cups, slip,
        # tilt, acceleration); the PCB's last position (the evidence of an instability, after which MuJoCo resets the state).
        self.Fault: tp.Optional[str] = None
        self.no_cups: tp.Optional[float] = None
        self.held: tp.Optional[tp.Tuple[float, int, float, float, float]] = None
        self.pcb_last = d.qpos[self.qpcb:self.qpcb + 3].copy()
        self.fc = np.zeros(6)
        # The fork on the bar (F11): the largest net force (its direction in the fork's frame, its time), squeeze, hinge load.
        self.fork_net, self.fork_net_direction, self.fork_net_t, self.fork_squeeze, self.handle_load = 0.0, np.zeros(3), 0.0, 0.0, 0.0
        self.handle_dof, self.handle_body = m.jnt_dofadr[m.joint('handle_hinge').id], m.body('mg06_handle').id
        # The robot's largest actuator torques and the model's limits (step 6).
        self.joint_torque = np.zeros(6)
        self.torque_limit = np.where(m.actuator_forcelimited[CONST_ROBOT_ACTUATORS] > 0, m.actuator_forcerange[CONST_ROBOT_ACTUATORS, 1], np.inf)
        # The vacuum switch (F11): its checks due; since when at least k cups (index k) have touched the PCB with the vacuum on;
        # the vacuum holds.
        self.Fault_Kind: tp.Optional[str] = None
        self.inputs = list(drive.Inputs)
        self.cups_since: tp.List[tp.Optional[float]] = [None] * (CONST_CUPS + 1)
        self.holds: tp.List[Hold_Str] = []
        # The per-step measurements (F11): the steps by label; the TCP's positions every 4 ms for its acceleration.
        self.steps: tp.Dict[str, Step_Physics_Str] = {}
        self.tcp_history: tp.List[np.ndarray] = []
        self.tcp_acceleration, self.tilt = 0.0, 0.0
        self.sub = max(1, int(round(CONST_SAMPLE_PERIOD / m.opt.timestep)))
        robot = {i for i in range(m.nbody) if m.body(i).name.startswith('aubo/')}
        loader = {m.body(n).id for n in CONST_LOADER_BODIES}
        self.robot_bodies, self.not_loader = robot, set(range(m.nbody)) - loader
        self.not_loader_pcb = self.not_loader - {self.PCB}
        self.collision = Robot_Collision_Cls(m, d)
        # The place into MG06 (B work): the holder's supports and side guides, the jaws, the PCB's free-joint speed address.
        base = m.body('mg06_base').id
        meshes = {g: m.mesh(m.geom_dataid[g]).name for g in range(m.ngeom)
                  if m.geom_bodyid[g] == base and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_MESH}
        holder = {name: g for g, name in meshes.items() if name.startswith('col_holder')}
        self.BASE, self.SUPPORTS = base, {holder[n]: n[4:] for n in CONST_HOLDER_SUPPORTS}
        self.GUIDES = {g: n[4:] for n, g in holder.items() if n not in CONST_HOLDER_SUPPORTS}
        self.JAWS = {m.body('jaw_L').id, m.body('jaw_R').id}
        self.PCB_GEOMS = [g for g in range(m.ngeom) if m.geom_bodyid[g] == self.PCB and m.geom_contype[g]]
        self.pcb_dof = m.jnt_dofadr[m.joint('pcb_free').id]
        self.place, self.jaw_gap_max, self.centring_x = Place_Physics_Str(), -math.inf, None

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
            if on:
                self.holds.append(Hold_Str(T_On=t))
            elif self.holds:
                self.holds[-1].T_Off = t
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
            hold = self.holds[-1] if self.holds else None
            if hold is not None and cups < hold.Min_Cups:
                hold.Min_Cups, hold.T_Min = cups, d.time
            if cups or self.rel0 is not None:
                R = d.xmat[TOOL].reshape(3, 3)
                rel = R.T @ (d.xpos[PCB] - d.xpos[TOOL])
                self.rel0 = rel if self.rel0 is None else self.rel0
                slip_now = np.linalg.norm(rel - self.rel0)
                self.slip = max(self.slip, slip_now)
        ff, net, normals, points = 0.0, np.zeros(3), np.zeros(3), 0
        record = self.steps.setdefault(ph, Step_Physics_Str(Name=ph, Start=t, Free=drive.Free(ph), Parallel=drive.Parallel(ph)))
        for i, c in enumerate(d.contact[:d.ncon]):
            k = self.classifier.Classify(c.geom1, c.geom2, drive.Fork_Allowed(ph))
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
                ff, points = ff + self.fc[0], points + 1
                # The contact frame's normal runs from geom 1 to geom 2, and its force acts on geom 2: the force on the bar.
                side = 1.0 if m.geom_bodyid[c.geom2] == self.handle_body else -1.0
                frame = c.frame.reshape(3, 3)
                net += side * frame.T @ self.fc[:3]
                normals += side * self.fc[0] * frame[0]
        if ff > self.fork_force:
            self.fork_force, self.fork_points = ff, points
        # The fork on the bar (F11): its net force, the squeeze between the slot's walls, the handle hinge's load.
        if float(np.linalg.norm(net)) > self.fork_net:
            self.fork_net, self.fork_net_t = float(np.linalg.norm(net)), d.time
            self.fork_net_direction = d.xmat[TOOL].reshape(3, 3).T @ net / self.fork_net
        self.fork_squeeze = max(self.fork_squeeze, 0.5 * (ff - float(np.linalg.norm(normals))))
        self.handle_load = max(self.handle_load, abs(float(d.qfrc_constraint[self.handle_dof])))
        self.joint_torque = np.maximum(self.joint_torque, np.abs(d.actuator_force[A_ROBOT]))
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
            self.__Loader_Distances(record)
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
        self.__Vacuum_Switch(t, cups)
        self.__Place(step)

    def __Place(self, step: int) -> None:
        """
        Description:
            The place into MG06 and the gripper operations around it (B work, items 7 and 9), when the drive has them: the
            side-guide clearance over the descent (every 4 ms), the jaw gap and the holder force at L_PLACE, the release (the
            jaws' last contact), the penetration while the PCB drops and settled, the seating Seat_Time after the release, the
            handover clamp's centring along the jaw axis, the re-grip's PCB-ledge gap.

        Args:
            (1) step [int]: The physics step's index.
        """

        m, d, cell, place, drive = self.Model, self.Data, self.Cell, self.place, self.Drive
        t = d.time
        if drive.Handover is not None:
            axis = d.xaxis[m.joint('jaw_R').id]
            if self.centring_x is None and t >= drive.Handover[0] - 0.5 * CONST_DT:
                self.centring_x = float(d.xpos[self.PCB] @ axis)
            elif self.centring_x is not None and math.isnan(place.Centring) and t >= drive.Handover[1] - 0.5 * CONST_DT:
                place.Centring = float(d.xpos[self.PCB] @ axis) - self.centring_x
        if drive.Regrip_End is not None and math.isnan(place.Regrip_Gap) and t >= drive.Regrip_End + CONST_REGRIP_SETTLE:
            place.Regrip_Gap = Ledge_Gap(m, d)
        if drive.Place is None:
            return
        down, reached, opened = drive.Place
        seat_at = place.Release + cell.Seat_Time
        if t < down or t > seat_at + CONST_SETTLED_WINDOW:
            return
        force, deepest, jaw_contact, others, supports = 0.0, 0.0, False, [], []
        for i, c in enumerate(d.contact[:d.ncon]):
            g1, g2 = c.geom1, c.geom2
            if self.PCB not in (m.geom_bodyid[g1], m.geom_bodyid[g2]):
                continue
            other = g2 if m.geom_bodyid[g1] == self.PCB else g1
            if m.geom_bodyid[other] in self.JAWS:
                jaw_contact = True
                others.append(m.body(m.geom_bodyid[other]).name)
            elif m.geom_bodyid[other] == self.BASE:
                mujoco.mj_contactForce(m, d, i, self.fc)
                force, deepest = force + self.fc[0], min(deepest, c.dist)
                if other in self.SUPPORTS:
                    supports.append(self.SUPPORTS[other])
                elif other in self.GUIDES:
                    others.append(self.GUIDES[other])
        if t <= place.Release and step % self.sub == 0:
            ft = np.zeros(6)
            for g, name in self.GUIDES.items():
                x = min(mujoco.mj_geomDistance(m, d, p, g, CONST_LOADER_CUT, ft) for p in self.PCB_GEOMS)
                if x < place.Guide_Min:
                    place.Guide_Min, place.Guide_Pair = x, '%s at %.3f s' % (name, t)
        if reached <= t <= opened:
            gap = d.qpos[self.qjl] + d.qpos[self.qjr]
            self.jaw_gap_max = max(self.jaw_gap_max, gap)
            place.Jaw_Closing = max(place.Jaw_Closing, self.jaw_gap_max - gap)
            place.Holder_Force = max(place.Holder_Force, force)
        if t >= opened and math.isinf(place.Release) and not jaw_contact:
            place.Release, place.Release_Z = t, float(d.xpos[self.PCB][2])
            return
        if math.isinf(place.Release):
            return
        speed = float(np.linalg.norm(d.qvel[self.pcb_dof:self.pcb_dof + 3]))
        if speed >= cell.Seat_Speed:
            place.Settle_Time = t - place.Release
        elif math.isinf(place.Settle_Time):
            place.Settle_Time = 0.0
        if t <= seat_at:
            place.Transient = min(place.Transient, deepest)
        if not place.Supports and t >= seat_at - 0.5 * m.opt.timestep:
            ft, q = np.zeros(6), d.qpos[self.qpcb + 3:self.qpcb + 7]
            place.Supports, place.Other_Contacts = sorted(set(supports)), sorted(set(others))
            place.Support_Gaps = {name: min(mujoco.mj_geomDistance(m, d, p, g, CONST_LOADER_CUT, ft) for p in self.PCB_GEOMS)
                                  for g, name in self.SUPPORTS.items()}
            place.Speed, place.Seated_Z = speed, float(d.xpos[self.PCB][2])
            place.Tilt = 2 * math.acos(min(1.0, abs(float(np.dot(q, self.Meta.Quat)))))
            place.Drop = place.Release_Z - place.Seated_Z
        elif t > seat_at:
            place.Settled = min(place.Settled, deepest)

    def __Loader_Distances(self, record: 'Step_Physics_Str') -> None:
        """
        Description:
            The step's closest robot - loader and carried PCB - loader distances now (F11): the loader's carriages and jaws, with
            the PCB where the program has it in the jaws in a free transfer; the PCB carried where the program has it on the cups.

        Args:
            (1) record [Step_Physics_Str(object)]: The step's record.
        """

        d, location = self.Data, self.Drive.PCB_Location(self.Data.time)
        # A free transfer keeps its distance to the PCB in the jaws too (Clearance.Loader_Free); in a LOCKED step at the loader the
        # cups touch the PCB in the jaws by design (the cups_pcb group), so the robot's distance is to the carriages and jaws only.
        jaws = location == 'jaws' and record.Free
        for field_name, mine, others in (('Robot_Loader', self.robot_bodies, self.not_loader_pcb if jaws else self.not_loader),
                                         ('PCB_Loader', {self.PCB} if location == 'cups' else None, self.not_loader)):
            if mine is None:
                continue
            pairs = self.collision.Close_Pairs(mine, CONST_LOADER_CUT, d, exclude=others)
            found = (pairs[0][0], d.time, '%s / %s' % (pairs[0][1], pairs[0][2])) if pairs else (CONST_LOADER_CUT, d.time, '')
            if getattr(record, field_name) is None or found[0] < getattr(record, field_name)[0]:
                setattr(record, field_name, found)

    def __Vacuum_Switch(self, t: float, cups: tp.Optional[int]) -> None:
        """
        Description:
            The simulated vacuum switch (F11): DI_VACUUM_OK is true once at least Vacuum_OK_Min_Cups cups have touched the PCB
            with the vacuum on for its Signal_Response. At the end of each WaitDI on it (the trajectory's timeline, which
            physics follows) a false input stops the run (Fault): the WaitDI would time out.

        Args:
            (1) t [float]: The step's start time (s).
            (2) cups [int]: The cups touching the PCB with the vacuum on, or None with the vacuum off.
        """

        d, half = self.Data, 0.5 * self.Model.opt.timestep
        for k in range(1, CONST_CUPS + 1):
            touching = cups is not None and cups >= k
            self.cups_since[k] = (t if self.cups_since[k] is None else self.cups_since[k]) if touching else None
        while self.inputs and self.Fault is None and d.time >= self.inputs[0][0] - half:
            _, step, text, response, need, timeout = self.inputs.pop(0)
            since = self.cups_since[need]
            if since is None or d.time - since < response - half:
                held = 'for %.3f s' % (d.time - since) if since is not None else 'not now'
                self.Fault, self.Fault_Kind = ('vacuum not OK at t = %.3f s, step %s, line "%s": %s of %d cups touch the PCB (%s needs %d '
                                               'for %.1f s: %s); the WaitDI would time out after %.1f s.'
                                               % (d.time, step, text, cups or 0, CONST_CUPS, CONST_VACUUM_OK, need, response, held, timeout),
                                               'input')

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
            self.Fault_Kind = 'lost'

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
                                Loader_Lag=self.loader_lag, Fork_Force=self.fork_force, Fork_Points=self.fork_points,
                                Fork_Net_Force=self.fork_net,
                                Fork_Net_Direction=self.fork_net_direction, Fork_Net_T=self.fork_net_t, Fork_Squeeze=self.fork_squeeze,
                                Handle_Load=self.handle_load, Joint_Torque=self.joint_torque, Torque_Limit=self.torque_limit, Penetration=self.pen,
                                Contacts=self.worst, Lid_Open=self.Lid_Open,
                                Lid_End=d.qpos[self.qa], Handle_End=d.qpos[self.qh], Steps=list(self.steps.values()),
                                Fault=self.Fault, Fault_Kind=self.Fault_Kind, Holds=self.holds,
                                Place=self.place if self.Drive.Place is not None else None)


def Run_Physics(model: mujoco.MjModel, drive: tp.Any, pcb_meta: PCB_Meta_Str, cell: Cell_Configuration_Str,
                progress: tp.Optional[tp.Callable[[float, str], None]] = None) -> 'Cycle_Result_Str':
    """
    Description:
        The physics run of a drive to its end (Physics_Run_Cls).

    Args:
        (1) model [mujoco.MjModel]: The compiled cell.
        (2) drive [Trajectory_Drive_Cls(object)]: What drives it.
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


