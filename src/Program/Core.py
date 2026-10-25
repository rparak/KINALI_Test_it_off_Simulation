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
# File Name: ../Program/Core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Hashlib (Secure hashes)
import hashlib
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# OS (Operating system interfaces)
import os
# Custom Lib.:
#   ../Configuration/Core - the cell, the PCB poses, the files
from Configuration.Core import Cell_Configuration_Str, PCB_Meta_Str, CONST_CELL_YAML, CONST_FORK_PATH
#   ../Configuration/Const - the repository root
from Configuration.Const import CONST_ROOT
#   ../Configuration/Program_Parser - the program's intermediate representation, the PCB locations
from Configuration.Program_Parser import (CONST_PCB_LOCATIONS, Program_Str, Command_Str, CONST_CONSTANTS_YAML, CONST_SIMULATION_YAML,
                                          CONST_OPTIMIZATION_YAML, CONST_WRIST_2)
#   ../Scene/Core - the crates, the robot's joints
from Scene.Core import PCB_In_Crate, Crate_Holding, CONST_ROBOT_JOINTS
#   ../Kinematics/Core - FK, IK, the PCB held by the cups
from Kinematics.Core import Robot_Kinematics_Cls, Quaternion_To_Matrix, PCB_For_TCP, RPY_To_Matrix, Rotation_Angle
#   ../Keypoints/Core - the points' MG06 state, poses and targets, the press plate's coupling, the contact groups, the point check
from Keypoints.Core import (Fixture_State, Site_Id, Point_Pose, Base_Pose, Target_Pose, CONST_POSE_POSITION, CONST_POSE_ANGLE, CONST_TOOL_POSITION,
                            CONST_TOOL_ANGLE)
#   ../Trajectory/Core - the trajectory format
from Trajectory.Core import Trajectory_Str, CONST_DT
#   ../Planner/Core - a planned spline's time-optimal timing (MoveSpline)
from Planner.Core import Spline_Path_Cls, Time_Path
#   ../Program/Const - the constants the build and the check share
from Program.Const import CONST_LOADER_HOME

"""
Description:
    Initialization of constants.

    The program at work (SPEC §2 item 4, FEATURES.md F8): Build_Program turns a program into a trajectory sampled
    every 4 ms - the robot's motion laws (MoveJ, MoveL, the recorded fork path), the loader and gripper profiles, the
    vacuum events, and the world state they imply (where the PCB is, MG06's lid and handle, the jaws' real position).
    The check of a trajectory and posing the cell at a sample are Program/Check.py. Every robot point is a keypoint's
    stored Joints: IK runs only along a MoveL or a recorded path, seeded from the previous sample. It prints nothing.
"""
# MoveL and Path IK: the position tolerance per sample (the orientation's is 10x), tighter than the keypoint IK's
# 1e-5 m so the finite-difference speeds and accelerations of the samples are not IK noise (1e-5 m between samples
# would read as about 2 rad/s^2). Pure DLS (w_null = 0): the 6-joint arm has no null space, and the null-space pull
# to the seed holds the solution about 5e-9 m off, so the solver ran to its 200-iteration limit (build 33 s -> 1.2 s,
# the same samples to the tolerance; measured 2026-10-24).
#   Unit [float] (m)
CONST_IK_TOL = 1e-8
# A MoveL or a Path ends exactly on its target keypoint's stored Joints: the last IK sample may differ from them by at
# most this per joint (the same IK branch differs by the IK tolerance, about 1e-5 rad; another branch by radians).
#   Unit [float] (rad)
CONST_SNAP_JOINTS = 1e-3
# A MoveL whose sampled joint speed or acceleration exceeds a limit is stretched by the factor that meets it, times
# this, and resampled (at most CONST_STRETCH_TRIES times).
#   Unit [float] (-), [int]
CONST_STRETCH_MARGIN, CONST_STRETCH_TRIES = 1.01, 8
# The PCB rules: the cups are on the PCB (vacuum on) when the PCB the TCP would hold is within this of the PCB's pose
# (the 1.5 mm press plus 0.5 mm); the PCB is between the jaws when it is within this of the jaws' hold pose.
#   Unit [float] (m)
CONST_PCB_TOLERANCE = 0.002
# The cause of each simulated digital input: the output whose switch-on starts its response time (Simulation.yaml
# Signal_Response); a WaitDI waits what is left of that time, nothing when it has passed (F11).
#   Unit [Dict<string, string>]
CONST_INPUT_CAUSE = {'DI_VACUUM_OK': 'DO_VACUUM'}
# The step the simulation appends after the program's last segment (Simulation.yaml End_Settle).
#   Unit [string]
CONST_END_SETTLE = 'End_Settle'
# The trajectory's units, joint order and TCP, stored in its metadata (SPEC §17: controller-friendly).
#   Unit [Dict<string, Any>]
CONST_META = {'units': {'length': 'm', 'angle': 'rad', 'time': 's', 'speed': 'rad/s'},
              'joint_order': CONST_ROBOT_JOINTS,
              'tcp': 'aubo/tool/suction_tcp: the cup face, 260.6 mm from the flange along tool +Z (SPEC §4.3)'}


class Program_Error_Cls(ValueError):
    """
    Description:
        A program cannot be built (a MoveL that leaves its IK family or does not converge, a stale target, a PCB rule
        broken). The message names the step, the command and what to do; a script prints it verbatim as an [ERROR] line.
    """


@dataclass
class Build_Note_Str:
    """
    Description:
        A note of the build the report lists: a stretched MoveL, the snap onto a target's stored joints, a velocity
        step at a recorded path's join.
    """

    # The step and the command.
    #   Unit [string]
    Step: str = ''
    Command: str = ''
    # What happened, as a sentence.
    #   Unit [string]
    Text: str = ''


def Min_Jerk(s: np.ndarray) -> np.ndarray:
    """
    Description:
        The min-jerk profile 10 s^3 - 15 s^4 + 6 s^5 of a normalised time, clipped to [0, 1].

    Args:
        (1) s [Vector<float>]: Normalised time.

    Returns:
        (1) parameter [Vector<float>]: The fraction of the move done.
    """

    s = np.clip(s, 0.0, 1.0)
    return s**3 * (10 - 15 * s + 6 * s * s)


def Min_Jerk_Rate(s: np.ndarray) -> np.ndarray:
    """
    Description:
        The derivative of Min_Jerk by the normalised time: 30 s^2 (1 - s)^2.

    Args:
        (1) s [Vector<float>]: Normalised time.

    Returns:
        (1) parameter [Vector<float>]: d(fraction) / ds.
    """

    s = np.clip(s, 0.0, 1.0)
    return 30 * s * s * (1 - s) ** 2


def Movej_Duration(q0: np.ndarray, q1: np.ndarray, cell: Cell_Configuration_Str, speed: tp.Optional[float] = None) -> float:
    """
    Description:
        The duration of a min-jerk joint move (SPEC §4.10, the legacy planner's rule): T = max(Movej_Min_Duration,
        max_j 1.875 |dq_j| / v_max_j, max_j sqrt(5.774 |dq_j| / a_max_j)), v_max = the speed fraction (the MoveJ's v=, or
        Cell.yaml Motion.Speed_Fraction) x the datasheet speeds, a_max the acceleration limits; 1.875 and 5.774 are
        min-jerk's peak speed and acceleration per unit of travel.

    Args:
        (1 - 2) q0, q1 [Vector<float> 1x6]: The start and the end (rad).
        (3) cell [Cell_Configuration_Str(object)]: The motion limits.
        (4) speed [float]: The fraction of the datasheet joint speeds, or None for Speed_Fraction.

    Returns:
        (1) parameter [float]: T (s), before rounding to whole samples.
    """

    dq = np.abs(np.asarray(q1) - np.asarray(q0))
    v_max = (cell.Speed_Fraction if speed is None else speed) * np.array(cell.Datasheet_Speeds)
    return max(cell.Movej_Min_Duration, float(np.max(1.875 * dq / v_max)), float(np.max(np.sqrt(5.774 * dq / np.array(cell.Acceleration_Limits)))))


def Slerp(q0: np.ndarray, q1: np.ndarray, s: float) -> np.ndarray:
    """
    Description:
        Spherical linear interpolation of two unit quaternions (w first) along the shorter arc.

    Args:
        (1 - 2) q0, q1 [Vector<float> 1x4]: The quaternions.
        (3) s [float]: The fraction, 0 .. 1.

    Returns:
        (1) parameter [Vector<float> 1x4]: The interpolated unit quaternion.
    """

    q1 = -q1 if q0 @ q1 < 0 else q1
    dot = float(np.clip(q0 @ q1, -1.0, 1.0))
    if dot > 1 - 1e-12:
        q = q0 + s * (q1 - q0)
        return q / np.linalg.norm(q)
    angle = math.acos(dot)
    return (math.sin((1 - s) * angle) * q0 + math.sin(s * angle) * q1) / math.sin(angle)


def File_Hash(path: str) -> str:
    """
    Description:
        The sha256 of a file's bytes, for the trajectory's metadata (an input changed = another hash).

    Args:
        (1) path [string]: The file.

    Returns:
        (1) parameter [string]: The hex digest.
    """

    with open(path, 'rb') as handle:
        return hashlib.sha256(handle.read()).hexdigest()


def Stale_Trajectory(trajectory: Trajectory_Str, program_path: str) -> tp.List[str]:
    """
    Description:
        The input files that changed since a trajectory was built (their sha256 against its metadata): a stale trajectory is
        refused by the viewers and the physics run.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) program_path [string]: Its program file.

    Returns:
        (1) parameter [List<string>]: The changed inputs' names; empty when it is current.
    """

    built = trajectory.Meta.get('inputs_sha256', {})
    now = {'program': program_path, 'constants': CONST_CONSTANTS_YAML, 'simulation': CONST_SIMULATION_YAML, 'cell': CONST_CELL_YAML,
           'fork_path': CONST_FORK_PATH}
    if 'optimization' in built:
        now['optimization'] = CONST_OPTIMIZATION_YAML
    return [name for name, path in now.items() if built.get(name) != File_Hash(path)]


def Executed_Commands(program: Program_Str) -> tp.List[tp.Tuple[int, str, Command_Str, str]]:
    """
    Description:
        The commands as the cell runs them, each one a trajectory segment: every line of every segment in order, a Run's
        job lines in its place (named '<Run line> > <job line>'), then the simulation's End_Settle hold as a last step.

    Args:
        (1) program [Program_Str(object)]: The program.

    Returns:
        (1) parameter [List<Tuple<int, string, Command_Str, string>>]: (step index, step name, command, its text).
    """

    out = []
    for index, segment in enumerate(program.Segments):
        for command in segment.Commands:
            if command.Kind == 'Run':
                out += [(index, segment.Key, line, '%s > %s' % (command.Text, line.Text)) for line in command.Job]
            else:
                out.append((index, segment.Key, command, command.Text))
    settle = program.Simulation.End_Settle
    if settle > 0:
        out.append((len(program.Segments), CONST_END_SETTLE, Command_Str(Kind='Wait', Station='Simulation', Duration=settle,
                                                                         Text='%s(%g)' % (CONST_END_SETTLE, settle)),
                                                                         '%s(%g)' % (CONST_END_SETTLE, settle)))
    return out


def Step_Names(program: Program_Str) -> tp.List[str]:
    """
    Description:
        The trajectory's steps: the segments' keys, then End_Settle when the simulation holds at the end.

    Args:
        (1) program [Program_Str(object)]: The program.

    Returns:
        (1) parameter [List<string>]: The names.
    """

    return [s.Key for s in program.Segments] + ([CONST_END_SETTLE] if program.Simulation.End_Settle > 0 else [])


def Validate_Constants(program: Program_Str, kinematics: Robot_Kinematics_Cls, cell: Cell_Configuration_Str, pcb_meta: PCB_Meta_Str) -> tp.List[str]:
    """
    Description:
        The checks of Constants.yaml against the model (Program_Parser checks everything else): every tool's Flange_Offset
        against its model site (CONST_TOOL_POSITION, CONST_TOOL_ANGLE), every point's Pose against FK(Joints, Tool)
        (CONST_POSE_POSITION, CONST_POSE_ANGLE), every derived point (Reference, PCB_Hold) against its target within the
        stale tolerance (Cell.yaml Clearance.Stale_Position, Stale_Angle), every loader position within the model's axis
        ranges and every gripper command within the jaws' range. Each problem names Constants.yaml, its line and the constant.

    Args:
        (1) program [Program_Str(object)]: The program (its constants and simulation data).
        (2) kinematics [Robot_Kinematics_Cls(object)]: FK on the model.
        (3) cell [Cell_Configuration_Str(object)]: The base pose and the tolerances.
        (4) pcb_meta [PCB_Meta_Str(object)]: The PCB poses (the PCB-hold targets).

    Returns:
        (1) parameter [List<string>]: The problems; empty when every constant matches the model.
    """

    c, s, model = program.Constants, program.Simulation, kinematics.Model
    file, problems = os.path.relpath(c.Path, CONST_ROOT) if os.path.isabs(c.Path) else c.Path, []
    flange = kinematics.Site_Flange
    for name, tool in c.Tools.items():
        p_f, R_f = kinematics.Site_Pose(np.zeros(6), flange)
        p_t, R_t = kinematics.Site_Pose(np.zeros(6), model.site(s.Tool_Site[name]).id)
        position = float(np.linalg.norm(R_f.T @ (p_t - p_f) - tool.Flange_Offset[:3]))
        angle = Rotation_Angle(RPY_To_Matrix(tool.Flange_Offset[3:]), R_f.T @ R_t)
        if position > CONST_TOOL_POSITION or angle > CONST_TOOL_ANGLE:
            problems.append('%s:%d: Tools.%s.Flange_Offset: %.4f mm / %.4f deg from the model site %s' % (file, tool.Line, name, 1000 * position,
                                                                                                    math.degrees(angle), s.Tool_Site[name]))
    for name, point in c.Points.items():
        position, rotation = Base_Pose(*Point_Pose(point, kinematics, s), cell)
        error, angle = float(np.linalg.norm(position - point.Pose[:3])), Rotation_Angle(RPY_To_Matrix(point.Pose[3:]), rotation)
        if error > CONST_POSE_POSITION or angle > CONST_POSE_ANGLE:
            problems.append('%s:%d: Points.%s.Pose: %.4f mm / %.4f deg from FK(Joints, %s) (at most %.2f mm / %.3f deg); print the corrected '
                            'Pose with Example/Keypoints/resolve_keypoint.py --name %s' % (file, point.Line, name, 1000 * error, math.degrees(angle),
                                                                                         point.Tool, 1000 * CONST_POSE_POSITION,
                                                                                         math.degrees(CONST_POSE_ANGLE), name))
        target = Target_Pose(name, c, s, pcb_meta, kinematics)
        if target is not None:
            p, R = Point_Pose(point, kinematics, s)
            stale, angle = float(np.linalg.norm(p - target[0])), Rotation_Angle(target[1], R)
            if stale > cell.Stale_Position or angle > cell.Stale_Angle:
                problems.append('%s:%d: Points.%s.%s: the point is %.3f mm / %.3f deg from its target (at most %.3f mm / %.3f deg)'
                                % (file, point.Line, name, 'Reference' if point.Reference else 'PCB_Hold (Simulation.yaml)', 1000 * stale,
                                   math.degrees(angle), 1000 * cell.Stale_Position, math.degrees(cell.Stale_Angle)))
    for name, position in c.Loader.items():
        for axis, joint in (('Y', 'y_axis'), ('Z', 'z_axis')):
            low, high = model.jnt_range[model.joint(joint).id]
            if not low <= getattr(position, axis) <= high:
                problems.append('%s:%d: Loader_Positions.%s.%s: %g m is outside the model\'s %s range [%g, %g] m'
                                % (file, position.Line, name, axis, getattr(position, axis), joint, low, high))
    low, high = model.jnt_range[model.joint('jaw_L').id]
    for name, command in c.Gripper.items():
        if not low <= command.Jaw <= high:
            problems.append('%s:%d: Gripper_Commands.%s.Jaw: %g m is outside the jaws\' range [%g, %g] m' % (file, command.Line, name,
                                                                                                             command.Jaw, low, high))
    return problems


class _Builder_Cls(object):
    """
    Description:
        The state of a program build: the samples so far, the current robot joints, loader and gripper commands,
        vacuum, PCB and MG06 state, and the rules that change them. Build_Program drives it command by command.

    Initialization of the Class:
        Args:
            (1) program [Program_Str(object)]: The program (its constants and simulation data).
            (2) cell [Cell_Configuration_Str(object)]: The motion limits and tolerances.
            (3) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
            (4) kinematics [Robot_Kinematics_Cls(object)]: FK and IK.

        Example:
            Initialization:
                # Initialization of the class.
                Cls = _Builder_Cls(program, cell, pcb_meta, kinematics)

            Features:
                # One command after the other, then the trajectory.
                Cls.Movej(command, step_index, segment)
    """

    def __init__(self, program: Program_Str, cell: Cell_Configuration_Str, pcb_meta: PCB_Meta_Str, kinematics: Robot_Kinematics_Cls) -> None:
        # << PRIVATE >> #
        self.__constants, self.__simulation = program.Constants, program.Simulation
        self.__cell, self.__meta, self.__kinematics = cell, pcb_meta, kinematics
        constants, simulation = program.Constants, program.Simulation
        model = kinematics.Model
        self.__lid_open = float(model.jnt_range[model.joint('lid_hinge').id][0])
        # The recorded paths: CLOSE the rows from 1 to the last CLOSE row, OPEN the OPEN rows after the join (SPEC §4.9).
        self.__path_logs, self.__path_rows = {}, {}
        for name, path in constants.Paths.items():
            recorded = np.load(path.File)
            log, phases = recorded['log'], np.array([str(p) for p in recorded['phase']])
            rows = np.where(np.char.startswith(phases, path.Phase))[0]
            if path.Phase == 'CLOSE':
                rows = np.arange(1, rows[-1] + 1)
            else:
                join = next(i for i in rows if log[i, 3] <= cell.Closed_Handle - cell.Open_Join_Margin)
                rows = rows[rows > join]
            self.__path_logs[name], self.__path_rows[name] = log, rows
        self.__v_max = cell.Speed_Fraction * np.array(cell.Datasheet_Speeds)
        self.__a_max = np.array(cell.Acceleration_Limits)
        self.__datasheet = np.array(cell.Datasheet_Speeds)
        self.__grid_points = program.Optimized.Topp_Grid_Points if program.Optimized is not None else 0
        # The loader's lane: the job started (Loader.Start) and not yet finished, one sample per robot sample.
        self.__job: tp.Optional[tp.Iterator[None]] = None
        # The jaws' stop on the PCB (Simulation.yaml Jaw_Actual).
        self.__grip = max(simulation.Jaw_Actual.values())
        # The current state.
        self.q = np.array(constants.Points[simulation.Start_Robot].Joints)
        self.y, self.z = constants.Loader[simulation.Start_Loader].Y, constants.Loader[simulation.Start_Loader].Z
        self.grip_command = constants.Gripper[simulation.Start_Gripper].Jaw
        self.vacuum = simulation.Start_Vacuum
        self.location = simulation.Start_PCB
        # The time each digital output went on (a simulated input's cause, CONST_INPUT_CAUSE).
        self.outputs_on: tp.Dict[str, float] = {}
        # The bellows' compression when the vacuum went on, and the TCP's position then (Held_Pose).
        self.press, self.press_tcp = 0.0, np.zeros(3)
        self.lid, self.handle = (self.__lid_open, 0.0) if simulation.Start_MG06 == 'Open' else (cell.Closed_Lid, cell.Closed_Handle)
        self.pcb = self.__Rest_Pose(self.location)
        # The samples: sample 0 is the start state.
        self.samples: tp.Dict[str, tp.List[tp.Any]] = {k: [] for k in ('q', 'qd', 'y', 'z', 'grip', 'vac', 'step', 'seg', 'loc', 'pcb', 'lid',
                                                                          'handle', 'jaw')}
        self.analytic: tp.List[tp.Tuple[int, int]] = []
        self.events: tp.List[tp.Tuple[float, str]] = []
        self.notes: tp.List[Build_Note_Str] = []
        self.segments: tp.List[tp.Tuple[int, str, int, int]] = []
        self.Append(self.q, np.zeros(6), 0, 0)

    # << PRIVATE >> #
    def __Rest_Pose(self, location: str) -> np.ndarray:
        """
        Description:
            The PCB pose at a resting location: on a crate floor, in the jaws at the current loader position, placed in
            MG06; for 'cups', held by the cups at the current joints with the frozen relative pose.

        Args:
            (1) location [string]: The location.

        Returns:
            (1) parameter [Vector<float> 1x7]: Position (m) and quaternion (w first).
        """

        m = self.__meta
        if location == 'input_crate':
            return np.r_[PCB_In_Crate('crate_pick')]
        if location == 'output_crate':
            return np.r_[PCB_In_Crate('crate_place')]
        if location == 'jaws':
            return np.r_[self.Jaw_Hold(), m.Quat]
        if location == 'mg06':
            return np.r_[m.Place_Pos, m.Quat]
        return self.Held_Pose(self.q)

    def Jaw_Hold(self) -> np.ndarray:
        """
        Description:
            The PCB centre held in the jaws at the current loader position: the hold pose (loader home) moved with the
            loader's Y and Z (measured: the place pose is the hold pose moved by place - home, to 1e-5 m).

        Returns:
            (1) parameter [Vector<float> 1x3]: The position (m).
        """

        home = self.__constants.Loader[CONST_LOADER_HOME]
        return np.array(self.__meta.Hold_Pos) + [0.0, self.y - home.Y, self.z - home.Z]

    def Held_Pose(self, q: np.ndarray) -> np.ndarray:
        """
        Description:
            The pose of the PCB on the cups at joints q: the nominal held pose (the cup face, SPEC §4.4), set back along the
            tool axis by the bellows' compression left: the compression at vacuum on (1.5 mm at a press, 0 at the handover)
            less the TCP's travel since - the sprung bellows extend first, so the PCB stays on its support for the first
            1.5 mm of the lift, then hangs at the cup face (SPEC §4.10).

        Args:
            (1) q [Vector<float> 1x6]: The joints (rad).

        Returns:
            (1) parameter [Vector<float> 1x7]: Position (m) and quaternion (w first).
        """

        p, R = self.__kinematics.Site_Pose(q)
        position, quat = PCB_For_TCP(p, R)
        left = max(0.0, self.press - float(np.linalg.norm(p - self.press_tcp)))
        return np.r_[position - left * R[:, 2], quat]

    def Between_Jaws(self, pcb: np.ndarray) -> bool:
        """
        Description:
            Whether a PCB pose is the jaws' hold pose at the current loader position (within CONST_PCB_TOLERANCE).

        Args:
            (1) pcb [Vector<float> 1x7]: The PCB pose.

        Returns:
            (1) parameter [bool]: True when it is between the jaws.
        """

        return float(np.linalg.norm(pcb[:3] - self.Jaw_Hold())) <= CONST_PCB_TOLERANCE

    def Append(self, q: np.ndarray, qd: np.ndarray, step: int, segment: int) -> None:
        """
        Description:
            Append one sample: a started loader job (Loader.Start) first moves on by one sample - the loader runs in
            parallel with the robot's lines - then the sample is recorded (Record).

        Args:
            (1 - 2) q, qd [Vector<float> 1x6]: The joints (rad) and their speeds (rad/s).
            (3 - 4) step, segment [int]: The step and segment indices.
        """

        if self.__job is not None:
            try:
                next(self.__job)
            except StopIteration:
                self.__job = None
        self.Record(q, qd, step, segment)

    def Record(self, q: np.ndarray, qd: np.ndarray, step: int, segment: int) -> None:
        """
        Description:
            Record one sample with the current loader, gripper, vacuum, PCB and MG06 state; the PCB on the cups follows
            the joints, the jaws stand at grip while the PCB is between them.

        Args:
            (1 - 2) q, qd [Vector<float> 1x6]: The joints (rad) and their speeds (rad/s).
            (3 - 4) step, segment [int]: The step and segment indices.
        """

        if self.location == 'cups':
            self.pcb = self.Held_Pose(q)
        elif self.location == 'jaws':
            self.pcb = np.r_[self.Jaw_Hold(), self.__meta.Quat]
        # The posed jaws are the command limited by the part: never closer than grip while the PCB is between them - in the jaws,
        # on the cups at the handover or the take, or resting in MG06 while the jaws release it or close on it.
        between = self.location == 'jaws' or (self.location in ('cups', 'mg06') and self.Between_Jaws(self.pcb))
        jaw = max(self.grip_command, self.__grip) if between else self.grip_command
        for key, value in (('q', q), ('qd', qd), ('y', self.y), ('z', self.z), ('grip', self.grip_command), ('vac', int(self.vacuum)), ('step', step),
                           ('seg', segment), ('loc', CONST_PCB_LOCATIONS.index(self.location)), ('pcb', self.pcb), ('lid', self.lid),
                           ('handle', self.handle), ('jaw', jaw)):
            self.samples[key].append(np.array(value, dtype=float) if key in ('q', 'qd', 'pcb') else value)
        self.q = np.array(q)

    def Fixture(self, target: str) -> tp.Tuple[float, float, float, float]:
        """
        Description:
            MG06 over a motion: from its state at the motion's start to the Fixture of the motion's target point (Open,
            Closed or its recorded row), along the motion's time law - the handle settles as the fork lets it go
            (FEATURES.md F8.5).

        Args:
            (1) target [string]: The point.

        Returns:
            (1 - 4) parameter [float]: The lid and handle at the start and at the end (rad).
        """

        lid, handle, _ = Fixture_State(self.__simulation.Points[target], self.__cell, self.__kinematics.Model)
        return self.lid, self.handle, lid, handle

    def Move_Fixture(self, state: tp.Tuple[float, float, float, float], s: float) -> None:
        """
        Description:
            Set MG06 at the fraction s of a motion (Fixture's start and end states).

        Args:
            (1) state [Tuple<float>]: The lid and handle at the start and at the end (rad).
            (2) s [float]: The fraction of the motion done (its time law), 0 .. 1.
        """

        lid_0, handle_0, lid_1, handle_1 = state
        self.lid, self.handle = (lid_1, handle_1) if s >= 1.0 else (lid_0 + (lid_1 - lid_0) * s, handle_0 + (handle_1 - handle_0) * s)

    def Fail(self, step: str, command: str, text: str) -> None:
        """
        Description:
            Raise the build's error, naming the step and the command.

        Args:
            (1 - 2) step, command [string]: Where.
            (3) text [string]: What is wrong and what to do.
        """

        raise Program_Error_Cls('Step %s, %s: %s' % (step, command, text))

    def Movej(self, command: Command_Str, step: int, segment: int) -> None:
        """
        Description:
            A min-jerk joint move to a point's Joints (Movej_Duration at the command's speed fraction, rounded to whole
            samples), ending exactly on them (like a MoveL or a Path), the speeds analytic.

        Args:
            (1) command [Command_Str(object)]: The command.
            (2 - 3) step, segment [int]: The indices.
        """

        q0, q1 = self.q.copy(), np.array(command.Joints)
        n = max(1, int(round(Movej_Duration(q0, q1, self.__cell, command.Speed) / CONST_DT)))
        fixture = self.Fixture(command.Target)
        first = len(self.samples['q'])
        for i in range(1, n + 1):
            s = i / n
            self.Move_Fixture(fixture, float(Min_Jerk(s)))
            # The last sample is the point's stored joints exactly (q0 + (q1 - q0) x 1.0 may differ from q1 by a rounding).
            self.Append(q1 if i == n else q0 + (q1 - q0) * Min_Jerk(s), (q1 - q0) * Min_Jerk_Rate(s) / (n * CONST_DT), step, segment)
        self.analytic.append((first, len(self.samples['q'])))

    def Movel_Samples(self, target: str, speed: float, scale: float, step: str, text: str) -> tp.Tuple[tp.List[np.ndarray], float]:
        """
        Description:
            The joint samples of a MoveL: the site on a straight line to the target keypoint's site pose, the orientation
            slerped, a min-jerk time law over T = max(Movel_Min_Duration, 1.875 L / speed) x scale, IK per sample seeded
            from the previous sample; the IK family and wrist_2's margin kept.

        Args:
            (1) target [string]: The keypoint.
            (2) speed [float]: The peak speed of the site (m/s).
            (3) scale [float]: The stretch of T (1 = none).
            (4 - 5) step, text [string]: The step and the command, for the errors.

        Returns:
            (1) parameter [List<Vector<float> 1x6>]: The samples (rad), the last one the IK's, not yet snapped.
            (2) parameter [float]: The line's length (m).
        """

        point = self.__constants.Points[target]
        site = Site_Id(self.__kinematics.Model, self.__simulation, point.Tool)
        p0, R0 = self.__kinematics.Site_Pose(self.q, site)
        p1, R1 = self.__kinematics.Site_Pose(np.array(point.Joints), site)
        q0, q1 = np.zeros(4), np.zeros(4)
        mujoco.mju_mat2Quat(q0, R0.flatten())
        mujoco.mju_mat2Quat(q1, R1.flatten())
        length = float(np.linalg.norm(p1 - p0))
        n = max(1, int(round(max(self.__cell.Movel_Min_Duration, 1.875 * length / speed) * scale / CONST_DT)))
        sign, q, out = np.sign(self.q[CONST_WRIST_2]), self.q.copy(), []
        for i in range(1, n + 1):
            s = float(Min_Jerk(i / n))
            q, ok = self.__kinematics.Solve_IK(p0 + (p1 - p0) * s, Quaternion_To_Matrix(Slerp(q0, q1, s)), q, site=site, tol=CONST_IK_TOL,
                                               w_null=0.0)
            if not ok:
                self.Fail(step, text, 'IK did not converge at sample %d of %d; the line leaves the reachable space - add a via '
                                      'keypoint or use Movej.' % (i, n))
            if abs(q[CONST_WRIST_2]) < self.__cell.Family_Margin or np.sign(q[CONST_WRIST_2]) != sign:
                self.Fail(step, text, 'wrist_2 reaches %.2f deg at sample %d of %d: a MoveL may not cross the IK family switch '
                                      '(SPEC §4.7); use Movej.' % (math.degrees(q[CONST_WRIST_2]), i, n))
            out.append(q.copy())
        return out, length

    def Snap(self, last: np.ndarray, target: str, step: str, text: str) -> np.ndarray:
        """
        Description:
            The target keypoint's stored Joints in place of the last IK sample, when the IK sample reaches the target's
            pose within the stale tolerance on the same branch (CONST_SNAP_JOINTS); else a stale-keypoint error.

        Args:
            (1) last [Vector<float> 1x6]: The last IK sample (rad).
            (2) target [string]: The keypoint.
            (3 - 4) step, text [string]: For the error and the note.

        Returns:
            (1) parameter [Vector<float> 1x6]: The stored joints (rad).
        """

        point = self.__constants.Points[target]
        stored = np.array(point.Joints)
        site = Site_Id(self.__kinematics.Model, self.__simulation, point.Tool)
        p, R = self.__kinematics.Site_Pose(last, site)
        p1, R1 = self.__kinematics.Site_Pose(stored, site)
        position = float(np.linalg.norm(p - p1))
        angle = float(np.arccos(np.clip((np.trace(R1.T @ R) - 1) / 2, -1.0, 1.0)))
        jump = float(np.max(np.abs(last - stored)))
        if np.sign(last[CONST_WRIST_2]) != np.sign(stored[CONST_WRIST_2]):
            self.Fail(step, text, 'the line keeps wrist_2 at %.1f deg but %s is in the other IK family (wrist_2 %.1f deg): a MoveL cannot '
                                  'switch families (SPEC §4.7); use Movej.' % (math.degrees(last[CONST_WRIST_2]), target,
                                                                              math.degrees(stored[CONST_WRIST_2])))
        if position > self.__cell.Stale_Position or angle > self.__cell.Stale_Angle or jump > CONST_SNAP_JOINTS:
            self.Fail(step, text, 'the motion ends %.3f mm / %.4f deg / %.5f rad from the stored joints of %s (stale point?); check it with '
                                  'Example/Keypoints/check_keypoints.py --name %s' % (1000 * position, math.degrees(angle), jump, target, target))
        return stored

    def Movel(self, command: Command_Str, step_name: str, step: int, segment: int) -> None:
        """
        Description:
            A MoveL (Movel_Samples), ending exactly on the target's stored joints (Snap); stretched while a sampled joint
            speed or acceleration (finite differences, from rest to rest) exceeds its limit, each stretch noted.

        Args:
            (1) command [Command_Str(object)]: The command.
            (2) step_name [string]: The step.
            (3 - 4) step, segment [int]: The indices.
        """

        scale = 1.0
        for _ in range(CONST_STRETCH_TRIES):
            out, length = self.Movel_Samples(command.Target, command.Speed, scale, step_name, command.Text)
            out[-1] = self.Snap(out[-1], command.Target, step_name, command.Text)
            q = np.vstack([self.q] + out + [out[-1]])
            qd = np.diff(q, axis=0) / CONST_DT
            qdd = np.diff(qd, axis=0) / CONST_DT
            factor = max(float(np.max(np.abs(qd) / self.__v_max)), math.sqrt(float(np.max(np.abs(qdd) / self.__a_max))))
            if factor <= 1.0:
                break
            scale *= factor * CONST_STRETCH_MARGIN
        else:
            self.Fail(step_name, command.Text, 'the joint limits are still exceeded after %d stretches; lower the Speed.' % CONST_STRETCH_TRIES)
        if scale > 1.0:
            self.notes.append(Build_Note_Str(step_name, command.Text, 'stretched x%.3f to %.2f s (a joint speed or acceleration limit), '
                                             'line %.1f mm' % (scale, len(out) * CONST_DT, 1000 * length)))
        fixture = self.Fixture(command.Target)
        for i, q in enumerate(out):
            self.Move_Fixture(fixture, float(Min_Jerk((i + 1) / len(out))))
            self.Append(q, np.zeros(6), step, segment)

    def Path(self, command: Command_Str, step_name: str, step: int, segment: int) -> None:
        """
        Description:
            A recorded fork path (SPEC §4.9): flange IK per recorded row seeded from the previous row, the recorded times
            linearly interpolated onto the 4 ms grid, MG06 as recorded; it must start at its start keypoint and ends
            exactly on its end keypoint (Snap). A path that starts moving is noted (a velocity step at the join).

        Args:
            (1) command [Command_Str(object)]: The command.
            (2) step_name [string]: The step.
            (3 - 4) step, segment [int]: The indices.
        """

        rows, first_keypoint, last_keypoint = self.__path_rows[command.Target], command.Path_Start, command.Path_End
        log, kinematics = self.__path_logs[command.Target], self.__kinematics
        p, R = kinematics.Site_Pose(self.q, kinematics.Site_Flange)
        before = log[rows[0] - 1]
        offset = float(np.linalg.norm(p - before[4:7]))
        if offset > self.__cell.Stale_Position:
            self.Fail(step_name, command.Text, 'the robot is %.3f mm from the path start (the pose of %s); move there first.'
                                               % (1000 * offset, first_keypoint))
        q, qs = self.q.copy(), [self.q.copy()]
        for i in rows:
            q, ok = kinematics.Solve_IK(log[i, 4:7], Quaternion_To_Matrix(log[i, 7:11]), q, site=kinematics.Site_Flange, tol=CONST_IK_TOL, w_null=0.0)
            if not ok:
                self.Fail(step_name, command.Text, 'flange IK did not converge at recorded row %d.' % i)
            qs.append(q.copy())
        qs[-1] = self.Snap(qs[-1], last_keypoint, step_name, command.Text)
        times = log[np.r_[rows[0] - 1, rows], 0] - before[0]
        lids, handles = log[np.r_[rows[0] - 1, rows], 1], log[np.r_[rows[0] - 1, rows], 3]
        start_speed = float(np.linalg.norm(log[rows[0], 4:7] - before[4:7]) / (log[rows[0], 0] - before[0]))
        if start_speed > 1e-4:
            self.notes.append(Build_Note_Str(step_name, command.Text, 'the recorded path starts moving: the flange at %.1f mm/s at its first '
                                             'row (a velocity step from rest; step 5 reports the tracking error and the force on the bar)'
                                             % (1000 * start_speed)))
        qs = np.array(qs)
        n = int(round(times[-1] / CONST_DT))
        for i in range(1, n + 1):
            # The last sample is the end keypoint exactly (the recording ends within half a sample of the grid).
            t = times[-1] if i == n else i * CONST_DT
            self.lid, self.handle = float(np.interp(t, times, lids)), float(np.interp(t, times, handles))
            self.Append(qs[-1] if i == n else np.array([np.interp(t, times, qs[:, j]) for j in range(6)]), np.zeros(6), step, segment)

    def Hold(self, duration: float, step: int, segment: int) -> None:
        """
        Description:
            Everything holds for a duration (rounded to whole samples).

        Args:
            (1) duration [float]: The time (s).
            (2 - 3) step, segment [int]: The indices.
        """

        for _ in range(int(round(duration / CONST_DT))):
            self.Append(self.q, np.zeros(6), step, segment)

    def Line_Steps(self, command: Command_Str, step_name: str) -> tp.Iterator[None]:
        """
        Description:
            The samples of one loader job line, as a generator: before each sample it sets the loader's channels. A loader
            axis or the gripper command moves min-jerk over its duration from its current value; a Wait holds. The
            gripper's PCB rules apply at the line's start: clamping on the PCB in MG06 takes it into the jaws; opening at
            MG06 places it there; opening while the cups hold it releases the jaws only; opening with the PCB in the jaws
            alone would drop it.

        Args:
            (1) command [Command_Str(object)]: The job line.
            (2) step_name [string]: The step, for the errors.
        """

        if command.Kind == 'Wait':
            for _ in range(int(round(command.Duration / CONST_DT))):
                yield
            return
        if command.Kind in ('MoveY', 'MoveZ'):
            channel, target = ('y' if command.Axis == 'Y' else 'z'), command.Position
        else:
            channel, target = 'grip_command', command.Jaw
            closing = target < self.grip_command
            at_place = self.location == 'mg06' and float(np.linalg.norm(np.array(self.__meta.Place_Pos) - self.Jaw_Hold())) <= CONST_PCB_TOLERANCE
            if closing and at_place:
                self.location = 'jaws'
            elif not closing and self.location == 'jaws':
                if float(np.linalg.norm(np.array(self.__meta.Place_Pos) - self.Jaw_Hold())) > CONST_PCB_TOLERANCE:
                    self.Fail(step_name, command.Text, 'opening the jaws away from MG06 with the PCB only in the jaws would drop it.')
                self.location, self.pcb = 'mg06', self.__Rest_Pose('mg06')
            self.events.append((len(self.samples['q']) * CONST_DT - CONST_DT, 'gripper_' + command.Target[2:].lower()))
        start = getattr(self, channel)
        n = max(1, int(round(command.Duration / CONST_DT)))
        for i in range(1, n + 1):
            setattr(self, channel, target if i == n else start + (target - start) * float(Min_Jerk(i / n)))
            yield

    def Loader_Or_Gripper(self, command: Command_Str, step_name: str, step: int, segment: int) -> None:
        """
        Description:
            A loader axis or the gripper command of a job run as a whole (Run): its samples (Line_Steps) while the robot
            holds.

        Args:
            (1) command [Command_Str(object)]: The command.
            (2) step_name [string]: The step.
            (3 - 4) step, segment [int]: The indices.
        """

        for _ in self.Line_Steps(command, step_name):
            self.Append(self.q, np.zeros(6), step, segment)

    def Start_Job(self, command: Command_Str, step_name: str) -> None:
        """
        Description:
            Loader.Start: the job's lines run on the loader's lane from the next sample on, one sample per robot sample,
            while the robot's lines go on.

        Args:
            (1) command [Command_Str(object)]: The Start line.
            (2) step_name [string]: The step, for the errors.
        """

        def Job() -> tp.Iterator[None]:
            """
            Description:
                The job's samples, line after line.
            """

            for line in command.Job:
                yield from self.Line_Steps(line, step_name)

        self.__job = Job()

    def Wait_Job(self, step: int, segment: int) -> None:
        """
        Description:
            Loader.WaitDone: the robot holds until the started job has taken its last sample.

        Args:
            (1 - 2) step, segment [int]: The indices.
        """

        while self.__job is not None:
            try:
                next(self.__job)
            except StopIteration:
                self.__job = None
                break
            self.Record(self.q, np.zeros(6), step, segment)

    def Movespline(self, command: Command_Str, step_name: str, step: int, segment: int) -> None:
        """
        Description:
            A planned joint spline (MoveSpline, F9): the cubic spline through its waypoints (Planner.Spline_Path_Cls) timed
            time-optimally within its v= of the datasheet joint speeds and the acceleration limits, at rest at both ends,
            on the program's TOPP grid (Planner.Time_Path); the speeds analytic. It must start where the robot stands and
            ends exactly on its end point's Joints; MG06 moves to the end point's Fixture along the path.

        Args:
            (1) command [Command_Str(object)]: The command.
            (2) step_name [string]: The step.
            (3 - 4) step, segment [int]: The indices.
        """

        waypoints = np.array(command.Waypoints)
        if float(np.max(np.abs(self.q - waypoints[0]))) > 1e-9:
            self.Fail(step_name, command.Text, 'the robot is not at the spline\'s start %s (%.2e rad away).'
                                               % (command.Path_Start, float(np.max(np.abs(self.q - waypoints[0])))))
        path = Spline_Path_Cls(waypoints)
        timed = Time_Path(path, command.Speed * self.__datasheet, self.__a_max, self.__grid_points, CONST_DT)
        fixture = self.Fixture(command.Path_End)
        first = len(self.samples['q'])
        for i in range(1, len(timed.Q)):
            self.Move_Fixture(fixture, float(timed.S[i] / path.Length))
            self.Append(timed.Q[i], timed.QD[i], step, segment)
        self.analytic.append((first, len(self.samples['q'])))

    def Vacuum(self, command: Command_Str, step_name: str) -> None:
        """
        Description:
            The vacuum output (SetDO on DO_VACUUM: its actuator) and its PCB rules: on, with the cups on the PCB (in a crate
            or in the jaws), the PCB goes onto the cups with the bellows' compression measured along the tool axis
            (Held_Pose); off, the PCB clamped in the jaws stays in them, the PCB
            in a crate lies on its floor; off anywhere else would drop it.

        Args:
            (1) command [Command_Str(object)]: The command.
            (2) step_name [string]: The step.
        """

        on = command.Value == 'ON'
        if on and not self.vacuum and self.location in ('input_crate', 'jaws'):
            held = np.r_[PCB_For_TCP(*self.__kinematics.Site_Pose(self.q))]
            if float(np.linalg.norm(held[:3] - self.pcb[:3])) > CONST_PCB_TOLERANCE:
                self.Fail(step_name, command.Text, 'the cups are %.1f mm from the PCB (%s): nothing to pick.'
                                                   % (1000 * np.linalg.norm(held[:3] - self.pcb[:3]), self.location))
            p, R = self.__kinematics.Site_Pose(self.q)
            self.press, self.press_tcp = max(0.0, float((held[:3] - self.pcb[:3]) @ R[:, 2])), p
            self.location = 'cups'
        elif on and not self.vacuum:
            self.Fail(step_name, command.Text, "vacuum on with no PCB under the cups (the PCB is in %s)." % self.location)
        elif not on and self.vacuum and self.location == 'cups':
            crate = Crate_Holding(self.pcb[:3])
            if self.Between_Jaws(self.pcb) and self.grip_command < self.__grip:
                self.location, self.pcb = 'jaws', self.__Rest_Pose('jaws')
            elif crate is not None:
                location = 'input_crate' if crate == 'crate_pick' else 'output_crate'
                drop = float(np.linalg.norm(self.__Rest_Pose(location)[:3] - self.pcb[:3]))
                if drop > CONST_PCB_TOLERANCE:
                    self.Fail(step_name, command.Text, 'vacuum off %.1f mm from the PCB\'s place on the crate floor would drop it; move down '
                                                       'first.' % (1000 * drop))
                self.location, self.pcb = location, self.__Rest_Pose(location)
            else:
                self.Fail(step_name, command.Text, 'vacuum off in free space would drop the PCB.')
        self.vacuum = on
        self.events.append((len(self.samples['q']) * CONST_DT - CONST_DT, '%s_%s' % (command.Actuator, command.Value.lower())))
        if on:
            self.outputs_on.setdefault(command.Target, len(self.samples['q']) * CONST_DT - CONST_DT)
        else:
            self.outputs_on.pop(command.Target, None)

    def Wait_Input(self, command: Command_Str, step_name: str, step: int, segment: int) -> None:
        """
        Description:
            A WaitDI: the input comes its simulated response time (Simulation.yaml Signal_Response) after its cause went on
            (CONST_INPUT_CAUSE), so the wait holds what is left of that time - nothing when it has passed, e.g. a second
            WaitDI after the first. An input whose cause is off never comes: the wait would time out.

        Args:
            (1) command [Command_Str(object)]: The command.
            (2) step_name [string]: The step.
            (3 - 4) step, segment [int]: The indices.
        """

        cause = CONST_INPUT_CAUSE[command.Target]
        if cause not in self.outputs_on:
            self.Fail(step_name, command.Text, '%s never comes: %s is off; the wait would time out after %g s.' % (command.Target, cause,
                                                                                                                   command.Timeout))
        elapsed = len(self.samples['q']) * CONST_DT - CONST_DT - self.outputs_on[cause]
        self.Hold(max(0.0, command.Duration - elapsed), step, segment)


def Build_Program(program: Program_Str, cell: Cell_Configuration_Str, pcb_meta: PCB_Meta_Str, kinematics: Robot_Kinematics_Cls,
                  meta: tp.Dict[str, tp.Any]) -> tp.Tuple[Trajectory_Str, tp.List[Build_Note_Str]]:
    """
    Description:
        The trajectory of a program: its constants first checked against the model (Validate_Constants); sample 0 is the
        simulation's start state, then each executed command's samples in order (Executed_Commands; a SetDO takes no time,
        a WaitDI holds what is left of its simulated response (Wait_Input), a Run its job's lines), every motion ending at
        rest; the joint speeds analytic in MoveJ, central differences elsewhere. A Test requires MG06 settled closed with the PCB inside.

    Args:
        (1) program [Program_Str(object)]: The program (its constants and simulation data).
        (2) cell [Cell_Configuration_Str(object)]: The motion limits, tolerances and the test duration.
        (3) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (4) kinematics [Robot_Kinematics_Cls(object)]: FK and IK (on the planning model).
        (5) meta [Dict<string, Any>]: Metadata for the trajectory (the inputs' hashes, the git commit, the platform).

    Returns:
        (1) parameter [Trajectory_Str(object)]: The trajectory.
        (2) parameter [List<Build_Note_Str>]: The notes (stretches, the recorded path's velocity step).
    """

    problems = Validate_Constants(program, kinematics, cell, pcb_meta)
    if problems:
        raise Program_Error_Cls('The constants do not match the model (%d problem(s)); correct them:\n  %s' % (len(problems), '\n  '.join(problems)))
    b = _Builder_Cls(program, cell, pcb_meta, kinematics)
    executed = Executed_Commands(program)
    for step_index, step_name, command, text in executed:
        segment = len(b.segments)
        first = len(b.samples['q']) if segment else 0
        if command.Kind == 'MoveJ':
            b.Movej(command, step_index, segment)
        elif command.Kind == 'MoveL':
            b.Movel(command, step_name, step_index, segment)
        elif command.Kind == 'MovePath':
            b.Path(command, step_name, step_index, segment)
        elif command.Kind == 'MoveSpline':
            b.Movespline(command, step_name, step_index, segment)
        elif command.Kind in ('MoveY', 'MoveZ', 'Gripper'):
            b.Loader_Or_Gripper(command, step_name, step_index, segment)
        elif command.Kind == 'Start':
            b.Start_Job(command, step_name)
        elif command.Kind == 'WaitDone':
            b.Wait_Job(step_index, segment)
        elif command.Kind == 'SetDO':
            b.Vacuum(command, step_name)
        elif command.Kind == 'WaitDI':
            b.Wait_Input(command, step_name, step_index, segment)
        else:
            if command.Kind == 'Test' and (b.location != 'mg06' or b.lid != cell.Closed_Lid or b.handle != cell.Closed_Handle):
                b.Fail(step_name, text, 'the test needs the PCB in MG06 and MG06 settled closed (PCB in %s, handle %.2f deg).'
                                        % (b.location, math.degrees(b.handle)))
            b.Hold(command.Duration, step_index, segment)
        b.segments.append((step_index, text, first, len(b.samples['q'])))
    # The joint speeds: central differences, zero where the robot holds (sample 0, every non-motion command) and at the
    # end of every MoveL and Path (they end at rest); analytic in MoveJ.
    q = np.array(b.samples['q'])
    qd = np.gradient(q, CONST_DT, axis=0) if len(q) > 1 else np.zeros_like(q)
    qd[0] = 0.0
    for (_, text, first, last), (_, _, command, _) in zip(b.segments, executed):
        if command.Kind in ('MoveL', 'MovePath'):
            qd[last - 1] = 0.0
        elif command.Kind != 'MoveJ':
            qd[first:last] = 0.0
    analytic = np.array(b.samples['qd'])
    for first, last in b.analytic:
        qd[first:last] = analytic[first:last]
    n = len(q)
    trajectory = Trajectory_Str(T=np.arange(n) * CONST_DT, Robot_Q=q, Robot_QD=qd, Loader_Y=np.array(b.samples['y']),
                                Loader_Z=np.array(b.samples['z']),
                                Gripper=np.array(b.samples['grip']), Vacuum=np.array(b.samples['vac'], np.int8),
                                Step_Index=np.array(b.samples['step'], np.int32), Segment_Id=np.array(b.samples['seg'], np.int32),
                                PCB_Location=np.array(b.samples['loc'], np.int8), PCB_Pose=np.array(b.samples['pcb']),
                                MG06_Lid=np.array(b.samples['lid']), MG06_Handle=np.array(b.samples['handle']), Jaw=np.array(b.samples['jaw']),
                                Step_Names=Step_Names(program), Segment_Step=np.array([s[0] for s in b.segments], np.int32),
                                Segment_Command=[s[1] for s in b.segments], Segment_Start=np.array([s[2] for s in b.segments], np.int32),
                                Segment_End=np.array([s[3] for s in b.segments], np.int32), Events=[(round(t, 6), e) for t, e in b.events],
                                Meta=dict(CONST_META, program=program.Name, **meta))
    return trajectory, b.notes


