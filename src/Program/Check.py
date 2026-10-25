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
# File Name: ../Program/Check.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
import dataclasses
from dataclasses import dataclass, field
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.:
#   ../Configuration/Core - the cell, the PCB poses
from Configuration.Core import Cell_Configuration_Str, PCB_Meta_Str
#   ../Configuration/Program_Parser - the program's intermediate representation, the PCB locations
from Configuration.Program_Parser import CONST_PCB_LOCATIONS, Program_Str
#   ../Scene/Core - the crates, the robot's joints
from Scene.Core import Crate_Holding, CONST_ROBOT_JOINTS
#   ../Kinematics/Core - FK, IK, the PCB held by the cups
from Kinematics.Core import Robot_Kinematics_Cls
#   ../Keypoints/Core - the points' MG06 state, poses and targets, the press plate's coupling, the contact groups, the point check
from Keypoints.Core import Press_For_Handle, Contact_Group, Check_Keypoint, Keypoint_Check_Str, CONST_MOUNT_BODY, CONST_MOUNT_GEOM_PREFIX
#   ../Collision/Core - exact distances, the loader's bodies
from Collision.Core import Robot_Collision_Cls, Ledge_Gap, CONST_LOADER_BODIES
#   ../Scene/Core - the cup block
from Scene.Core import Cup_Block_Geom
#   ../Trajectory/Core - the trajectory format
from Trajectory.Core import Trajectory_Str, CONST_DT
#   ../Program/Core - the build: its notes, the executed commands, the step names
from Program.Core import Build_Note_Str, Executed_Commands, Step_Names
#   ../Program/Const - the constants the build and the check share
from Program.Const import CONST_LOADER_HOME

"""
Description:
    Initialization of constants.

    The check of a program's trajectory (SPEC §2 item 4, §7.9) on the planning model: per sample the exact distances, the
    contact groups, the joint and loader limits, the jaws, the handover and the re-grip, and its report; and what the
    check measures - the cell posed at a sample (also the playback's), the loader where its actuators put it, the
    robot-loader distance. It prints nothing.
"""
# The check reports every pair within this distance (the clearance decides the verdict), so a near pair outside the
# clearance is visible too.
#   Unit [float] (m)
CONST_REPORT_CUT = 0.01
# The MG06 lid and handle: the farthest point from their hinges, for the check's motion bound.
#   Unit [float] (m)
CONST_MG06_REACH = 0.5
# The check's speed and acceleration limits: a MoveJ may exceed them by its duration's rounding to whole 4 ms samples (the
# legacy rule: up to half a sample shorter, so speeds up to 1 + 0.5 / n, accelerations its square); finite differences
# of a sampled min-jerk may exceed the continuous peak by this fraction (measured at most 0.08 % on the default program).
#   Unit [float] (-)
CONST_DIFFERENCE_TOLERANCE = 1e-3
# The keypoint whose handover is checked for a press (owner's B1, SPEC §4.7; the ledge gap: Collision.Core.Ledge_Gap).
#   Unit [string]
CONST_HANDOVER_KEYPOINT = 'P_LOADER_PLACE'
# The loader's axes and their actuators in the model (the actuator model, Loader_Actual), and the distance up to which a
# robot-loader pair is measured (Robot_Loader_Distance).
#   Unit [Tuple<Tuple<string>>], [float] (m)
CONST_LOADER_AXES, CONST_LOADER_CUT = (('y_axis', 'y_servo'), ('z_axis', 'z_servo')), 0.05
# The gating rule's sampling of the robot's motion and the loader's job: every 5th sample (20 ms) and the last.
#   Unit [int] (samples)
CONST_GATING_STRIDE = 5


def Pose_Sample(kinematics: Robot_Kinematics_Cls, trajectory: Trajectory_Str, i: int, data: tp.Optional[mujoco.MjData] = None) -> None:
    """
    Description:
        Pose the whole cell at sample i of a trajectory (kinematics, no physics): the robot, the loader axes and the jaws'
        real position, the PCB's pose, MG06's lid, handle and press plate (the model's coupling); ends with mj_kinematics.

    Args:
        (1) kinematics [Robot_Kinematics_Cls(object)]: The model and the default data.
        (2) trajectory [Trajectory_Str(object)]: The trajectory.
        (3) i [int]: The sample.
        (4) data [mujoco.MjData]: The data to pose (default: the kinematics' data).
    """

    d = kinematics.Data if data is None else data
    model = kinematics.Model
    handle = float(trajectory.MG06_Handle[i])
    kinematics.Set_Scene(d, lid=float(trajectory.MG06_Lid[i]), handle=handle, press=Press_For_Handle(model, handle), y=float(trajectory.Loader_Y[i]),
                         z=float(trajectory.Loader_Z[i]), jaw=float(trajectory.Jaw[i]), pcb=trajectory.PCB_Pose[i])
    d.qpos[kinematics.Q_Address] = trajectory.Robot_Q[i]
    mujoco.mj_kinematics(model, d)


def Jaw_Posing_Faults(trajectory: Trajectory_Str, grip: float) -> tp.List[tp.Tuple[int, str]]:
    """
    Description:
        The posed jaws against their rule: during every gripper operation (the command ramping) the posed jaw opens
        monotonically on an open and closes monotonically on a close, and it is never closer than grip while the PCB is in the
        jaws.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) grip [float]: Where the jaws stop on the PCB (m; Simulation.yaml Jaw_Actual).

    Returns:
        (1) parameter [List<Tuple<int, string>>]: The sample and a sentence per fault; empty when the posing follows the rule.
    """

    command, jaw, out = np.asarray(trajectory.Gripper), np.asarray(trajectory.Jaw), []
    moving = np.flatnonzero(np.abs(np.diff(command)) > 0.0)
    for operation in np.split(moving, np.flatnonzero(np.diff(moving) > 1) + 1) if moving.size else []:
        first, last = int(operation[0]), int(operation[-1]) + 1
        opening = command[last] > command[first]
        steps = np.diff(jaw[first:last + 1])
        if np.any(steps < 0.0) if opening else np.any(steps > 0.0):
            worst = int(np.argmin(steps) if opening else np.argmax(steps)) + first
            out.append((worst + 1, 'the posed jaws %s by %.2f mm at %.3f s while the gripper %s (%.3f-%.3f s)' % (
                'close' if opening else 'open', 1000 * abs(jaw[worst + 1] - jaw[worst]), trajectory.T[worst + 1],
                'opens' if opening else 'closes', trajectory.T[first], trajectory.T[last])))
    inside = np.flatnonzero((np.asarray(trajectory.PCB_Location) == CONST_PCB_LOCATIONS.index('jaws')) & (jaw < grip - 1e-12))
    if inside.size:
        out.append((int(inside[0]), 'the posed jaws are %.2f mm closer than grip with the PCB in them at %.3f s' % (
            1000 * (grip - jaw[inside].min()), trajectory.T[inside[0]])))
    return out


def Jaw_Speed_Faults(trajectory: Trajectory_Str, limit: float) -> tp.List[tp.Tuple[int, str]]:
    """
    Description:
        The gripper operations whose command ramps faster than the gripper's largest jaw speed (Cell.yaml Motion.Jaw_Speed_Max,
        the SCHUNK EGI 040-EI data sheet): the peak of the sampled command's finite differences.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) limit [float]: The largest jaw speed (m/s per jaw).

    Returns:
        (1) parameter [List<Tuple<int, string>>]: The sample and a sentence per fault; empty when every ramp is within it.
    """

    command, out = np.asarray(trajectory.Gripper), []
    speed = np.abs(np.diff(command)) / CONST_DT
    moving = np.flatnonzero(speed > 0.0)
    for operation in np.split(moving, np.flatnonzero(np.diff(moving) > 1) + 1) if moving.size else []:
        peak = int(operation[np.argmax(speed[operation])])
        if speed[peak] > limit * (1 + CONST_DIFFERENCE_TOLERANCE):
            out.append((peak + 1, 'the gripper command ramps at %.1f mm/s at %.3f s, faster than Motion.Jaw_Speed_Max %.0f mm/s (%.3f-%.3f s)'
                        % (1000 * speed[peak], trajectory.T[peak + 1], 1000 * limit, trajectory.T[operation[0]], trajectory.T[operation[-1] + 1])))
    return out


def Loader_Actual(trajectory: Trajectory_Str, model: mujoco.MjModel) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        Where the loader's axes are along a trajectory (F11): their commands through the model's actuators - the filter
        (act' = (u - act) / tau), the position servo (kp (act - q) - kv q'), the axis' damping, its mass and its gravity load at
        the start - stepped at the model's timestep with the command linear between samples. Against the physics run of the
        optimized program (2026-10-25) within 0.17 mm (y) and 1.04 mm (z, its load changes with the PCB); the command alone
        is up to 22.8 mm off, a first-order filter alone 9.6 mm.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) model [mujoco.MjModel]: The model (its actuators, masses and timestep).

    Returns:
        (1 - 2) parameter [Vector<float> n]: y, z per sample (m).
    """

    data = mujoco.MjData(model)
    data.qpos[model.jnt_qposadr[model.joint('y_axis').id]] = trajectory.Loader_Y[0]
    data.qpos[model.jnt_qposadr[model.joint('z_axis').id]] = trajectory.Loader_Z[0]
    mujoco.mj_forward(model, data)
    h = model.opt.timestep
    sub = int(round(CONST_DT / h))
    out = []
    for (joint, actuator), command in zip(CONST_LOADER_AXES, (trajectory.Loader_Y, trajectory.Loader_Z)):
        dof, a = model.jnt_dofadr[model.joint(joint).id], model.actuator(actuator).id
        unit, row = np.zeros(model.nv), np.zeros(model.nv)
        unit[dof] = 1.0
        mujoco.mj_mulM(model, data, row, unit)
        mass, load = row[dof], data.qfrc_bias[dof]
        kp, kv, tau = model.actuator_gainprm[a][0], -model.actuator_biasprm[a][2], model.actuator_dynprm[a][0]
        damping = model.dof_damping[dof]
        q, v, act = float(command[0]), 0.0, float(command[0])
        axis = np.empty(len(command))
        axis[0] = q
        for i in range(1, len(command)):
            for k in range(sub):
                u = command[i - 1] + (command[i] - command[i - 1]) * (k / sub)
                force = kp * (act - q) - (kv + damping) * v - load
                act += h * (u - act) / tau
                v += h * force / mass
                q += h * v
            axis[i] = q
        out.append(axis)
    return out[0], out[1]


def Loader_Actual_Trajectory(trajectory: Trajectory_Str, model: mujoco.MjModel) -> Trajectory_Str:
    """
    Description:
        The trajectory with the loader where its actuators put it (Loader_Actual) and the PCB in its jaws moved with it (F11).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) model [mujoco.MjModel]: The model.

    Returns:
        (1) parameter [Trajectory_Str(object)]: A copy with Loader_Y, Loader_Z and PCB_Pose replaced.
    """

    y, z = Loader_Actual(trajectory, model)
    pcb = trajectory.PCB_Pose.copy()
    jaws = trajectory.PCB_Location == CONST_PCB_LOCATIONS.index('jaws')
    pcb[jaws, 1] += (y - trajectory.Loader_Y)[jaws]
    pcb[jaws, 2] += (z - trajectory.Loader_Z)[jaws]
    return dataclasses.replace(trajectory, Loader_Y=y, Loader_Z=z, PCB_Pose=pcb)


def Robot_Loader_Distance(trajectory: Trajectory_Str, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls,
                          samples: tp.Iterable[int]) -> tp.Tuple[float, int, str]:
    """
    Description:
        The smallest exact distance between the robot (with the PCB it carries) and the loader (its carriages and jaws, with
        the PCB they hold) over samples (I8, Clearance.Loader_Free).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory (Loader_Actual_Trajectory for the loader's real position).
        (2) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (3) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (4) samples [List<int>]: The samples.

    Returns:
        (1) parameter [float]: The distance (m), CONST_LOADER_CUT when nothing is closer.
        (2) parameter [int]: Its sample (-1 when none).
        (3) parameter [string]: The pair.
    """

    model, data = kinematics.Model, kinematics.Data
    robot, pcb = collision.Robot_Bodies, model.body('pcb').id
    loader = {model.body(name).id for name in CONST_LOADER_BODIES}
    best = (CONST_LOADER_CUT, -1, '')
    for i in samples:
        Pose_Sample(kinematics, trajectory, i)
        location = CONST_PCB_LOCATIONS[trajectory.PCB_Location[i]]
        mine = robot | ({pcb} if location == 'cups' else set())
        theirs = loader | ({pcb} if location == 'jaws' else set())
        others = set(range(model.nbody)) - theirs
        for distance, body_1, body_2, _, _ in collision.Close_Pairs(mine, CONST_LOADER_CUT, data, exclude=others):
            if distance < best[0]:
                best = (distance, i, '%s / %s' % (body_1, body_2))
    return best


def _Boxes(model: mujoco.MjModel, data: mujoco.MjData, geoms: np.ndarray) -> np.ndarray:
    """
    Description:
        The world-aligned boxes of geoms in the data's kinematics (each geom's own box, geom_aabb, rotated).

    Args:
        (1) model [mujoco.MjModel]: The model.
        (2) data [mujoco.MjData]: Its data, after mj_kinematics.
        (3) geoms [Vector<int> k]: The geoms.

    Returns:
        (1) parameter [Matrix<float> kx2x3]: Per geom its lower and upper corner (m).
    """

    R = data.geom_xmat[geoms].reshape(-1, 3, 3)
    centre = data.geom_xpos[geoms] + np.einsum('kij,kj->ki', R, model.geom_aabb[geoms, :3])
    half = np.einsum('kij,kj->ki', np.abs(R), model.geom_aabb[geoms, 3:])
    return np.stack([centre - half, centre + half], axis=1)


def Swept_Pairs(trajectory: Trajectory_Str, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, robot_samples: tp.Sequence[int],
                loader_samples: tp.Sequence[int], cut: float) -> tp.Tuple[np.ndarray, tp.Dict[tp.Tuple[int, int], str]]:
    """
    Description:
        The smallest exact distance between the robot (with the PCB its cups carry) at each robot sample and the loader (with the
        PCB in its jaws) at each loader sample - the robot against the volume the loader sweeps and the loader against the
        volume the robot sweeps, whatever the timing between them (the gating rule, F15, F16). A pair of samples is measured
        only where a robot geom's box comes within cut of a loader geom's box.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory (Loader_Actual_Trajectory for the loader's real position).
        (2) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (3) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (4) robot_samples [List<int>]: The robot's samples.
        (5) loader_samples [List<int>]: The loader's samples.
        (6) cut [float]: The distance up to which pairs are measured (m).

    Returns:
        (1) parameter [Matrix<float> RxL]: The distances (m; cut where nothing is closer).
        (2) parameter [Dict<Tuple<int, int>, string>]: The closest pair of bodies by (robot, loader) index, where measured.
    """

    tr, model, data = trajectory, kinematics.Model, kinematics.Data
    address = kinematics.Scene_Address
    pcb = model.body('pcb').id
    robot, loader = collision.Robot_Bodies, {model.body(name).id for name in CONST_LOADER_BODIES}
    collidable = (model.geom_contype > 0) | (model.geom_conaffinity > 0)
    geoms = lambda bodies: np.flatnonzero(collidable & np.isin(model.geom_bodyid, list(bodies)))
    jaws, cups = CONST_PCB_LOCATIONS.index('jaws'), CONST_PCB_LOCATIONS.index('cups')
    loader_boxes = []
    for j in loader_samples:
        Pose_Sample(kinematics, tr, int(j))
        loader_boxes.append(_Boxes(model, data, geoms(loader | ({pcb} if tr.PCB_Location[j] == jaws else set()))))
    distances, pairs = np.full((len(robot_samples), len(loader_samples)), cut), {}
    for r, i in enumerate(robot_samples):
        Pose_Sample(kinematics, tr, int(i))
        mine = robot | ({pcb} if tr.PCB_Location[i] == cups else set())
        boxes = _Boxes(model, data, geoms(mine))
        for l, (j, theirs_boxes) in enumerate(zip(loader_samples, loader_boxes)):
            near = np.all((boxes[:, None, 0] - cut <= theirs_boxes[None, :, 1]) & (boxes[:, None, 1] + cut >= theirs_boxes[None, :, 0]), axis=2)
            if not near.any():
                continue
            Pose_Sample(kinematics, tr, int(i))
            data.qpos[address['y_axis']], data.qpos[address['z_axis']] = tr.Loader_Y[j], tr.Loader_Z[j]
            data.qpos[address['jaw_L']] = data.qpos[address['jaw_R']] = tr.Jaw[j]
            if tr.PCB_Location[j] == jaws:
                data.qpos[address['pcb_free']:address['pcb_free'] + 7] = tr.PCB_Pose[j]
            mujoco.mj_kinematics(model, data)
            theirs = loader | ({pcb} if tr.PCB_Location[j] == jaws else set())
            for distance, body_1, body_2, _, _ in collision.Close_Pairs(mine, cut, data, exclude=set(range(model.nbody)) - theirs)[:1]:
                distances[r, l], pairs[(r, l)] = distance, '%s / %s' % (body_1, body_2)
    return distances, pairs


@dataclass
class Gating_Fault_Str:
    """
    Description:
        A parallel part whose separation rests on timing (the gating rule, F15, F16): while a loader job may run (from its
        Loader.Start to the end of its Loader.WaitDone) the robot - moving or holding - comes closer than
        Clearance.Loader_Free to a pose the job sweeps at another time. 'robot enters' where the loader has been (a slower
        loader would meet it), or 'loader enters' where the robot has been (a slower robot - a speed override - would meet
        it). Either is a fault.
    """

    # The step, the robot's line where it first is at such a pair (a motion, or the line it holds at), the job, 'robot enters'
    # or 'loader enters'.
    #   Unit [string]
    Step: str = ''
    Motion: str = ''
    Job: str = ''
    Kind: str = ''
    # The closest distance, the robot's and the loader's times then, the pair; the smallest time between the two objects at a
    # pair closer than Loader_Free (the later one's lead); the robot's first and last time at such a pair.
    #   Unit [float] (m), [float] (s), [float] (s), [string], [float] (s), [Tuple<float>] (s)
    Distance: float = 0.0
    Robot_T: float = 0.0
    Loader_T: float = 0.0
    Pair: str = ''
    Lead: float = math.nan
    Robot_Span: tp.Tuple[float, float] = (math.nan, math.nan)


def Gating_Faults(trajectory: Trajectory_Str, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls,
                  cell: Cell_Configuration_Str) -> tp.List[Gating_Fault_Str]:
    """
    Description:
        The gating rule (owner 2026-10-25, F15; symmetric, F16): while a parallel loader job may run (from its Loader.Start to
        the end of its Loader.WaitDone) neither object may come where the other has been or will be - the robot (moving or
        holding, with a carried PCB) at least Clearance.Loader_Free from every pose the job sweeps (the loader where its
        actuators put it), every CONST_GATING_STRIDE samples and the last (Swept_Pairs, the robot's repeated poses measured
        once). A pair closer than that rests on timing: on the loader's speed when the robot comes later ('robot enters'), on
        the robot's when the loader comes later ('loader enters'). The order must come from the program: the robot's motion
        out of the way before the job's Loader.Start, or the job's Loader.WaitDone (or an input from the loader) before the
        robot's motion in.

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (3) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (4) cell [Cell_Configuration_Str(object)]: Clearance.Loader_Free.

    Returns:
        (1) parameter [List<Gating_Fault_Str(object)>]: The faults, per job and kind.
    """

    tr = trajectory
    texts = tr.Segment_Command
    starts = [(k, text[len('Loader.Start('):-1]) for k, text in enumerate(texts) if text.startswith('Loader.Start(')]
    if not starts:
        return []
    actual = Loader_Actual_Trajectory(tr, kinematics.Model)
    cups = CONST_PCB_LOCATIONS.index('cups')
    out = []
    for k, job in starts:
        done = texts.index('Loader.WaitDone(%s)' % job, k)
        first, last = int(tr.Segment_Start[k]), int(tr.Segment_End[done]) - 1
        samples = np.array(sorted(set(range(first, last + 1, CONST_GATING_STRIDE)) | {last}))
        poses: tp.Dict[tp.Tuple[bytes, bool], int] = {}
        rows = np.array([poses.setdefault((tr.Robot_Q[i].tobytes(), bool(tr.PCB_Location[i] == cups)), len(poses)) for i in samples])
        unique = [int(samples[np.flatnonzero(rows == u)[0]]) for u in range(len(poses))]
        distances, pairs = Swept_Pairs(actual, kinematics, collision, unique, samples, cell.Loader_Free)
        D = distances[rows]
        T = tr.T[samples]
        for kind, side in (('robot enters', T[None, :] <= T[:, None]), ('loader enters', T[None, :] > T[:, None])):
            close = (D < cell.Loader_Free) & side
            if not close.any():
                continue
            r, l = np.unravel_index(int(np.argmin(np.where(close, D, np.inf))), D.shape)
            inside = np.flatnonzero(close.any(axis=1))
            segment = min(int(np.searchsorted(tr.Segment_End, int(samples[inside[0]]), side='right')), len(texts) - 1)
            robot_times = T[inside]
            out.append(Gating_Fault_Str(Step=tr.Step_Names[tr.Segment_Step[segment]], Motion=texts[segment], Job=job, Kind=kind,
                                        Distance=float(D[r, l]), Robot_T=float(T[r]), Loader_T=float(T[l]), Pair=pairs[(int(rows[r]), int(l))],
                                        Lead=float(np.min(np.abs(T[None, :] - T[:, None])[close])),
                                        Robot_Span=(float(robot_times[0]), float(robot_times[-1]))))
    return out


@dataclass
class Group_Distance_Str:
    """
    Description:
        The closest pair of a contact group (or of the pairs no allowed group covers) in a step, and where it occurs.
    """

    # The group ('' for the pairs no allowed group covers).
    #   Unit [string]
    Group: str = ''
    # The exact distance (negative = overlap), the sample and its time, the pair.
    #   Unit [float] (m), [int], [float] (s), [string]
    Distance: float = 0.0
    Sample: int = 0
    Time: float = 0.0
    Pair: str = ''


@dataclass
class Step_Check_Str:
    """
    Description:
        The check of one program step: the distances per allowed group and the closest pair no allowed group covers (each
        with its sample), the peak joint speed and acceleration as fractions of the limits, the failures, the verdict.
    """

    # The step, its start and its duration.
    #   Unit [string], [float] (s), [float] (s)
    Name: str = ''
    Start: float = 0.0
    Duration: float = 0.0
    # The step's declared groups: the closest pair of each (None when no pair of the group came within CONST_REPORT_CUT).
    #   Unit [Dict<string, Group_Distance_Str>]
    Allowed: tp.Dict[str, tp.Optional[Group_Distance_Str]] = field(default_factory=dict)
    # The closest pair no declared group covers (None when nothing came within CONST_REPORT_CUT).
    #   Unit [Group_Distance_Str(object)]
    Closest: tp.Optional[Group_Distance_Str] = None
    # The peak joint speed and acceleration of its MoveJ / MoveL samples, and of its recorded paths, as fractions of the
    # Cell.yaml limits (0 when it has none).
    #   Unit [float] (-)
    Peak_Speed: float = 0.0
    Peak_Acceleration: float = 0.0
    Path_Speed: float = 0.0
    Path_Acceleration: float = 0.0
    # The samples measured exactly.
    #   Unit [int]
    Checked: int = 0
    # A free transfer's closest pair to the loader, the loader at its actuator model's position (F11), or None.
    #   Unit [Group_Distance_Str(object)]
    Loader: tp.Optional[Group_Distance_Str] = None
    # Every failed rule, as a sentence.
    #   Unit [List<string>]
    Failures: tp.List[str] = field(default_factory=list)
    # PASS or FAIL.
    #   Unit [string]
    Verdict: str = ''


@dataclass
class Handover_Check_Str:
    """
    Description:
        The no-press check of the handover (owner's B1, SPEC §4.7): the PCB-ledge gap with the jaws at grip over the MoveL into
        P_LOADER_PLACE (its minimum: the gap at the point plus the PCB's height above it along the plate normal), at P_LOADER_PLACE, and
        while the jaws clamp the PCB on the cups (min, max); the PCB centre's lateral offset from the jaw centreline there.
    """

    # The gaps.
    #   Unit [float] (m)
    Line_Min: float = 0.0
    At_Handover: float = 0.0
    Clamped_Min: float = 0.0
    Clamped_Max: float = 0.0
    # The PCB centre's offset from the jaw centreline along the jaw axis at P_LOADER_PLACE (item 1).
    #   Unit [float] (m)
    Lateral: float = 0.0
    # The failures; empty when it passes.
    #   Unit [List<string>]
    Failures: tp.List[str] = field(default_factory=list)


@dataclass
class Regrip_Check_Str:
    """
    Description:
        The re-grip from MG06 (item 5): the PCB-ledge gap with the jaws at grip about the PCB resting in MG06 (the handover's
        no-press rule), and the open jaws' closest distance to the PCB and the MG06 holder over the descent before it.
    """

    # The gap; the descent's closest distance and its pair.
    #   Unit [float] (m), [float] (m), [string]
    Gap: float = 0.0
    Descent_Min: float = 0.0
    Descent_Pair: str = ''
    # The failures; empty when it passes.
    #   Unit [List<string>]
    Failures: tp.List[str] = field(default_factory=list)


@dataclass
class Program_Check_Str:
    """
    Description:
        The check of a program's trajectory: per step, the handover, the keypoints the program uses, the build's notes.
    """

    # The program and its cycle time.
    #   Unit [string], [float] (s)
    Program: str = ''
    Cycle_Time: float = 0.0
    # The step checks, in order.
    #   Unit [List<Step_Check_Str>]
    Steps: tp.List[Step_Check_Str] = field(default_factory=list)
    # The handover's no-press check (None when the program has no MoveL into P_LOADER_PLACE).
    #   Unit [Handover_Check_Str(object)]
    Handover: tp.Optional[Handover_Check_Str] = None
    # The re-grip from MG06 (None when the jaws never close on the PCB resting in MG06).
    #   Unit [Regrip_Check_Str(object)]
    Regrip: tp.Optional[Regrip_Check_Str] = None
    # The keypoint check of every keypoint the program uses.
    #   Unit [List<Keypoint_Check_Str>]
    Keypoints: tp.List[Keypoint_Check_Str] = field(default_factory=list)
    # The build's notes.
    #   Unit [List<Build_Note_Str>]
    Notes: tp.List[Build_Note_Str] = field(default_factory=list)
    # The samples measured exactly, of all.
    #   Unit [int]
    Checked: int = 0
    # PASS or FAIL.
    #   Unit [string]
    Verdict: str = ''


def _Peaks(trajectory: Trajectory_Str, first: int, last: int, cell: Cell_Configuration_Str) -> tp.Tuple[float, float]:
    """
    Description:
        The peak joint speed and acceleration of one motion segment as fractions of the limits: finite differences over the
        segment from its start sample (the previous end, at rest) to its end held (at rest).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The trajectory.
        (2 - 3) first, last [int]: The segment's samples (end exclusive).
        (4) cell [Cell_Configuration_Str(object)]: The limits.

    Returns:
        (1 - 2) parameter [float]: max |qd| / v_max, max |qdd| / a_max.
    """

    q = np.vstack([trajectory.Robot_Q[max(first - 1, 0)], trajectory.Robot_Q[first:last], trajectory.Robot_Q[last - 1]])
    qd = np.diff(q, axis=0) / CONST_DT
    qdd = np.diff(qd, axis=0) / CONST_DT
    v_max = cell.Speed_Fraction * np.array(cell.Datasheet_Speeds)
    return float(np.max(np.abs(qd) / v_max)), float(np.max(np.abs(qdd) / np.array(cell.Acceleration_Limits)))


def Check_Trajectory(trajectory: Trajectory_Str, program: Program_Str, notes: tp.List[Build_Note_Str], kinematics: Robot_Kinematics_Cls,
                     collision: Robot_Collision_Cls, cell: Cell_Configuration_Str, pcb_meta: PCB_Meta_Str,
                     free_steps: tp.Optional[tp.Set[str]] = None) -> Program_Check_Str:
    """
    Description:
        The check of a program's trajectory (FEATURES.md F8), on the planning model (the table included):
        - every sample where a robot geom, the carried PCB, the loader, the jaws or MG06 may have moved more than
          Clearance.Check_Step since the last measured sample (and every segment end) is posed and measured exactly: the
          robot's geoms, and the PCB's when the cups carry it (to everything else) or the jaws hold it (to the workstation:
          the table, the crates); every pair within the clearance fails unless its group is one the step allows, and an
          allowed pair fails deeper than its group's Allowed_Depth; every allowed group must occur in the step; per step and
          group the closest pair and where it occurs is kept (pairs up to CONST_REPORT_CUT);
        - the robot joints within the model's limits; the peak joint speed and acceleration of every MoveJ (allowing its
          rounding to whole samples) and MoveL within the Cell.yaml limits; the recorded paths' peaks reported, not limited;
        - the handover's no-press rule (B1): the PCB-ledge gap within Handover.Ledge_Gap +- Ledge_Tolerance at
          P_LOADER_PLACE and while the jaws clamp the PCB on the cups, and no smaller along the MoveL into it;
        - the keypoint check of every keypoint the program uses (stale, Check, limits, clearance in its context);
        - in each free transfer (free_steps, Configuration.Program_Parser.Free_Steps; F11), every sample: the robot and the
          carried PCB keep Clearance.Loader_Free + Tracking_Margin from the loader and the PCB in its jaws, the loader posed
          where its actuators put it (Loader_Actual_Trajectory).

    Args:
        (1) trajectory [Trajectory_Str(object)]: The built trajectory.
        (2) program [Program_Str(object)]: Its program.
        (3) notes [List<Build_Note_Str>]: The build's notes.
        (4) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (5) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (6) cell [Cell_Configuration_Str(object)]: The clearance, the depths, the limits, the handover gap.
        (7) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (8) free_steps [Set<string>]: The free transfers' segment keys (none: no loader rule).

    Returns:
        (1) parameter [Program_Check_Str(object)]: The result.
    """

    model, data = kinematics.Model, kinematics.Data
    tr, robot = trajectory, collision.Robot_Bodies
    pcb_body = model.body('pcb').id
    not_workstation = {i for i in range(model.nbody) if model.body(i).name != 'workstation'}
    cup_block = str(Cup_Block_Geom(model))
    lower, upper = kinematics.Limits
    executed = Executed_Commands(program)
    commands = [command for _, _, command, _ in executed]
    texts = [text for _, _, _, text in executed]
    allow = [s.Allow_Contact for s in program.Segments] + [[]]
    ends = set(int(e) - 1 for e in tr.Segment_End if e > 0)
    result = Program_Check_Str(Program=program.Name, Cycle_Time=float(tr.T[-1]), Notes=list(notes))
    for step_index, name in enumerate(Step_Names(program)):
        samples = np.flatnonzero(tr.Step_Index == step_index)
        check = Step_Check_Str(Name=name, Start=float(tr.T[samples[0]]), Duration=float(len(samples) * CONST_DT),
                               Allowed={g: None for g in allow[step_index]})
        result.Steps.append(check)
    # The motion bound: the robot's reach from its first joint (with the carried PCB), the scene's moving parts.
    last = None
    for i in range(len(tr.T)):
        if last is not None and i not in ends:
            Pose_Sample(kinematics, tr, last)
            anchor = data.xanchor[model.joint('aubo/' + CONST_ROBOT_JOINTS[0]).id]
            reach = max(float(np.linalg.norm(data.geom_xpos[g] - anchor) + model.geom_rbound[g])
                        for g in range(model.ngeom) if model.geom_bodyid[g] in robot | {pcb_body})
            bound = (float(np.sum(np.abs(tr.Robot_Q[i] - tr.Robot_Q[last]))) * reach + abs(tr.Loader_Y[i] - tr.Loader_Y[last])
                     + abs(tr.Loader_Z[i] - tr.Loader_Z[last]) + abs(tr.Jaw[i] - tr.Jaw[last])
                     + (abs(tr.MG06_Lid[i] - tr.MG06_Lid[last]) + abs(tr.MG06_Handle[i] - tr.MG06_Handle[last])) * CONST_MG06_REACH)
            if bound <= cell.Check_Step:
                continue
        last = i
        check = result.Steps[tr.Step_Index[i]]
        check.Checked += 1
        Pose_Sample(kinematics, tr, i)
        if np.any(tr.Robot_Q[i] < lower) or np.any(tr.Robot_Q[i] > upper):
            check.Failures.append('joints outside the model limits at sample %d (%.3f s)' % (i, tr.T[i]))
        pairs = collision.Close_Pairs(robot, CONST_REPORT_CUT, data)
        location = CONST_PCB_LOCATIONS[tr.PCB_Location[i]]
        if location == 'cups':
            pairs += collision.Close_Pairs({pcb_body}, CONST_REPORT_CUT, data, exclude=robot)
        elif location == 'jaws':
            pairs += collision.Close_Pairs({pcb_body}, CONST_REPORT_CUT, data, exclude=not_workstation)
        in_crate = Crate_Holding(data.xpos[pcb_body]) is not None
        for distance, body_1, body_2, geom_1, geom_2 in pairs:
            if body_1 == CONST_MOUNT_BODY and geom_2.startswith(CONST_MOUNT_GEOM_PREFIX):
                continue
            group = Contact_Group({body_1, body_2}, {geom_1, geom_2}, cup_block, in_crate)
            pair = '%s / %s' % (body_1 if geom_1.isdigit() else geom_1, body_2 if geom_2.isdigit() else geom_2)
            found = Group_Distance_Str(Group=group or '', Distance=distance, Sample=i, Time=float(tr.T[i]), Pair=pair)
            if group in check.Allowed:
                if check.Allowed[group] is None or distance < check.Allowed[group].Distance:
                    check.Allowed[group] = found
            elif check.Closest is None or distance < check.Closest.Distance:
                check.Closest = found
        result.Checked += 1
    for step_index, check in enumerate(result.Steps):
        if check.Closest is not None and check.Closest.Distance < cell.Clearance:
            c = check.Closest
            why = '' if not c.Group else ' (group %s not allowed in this step)' % c.Group
            check.Failures.append('%s %.2f mm apart at sample %d (%.3f s); clearance %.1f mm%s'
                                  % (c.Pair, 1000 * c.Distance, c.Sample, c.Time, 1000 * cell.Clearance, why))
        for group, found in check.Allowed.items():
            if found is None or found.Distance >= cell.Clearance:
                check.Failures.append('allowed contact %s does not occur in the step (allow only what is present)' % group)
            elif -found.Distance > cell.Allowed_Depth[group]:
                check.Failures.append('%s %.2f mm deep at sample %d (%.3f s), at most %.2f mm' % (group, -1000 * found.Distance, found.Sample,
                                                                                                found.Time, 1000 * cell.Allowed_Depth[group]))
        for segment, command in enumerate(commands):
            first, end = int(tr.Segment_Start[segment]), int(tr.Segment_End[segment])
            if tr.Segment_Step[segment] != step_index or command.Kind not in ('MoveJ', 'MoveL', 'MovePath', 'MoveSpline') or end <= first:
                continue
            speed, acceleration = _Peaks(tr, max(first, 1), end, cell)
            if command.Kind == 'MovePath':
                check.Path_Speed, check.Path_Acceleration = max(check.Path_Speed, speed), max(check.Path_Acceleration, acceleration)
                continue
            check.Peak_Speed, check.Peak_Acceleration = max(check.Peak_Speed, speed), max(check.Peak_Acceleration, acceleration)
            # A MoveJ or a MoveSpline runs at its v= (of the datasheet speeds; the limits are at Speed_Fraction); a MoveJ's rule
            # rounds its duration to whole samples: up to half a sample shorter, so up to this much faster (a MoveSpline's
            # timing is stretched to whole samples instead).
            rounding = 1.0 + 0.5 / (end - max(first, 1)) if command.Kind == 'MoveJ' else 1.0
            fraction = command.Speed / cell.Speed_Fraction if command.Kind in ('MoveJ', 'MoveSpline') else 1.0
            if speed > fraction * rounding * (1 + CONST_DIFFERENCE_TOLERANCE) or acceleration > rounding**2 * (1 + CONST_DIFFERENCE_TOLERANCE):
                check.Failures.append('%s: peak joint speed %.3f / acceleration %.3f of the limits (allowed %.3f / %.3f)'
                                      % (texts[segment], speed, acceleration, fraction * rounding, rounding**2))
    # B1: the handover's no-press rule.
    for segment, command in enumerate(commands):
        if command.Kind != 'MoveL' or command.Target != CONST_HANDOVER_KEYPOINT:
            continue
        first, end = int(tr.Segment_Start[segment]), int(tr.Segment_End[segment])
        grip = max(program.Simulation.Jaw_Actual.values())
        home = program.Constants.Loader[CONST_LOADER_HOME]
        # Item 1: along the descent only the PCB's height above the finger ledges counts - the gap at the point plus the PCB's
        # height above its end position along the plate normal (the jaws are open during the descent, so a jaw posed at grip
        # beside the board would measure its lead-in faces, not the ledges under it).
        Pose_Sample(kinematics, tr, end - 1)
        axis = data.xaxis[model.joint('jaw_R').id].copy()
        hold = np.array(pcb_meta.Hold_Pos) + [0.0, tr.Loader_Y[end - 1] - home.Y, tr.Loader_Z[end - 1] - home.Z]
        lateral = float((tr.PCB_Pose[end - 1][:3] - hold) @ axis)
        handle = float(tr.MG06_Handle[end - 1])
        kinematics.Set_Scene(data, lid=float(tr.MG06_Lid[end - 1]), handle=handle, press=Press_For_Handle(model, handle),
                             y=float(tr.Loader_Y[end - 1]), z=float(tr.Loader_Z[end - 1]), jaw=grip, pcb=tr.PCB_Pose[end - 1])
        mujoco.mj_kinematics(model, data)
        at_point, normal = Ledge_Gap(model, data), data.xmat[model.body('pcb').id].reshape(3, 3)[:, 2].copy()
        line = [at_point + float((tr.PCB_Pose[i][:3] - tr.PCB_Pose[end - 1][:3]) @ normal) for i in range(first, end)]
        clamped = []
        for i in range(end, len(tr.T)):
            if CONST_PCB_LOCATIONS[tr.PCB_Location[i]] != 'cups':
                break
            if tr.Jaw[i] == grip:
                Pose_Sample(kinematics, tr, i)
                clamped.append(Ledge_Gap(model, data))
        # The gap at the point itself is measured on the PCB as it is, as while clamped.
        handover = Handover_Check_Str(Line_Min=min(line), At_Handover=at_point, Clamped_Min=min(clamped) if clamped else math.nan,
                                      Clamped_Max=max(clamped) if clamped else math.nan, Lateral=lateral)
        low, high = cell.Ledge_Gap - cell.Ledge_Tolerance, cell.Ledge_Gap + cell.Ledge_Tolerance
        if not low <= handover.At_Handover <= high:
            handover.Failures.append('the PCB-ledge gap at %s is %.3f mm, not %.1f +- %.1f mm%s' % (
                CONST_HANDOVER_KEYPOINT, 1000 * handover.At_Handover, 1000 * cell.Ledge_Gap, 1000 * cell.Ledge_Tolerance,
                ' (a press)' if handover.At_Handover < low else ''))
        if handover.Line_Min < low:
            handover.Failures.append('the PCB-ledge gap falls to %.3f mm along %s (a press)' % (1000 * handover.Line_Min, command.Text))
        if not clamped or not low <= handover.Clamped_Min <= handover.Clamped_Max <= high:
            handover.Failures.append('the PCB-ledge gap while the jaws clamp is %s, not %.1f +- %.1f mm' % (
                'never measured (the jaws never reach grip with the PCB on the cups)' if not clamped else
                '%.3f .. %.3f mm' % (1000 * handover.Clamped_Min, 1000 * handover.Clamped_Max), 1000 * cell.Ledge_Gap, 1000 * cell.Ledge_Tolerance))
        if abs(handover.Lateral) > cell.Lateral_Max:
            handover.Failures.append('the PCB centre is %.3f mm from the jaw centreline at %s, more than Handover.Lateral_Max %.1f mm: the '
                                     'jaws would pull it off the cups as they centre it' % (1000 * abs(handover.Lateral), CONST_HANDOVER_KEYPOINT,
                                                                                            1000 * cell.Lateral_Max))
        result.Handover = handover
    # Item 5: the re-grip from MG06 - the jaws closing on the PCB resting in MG06.
    moving = np.flatnonzero(np.abs(np.diff(tr.Gripper)) > 0.0)
    mg06, jaw_bodies = CONST_PCB_LOCATIONS.index('mg06'), {model.body('jaw_L').id, model.body('jaw_R').id}
    others = {b for b in range(model.nbody) if b not in (pcb_body, model.body('mg06_base').id)}
    for operation in np.split(moving, np.flatnonzero(np.diff(moving) > 1) + 1) if moving.size else []:
        first, last = int(operation[0]), int(operation[-1]) + 1
        if tr.Gripper[last] >= tr.Gripper[first] or tr.PCB_Location[first] != mg06:
            continue
        grip = max(program.Simulation.Jaw_Actual.values())
        handle = float(tr.MG06_Handle[last])
        kinematics.Set_Scene(data, lid=float(tr.MG06_Lid[last]), handle=handle, press=Press_For_Handle(model, handle), y=float(tr.Loader_Y[last]),
                             z=float(tr.Loader_Z[last]), jaw=grip, pcb=tr.PCB_Pose[first])
        mujoco.mj_kinematics(model, data)
        regrip = Regrip_Check_Str(Gap=Ledge_Gap(model, data), Descent_Min=math.inf)
        # The descent: back from the close while the loader stands or comes down at the same Y with the PCB in MG06.
        start = first
        while start > 0 and tr.PCB_Location[start - 1] == mg06 and tr.Loader_Y[start - 1] == tr.Loader_Y[start] \
                and tr.Loader_Z[start - 1] >= tr.Loader_Z[start]:
            start -= 1
        for i in range(start, first + 1):
            Pose_Sample(kinematics, tr, i)
            for distance, body_1, body_2, _, geom_2 in collision.Close_Pairs(jaw_bodies, CONST_REPORT_CUT, data, exclude=others)[:1]:
                if distance < regrip.Descent_Min:
                    other = body_2 if geom_2.isdigit() else geom_2
                    regrip.Descent_Min, regrip.Descent_Pair = distance, '%s / %s at %.3f s' % (body_1, other, tr.T[i])
        low, high = cell.Ledge_Gap - cell.Ledge_Tolerance, cell.Ledge_Gap + cell.Ledge_Tolerance
        if not low <= regrip.Gap <= high:
            regrip.Failures.append('the PCB-ledge gap at the re-grip from MG06 is %.3f mm, not %.1f +- %.1f mm%s' % (
                1000 * regrip.Gap, 1000 * cell.Ledge_Gap, 1000 * cell.Ledge_Tolerance,
                ' (the jaws clamp on the ledge corners)' if regrip.Gap < low else ''))
        if regrip.Descent_Min <= 0.0:
            regrip.Failures.append('the open jaws touch %s during the descent to the re-grip (%.3f mm)'
                                   % (regrip.Descent_Pair, 1000 * regrip.Descent_Min))
        result.Regrip = regrip
    # The posed jaws (the command limited by the part) and the gripper's jaw speed: their rules, in the step of each fault.
    for i, fault in Jaw_Posing_Faults(tr, max(program.Simulation.Jaw_Actual.values())) + Jaw_Speed_Faults(tr, cell.Jaw_Speed_Max):
        result.Steps[int(tr.Step_Index[i])].Failures.append(fault)
    used = []
    for command in commands:
        names = [command.Target] if command.Kind in ('MoveJ', 'MoveL') else []
        names += [command.Path_Start, command.Path_End] if command.Kind in ('MovePath', 'MoveSpline') else []
        used += [n for n in names if n not in used]
    start = program.Simulation.Start_Robot
    used = [start] + [n for n in used if n != start]
    result.Keypoints = [Check_Keypoint(name, kinematics, collision, program.Constants, program.Simulation, cell, pcb_meta) for name in used]
    # F11: the free transfers' room around the loader, the loader where its actuators put it.
    if free_steps:
        actual, need = Loader_Actual_Trajectory(tr, model), cell.Loader_Free + cell.Tracking_Margin
        for step_index, check in enumerate(result.Steps):
            if check.Name not in free_steps:
                continue
            distance, i, pair = Robot_Loader_Distance(actual, kinematics, collision, np.flatnonzero(tr.Step_Index == step_index))
            check.Loader = Group_Distance_Str(Group='loader', Distance=distance, Sample=i, Time=float(tr.T[i]) if i >= 0 else 0.0, Pair=pair)
            if distance < need:
                check.Failures.append('%s %.2f mm from the loader at sample %d (%.3f s), the loader at its actuator model\'s position; a free '
                                      'transfer keeps Clearance.Loader_Free %.1f mm + Tracking_Margin %.1f mm'
                                      % (pair, 1000 * distance, i, tr.T[i], 1000 * cell.Loader_Free, 1000 * cell.Tracking_Margin))
    for check in result.Steps:
        check.Verdict = 'FAIL' if check.Failures else 'PASS'
    failed = any(c.Failures for c in result.Steps) or (result.Handover is not None and result.Handover.Failures) \
        or (result.Regrip is not None and result.Regrip.Failures) or any(k.Verdict == 'FAIL' for k in result.Keypoints)
    result.Verdict = 'FAIL' if failed else 'PASS'
    return result


def _Distance_Text(found: tp.Optional[Group_Distance_Str], cut: float = CONST_REPORT_CUT) -> str:
    """
    Description:
        A closest pair as report text: '-1.48 mm at 819 (3.276 s) aubo/tool/cup1 / pcb_comp', or '> 10 mm'.

    Args:
        (1) found [Group_Distance_Str(object)]: The pair, or None.
        (2) cut [float]: The distance pairs were measured up to (m).

    Returns:
        (1) parameter [string]: The text.
    """

    if found is None:
        return '> %g mm' % (1000 * cut)
    return '%.2f mm at %d (%.3f s) %s' % (1000 * found.Distance, found.Sample, found.Time, found.Pair)


def Format_Check_Report(result: Program_Check_Str, trajectory: Trajectory_Str, cell: Cell_Configuration_Str) -> tp.List[str]:
    """
    Description:
        The program check as text: a header (the program, the cycle time, the samples); per step its start, duration, peak
        joint speed and acceleration (fractions of the limits; a recorded path's in brackets), verdict, then per allowed
        group and for the closest pair no allowed group covers, the smallest distance with its sample and time; the
        handover's PCB-ledge gaps; the build's notes (stretched MoveLs, the recorded path's velocity step); the events; the
        keypoints used; every failure; the project's sentence on simulation results.

    Args:
        (1) result [Program_Check_Str(object)]: The check.
        (2) trajectory [Trajectory_Str(object)]: The trajectory (its events and size).
        (3) cell [Cell_Configuration_Str(object)]: The clearance, for the header.

    Returns:
        (1) parameter [List<string>]: The lines.
    """

    lines = ['program %s: %d steps, %d samples (4 ms), cycle time %.3f s; %d samples measured exactly (clearance %.1f mm, pairs '
             'reported up to %g mm)' % (result.Program, len(result.Steps), len(trajectory.T), result.Cycle_Time, result.Checked,
                                         1000 * cell.Clearance, 1000 * CONST_REPORT_CUT)]
    if any(s.Loader is not None for s in result.Steps):
        lines.append('free transfers to the loader (every sample, the loader at its actuator model\'s position): at least Loader_Free %.1f mm + '
                     'Tracking_Margin %.1f mm, pairs up to %g mm' % (1000 * cell.Loader_Free, 1000 * cell.Tracking_Margin, 1000 * CONST_LOADER_CUT))
    lines.append('%-20s %8s %7s %13s %15s %s' % ('step', 'start s', 'dur s', 'peak v / a', '(recorded path)', 'verdict'))
    for s in result.Steps:
        path = '(%.2f / %.2f)' % (s.Path_Speed, s.Path_Acceleration) if s.Path_Speed else ''
        lines.append('%-20s %8.3f %7.3f %6.3f / %.3f %15s %s' % (s.Name, s.Start, s.Duration, s.Peak_Speed, s.Peak_Acceleration, path, s.Verdict))
        for group, found in s.Allowed.items():
            lines.append('    %-12s %s' % (group, _Distance_Text(found)))
        lines.append('    %-12s %s' % ('not allowed', _Distance_Text(s.Closest)))
        if s.Loader is not None:
            lines.append('    %-12s %s' % ('loader', _Distance_Text(s.Loader if s.Loader.Sample >= 0 else None, CONST_LOADER_CUT)))
    if result.Handover is not None:
        h = result.Handover
        lines.append('handover (no press, %.1f +- %.1f mm): PCB-ledge gap with the jaws at grip - along the MoveL into %s (the PCB\'s '
                     'height above the ledges) min %.3f mm, at it %.3f mm, while clamped %.3f .. %.3f mm; PCB centre %.3f mm from the jaw '
                     'centreline (<= %.1f mm): %s' % (1000 * cell.Ledge_Gap, 1000 * cell.Ledge_Tolerance, CONST_HANDOVER_KEYPOINT, 1000 * h.Line_Min,
                                  1000 * h.At_Handover, 1000 * h.Clamped_Min, 1000 * h.Clamped_Max, 1000 * abs(h.Lateral), 1000 * cell.Lateral_Max,
                                  'FAIL' if h.Failures else 'PASS'))
    if result.Regrip is not None:
        r = result.Regrip
        lines.append('re-grip from MG06 (no press, %.1f +- %.1f mm): PCB-ledge gap with the jaws at grip %.3f mm; the open jaws over the descent '
                     '%s from the PCB and the holder: %s' % (
                         1000 * cell.Ledge_Gap, 1000 * cell.Ledge_Tolerance, 1000 * r.Gap,
                         'more than %.0f mm' % (1000 * CONST_REPORT_CUT) if math.isinf(r.Descent_Min)
                         else '%.2f mm (%s)' % (1000 * r.Descent_Min, r.Descent_Pair),
                         'FAIL' if r.Failures else 'PASS'))
        lines += ['  FAIL %s' % f for f in r.Failures]
    stretched = [n for n in result.Notes if n.Text.startswith('stretched')]
    lines.append('stretched MoveLs: %s' % ('none' if not stretched else '; '.join('%s %s %s' % (n.Step, n.Command, n.Text) for n in stretched)))
    lines += ['note: %s, %s: %s' % (n.Step, n.Command, n.Text) for n in result.Notes if n not in stretched]
    lines.append('events: %s' % ', '.join('%.3f s %s' % (t, e) for t, e in trajectory.Events))
    verdicts = [k.Verdict for k in result.Keypoints]
    lines.append('keypoints used: %d (%d FAIL, %d WARN, %d PASS; Example/Keypoints/check_keypoints.py shows each)'
                 % (len(verdicts), verdicts.count('FAIL'), verdicts.count('WARN'), verdicts.count('PASS')))
    for s in result.Steps:
        lines += ['  %s: %s' % (s.Name, f) for f in s.Failures]
    if result.Handover is not None:
        lines += ['  handover: %s' % f for f in result.Handover.Failures]
    for k in result.Keypoints:
        lines += ['  keypoint %s: %s' % (k.Name, f) for f in k.Failures]
    lines.append('%s: the program check %s. Simulation results only: not validated for the real cell.'
                 % (result.Program, 'FAILED' if result.Verdict == 'FAIL' else 'passed'))
    return lines
