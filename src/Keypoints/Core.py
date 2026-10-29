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
# File Name: ../Keypoints/Core.py
## =========================================================================== ##

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
# Custom Lib.:
#   ../Configuration/Core, ../Configuration/Program_Parser - the cell, the PCB poses, the constants and the simulation data
from Configuration.Core import Cell_Configuration_Str, PCB_Meta_Str
from Configuration.Program_Parser import Constants_Str, Simulation_Str, Point_Str, Point_Scene_Str, CONST_WRIST_2
#   ../Scene/Core - the crates
from Scene.Core import PCB_In_Crate, Cup_Block_Geom, Crate_Holding
#   ../Kinematics/Core - FK, IK, the PCB held by the cups, rotations
from Kinematics.Core import Robot_Kinematics_Cls, TCP_For_PCB, PCB_For_TCP, RPY_To_Matrix, Matrix_To_RPY, Rotation_Angle
#   ../Collision/Core - contacts of the robot and of the carried PCB
from Collision.Core import Robot_Collision_Cls

"""
Description:
    Initialization of constants.

    The robot points of Configuration/Constants.yaml at work, in their scenes of Configuration/Simulation.yaml (SPEC §2
    item 3, §7.7): a point's pose in the robot base frame, its Reference or PCB-hold target, its MG06 state, posing the
    whole cell at a point (robot, loader, jaws, PCB, MG06), the point check, and the IK resolution of a point's joints.
    Posing and playback use a point's Joints only; IK runs only in Resolve_Keypoint, whose result a person pastes into
    the file. It prints nothing.
"""
# The joints posed for the scene.
#   Unit [string]
CONST_LID, CONST_HANDLE, CONST_PRESS = 'lid_hinge', 'handle_hinge', 'press_slide'
# A point's Pose against FK(Joints, Tool): the Pose is written from FK to 9 decimals, so these bound a stale Pose, not
# rounding (owner, 2026-10-24, F10).
#   Unit [float] (m), [float] (rad)
CONST_POSE_POSITION, CONST_POSE_ANGLE = 1e-5, math.radians(1e-3)
# A tool's Flange_Offset against its model site (Simulation.yaml Tool_Site), as tight.
#   Unit [float] (m), [float] (rad)
CONST_TOOL_POSITION, CONST_TOOL_ANGLE = 1e-5, math.radians(1e-3)
# The robot's mount: its base (no joint) stands on the pedestal plate, distance 0 by construction. The one pair
# never checked (a single legal value, as the template's base-env_stand exclusion): the base link against the
# plate's collision geoms.
#   Unit [Tuple<string>]
CONST_MOUNT_BODY, CONST_MOUNT_GEOM_PREFIX = 'aubo/base_link', 'ws_col_deska_robot'


class Keypoints_Error_Cls(ValueError):
    """
    Description:
        A point cannot be posed or resolved (an unknown name or location, a resolution that would change the IK
        family). The message names the point and what to do.
    """


@dataclass
class Keypoint_Check_Str:
    """
    Description:
        The check of one point (SPEC §2 item 3): each rule's measured value and the verdict.
    """

    # The point.
    #   Unit [string]
    Name: str = ''
    # Joints within the model's limits; the IK family (wrist_2's sign) and whether wrist_2 is within the family margin.
    #   Unit [bool], [string], [bool]
    Limits_Ok: bool = True
    Family: str = ''
    Family_Near_Zero: bool = False
    # FK(Joints, Tool) against the Pose.
    #   Unit [float] (m), [float] (rad)
    Check_Error: float = 0.0
    Check_Angle: float = 0.0
    # FK against the Reference or PCB-hold target (None when the point has neither).
    #   Unit [float] (m), [float] (rad)
    Stale_Position: tp.Optional[float] = None
    Stale_Angle: tp.Optional[float] = None
    # The smallest distance of a contact that is not allowed (the clearance, if none), and its pair.
    #   Unit [float] (m), [string]
    Min_Distance: float = 0.0
    Min_Pair: str = ''
    # The allowed contacts found: (group, deepest distance).
    #   Unit [List<Tuple<string, float>>] (-, m)
    Allowed: tp.List[tp.Tuple[str, float]] = field(default_factory=list)
    # Every failed rule, as a sentence; empty when the point passes.
    #   Unit [List<string>]
    Failures: tp.List[str] = field(default_factory=list)
    # PASS, WARN (passes with a warning) or FAIL.
    #   Unit [string]
    Verdict: str = ''


def Site_Id(model: mujoco.MjModel, simulation: Simulation_Str, tool: str) -> int:
    """
    Description:
        The model site of a tool (Simulation.yaml Tool_Site).

    Args:
        (1) model [mujoco.MjModel]: The model.
        (2) simulation [Simulation_Str(object)]: The simulation data.
        (3) tool [string]: The tool (T_...).

    Returns:
        (1) parameter [int]: The site id.
    """

    return model.site(simulation.Tool_Site[tool]).id


def Family(joints: tp.Sequence[float]) -> str:
    """
    Description:
        The IK family of joints: wrist_up (wrist_2 > 0, PCB handling) or wrist_down (the MG06 fork; SPEC §4.7).

    Args:
        (1) joints [Vector<float> 1x6]: The joints (rad).

    Returns:
        (1) parameter [string]: wrist_up or wrist_down.
    """

    return 'wrist_up' if joints[CONST_WRIST_2] > 0 else 'wrist_down'


def Base_Pose(position: np.ndarray, rotation: np.ndarray, cell: Cell_Configuration_Str) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        A world pose (the MG06 fixture frame) in the robot base frame (Cell.yaml Robot.Base_Position, Base_Yaw).

    Args:
        (1 - 2) position, rotation [Vector<float> 1x3, Matrix<float> 3x3]: The world pose (m).
        (3) cell [Cell_Configuration_Str(object)]: The base pose.

    Returns:
        (1) parameter [Vector<float> 1x3]: The position in the base frame (m).
        (2) parameter [Matrix<float> 3x3]: The orientation in the base frame.
    """

    yaw = cell.Base_Yaw
    R_base = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
    return R_base.T @ (np.asarray(position) - np.array(cell.Base_Position)), R_base.T @ rotation


def Point_Pose(point: Point_Str, kinematics: Robot_Kinematics_Cls, simulation: Simulation_Str) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        FK(Joints, Tool) of a point in the world (the MG06 fixture frame).

    Args:
        (1) point [Point_Str(object)]: The point.
        (2) kinematics [Robot_Kinematics_Cls(object)]: FK.
        (3) simulation [Simulation_Str(object)]: The tools' model sites.

    Returns:
        (1) parameter [Vector<float> 1x3]: The position (m).
        (2) parameter [Matrix<float> 3x3]: The orientation.
    """

    return kinematics.Site_Pose(np.array(point.Joints), Site_Id(kinematics.Model, simulation, point.Tool))


def PCB_Pose(location: str, pcb_meta: PCB_Meta_Str, kinematics: Robot_Kinematics_Cls, joints: np.ndarray) -> np.ndarray:
    """
    Description:
        The PCB pose at a named location: flat on a crate floor (input_crate, output_crate), at the hold pose in the jaws
        (jaws), placed in MG06 (mg06), or held by the cups at the given joints (cups).

    Args:
        (1) location [string]: The location.
        (2) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (3) kinematics [Robot_Kinematics_Cls(object)]: FK of the cups (for 'cups').
        (4) joints [Vector<float> 1x6]: The robot joints (rad), for 'cups'.

    Returns:
        (1) parameter [Vector<float> 1x7]: Position (m) and quaternion (w first).
    """

    if location == 'input_crate':
        return np.r_[PCB_In_Crate('crate_pick')]
    if location == 'output_crate':
        return np.r_[PCB_In_Crate('crate_place')]
    if location == 'jaws':
        return np.r_[pcb_meta.Hold_Pos, pcb_meta.Quat]
    if location == 'mg06':
        return np.r_[pcb_meta.Place_Pos, pcb_meta.Quat]
    if location == 'cups':
        return np.r_[PCB_For_TCP(*kinematics.Site_Pose(np.asarray(joints)))]
    raise Keypoints_Error_Cls("The PCB location '%s' is not one of input_crate, cups, jaws, mg06, output_crate." % location)


def Target_Pose(name: str, constants: Constants_Str, simulation: Simulation_Str, pcb_meta: PCB_Meta_Str,
                kinematics: Robot_Kinematics_Cls) -> tp.Optional[tp.Tuple[np.ndarray, np.ndarray]]:
    """
    Description:
        The world pose a point's definition asks for, at its tool: its Reference's FK moved by its Offset along the
        Reference's tool axes, or the TCP holding the PCB at its PCB_Hold location moved by that offset along the tool
        axes; None when it has neither.

    Args:
        (1) name [string]: The point.
        (2) constants [Constants_Str(object)]: The points.
        (3) simulation [Simulation_Str(object)]: The scenes, the tools' sites.
        (4) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (5) kinematics [Robot_Kinematics_Cls(object)]: FK.

    Returns:
        (1) parameter [Tuple<Vector<float> 1x3, Matrix<float> 3x3>]: The target position (m) and orientation, or None.
    """

    point, scene = constants.Points[name], simulation.Points[name]
    if point.Reference is not None:
        position, rotation = Point_Pose(constants.Points[point.Reference], kinematics, simulation)
        offset = point.Offset
    elif scene.Hold_Location is not None:
        pose = PCB_Pose(scene.Hold_Location, pcb_meta, kinematics, np.zeros(6))
        position, rotation = TCP_For_PCB(pose[:3], pose[3:])
        offset = scene.Hold_Offset
    else:
        return None
    return position + rotation @ np.array(offset), rotation


def Press_For_Handle(model: mujoco.MjModel, handle: float) -> float:
    """
    Description:
        The MG06 press plate's position for a handle angle, through the model's joint equality coupling them (a
        polynomial in the handle angle).

    Args:
        (1) model [mujoco.MjModel]: The model.
        (2) handle [float]: The handle angle (rad).

    Returns:
        (1) parameter [float]: The press plate (m).
    """

    press_id, handle_id = model.joint(CONST_PRESS).id, model.joint(CONST_HANDLE).id
    coupling = next(i for i in range(model.neq) if model.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT
                    and model.eq_obj1id[i] == press_id and model.eq_obj2id[i] == handle_id)
    return float(sum(model.eq_data[coupling][k] * handle**k for k in range(5)))


def Fixture_State(scene: Point_Scene_Str, cell: Cell_Configuration_Str, model: mujoco.MjModel,
                  fixture: tp.Optional[str] = None) -> tp.Tuple[float, float, float]:
    """
    Description:
        The MG06 lid, handle and press plate of a point's Fixture (or of the named state fixture, which overrides it):
        Open = the lid at its open stop (the model) and the handle up; Closed = Cell.yaml MG06; the recorded state as
        given. The press plate follows the handle through the model's equality.

    Args:
        (1) scene [Point_Scene_Str(object)]: The point's scene.
        (2) cell [Cell_Configuration_Str(object)]: The closed state.
        (3) model [mujoco.MjModel]: The model (the lid's stop, the press coupling).
        (4) fixture [string]: 'Open' or 'Closed' to override, or None.

    Returns:
        (1) parameter [Tuple<float>]: lid (rad), handle (rad), press (m).
    """

    state = fixture or scene.Fixture
    if state == 'Open':
        lid, handle = float(model.jnt_range[model.joint(CONST_LID).id][0]), 0.0
    elif state == 'Closed':
        lid, handle = cell.Closed_Lid, cell.Closed_Handle
    elif state == 'Recorded':
        lid, handle = scene.Fixture_Lid, scene.Fixture_Handle
    else:
        raise Keypoints_Error_Cls("The MG06 state '%s' is not Open or Closed." % state)
    return lid, handle, Press_For_Handle(model, handle)


def Jaw_Position(gripper: str, constants: Constants_Str, simulation: Simulation_Str) -> float:
    """
    Description:
        Where the jaws stand for a gripper command in a posed scene: its Jaw_Actual (on the PCB) when it has one, else its
        command.

    Args:
        (1) gripper [string]: The gripper command (G_...).
        (2) constants [Constants_Str(object)]: The commands.
        (3) simulation [Simulation_Str(object)]: Jaw_Actual.

    Returns:
        (1) parameter [float]: The jaw position, per jaw (m).
    """

    return simulation.Jaw_Actual.get(gripper, constants.Gripper[gripper].Jaw)


def Pose_Cell(kinematics: Robot_Kinematics_Cls, constants: Constants_Str, simulation: Simulation_Str, cell: Cell_Configuration_Str,
              pcb_meta: PCB_Meta_Str, name: str, loader: tp.Optional[str] = None, gripper: tp.Optional[str] = None, pcb: tp.Optional[str] = None,
              fixture: tp.Optional[str] = None) -> Point_Str:
    """
    Description:
        Pose the whole cell at a point in the kinematics' data: the robot at its Joints, the suction cups' bellows at rest,
        the loader axes and jaws, the PCB at its location, MG06 in its state - from the point's scene (Simulation.yaml), each
        overridable. Ends with mj_forward.

    Args:
        (1) kinematics [Robot_Kinematics_Cls(object)]: The model and the data to pose.
        (2) constants [Constants_Str(object)]: The points, loader positions and gripper commands.
        (3) simulation [Simulation_Str(object)]: The scenes.
        (4) cell [Cell_Configuration_Str(object)]: The cell configuration.
        (5) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (6) name [string]: The point.
        (7 - 10) loader, gripper, pcb, fixture [string]: Overrides of the scene, or None.

    Returns:
        (1) parameter [Point_Str(object)]: The point posed.
    """

    if name not in constants.Points:
        raise Keypoints_Error_Cls("There is no point '%s' in Configuration/Constants.yaml; the points are %s." % (name, ', '.join(constants.Points)))
    point, scene = constants.Points[name], simulation.Points[name]
    model, data = kinematics.Model, kinematics.Data
    loader_name, gripper_name = loader or scene.Loader, gripper or scene.Gripper
    if loader_name not in constants.Loader or gripper_name not in constants.Gripper:
        raise Keypoints_Error_Cls("Unknown loader position '%s' or gripper command '%s'; they are %s and %s."
                                  % (loader_name, gripper_name, ', '.join(constants.Loader), ', '.join(constants.Gripper)))
    joints = np.array(point.Joints)
    mujoco.mj_resetData(model, data)
    lid, handle, press = Fixture_State(scene, cell, model, fixture)
    position = constants.Loader[loader_name]
    kinematics.Set_Scene(data, lid=lid, handle=handle, press=press, y=position.Y, z=position.Z, jaw=Jaw_Position(gripper_name, constants, simulation),
                         pcb=PCB_Pose(pcb or scene.PCB, pcb_meta, kinematics, joints))
    data.qpos[kinematics.Q_Address] = joints
    mujoco.mj_forward(model, data)
    return point


def Contact_Group(names: tp.Set[str], geoms: tp.Set[str], cup_block: str, pcb_in_crate: bool) -> tp.Optional[str]:
    """
    Description:
        The contact group of a pair of bodies (and their geoms), or None for a pair no group covers. cups_pcb is
        the VGC10's face on the PCB: a cup, or the cup block (not the fork, which is on the same tool body);
        pcb_crate the PCB on a crate, and on the table boxes (Collision.CONST_TABLE_BOXES) only while the PCB is
        inside a crate, its floor between them (owner, 2026-10-24); pcb_jaws the PCB on the loader's jaws.

    Args:
        (1) names [Set<string>]: The two bodies.
        (2) geoms [Set<string>]: The two geoms.
        (3) cup_block [string]: The cup block's geom (its id as text, it has no name).
        (4) pcb_in_crate [bool]: The PCB is inside a crate (Scene.Crate_Holding).

    Returns:
        (1) parameter [string]: cups_pcb, handle_bar, pcb_crate, pcb_jaws, or None.
    """

    if 'pcb' in names and (any(n.startswith('aubo/tool/vgc10_cup') for n in names) or cup_block in geoms):
        return 'cups_pcb'
    if names == {'aubo/tool/ee_tool', 'mg06_handle'}:
        return 'handle_bar'
    if 'pcb' in names and 'workstation' in names and any(g.startswith('crate_') for g in geoms):
        return 'pcb_crate'
    if 'pcb' in names and 'workstation' in names and pcb_in_crate and any(g.startswith('table_') for g in geoms):
        return 'pcb_crate'
    if 'pcb' in names and names & {'jaw_L', 'jaw_R'}:
        return 'pcb_jaws'
    return None


def Check_Keypoint(name: str, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, constants: Constants_Str,
                   simulation: Simulation_Str, cell: Cell_Configuration_Str, pcb_meta: PCB_Meta_Str,
                   fixture: tp.Optional[str] = None) -> Keypoint_Check_Str:
    """
    Description:
        The check of one point (SPEC §2 item 3, §7.7), posed in its scene (or the Fixture override): joint limits (the
        model); the IK family (the sign of wrist_2, a WARNING within the family margin); the Pose against FK(Joints, Tool)
        (CONST_POSE_POSITION, CONST_POSE_ANGLE); the Reference or PCB-hold target (stale beyond Clearance.Stale_Position /
        Stale_Angle); the clearance of the robot and tool - and of the PCB when the cups carry it - to the scene, every
        contact closer than the clearance failing unless its group is in the scene's Contacts and no deeper than that
        group's allowed depth. Distances are exact (Collision.Close_Pairs); the robot's mount is not checked.

    Args:
        (1) name [string]: The point.
        (2) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (3) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (4) constants [Constants_Str(object)]: The constants.
        (5) simulation [Simulation_Str(object)]: The scenes, the tools' sites.
        (6) cell [Cell_Configuration_Str(object)]: The tolerances.
        (7) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (8) fixture [string]: 'Open' or 'Closed' to override the point's MG06 state, or None.

    Returns:
        (1) parameter [Keypoint_Check_Str(object)]: The result.
    """

    point, scene = constants.Points[name], simulation.Points[name]
    joints = np.array(point.Joints)
    result = Keypoint_Check_Str(Name=name, Family=Family(joints))
    lower, upper = kinematics.Limits
    if np.any(joints < lower) or np.any(joints > upper):
        result.Limits_Ok = False
        result.Failures.append('joints outside the model limits: %s' % ', '.join('J%d %.2f deg' % (i + 1, math.degrees(q))
                                                                            for i, q in enumerate(joints) if not lower[i] <= q <= upper[i]))
    result.Family_Near_Zero = abs(joints[CONST_WRIST_2]) < cell.Family_Margin
    position, rotation = Base_Pose(*Point_Pose(point, kinematics, simulation), cell)
    result.Check_Error = float(np.linalg.norm(position - point.Pose[:3]))
    result.Check_Angle = Rotation_Angle(RPY_To_Matrix(point.Pose[3:]), rotation)
    if result.Check_Error > CONST_POSE_POSITION or result.Check_Angle > CONST_POSE_ANGLE:
        result.Failures.append('Pose %.4f mm / %.4f deg from FK(Joints, %s) (at most %.2f mm / %.3f deg); print the corrected Pose with '
                               'Example/cell.py point %s' % (1000 * result.Check_Error, math.degrees(result.Check_Angle),
                                                                                  point.Tool, 1000 * CONST_POSE_POSITION,
                                                                                  math.degrees(CONST_POSE_ANGLE), name))
    target = Target_Pose(name, constants, simulation, pcb_meta, kinematics)
    if target is not None:
        p, R = Point_Pose(point, kinematics, simulation)
        result.Stale_Position = float(np.linalg.norm(p - target[0]))
        result.Stale_Angle = Rotation_Angle(target[1], R)
        if result.Stale_Position > cell.Stale_Position or result.Stale_Angle > cell.Stale_Angle:
            result.Failures.append('stale: %.3f mm / %.3f deg from its %s target; print the corrected lines with '
                                   'Example/cell.py point %s' % (1000 * result.Stale_Position,
                                                                                        math.degrees(result.Stale_Angle),
                                                                                     'Reference' if point.Reference else 'PCB_Hold', name))
    # The clearance, posed in the scene: exact distances of the robot's geoms, and of the carried PCB's.
    Pose_Cell(kinematics, constants, simulation, cell, pcb_meta, name, fixture=fixture)
    contacts = collision.Close_Pairs(collision.Robot_Bodies, cell.Clearance, kinematics.Data)
    if scene.PCB == 'cups':
        contacts += collision.Close_Pairs({kinematics.Model.body('pcb').id}, cell.Clearance, kinematics.Data, exclude=collision.Robot_Bodies)
    allowed, closest = {}, (cell.Clearance, '')
    cup_block = str(Cup_Block_Geom(kinematics.Model))
    in_crate = Crate_Holding(kinematics.Data.xpos[kinematics.Model.body('pcb').id]) is not None
    for distance, body_1, body_2, geom_1, geom_2 in contacts:
        if body_1 == CONST_MOUNT_BODY and geom_2.startswith(CONST_MOUNT_GEOM_PREFIX):
            continue
        group = Contact_Group({body_1, body_2}, {geom_1, geom_2}, cup_block, in_crate)
        if group is not None and group in scene.Contacts:
            allowed[group] = min(allowed.get(group, cell.Clearance), distance)
        elif distance < closest[0]:
            closest = (distance, '%s / %s' % (body_1 if geom_1.isdigit() else geom_1, body_2 if geom_2.isdigit() else geom_2))
    result.Min_Distance, result.Min_Pair = closest
    result.Allowed = sorted(allowed.items())
    if closest[1]:
        result.Failures.append('%s %.2f mm apart (clearance %.1f mm)' % (closest[1], 1000 * closest[0], 1000 * cell.Clearance))
    for group, distance in result.Allowed:
        if -distance > cell.Allowed_Depth[group]:
            result.Failures.append('allowed contact %s is %.2f mm deep (at most %.2f mm)' % (group, -1000 * distance,
                                                                                          1000 * cell.Allowed_Depth[group]))
    result.Verdict = 'FAIL' if result.Failures else ('WARN' if result.Family_Near_Zero else 'PASS')
    return result


def Check_Keypoints(kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, constants: Constants_Str, simulation: Simulation_Str,
                    cell: Cell_Configuration_Str, pcb_meta: PCB_Meta_Str) -> tp.List[Keypoint_Check_Str]:
    """
    Description:
        The check of every robot point, in the file's order.

    Args:
        (1) kinematics [Robot_Kinematics_Cls(object)]: On the planning model.
        (2) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (3) constants [Constants_Str(object)]: The constants.
        (4) simulation [Simulation_Str(object)]: The scenes.
        (5) cell [Cell_Configuration_Str(object)]: The tolerances.
        (6) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.

    Returns:
        (1) parameter [List<Keypoint_Check_Str>]: One result per point.
    """

    return [Check_Keypoint(name, kinematics, collision, constants, simulation, cell, pcb_meta) for name in constants.Points]


def Format_Check_Table(results: tp.List[Keypoint_Check_Str]) -> tp.List[str]:
    """
    Description:
        The check as a table: point, family (? within the family margin), limits, the Pose error against FK, the stale
        error, the closest not-allowed contact (or the clearance), the allowed contacts with their depth, the verdict; then
        each failure.

    Args:
        (1) results [List<Keypoint_Check_Str>]: The results.

    Returns:
        (1) parameter [List<string>]: The lines.
    """

    lines = ['%-21s %-10s %-6s %8s %10s %-38s %-28s %s' % ('keypoint', 'family', 'limits', 'check', 'stale', 'closest (not allowed)',
                                                           'allowed contacts', 'verdict')]
    for r in results:
        family = r.Family + ('?' if r.Family_Near_Zero else '')
        stale = '-' if r.Stale_Position is None else '%.3f mm' % (1000 * r.Stale_Position)
        closest = ('%.2f mm %s' % (1000 * r.Min_Distance, r.Min_Pair)) if r.Min_Pair else '>= %.1f mm' % (1000 * r.Min_Distance)
        allowed = ', '.join('%s %.2f' % (g, 1000 * d) for g, d in r.Allowed) or '-'
        lines.append('%-21s %-10s %-6s %5.3f mm %10s %-38s %-28s %s' % (r.Name, family, 'ok' if r.Limits_Ok else 'OUT', 1000 * r.Check_Error,
                                                                     stale, closest[:38], allowed[:28], r.Verdict))
    for r in results:
        lines += ['  %s: %s' % (r.Name, failure) for failure in r.Failures]
        if r.Family_Near_Zero:
            lines.append('  %s: WARNING wrist_2 within the family margin of zero (the family switch, SPEC §4.7)' % r.Name)
    return lines


def Resolve_Keypoint(name: str, kinematics: Robot_Kinematics_Cls, constants: Constants_Str, simulation: Simulation_Str, pcb_meta: PCB_Meta_Str,
                     cell: Cell_Configuration_Str, seed: tp.Optional[str] = None) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        The joints that reach a point's Reference or PCB-hold target (IK on its tool's site, seeded from its own joints or
        another point's, refused when it does not converge or changes the IK family), and the Pose they give.

    Args:
        (1) name [string]: The point (with a Reference or a PCB_Hold).
        (2) kinematics [Robot_Kinematics_Cls(object)]: IK.
        (3) constants [Constants_Str(object)]: The points.
        (4) simulation [Simulation_Str(object)]: The scenes, the tools' sites.
        (5) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (6) cell [Cell_Configuration_Str(object)]: The base pose.
        (7) seed [string]: The point whose joints seed the IK, or None for the point's own.

    Returns:
        (1) parameter [Vector<float> 1x6]: The joints (rad).
        (2) parameter [Vector<float> 1x6]: The Pose: x, y, z (m), roll, pitch, yaw (rad) in the robot base frame.
    """

    point = constants.Points[name]
    target = Target_Pose(name, constants, simulation, pcb_meta, kinematics)
    if target is None:
        raise Keypoints_Error_Cls('Point %s has no Reference or PCB_Hold target to resolve; its Joints are its definition.' % name)
    start = np.array(constants.Points[seed or name].Joints)
    site = Site_Id(kinematics.Model, simulation, point.Tool)
    joints, ok = kinematics.Solve_IK(target[0], target[1], start, site=site, q_ref=start, iters=500)
    if not ok:
        raise Keypoints_Error_Cls('Point %s: IK did not reach its target from the seed %s; try another --seed.' % (name, seed or name))
    if Family(joints) != Family(point.Joints):
        raise Keypoints_Error_Cls('Point %s: the IK solution is in the other family (wrist_2 %.2f deg); try another --seed.'
                                  % (name, math.degrees(joints[CONST_WRIST_2])))
    position, rotation = Base_Pose(*kinematics.Site_Pose(joints, site), cell)
    return joints, np.r_[position, Matrix_To_RPY(rotation)]
