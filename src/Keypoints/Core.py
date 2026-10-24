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
#   ../Configuration/Core - the configuration's structures
from Configuration.Core import Cell_Configuration_Str, Keypoints_Str, Robot_Keypoint_Str, PCB_Meta_Str
#   ../Scene/Core - the crates
from Scene.Core import PCB_In_Crate, Cup_Block_Geom
#   ../Kinematics/Core - FK, IK, the PCB held by the cups
from Kinematics.Core import Robot_Kinematics_Cls, TCP_For_PCB, PCB_For_TCP, Quaternion_To_Matrix
#   ../Collision/Core - contacts of the robot and of the carried PCB
from Collision.Core import Robot_Collision_Cls

"""
Description:
    Initialization of constants.

    The keypoints of Configuration/Keypoints.yaml at work (SPEC §2 item 3): the target of a Relative_To, the
    MG06 state of a Fixture, posing the whole cell at a keypoint (robot, loader, jaws, PCB, MG06), the
    keypoint check, and the IK resolution of a keypoint's joints. Posing and playback use a keypoint's Joints
    only; IK runs only in Resolve_Keypoint, whose result a person pastes into the file. It prints nothing.
"""
# The sites a keypoint's Check and Relative_To name.
#   Unit [Dict<string, string>]
CONST_FRAME_SITES = {'suction_tcp': 'aubo/tool/suction_tcp', 'tool_flange': 'aubo/tool/tool_flange'}
# The joints posed for the scene.
#   Unit [string]
CONST_LID, CONST_HANDLE, CONST_PRESS = 'lid_hinge', 'handle_hinge', 'press_slide'
# wrist_2's index in the joint vector (its sign decides the IK family, SPEC §4.7).
#   Unit [int]
CONST_WRIST_2 = 4
# The robot's mount: its base (no joint) stands on the pedestal plate, distance 0 by construction. The one pair
# never checked (a single legal value, as the template's base-env_stand exclusion): the base link against the
# plate's collision geoms.
#   Unit [Tuple<string>]
CONST_MOUNT_BODY, CONST_MOUNT_GEOM_PREFIX = 'aubo/base_link', 'ws_col_deska_robot'


class Keypoints_Error_Cls(ValueError):
    """
    Description:
        A keypoint cannot be posed or resolved (an unknown name or location, a resolution that would change
        the IK family). The message names the keypoint and what to do.
    """


@dataclass
class Keypoint_Check_Str:
    """
    Description:
        The check of one keypoint (SPEC §2 item 3): each rule's measured value and the verdict.
    """

    # The keypoint.
    #   Unit [string]
    Name: str = ''
    # Joints within the model's limits; the IK family as declared; wrist_2 within the family margin of zero.
    #   Unit [bool]
    Limits_Ok: bool = True
    Family_Ok: bool = True
    Family_Near_Zero: bool = False
    # FK at the Check site against the Check position.
    #   Unit [float] (m)
    Check_Error: float = 0.0
    # FK against the Relative_To target (None when the keypoint has none).
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
    # Every failed rule, as a sentence; empty when the keypoint passes.
    #   Unit [List<string>]
    Failures: tp.List[str] = field(default_factory=list)
    # PASS, WARN (passes with a warning) or FAIL.
    #   Unit [string]
    Verdict: str = ''


def Site_Id(model: mujoco.MjModel, frame: str) -> int:
    """
    Description:
        The model site of a keypoint frame name.

    Args:
        (1) model [mujoco.MjModel]: The model.
        (2) frame [string]: suction_tcp or tool_flange.

    Returns:
        (1) parameter [int]: The site id.
    """

    return model.site(CONST_FRAME_SITES[frame]).id


def PCB_Pose(location: str, pcb_meta: PCB_Meta_Str, kinematics: Robot_Kinematics_Cls, joints: np.ndarray) -> np.ndarray:
    """
    Description:
        The PCB pose at a named location: flat on a crate floor (input_crate, output_crate), at the hold pose
        in the jaws (jaws), placed in MG06 (mg06), or held by the cups at the given joints (cups).

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


def Target_Pose(keypoint: Robot_Keypoint_Str, keypoints: Keypoints_Str, pcb_meta: PCB_Meta_Str,
                kinematics: Robot_Kinematics_Cls) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        The pose a keypoint's Relative_To asks for, at its frame's site: the TCP holding the PCB at a location
        (pcb_hold@<location>) or another keypoint's site pose, moved by the offset along that frame's axes.

    Args:
        (1) keypoint [Robot_Keypoint_Str(object)]: The keypoint (with a Relative_To).
        (2) keypoints [Keypoints_Str(object)]: All keypoints (a keypoint target).
        (3) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (4) kinematics [Robot_Kinematics_Cls(object)]: FK.

    Returns:
        (1) parameter [Vector<float> 1x3]: The target position (m).
        (2) parameter [Matrix<float> 3x3]: The target orientation.
    """

    target = keypoint.Relative_Target
    if target.startswith('pcb_hold@'):
        pose = PCB_Pose(target[len('pcb_hold@'):], pcb_meta, kinematics, np.zeros(6))
        position, rotation = TCP_For_PCB(pose[:3], pose[3:])
        if keypoint.Relative_Frame != 'suction_tcp':
            raise Keypoints_Error_Cls('Keypoint %s: a pcb_hold target is a suction_tcp pose; set Relative_To.Frame to suction_tcp.' % keypoint.Name)
    else:
        position, rotation = kinematics.Site_Pose(np.array(keypoints.Robot[target].Joints), Site_Id(kinematics.Model, keypoint.Relative_Frame))
    return position + rotation @ np.array(keypoint.Relative_Offset), rotation


def Fixture_State(keypoint: Robot_Keypoint_Str, cell: Cell_Configuration_Str, model: mujoco.MjModel,
                  fixture: tp.Optional[str] = None) -> tp.Tuple[float, float, float]:
    """
    Description:
        The MG06 lid, handle and press plate of a keypoint's Fixture (or of the named state fixture, which
        overrides it): Open = the lid at its open stop (the model) and the handle up; Closed = Cell.yaml MG06;
        the recorded state as given. The press plate follows the handle through the model's equality.

    Args:
        (1) keypoint [Robot_Keypoint_Str(object)]: The keypoint.
        (2) cell [Cell_Configuration_Str(object)]: The closed state.
        (3) model [mujoco.MjModel]: The model (the lid's stop, the press coupling).
        (4) fixture [string]: 'Open' or 'Closed' to override, or None.

    Returns:
        (1) parameter [Tuple<float>]: lid (rad), handle (rad), press (m).
    """

    state = fixture or keypoint.Fixture
    if state == 'Open':
        lid, handle = float(model.jnt_range[model.joint(CONST_LID).id][0]), 0.0
    elif state == 'Closed':
        lid, handle = cell.Closed_Lid, cell.Closed_Handle
    elif state == 'Recorded':
        lid, handle = keypoint.Fixture_Lid, keypoint.Fixture_Handle
    else:
        raise Keypoints_Error_Cls("The MG06 state '%s' is not Open or Closed." % state)
    press_id, handle_id = model.joint(CONST_PRESS).id, model.joint(CONST_HANDLE).id
    coupling = next(i for i in range(model.neq) if model.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT
                    and model.eq_obj1id[i] == press_id and model.eq_obj2id[i] == handle_id)
    press = float(sum(model.eq_data[coupling][k] * handle**k for k in range(5)))
    return lid, handle, press


def Pose_Cell(kinematics: Robot_Kinematics_Cls, keypoints: Keypoints_Str, cell: Cell_Configuration_Str, pcb_meta: PCB_Meta_Str,
              name: str, loader: tp.Optional[str] = None, gripper: tp.Optional[str] = None, pcb: tp.Optional[str] = None,
              fixture: tp.Optional[str] = None) -> Robot_Keypoint_Str:
    """
    Description:
        Pose the whole cell at a keypoint in the kinematics' data: the robot at its Joints, the suction cups'
        bellows at rest, the loader axes and jaws, the PCB at its location, MG06 in its state - from the
        keypoint's Context and Fixture, each overridable. Ends with mj_forward.

    Args:
        (1) kinematics [Robot_Kinematics_Cls(object)]: The model and the data to pose.
        (2) keypoints [Keypoints_Str(object)]: The keypoints.
        (3) cell [Cell_Configuration_Str(object)]: The cell configuration.
        (4) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (5) name [string]: The keypoint.
        (6 - 9) loader, gripper, pcb, fixture [string]: Overrides of the Context and the Fixture, or None.

    Returns:
        (1) parameter [Robot_Keypoint_Str(object)]: The keypoint posed.
    """

    if name not in keypoints.Robot:
        raise Keypoints_Error_Cls("There is no keypoint '%s' in Configuration/Keypoints.yaml; the keypoints are %s."
                                  % (name, ', '.join(keypoints.Robot)))
    keypoint = keypoints.Robot[name]
    model, data = kinematics.Model, kinematics.Data
    loader_name, gripper_name = loader or keypoint.Loader, gripper or keypoint.Gripper
    if loader_name not in keypoints.Loader or gripper_name not in keypoints.Gripper:
        raise Keypoints_Error_Cls("Unknown loader position '%s' or gripper state '%s'; they are %s and %s."
                                  % (loader_name, gripper_name, ', '.join(keypoints.Loader), ', '.join(keypoints.Gripper)))
    joints = np.array(keypoint.Joints)
    mujoco.mj_resetData(model, data)
    lid, handle, press = Fixture_State(keypoint, cell, model, fixture)
    position = keypoints.Loader[loader_name]
    kinematics.Set_Scene(data, lid=lid, handle=handle, press=press, y=position.Y, z=position.Z, jaw=keypoints.Gripper[gripper_name],
                         pcb=PCB_Pose(pcb or keypoint.PCB, pcb_meta, kinematics, joints))
    data.qpos[kinematics.Q_Address] = joints
    mujoco.mj_forward(model, data)
    return keypoint


def _Group(names: tp.Set[str], geoms: tp.Set[str], cup_block: str) -> tp.Optional[str]:
    """
    Description:
        The contact group of a pair of bodies (and their geoms), or None for a pair no group covers. cups_pcb is
        the VGC10's face on the PCB: a cup, or the cup block (not the fork, which is on the same tool body).

    Args:
        (1) names [Set<string>]: The two bodies.
        (2) geoms [Set<string>]: The two geoms.
        (3) cup_block [string]: The cup block's geom (its id as text, it has no name).

    Returns:
        (1) parameter [string]: cups_pcb, handle_bar, pcb_crate, or None.
    """

    if 'pcb' in names and (any(n.startswith('aubo/tool/vgc10_cup') for n in names) or cup_block in geoms):
        return 'cups_pcb'
    if names == {'aubo/tool/ee_tool', 'mg06_handle'}:
        return 'handle_bar'
    if 'pcb' in names and 'workstation' in names and any(g.startswith('crate_') for g in geoms):
        return 'pcb_crate'
    return None


def Check_Keypoint(name: str, kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, keypoints: Keypoints_Str,
                   cell: Cell_Configuration_Str, pcb_meta: PCB_Meta_Str, fixture: tp.Optional[str] = None) -> Keypoint_Check_Str:
    """
    Description:
        The check of one keypoint (SPEC §2 item 3), posed in its Context and Fixture (or the Fixture override):
        joint limits (the model); the IK family (the sign of wrist_2, a WARNING within the family margin); the
        Check value; the Relative_To target (stale); the clearance of the robot and tool - and of the PCB when the
        cups carry it - to the scene, every contact closer than the clearance failing unless its group is in
        Allow_Contact and no deeper than that group's allowed depth. Distances are exact (Collision.Close_Pairs);
        the robot's mount (its base on the pedestal plate) is not checked.

    Args:
        (1) name [string]: The keypoint.
        (2) kinematics [Robot_Kinematics_Cls(object)]: On the cell model.
        (3) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (4) keypoints [Keypoints_Str(object)]: The keypoints.
        (5) cell [Cell_Configuration_Str(object)]: The tolerances.
        (6) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (7) fixture [string]: 'Open' or 'Closed' to override the keypoint's MG06 state, or None.

    Returns:
        (1) parameter [Keypoint_Check_Str(object)]: The result.
    """

    keypoint = keypoints.Robot[name]
    joints = np.array(keypoint.Joints)
    result = Keypoint_Check_Str(Name=name)
    lower, upper = kinematics.Limits
    if np.any(joints < lower) or np.any(joints > upper):
        result.Limits_Ok = False
        result.Failures.append('joints outside the model limits: %s' % ', '.join('J%d %.2f deg' % (i + 1, math.degrees(q))
                                                                            for i, q in enumerate(joints) if not lower[i] <= q <= upper[i]))
    wrist_2 = joints[CONST_WRIST_2]
    result.Family_Near_Zero = abs(wrist_2) < cell.Family_Margin
    result.Family_Ok = result.Family_Near_Zero or (wrist_2 > 0) == (keypoint.Family == 'wrist_up')
    if not result.Family_Ok:
        result.Failures.append('family %s, but wrist_2 is %.2f deg' % (keypoint.Family, math.degrees(wrist_2)))
    position, _ = kinematics.Site_Pose(joints, Site_Id(kinematics.Model, keypoint.Check_Frame))
    result.Check_Error = float(np.linalg.norm(position - keypoint.Check_Position))
    if result.Check_Error > cell.Check_Position:
        result.Failures.append('%s %.3f mm from its Check position (tolerance %.3f mm)' % (keypoint.Check_Frame, 1000 * result.Check_Error,
                                                                                           1000 * cell.Check_Position))
    if keypoint.Relative_Target is not None:
        target, target_rotation = Target_Pose(keypoint, keypoints, pcb_meta, kinematics)
        p, R = kinematics.Site_Pose(joints, Site_Id(kinematics.Model, keypoint.Relative_Frame))
        result.Stale_Position = float(np.linalg.norm(p - target))
        result.Stale_Angle = float(np.arccos(np.clip((np.trace(target_rotation.T @ R) - 1) / 2, -1.0, 1.0)))
        if result.Stale_Position > cell.Stale_Position or result.Stale_Angle > cell.Stale_Angle:
            result.Failures.append('stale: %.3f mm / %.3f deg from its Relative_To target; print the corrected line with '
                                   'Example/Keypoints/resolve_keypoint.py --name %s' % (1000 * result.Stale_Position,
                                                                                     math.degrees(result.Stale_Angle), name))
    # The clearance, posed in the scene: exact distances of the robot's geoms, and of the carried PCB's.
    Pose_Cell(kinematics, keypoints, cell, pcb_meta, name, fixture=fixture)
    contacts = collision.Close_Pairs(collision.Robot_Bodies, cell.Clearance, kinematics.Data)
    if keypoint.PCB == 'cups':
        contacts += collision.Close_Pairs({kinematics.Model.body('pcb').id}, cell.Clearance, kinematics.Data, exclude=collision.Robot_Bodies)
    allowed, closest = {}, (cell.Clearance, '')
    cup_block = str(Cup_Block_Geom(kinematics.Model))
    for distance, body_1, body_2, geom_1, geom_2 in contacts:
        if body_1 == CONST_MOUNT_BODY and geom_2.startswith(CONST_MOUNT_GEOM_PREFIX):
            continue
        group = _Group({body_1, body_2}, {geom_1, geom_2}, cup_block)
        if group is not None and group in keypoint.Allow_Contact:
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


def Check_Keypoints(kinematics: Robot_Kinematics_Cls, collision: Robot_Collision_Cls, keypoints: Keypoints_Str,
                    cell: Cell_Configuration_Str, pcb_meta: PCB_Meta_Str) -> tp.List[Keypoint_Check_Str]:
    """
    Description:
        The check of every robot keypoint, in the file's order.

    Args:
        (1) kinematics [Robot_Kinematics_Cls(object)]: On the cell model.
        (2) collision [Robot_Collision_Cls(object)]: On the same model and data.
        (3) keypoints [Keypoints_Str(object)]: The keypoints.
        (4) cell [Cell_Configuration_Str(object)]: The tolerances.
        (5) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.

    Returns:
        (1) parameter [List<Keypoint_Check_Str>]: One result per keypoint.
    """

    return [Check_Keypoint(name, kinematics, collision, keypoints, cell, pcb_meta) for name in keypoints.Robot]


def Format_Check_Table(results: tp.List[Keypoint_Check_Str], keypoints: Keypoints_Str) -> tp.List[str]:
    """
    Description:
        The check as a table: keypoint, family, limits, Check error, stale error, the closest not-allowed
        contact (or the clearance), the allowed contacts with their depth, the verdict; then each failure.

    Args:
        (1) results [List<Keypoint_Check_Str>]: The results.
        (2) keypoints [Keypoints_Str(object)]: The keypoints (their family).

    Returns:
        (1) parameter [List<string>]: The lines.
    """

    lines = ['%-21s %-10s %-6s %8s %10s %-38s %-28s %s' % ('keypoint', 'family', 'limits', 'check', 'stale', 'closest (not allowed)',
                                                           'allowed contacts', 'verdict')]
    for r in results:
        family = keypoints.Robot[r.Name].Family + ('?' if r.Family_Near_Zero else ('' if r.Family_Ok else '!'))
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


def Resolve_Keypoint(name: str, kinematics: Robot_Kinematics_Cls, keypoints: Keypoints_Str, pcb_meta: PCB_Meta_Str,
                     seed: tp.Optional[str] = None) -> np.ndarray:
    """
    Description:
        The joints that reach a keypoint's Relative_To target: IK on its frame's site, seeded from the keypoint's
        own joints (or another keypoint's), refused when it does not converge or changes the IK family.

    Args:
        (1) name [string]: The keypoint (with a Relative_To).
        (2) kinematics [Robot_Kinematics_Cls(object)]: IK.
        (3) keypoints [Keypoints_Str(object)]: The keypoints.
        (4) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (5) seed [string]: The keypoint whose joints seed the IK, or None for the keypoint's own.

    Returns:
        (1) parameter [Vector<float> 1x6]: The joints (rad).
    """

    keypoint = keypoints.Robot[name]
    if keypoint.Relative_Target is None:
        raise Keypoints_Error_Cls('Keypoint %s has no Relative_To target to resolve; its Joints are its definition.' % name)
    target, rotation = Target_Pose(keypoint, keypoints, pcb_meta, kinematics)
    start = np.array(keypoints.Robot[seed or name].Joints)
    joints, ok = kinematics.Solve_IK(target, rotation, start, site=Site_Id(kinematics.Model, keypoint.Relative_Frame), q_ref=start, iters=500)
    if not ok:
        raise Keypoints_Error_Cls('Keypoint %s: IK did not reach its Relative_To target from the seed %s; try another --seed.' % (name, seed or name))
    if (joints[CONST_WRIST_2] > 0) != (keypoint.Family == 'wrist_up'):
        raise Keypoints_Error_Cls('Keypoint %s: the IK solution is in the other family (wrist_2 %.2f deg); try another --seed.'
                                  % (name, math.degrees(joints[CONST_WRIST_2])))
    return joints
