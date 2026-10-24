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
# File Name: ../Kinematics/Core.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Typing (Support for type hints)
import typing as tp
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.: the robot's joint names and the PCB / tool frames
#   ../Scene/Core
from Scene.Core import CONST_ROBOT_JOINTS, CONST_SUCTION_IN_PCB, CONST_R_TOOL_IN_PCB

"""
Description:
    Initialization of constants.

    Kinematics of the AUBO i5 in the cell (SPEC §7.3): site poses of a robot configuration,
    damped-least-squares IK on the suction TCP or the flange with a null-space pull to a reference,
    and the TCP pose that holds the PCB at a given pose. It works on a model and data handed to it.
"""
# The suction TCP (cup face, 260.6 mm from the flange) and the flange sites.
#   Unit [string]
CONST_SITE_TCP = 'aubo/tool/suction_tcp'
CONST_SITE_FLANGE = 'aubo/tool/tool_flange'
# The scene joints set by Set_Scene: MG06 lid, press plate, handle; loader Y, Z, jaws; the free PCB.
#   Unit [List<string>]
CONST_SCENE_JOINTS = ['lid_hinge', 'press_slide', 'handle_hinge', 'y_axis', 'z_axis', 'jaw_L', 'jaw_R', 'pcb_free']
# DLS IK: damping, convergence tolerance (position; the orientation tolerance is 10x), null-space gain,
# default iteration count, and the position error below which a non-converged result still counts.
#   Unit [float] (-), [float] (m), [float] (-), [int], [float] (m)
CONST_IK_DAMPING, CONST_IK_TOL, CONST_IK_NULL_GAIN, CONST_IK_ITERATIONS, CONST_IK_ACCEPT = 1e-4, 1e-5, 0.02, 200, 1e-3


class Robot_Kinematics_Cls(object):
    """
    Description:
        Site poses and IK of the AUBO i5 on a given cell model; the scene joints (MG06, loader, PCB) are
        set with Set_Scene.

    Initialization of the Class:
        Args:
            (1) model [mujoco.MjModel]: The compiled cell.
            (2) data [mujoco.MjData]: Its data (default: a new one).

        Example:
            Initialization:
                # Initialization of the class.
                Cls = Robot_Kinematics_Cls(model)

            Features:
                # The TCP pose of a configuration, and IK back to it.
                p, R = Cls.Site_Pose(q)
                q_ik, ok = Cls.Solve_IK(p, R, q0)
    """

    def __init__(self, model: mujoco.MjModel, data: tp.Optional[mujoco.MjData] = None) -> None:
        # << PRIVATE >> #
        self.__model = model
        self.__data = mujoco.MjData(model) if data is None else data
        joint_ids = [model.joint('aubo/' + j).id for j in CONST_ROBOT_JOINTS]
        self.__q_address = np.array([model.jnt_qposadr[j] for j in joint_ids])
        self.__dof_address = np.array([model.jnt_dofadr[j] for j in joint_ids])
        self.__lower, self.__upper = model.jnt_range[joint_ids, 0], model.jnt_range[joint_ids, 1]
        self.__site_tcp = model.site(CONST_SITE_TCP).id
        self.__site_flange = model.site(CONST_SITE_FLANGE).id
        self.__scene_address = {n: model.jnt_qposadr[model.joint(n).id] for n in CONST_SCENE_JOINTS}

    @property
    def Model(self) -> mujoco.MjModel:
        """
        Description:
            The model.

        Returns:
            (1) parameter [mujoco.MjModel]: The compiled cell.
        """

        return self.__model

    @property
    def Data(self) -> mujoco.MjData:
        """
        Description:
            The default data.

        Returns:
            (1) parameter [mujoco.MjData]: The data used when none is passed.
        """

        return self.__data

    @property
    def Q_Address(self) -> np.ndarray:
        """
        Description:
            The qpos addresses of the six robot joints.

        Returns:
            (1) parameter [Vector<int> 1x6]: The addresses.
        """

        return self.__q_address

    @property
    def Dof_Address(self) -> np.ndarray:
        """
        Description:
            The dof addresses of the six robot joints.

        Returns:
            (1) parameter [Vector<int> 1x6]: The addresses.
        """

        return self.__dof_address

    @property
    def Limits(self) -> tp.Tuple[np.ndarray, np.ndarray]:
        """
        Description:
            The joint limits.

        Returns:
            (1 - 2) parameter [Vector<float> 1x6]: Lower and upper limits (rad).
        """

        return self.__lower, self.__upper

    @property
    def Site_TCP(self) -> int:
        """
        Description:
            The id of the suction TCP site.

        Returns:
            (1) parameter [int]: The site id.
        """

        return self.__site_tcp

    @property
    def Site_Flange(self) -> int:
        """
        Description:
            The id of the flange site.

        Returns:
            (1) parameter [int]: The site id.
        """

        return self.__site_flange

    @property
    def Scene_Address(self) -> tp.Dict[str, int]:
        """
        Description:
            The qpos addresses of the scene joints (CONST_SCENE_JOINTS).

        Returns:
            (1) parameter [Dict<string, int>]: Joint name -> qpos address.
        """

        return dict(self.__scene_address)

    def Set_Scene(self, data: tp.Optional[mujoco.MjData] = None, lid: float = -0.6974, handle: float = 0.0, press: float = 0.0,
                  y: float = -0.1, z: float = 0.0, jaw: float = 0.035, pcb: tp.Optional[np.ndarray] = None) -> None:
        """
        Description:
            Set the scene joints: MG06 lid, handle and press plate, loader axes and jaws, and the PCB pose
            (or the PCB moved out of the way, to (0, 3, 3) m).

        Args:
            (1) data [mujoco.MjData]: The data (default: the class's).
            (2 - 4) lid, handle, press [float]: MG06 joints (rad, rad, m).
            (5 - 7) y, z, jaw [float]: Loader Y, Z and each jaw (m).
            (8) pcb [Vector<float> 1x7]: The PCB's position and quaternion, or None.
        """

        d = self.__data if data is None else data
        a = self.__scene_address
        d.qpos[a['lid_hinge']], d.qpos[a['handle_hinge']], d.qpos[a['press_slide']] = lid, handle, press
        d.qpos[a['y_axis']], d.qpos[a['z_axis']], d.qpos[a['jaw_L']], d.qpos[a['jaw_R']] = y, z, jaw, jaw
        if pcb is not None:
            d.qpos[a['pcb_free']:a['pcb_free'] + 7] = pcb
        else:
            d.qpos[a['pcb_free']:a['pcb_free'] + 3] = [0, 3, 3]

    def Site_Pose(self, q: np.ndarray, site: tp.Optional[int] = None,
                  data: tp.Optional[mujoco.MjData] = None) -> tp.Tuple[np.ndarray, np.ndarray]:
        """
        Description:
            The pose of a site for a robot configuration (kinematics only).

        Args:
            (1) q [Vector<float> 1x6]: The joints (rad).
            (2) site [int]: The site id (default: the suction TCP).
            (3) data [mujoco.MjData]: The data (default: the class's).

        Returns:
            (1) parameter [Vector<float> 1x3]: The site position (m).
            (2) parameter [Matrix<float> 3x3]: Its orientation.
        """

        site = self.__site_tcp if site is None else site
        d = self.__data if data is None else data
        d.qpos[self.__q_address] = q
        mujoco.mj_kinematics(self.__model, d)
        return d.site_xpos[site].copy(), d.site_xmat[site].reshape(3, 3).copy()

    def Solve_IK(self, pos: np.ndarray, mat: np.ndarray, q0: np.ndarray, site: tp.Optional[int] = None,
                 q_ref: tp.Optional[np.ndarray] = None, iters: int = CONST_IK_ITERATIONS, tol: float = CONST_IK_TOL,
                 data: tp.Optional[mujoco.MjData] = None, w_null: float = CONST_IK_NULL_GAIN) -> tp.Tuple[np.ndarray, bool]:
        """
        Description:
            Damped-least-squares IK of a site to a pose, with a null-space pull towards q_ref, the joints
            clipped to their limits at each step.

        Args:
            (1 - 2) pos, mat [Vector<float> 1x3, Matrix<float> 3x3]: The target position (m) and orientation.
            (3) q0 [Vector<float> 1x6]: The start (rad).
            (4) site [int]: The site id (default: the suction TCP).
            (5) q_ref [Vector<float> 1x6]: The null-space reference (default: the start q0).
            (6) iters [int]: The iteration limit.
            (7) tol [float]: The position tolerance (m); the orientation tolerance is 10 x tol.
            (8) data [mujoco.MjData]: The data (default: the class's).
            (9) w_null [float]: The null-space gain.

        Returns:
            (1) parameter [Vector<float> 1x6]: The joints (rad).
            (2) parameter [bool]: True when converged, or within CONST_IK_ACCEPT in position at the end.
        """

        site = self.__site_tcp if site is None else site
        d = self.__data if data is None else data
        m = self.__model
        q = np.array(q0, float)
        q_ref = q if q_ref is None else np.asarray(q_ref)
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        for _ in range(iters):
            d.qpos[self.__q_address] = q
            mujoco.mj_kinematics(m, d)
            mujoco.mj_comPos(m, d)
            ep = pos - d.site_xpos[site]
            cur = d.site_xmat[site].reshape(3, 3)
            er = 0.5 * (np.cross(cur[:, 0], mat[:, 0]) + np.cross(cur[:, 1], mat[:, 1]) + np.cross(cur[:, 2], mat[:, 2]))
            e = np.r_[ep, er]
            if np.linalg.norm(ep) < tol and np.linalg.norm(er) < 10 * tol:
                return q, True
            mujoco.mj_jacSite(m, d, jp, jr, site)
            J = np.vstack([jp[:, self.__dof_address], jr[:, self.__dof_address]])
            JJ = J @ J.T + CONST_IK_DAMPING * np.eye(6)
            dq = J.T @ np.linalg.solve(JJ, e)
            N = np.eye(6) - J.T @ np.linalg.solve(JJ, J)
            dq += N @ (w_null * (q_ref - q))
            q = np.clip(q + dq, self.__lower, self.__upper)
        d.qpos[self.__q_address] = q
        mujoco.mj_kinematics(m, d)
        ok = np.linalg.norm(pos - d.site_xpos[site]) < CONST_IK_ACCEPT
        return q, ok


def Quaternion_To_Matrix(q: np.ndarray) -> np.ndarray:
    """
    Description:
        A quaternion (w first) as a rotation matrix.

    Args:
        (1) q [Vector<float> 1x4]: The quaternion.

    Returns:
        (1) parameter [Matrix<float> 3x3]: The rotation matrix.
    """

    m = np.zeros(9)
    mujoco.mju_quat2Mat(m, np.asarray(q, float))
    return m.reshape(3, 3)


def TCP_For_PCB(pcb_pos: np.ndarray, pcb_quat: np.ndarray) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        The suction TCP pose that holds the PCB at the given pose (SPEC §4.4: the TCP in the PCB frame,
        tool axes diag(1, -1, -1)).

    Args:
        (1 - 2) pcb_pos, pcb_quat [Vector<float> 1x3, Vector<float> 1x4]: The PCB pose (m, w first).

    Returns:
        (1) parameter [Vector<float> 1x3]: The TCP position (m).
        (2) parameter [Matrix<float> 3x3]: The TCP orientation.
    """

    R = Quaternion_To_Matrix(pcb_quat)
    return pcb_pos + R @ CONST_SUCTION_IN_PCB, R @ CONST_R_TOOL_IN_PCB


def PCB_For_TCP(tcp_pos: np.ndarray, tcp_mat: np.ndarray) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        The PCB pose held by the cups at a suction TCP pose: the inverse of TCP_For_PCB.

    Args:
        (1 - 2) tcp_pos, tcp_mat [Vector<float> 1x3, Matrix<float> 3x3]: The TCP pose (m).

    Returns:
        (1) parameter [Vector<float> 1x3]: The PCB centre (m).
        (2) parameter [Vector<float> 1x4]: Its quaternion, w first.
    """

    R = tcp_mat @ CONST_R_TOOL_IN_PCB.T
    quat = np.zeros(4)
    mujoco.mju_mat2Quat(quat, R.flatten())
    return tcp_pos - R @ CONST_SUCTION_IN_PCB, quat



def RPY_To_Matrix(rpy: tp.Sequence[float]) -> np.ndarray:
    """
    Description:
        A rotation from fixed-axis roll, pitch, yaw: R = Rz(yaw) Ry(pitch) Rx(roll) (the Constants.yaml Pose convention).

    Args:
        (1) rpy [Vector<float> 1x3]: Roll, pitch, yaw (rad).

    Returns:
        (1) parameter [Matrix<float> 3x3]: The rotation matrix.
    """

    (cr, sr), (cp, sp), (cy, sy) = [(np.cos(a), np.sin(a)) for a in rpy]
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def Matrix_To_RPY(R: np.ndarray) -> np.ndarray:
    """
    Description:
        Fixed-axis roll, pitch, yaw of a rotation, R = Rz(yaw) Ry(pitch) Rx(roll) (the inverse of RPY_To_Matrix); roll and yaw
        in (-pi, pi], pitch in [-pi/2, pi/2].

    Args:
        (1) R [Matrix<float> 3x3]: The rotation matrix.

    Returns:
        (1) parameter [Vector<float> 1x3]: Roll, pitch, yaw (rad).
    """

    rpy = np.array([np.arctan2(R[2, 1], R[2, 2]), np.arcsin(-np.clip(R[2, 0], -1.0, 1.0)), np.arctan2(R[1, 0], R[0, 0])])
    return np.where(rpy <= -np.pi, rpy + 2.0 * np.pi, rpy)


def Degrees_Text(angle: float, decimals: int = 9) -> str:
    """
    Description:
        An angle as degrees text normalized to (-180, 180] after the rounding (-179.9999999999 is written 180.000000000), the
        spelling of a Constants.yaml Pose angle.

    Args:
        (1) angle [float]: The angle (rad).
        (2) decimals [int]: The decimals written.

    Returns:
        (1) parameter [string]: The text.
    """

    degrees = math.degrees(angle) % 360.0
    text = '%.*f' % (decimals, degrees - 360.0 if degrees > 180.0 else degrees)
    return '%.*f' % (decimals, 180.0) if float(text) <= -180.0 else text


def Rotation_Angle(R_1: np.ndarray, R_2: np.ndarray) -> float:
    """
    Description:
        The angle of the rotation between two orientations.

    Args:
        (1 - 2) R_1, R_2 [Matrix<float> 3x3]: The orientations.

    Returns:
        (1) parameter [float]: The angle (rad).
    """

    return float(np.arccos(np.clip((np.trace(R_1.T @ R_2) - 1) / 2, -1.0, 1.0)))
