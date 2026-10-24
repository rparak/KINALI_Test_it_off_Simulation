"""Legacy shim: the names of the former robot_kin.py, mapped onto src/ (Kinematics.Core, Collision.Core) for
legacy/plan_cell.py, which keeps its code unchanged until step 6. One planning model and one MjData are shared
by the kinematics and the collision queries, as in the former module."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))
import mujoco
import cell   # noqa: F401  (routes MuJoCo's log into logger/)
from Kinematics.Core import Robot_Kinematics_Cls, Quaternion_To_Matrix, TCP_For_PCB
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls, CONST_PLANNING_CLEARANCE

M = Build_Planning_Model()
D = mujoco.MjData(M)
_kin = Robot_Kinematics_Cls(M, D)
_col = Robot_Collision_Cls(M, D)
QADR, DADR = _kin.Q_Address, _kin.Dof_Address
LO, HI = _kin.Limits
SITE_TCP, SITE_FLANGE = _kin.Site_TCP, _kin.Site_Flange
ROBOT_BODIES = _col.Robot_Bodies
CLEAR = CONST_PLANNING_CLEARANCE
_a = _kin.Scene_Address
QA, QP, QH, QY, QZ, QJL, QJR, QPCB = (_a[n] for n in ['lid_hinge', 'press_slide', 'handle_hinge', 'y_axis', 'z_axis', 'jaw_L', 'jaw_R', 'pcb_free'])


def set_scene(d=D, lid=-0.6974, handle=0.0, press=0.0, y=-0.1, z=0.0, jaw=0.035, pcb=None):
    _kin.Set_Scene(d, lid=lid, handle=handle, press=press, y=y, z=z, jaw=jaw, pcb=pcb)


def site_pose(q, site=SITE_TCP, d=D):
    return _kin.Site_Pose(q, site, d)


def ik(pos, mat, q0, site=SITE_TCP, q_ref=None, iters=200, tol=1e-5, d=D, w_null=0.02):
    return _kin.Solve_IK(pos, mat, q0, site=site, q_ref=q_ref, iters=iters, tol=tol, data=d, w_null=w_null)


def contacts(q, d=D, ignore_pcb=False, margin=0.0):
    return _col.Contacts(q, d, ignore_pcb=ignore_pcb, margin=margin)


def clearance(q, d=D, cut=0.05, skip=()):
    return _col.Clearance(q, d, cut=cut, skip=skip)


quat2mat, tcp_for_pcb = Quaternion_To_Matrix, TCP_For_PCB
