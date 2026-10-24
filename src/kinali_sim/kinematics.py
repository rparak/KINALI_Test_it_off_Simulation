"""Kinematics helpers for the AUBO i5 in the MG06 cell: IK (damped least squares), Cartesian targets for the
suction TCP / flange, and collision checks of a robot configuration against the scene."""
import numpy as np, mujoco
import cell   # legacy/cell.py (F3.3 transition; F3.4 replaces this module)

M = cell.build_model()
D = mujoco.MjData(M)
JID = [M.joint('aubo/' + j).id for j in cell.ROBOT_JOINTS]
QADR = np.array([M.jnt_qposadr[j] for j in JID]); DADR = np.array([M.jnt_dofadr[j] for j in JID])
LO, HI = M.jnt_range[JID, 0], M.jnt_range[JID, 1]
SITE_TCP = M.site('aubo/tool/suction_tcp').id
SITE_FLANGE = M.site('aubo/tool/tool_flange').id
ROBOT_BODIES = {i for i in range(M.nbody) if M.body(i).name.startswith('aubo/')}
CLEAR = 0.003                                   # planning clearance: robot closer than 3 mm to anything = collision
for _g in range(M.ngeom):
    if M.geom_bodyid[_g] in ROBOT_BODIES or M.body(M.geom_bodyid[_g]).name == 'pcb': M.geom_margin[_g] = 0.006
J = lambda n: M.jnt_qposadr[M.joint(n).id]
QA, QP, QH, QY, QZ, QJL, QJR, QPCB = (J(n) for n in ['lid_hinge', 'press_slide', 'handle_hinge', 'y_axis', 'z_axis', 'jaw_L', 'jaw_R', 'pcb_free'])


def set_scene(d=D, lid=-0.6974, handle=0.0, press=0.0, y=-0.1, z=0.0, jaw=0.035, pcb=None):
    d.qpos[QA], d.qpos[QH], d.qpos[QP] = lid, handle, press
    d.qpos[QY], d.qpos[QZ], d.qpos[QJL], d.qpos[QJR] = y, z, jaw, jaw
    if pcb is not None: d.qpos[QPCB:QPCB + 7] = pcb
    else: d.qpos[QPCB:QPCB + 3] = [0, 3, 3]                      # out of the way


def site_pose(q, site=SITE_TCP, d=D):
    d.qpos[QADR] = q; mujoco.mj_kinematics(M, d)
    return d.site_xpos[site].copy(), d.site_xmat[site].reshape(3, 3).copy()


def ik(pos, mat, q0, site=SITE_TCP, q_ref=None, iters=200, tol=1e-5, d=D, w_null=0.02):
    q = np.array(q0, float); q_ref = q if q_ref is None else np.asarray(q_ref)
    jp, jr = np.zeros((3, M.nv)), np.zeros((3, M.nv))
    for _ in range(iters):
        d.qpos[QADR] = q; mujoco.mj_kinematics(M, d); mujoco.mj_comPos(M, d)
        ep = pos - d.site_xpos[site]
        cur = d.site_xmat[site].reshape(3, 3); er = 0.5 * (np.cross(cur[:, 0], mat[:, 0]) + np.cross(cur[:, 1], mat[:, 1]) + np.cross(cur[:, 2], mat[:, 2]))
        e = np.r_[ep, er]
        if np.linalg.norm(ep) < tol and np.linalg.norm(er) < 10 * tol: return q, True
        mujoco.mj_jacSite(M, d, jp, jr, site)
        Jm = np.vstack([jp[:, DADR], jr[:, DADR]])
        lam = 1e-4
        JJ = Jm @ Jm.T + lam * np.eye(6)
        dq = Jm.T @ np.linalg.solve(JJ, e)
        N = np.eye(6) - Jm.T @ np.linalg.solve(JJ, Jm)
        dq += N @ (w_null * (q_ref - q))
        q = np.clip(q + dq, LO, HI)
    d.qpos[QADR] = q; mujoco.mj_kinematics(M, d)
    ok = np.linalg.norm(pos - d.site_xpos[site]) < 1e-3
    return q, ok


def contacts(q, d=D, ignore_pcb=False, margin=0.0):
    """robot-related contacts (penetration below -margin) for configuration q in the current scene state"""
    d.qpos[QADR] = q; mujoco.mj_forward(M, d); out = []
    for c in d.contact[:d.ncon]:
        b1, b2 = M.geom_bodyid[c.geom1], M.geom_bodyid[c.geom2]
        if not ({b1, b2} & ROBOT_BODIES): continue
        names = {M.body(b1).name, M.body(b2).name}
        if ignore_pcb and 'pcb' in names: continue
        if c.dist < -margin: out.append((c.dist, M.body(b1).name, M.body(b2).name, M.geom(c.geom1).name or str(c.geom1), M.geom(c.geom2).name or str(c.geom2)))
    return sorted(out)


def clearance(q, d=D, cut=0.05, skip=()):
    """smallest distance between robot geoms and other collidable geoms (approximate, bounding-sphere filtered)"""
    d.qpos[QADR] = q; mujoco.mj_kinematics(M, d)
    rob = [g for g in range(M.ngeom) if M.geom_bodyid[g] in ROBOT_BODIES and M.geom_contype[g]]
    oth = [g for g in range(M.ngeom) if M.geom_bodyid[g] not in ROBOT_BODIES and (M.geom_contype[g] or M.geom_conaffinity[g])
           and M.body(M.geom_bodyid[g]).name not in skip]
    best, ft = (cut, None), np.zeros(6)
    for a in rob:
        for b in oth:
            if not ((M.geom_contype[a] & M.geom_conaffinity[b]) or (M.geom_contype[b] & M.geom_conaffinity[a])): continue
            if np.linalg.norm(d.geom_xpos[a] - d.geom_xpos[b]) - M.geom_rbound[a] - M.geom_rbound[b] > cut: continue
            x = mujoco.mj_geomDistance(M, d, a, b, cut, ft)
            if x < best[0]: best = (x, (M.body(M.geom_bodyid[a]).name, M.geom(b).name or M.body(M.geom_bodyid[b]).name))
    return best


def quat2mat(q):
    m = np.zeros(9); mujoco.mju_quat2Mat(m, np.asarray(q, float)); return m.reshape(3, 3)


def tcp_for_pcb(pcb_pos, pcb_quat):
    """suction TCP pose that holds the PCB at the given pose"""
    R = quat2mat(pcb_quat)
    return pcb_pos + R @ cell.SUCTION_IN_PCB, R @ cell.R_TOOL_IN_PCB
