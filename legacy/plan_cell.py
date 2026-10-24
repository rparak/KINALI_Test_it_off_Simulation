"""Plan the whole cell cycle (robot joint trajectory + loader moves + suction events) and save it to
legacy/cell_plan.npz.  The robot motion is computed offline with IK on the same MuJoCo model and every robot
segment is checked for collisions against the scene in the state it will have at that time.

Cycle:
  1. robot: home -> above crate 1 -> down onto the PCB -> vacuum on -> up -> over the loader -> down into the
     open jaws (PCB at the hold pose) | loader closes the jaws | vacuum off -> robot up -> home
  2. loader: up -> towards MG06 -> down into the holder -> open -> up -> park (y = -199.5 mm, z = +20 mm)
  3. robot: home -> fork start pose -> close MG06 (lid down, handle 29 deg) -> leave -> come back -> open MG06
     (recorded fork path of the MG06 project, followed with IK) -> home
  4. loader: up -> to MG06 -> down -> close jaws (pick) -> up -> home
  5. robot: home -> above the PCB in the loader -> down -> vacuum on | loader opens | robot up -> above crate 2
     -> down -> vacuum off -> up -> home
usage: python legacy/plan_cell.py [--out FILE]      (default output: legacy/cell_plan.npz)
"""
import os, sys, json, numpy as np, mujoco
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))   # runs without pip install -e
from kinali_sim import model as cell, kinematics as K, paths

P = cell.P
DT = 0.004
Y0, YP, ZP = P['y_home'], P['y_place'], P['z_place']
Y_PARK, Z_PARK = -0.1995, 0.020
J_GRIP, J_OPEN = P['jaw_grip'], P['jaw_open']
Z_TRAVEL, J_CLAMP = ZP + 0.012, P['jaw_grip'] - 0.012
SPEED = 0.6                                                 # fraction of the AUBO i5 datasheet joint speeds used in the plan
VMAX = SPEED * cell.VMAX_DATASHEET                          # 2.3 / 2.4 rad/s
AMAX = np.array([3.0, 3.0, 3.0, 6.0, 6.0, 6.0])             # rad/s^2 (assumption, not in the datasheet)
FORK = np.load(paths.FORK_PATH)
FLOG, FPH = FORK['log'], FORK['phase']

# family of IK solutions used throughout (elbow up, wrist down) - seeds from an IK survey
# handling the PCB: 'wrist up' family (wrist 2 > 0); MG06 fork path: 'wrist down' family (wrist 2 < 0)
SEED = {'place': [1.05, 0.06, 2.2, 0.57, 1.57, 0.26], 'handover': [-0.41, 0.14, 2.05, 0.28, 1.43, -1.19],
        'fork': [-1.13, 0.01, 1.13, -2.02, -1.92, 1.15], 'pick': [0.46, -0.13, 2.0, 0.56, 1.57, -0.32]}
FLIP = np.diag([-1.0, -1.0, 1.0])                          # tool turned 180 deg about its axis
H_CLOSED, LID_CLOSED = 28.65, -0.5                         # handle / lid angle after closing (measured in the cell simulation; the
                                                           # lid rises 0.5 deg within the 1.5 mm lock clearance)


def mj(s): s = np.clip(s, 0, 1); return s**3 * (10 - 15 * s + 6 * s * s)


class Plan:
    def __init__(self):
        self.t = 0.0; self.q = []; self.loader = {'y': [], 'z': [], 'jaw': []}; self.events = []; self.phases = []
        self.lstate = dict(y=Y0, z=0.0, jaw=J_OPEN); self.qcur = None; self.fix = dict(lid=-0.6974, handle=0.0, press=0.0)
        self.pcb = ('crate', None); self.rel = None; self.worst = (1.0, None, '')

    # ---------- bookkeeping ----------
    def phase(self, txt): self.phases.append((round(self.t, 3), txt))

    def hold(self, T):
        n = int(round(T / DT)); self.q += [self.qcur] * n; self.t += n * DT

    def loader_move(self, ch, target, T, wait=True, actual=None):
        self.loader[ch].append((self.t, T, target)); self.lstate[ch] = target if actual is None else actual
        if wait: self.hold(T)

    def event(self, name): self.events.append((round(self.t, 4), name))

    # ---------- collision check of one robot configuration ----------
    def check(self, q, label, fix=None, record=True):
        fx = fix or self.fix
        K.set_scene(lid=fx['lid'], handle=fx['handle'], press=fx['press'], y=self.lstate['y'], z=self.lstate['z'], jaw=self.lstate['jaw'],
                    pcb=self.pcb_pose(q))
        c = K.contacts(q, ignore_pcb=True, margin=-K.CLEAR)
        c = [x for x in c if not ('pcb' in (x[1], x[2]))]
        if self.pcb[0] == 'attached':                           # the carried PCB must not hit anything but the tool
            K.D.qpos[K.QADR] = q; mujoco.mj_forward(K.M, K.D)
            for cc in K.D.contact[:K.D.ncon]:
                b = {K.M.body(K.M.geom_bodyid[cc.geom1]).name, K.M.body(K.M.geom_bodyid[cc.geom2]).name}
                if 'pcb' in b and not any(n.startswith('aubo/') for n in b) and cc.dist < K.CLEAR:
                    c.append((cc.dist, *sorted(b), '', ''))
        if record and c and c[0][0] < self.worst[0]: self.worst = (c[0][0], c[0][1:3], label)
        return c

    def pcb_pose(self, q):
        if self.pcb[0] == 'attached':
            p, R = K.site_pose(q, K.SITE_TCP)
            Rp = R @ self.rel_R.T; pp = p - Rp @ cell.SUCTION_IN_PCB
            qq = np.zeros(4); mujoco.mju_mat2Quat(qq, Rp.flatten()); return np.r_[pp, qq]
        return None

    # ---------- motions ----------
    def _jpath(self, q0, q1, vscale=1.0):
        dq = np.abs(np.asarray(q1) - q0)       # minimum-jerk: v_peak = 1.875 dq/T, a_peak = 5.77 dq/T^2
        T = max(0.5, float(np.max(1.875 * dq / (VMAX * vscale))), float(np.max(np.sqrt(5.774 * dq / AMAX)))); n = int(round(T / DT))
        return [q0 + (q1 - q0) * mj(i / n) for i in range(1, n + 1)]

    def _bad(self, path, label):
        bad = []
        for i, q in enumerate(path):
            if i % 10 == 9 or i == len(path) - 1:
                c = self.check(q, label, record=False)
                if c: bad.append((round((i + 1) * DT, 2), round(1000 * c[0][0], 1), c[0][1:3]))
        return bad

    def joint_move(self, q1, label, vscale=1.0, site=None):
        """joint-space move; if the straight joint path collides, insert a via point lifted above the midpoint"""
        site = K.SITE_TCP if site is None else site
        q0 = self.qcur; q1 = np.asarray(q1, float)
        segs = [self._jpath(q0, q1, vscale)]; bad = self._bad(segs[0], label); via = ''
        if bad:
            qm = (q0 + q1) / 2; pm, Rm = K.site_pose(qm, site)
            p0, _ = K.site_pose(q0, site); p1, _ = K.site_pose(q1, site)
            for dz in (0.08, 0.15, 0.22, 0.30):
                for shift in (np.zeros(3), 0.5 * (cell.ROBOT_BASE - pm) * [1, 1, 0]):
                    pv = pm + shift; pv[2] = max(p0[2], p1[2], pm[2]) + dz
                    qv, ok = K.ik(pv, Rm, qm, site=site, q_ref=qm, iters=400)
                    if not ok: continue
                    s1, s2 = self._jpath(q0, qv, vscale), self._jpath(qv, q1, vscale)
                    b = self._bad(s1, label) + self._bad(s2, label)
                    if not b: segs, bad, via = [s1, s2], [], f' (via {np.round(pv, 2)})'; break
                if not bad: break
        T = 0
        for sg in segs:
            for q in sg: self.q.append(q); self.check(q, label)
            T += len(sg) * DT
        self.t += T; self.qcur = q1
        print(f'  {label:42s} {T:5.2f}s ' + ('ok' + via if not bad else f'COLLISION {bad[:2]}'))
        return not bad

    def linear_move(self, p1, R1, label, speed=0.08, site=None, bar_contact=False):
        site = K.SITE_TCP if site is None else site
        p0, R0 = K.site_pose(self.qcur, site); L = np.linalg.norm(p1 - p0)
        T = max(0.6, 1.875 * L / speed); n = int(round(T / DT)); q = self.qcur; bad = []
        q0r = np.zeros(4); mujoco.mju_mat2Quat(q0r, R0.flatten()); q1r = np.zeros(4); mujoco.mju_mat2Quat(q1r, np.asarray(R1).flatten())
        if q0r @ q1r < 0: q1r = -q1r
        for i in range(1, n + 1):
            s = mj(i / n); qq = (1 - s) * q0r + s * q1r; qq /= np.linalg.norm(qq)
            q, ok = K.ik(p0 + (p1 - p0) * s, K.quat2mat(qq), q, site=site, q_ref=q)
            if not ok: print('   IK failed', label, i);
            self.q.append(q)
            if i % 10 == 0 or i == n:
                c = self.check(q, label)
                if bar_contact: c = [x for x in c if not ({'mg06_handle', 'aubo/tool/ee_tool'} == {x[1], x[2]})]
                if c: bad.append((round(i * DT, 2), round(1000 * c[0][0], 1), c[0][1:3]))
        self.t += n * DT; self.qcur = q
        print(f'  {label:42s} {T:5.2f}s ' + ('ok' if not bad else f'COLLISION {bad[:2]}'))

    def replay(self, rows, name):
        """follow recorded flange poses (IK per sample) with the recorded timing"""
        q = self.qcur; qs = [q]; ts = [FLOG[rows[0] - 1, 0] if rows[0] > 0 else 0.0]; bad = []
        self.phases.append((round(self.t, 3), 'robot: ' + name))
        for k, i in enumerate(rows):
            row = FLOG[i]; q, ok = K.ik(row[4:7], K.quat2mat(row[7:11]), q, site=K.SITE_FLANGE, q_ref=q)
            if not ok: print('   IK failed on the fork path', i)
            self.fix = dict(lid=row[1], press=row[2], handle=row[3])
            if k % 5 == 0:
                c = [x for x in self.check(q, name) if not ({'mg06_handle', 'aubo/tool/ee_tool'} == {x[1], x[2]})]  # fork on the bar
                if c: bad.append((round(row[0], 2), round(1000 * c[0][0], 1), c[0][1:3]))
            qs.append(q); ts.append(row[0])
        ts = np.array(ts) - ts[0]; qs = np.array(qs); tt = np.arange(DT, ts[-1] + 1e-9, DT)
        for x in tt: self.q.append(np.array([np.interp(x, ts, qs[:, j]) for j in range(6)]))
        self.t += len(tt) * DT; self.qcur = qs[-1]
        print(f'  {name:42s} {ts[-1]:5.2f}s ' + ('ok' if not bad else f'COLLISION {bad[:3]}') + f'  max joint step {np.round(np.max(np.abs(np.diff(qs, axis=0)), axis=0), 3)}')

    # ---------- helpers ----------
    def solve(self, p, R, seed, site=None):
        site = K.SITE_TCP if site is None else site; seed = np.array(seed, float); rng = np.random.default_rng(0); best = None
        for k in range(60):
            q0 = seed if k == 0 else seed + rng.uniform(-0.6, 0.6, 6)
            q, ok = K.ik(p, R, q0, site=site, q_ref=seed, iters=500)
            if ok and (best is None or np.linalg.norm(q - seed) < np.linalg.norm(best - seed)): best = q
            if ok and np.linalg.norm(q - seed) < 0.5: break
        assert best is not None, ('IK failed', p)
        return best

    def tool_R(self, pcb_quat, seed):
        """tool orientation for holding the PCB; turn the tool 180 deg if that keeps wrist 3 nearer to the seed"""
        Rp = K.quat2mat(pcb_quat)
        return Rp @ cell.R_TOOL_IN_PCB


def main():
    pl = Plan()
    # home: tool down, between the robot and the crates
    R_down = np.diag([1.0, -1.0, -1.0])
    q_home = pl.solve(np.array([0.30, -0.52, 0.40]), R_down, SEED['place']); pl.qcur = q_home
    pl.check(q_home, 'home')

    # ---------- 1. pick from crate 1, hand over to the loader ----------
    pl.phase('robot: pick the PCB from crate 1 (vacuum)')
    pp, pq = cell.pcb_in_crate('crate_pick'); tp, tR = K.tcp_for_pcb(pp, pq)
    PRESS = 0.0015                                  # cups compressed 1.5 mm (bellows) when the vacuum switches on
    q_pa = pl.solve(tp + [0, 0, 0.12], tR, SEED['pick'])
    pl.joint_move(q_pa, 'home -> above crate 1')
    pl.linear_move(tp + [0, 0, 0.015], tR, 'down into crate 1', speed=0.20)
    pl.linear_move(tp - [0, 0, PRESS], tR, 'cups onto the PCB', speed=0.03)
    pl.event('suction_on'); pl.pcb = ('attached', None); pl.rel_R = cell.R_TOOL_IN_PCB; pl.hold(0.3)
    pl.linear_move(tp + [0, 0, 0.12], tR, 'up out of the crate', speed=0.20)
    pl.phase('robot: put the PCB into the loader gripper')
    hp, hq = np.array(P['hold_pos']), np.array(P['quat']); htp, htR = K.tcp_for_pcb(hp, hq); nrm = K.quat2mat(hq)[:, 2]
    q_hpre = pl.solve(htp + 0.06 * nrm, htR, SEED['handover'])
    # via point above the loader so the PCB does not swing over the crates / MG06 front
    q_via = pl.solve(htp + 0.16 * nrm, htR, SEED['handover'])
    pl.joint_move(q_via, 'above crate -> above the loader')
    pl.linear_move(htp + 0.015 * nrm, htR, 'down to the loader', speed=0.20)
    pl.linear_move(htp, htR, 'PCB into the open jaws (hold pose)', speed=0.03)   # no press: the board must sit above the ledges
    pl.phase('loader: close gripper'); pl.loader_move('jaw', J_CLAMP, 0.35, actual=J_GRIP); pl.hold(0.25)
    pl.event('suction_off'); pl.pcb = ('loader', None); pl.hold(0.2)
    pl.phase('robot: away')
    pl.linear_move(htp + 0.16 * nrm, htR, 'robot up from the loader', speed=0.20); pl.joint_move(q_home, '-> home')

    # ---------- 2. loader places the PCB into MG06 and parks ----------
    pl.phase('loader: up, towards MG06'); pl.loader_move('z', Z_TRAVEL, 0.8, wait=False); pl.hold(0.45)
    pl.loader_move('y', YP, 2.0, wait=False); pl.hold(2.1)
    pl.phase('loader: down into the holder'); pl.loader_move('z', ZP, 0.6); pl.hold(0.1)
    pl.phase('loader: open gripper (place)'); pl.loader_move('jaw', J_OPEN, 0.35); pl.pcb = ('mg06', None); pl.hold(0.1)
    pl.phase('loader: up, to park y = -199.5 mm'); pl.loader_move('z', Z_TRAVEL, 0.5)
    pl.loader_move('y', Y_PARK, 1.8, wait=False); pl.hold(0.95); pl.loader_move('z', Z_PARK, 0.8); pl.hold(0.15)

    # ---------- 3. robot closes and opens MG06 with the fork ----------
    pl.phase('robot: to the MG06 handle')
    f0 = FLOG[0]; q_f0 = pl.solve(f0[4:7], K.quat2mat(f0[7:11]), SEED['fork'], site=K.SITE_FLANGE)
    q_fvia = pl.solve(f0[4:7] + [0.10, -0.10, 0.10], K.quat2mat(f0[7:11]), SEED['fork'], site=K.SITE_FLANGE)
    pl.joint_move(q_fvia, 'home -> above the handle', site=K.SITE_FLANGE); pl.joint_move(q_f0, '-> fork start pose')
    # recorded flange path of the MG06 project: CLOSE and OPEN are replayed (IK per sample); the robot leaves and
    # comes back on a new path planned around the actual closed handle (bar centred in the slot, 5 mm play each side)
    phs = np.array([str(x) for x in FPH])
    i_close = np.where(np.char.startswith(phs, 'CLOSE'))[0]; i_open = np.where(np.char.startswith(phs, 'OPEN'))[0]
    pl.replay(np.r_[np.arange(1, i_close[0]), i_close], 'close MG06 (lid down, handle 29 deg)')
    pl.fix = dict(lid=np.radians(LID_CLOSED), handle=np.radians(H_CLOSED), press=0.012 * (np.radians(H_CLOSED) / 1.047) ** 3)
    p_end, R_end = K.site_pose(pl.qcur, K.SITE_FLANGE)
    K.D.qpos[K.QA], K.D.qpos[K.QH] = np.radians(LID_CLOSED), np.radians(H_CLOSED); mujoco.mj_kinematics(K.M, K.D)
    bar = K.D.site_xpos[K.M.site('handle_grasp').id].copy(); b_t = R_end.T @ (bar - p_end)
    # centre the bar in the slot (tool x = 0.12) and lift it 2 mm off the slot bottom (tool y = 0.064 -> 0.066)
    p_c = p_end + R_end[:, 0] * (b_t[0] - 0.12) - R_end[:, 1] * (0.066 - b_t[1])
    p_out = p_c - 0.09 * R_end[:, 1]; p_up = p_out + [0, 0, 0.12]
    pl.linear_move(p_c, R_end, 'bar to the slot centre', speed=0.02, site=K.SITE_FLANGE, bar_contact=True)
    pl.phase('robot: leave MG06 (fixture closed)')
    pl.linear_move(p_out, R_end, 'fork out of the bar (along the slot)', speed=0.12, site=K.SITE_FLANGE)
    pl.linear_move(p_up, R_end, 'up', speed=0.25, site=K.SITE_FLANGE)
    for hd in (H_CLOSED - 1.0, H_CLOSED, H_CLOSED + 1.0):     # robustness: clearance if the handle stands 1 deg off
        fx = dict(lid=np.radians(LID_CLOSED), handle=np.radians(hd), press=0.0); worst = 1
        for pp_ in np.linspace(p_c, p_out, 15):
            q_, _ = K.ik(pp_, R_end, pl.qcur, site=K.SITE_FLANGE, q_ref=pl.qcur); K.set_scene(**fx, y=Y_PARK, z=Z_PARK)
            worst = min(worst, K.clearance(q_, cut=0.03)[0])
        print(f'     clearance fork-MG06 on the way out if the handle is at {hd:.1f} deg: {1000*worst:.1f} mm')
    pl.phase('robot: away, MG06 closed'); pl.hold(1.0)
    pl.phase('robot: back to the handle (same path)')
    pl.linear_move(p_out, R_end, 'down', speed=0.25, site=K.SITE_FLANGE)
    pl.linear_move(p_c, R_end, 'fork onto the bar (along the slot)', speed=0.12, site=K.SITE_FLANGE)
    # join the recorded opening path where its handle angle has already passed the actual closed angle, so the
    # slot wall meets the bar while moving upwards (no push down onto the closed handle)
    k = next(i for i in i_open if np.degrees(FLOG[i, 3]) <= H_CLOSED - 0.2)
    ok_ = FLOG[k]; pl.fix = dict(lid=ok_[1], press=ok_[2], handle=ok_[3]); pl.phase('robot: open MG06')
    pl.linear_move(ok_[4:7], K.quat2mat(ok_[7:11]), 'slot wall onto the bar (upwards)', speed=0.03, site=K.SITE_FLANGE, bar_contact=True)
    pl.replay(i_open[i_open > k], 'open MG06')
    pl.fix = dict(lid=-0.6974, handle=0.0, press=0.0)
    pl.phase('robot: home'); pl.joint_move(q_fvia, 'away from the handle', site=K.SITE_FLANGE); pl.joint_move(q_home, '-> home', site=K.SITE_FLANGE)

    # ---------- 4. loader picks the PCB and returns ----------
    pl.phase('loader: back to MG06'); pl.loader_move('z', Z_TRAVEL, 0.6, wait=False); pl.hold(0.4)
    pl.loader_move('y', YP, 2.2, wait=False); pl.hold(2.3)
    pl.phase('loader: down, close gripper (pick)'); pl.loader_move('z', ZP, 0.6); pl.hold(0.1)
    pl.loader_move('jaw', J_CLAMP, 0.35, actual=J_GRIP); pl.pcb = ('loader', None); pl.hold(0.2)
    pl.phase('loader: up, back home'); pl.loader_move('z', Z_TRAVEL, 0.6)
    pl.loader_move('y', Y0, 2.0, wait=False); pl.hold(1.5); pl.loader_move('z', 0.0, 0.8); pl.hold(0.2)

    # ---------- 5. robot takes the PCB and puts it into crate 2 ----------
    pl.phase('robot: take the PCB from the loader (vacuum)')
    pl.joint_move(q_via, 'home -> above the loader'); pl.linear_move(htp + 0.015 * nrm, htR, 'down to the loader', speed=0.20)
    pl.linear_move(htp - PRESS * nrm, htR, 'cups onto the PCB', speed=0.03)
    pl.event('suction_on'); pl.pcb = ('attached', None); pl.hold(0.3)
    pl.phase('loader: open gripper'); pl.loader_move('jaw', J_OPEN, 0.35); pl.hold(0.15)
    pl.phase('robot: PCB into crate 2')
    pl.linear_move(htp + 0.03 * nrm, htR, 'up out of the jaws', speed=0.05)
    pl.linear_move(htp + 0.16 * nrm, htR, 'up', speed=0.20)
    cp, cq = cell.pcb_in_crate('crate_place'); ctp, ctR = K.tcp_for_pcb(cp + [0, 0, 0.001], cq)
    q_ca = pl.solve(ctp + [0, 0, 0.12], ctR, SEED['place'])
    pl.joint_move(q_ca, '-> above crate 2')
    pl.linear_move(ctp + [0, 0, 0.015], ctR, 'down into crate 2', speed=0.20)
    pl.linear_move(ctp, ctR, 'PCB down to the crate floor', speed=0.03)
    pl.event('suction_off'); pl.pcb = ('crate2', None); pl.hold(0.3)
    pl.linear_move(ctp + [0, 0, 0.12], ctR, 'up', speed=0.20)
    pl.phase('robot: home, cycle finished'); pl.joint_move(q_home, '-> home'); pl.hold(0.8)

    q = np.array(pl.q)
    print('total %.1f s, worst robot contact %s' % (pl.t, 'none' if pl.worst[1] is None else f'{1000*pl.worst[0]:.1f} mm {pl.worst[1]} ({pl.worst[2]})'))
    print('joint range used:', np.round(q.min(0), 2), np.round(q.max(0), 2))
    v = np.abs(np.diff(q, axis=0)) / DT; print('max joint speed rad/s:', np.round(v.max(0), 2))
    out = sys.argv[sys.argv.index('--out') + 1] if '--out' in sys.argv else paths.LEGACY_PLAN
    np.savez(out, dt=DT, q=q, q_home=q_home,
             loader=json.dumps(pl.loader), events=json.dumps(pl.events), phases=json.dumps(pl.phases),
             J_OPEN=J_OPEN, T=pl.t)


if __name__ == '__main__':
    main()
