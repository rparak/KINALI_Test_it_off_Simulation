"""Full cell cycle with the AUBO i5 (plan from tools/plan_cell.py, physics in MuJoCo):
left crate -> robot (vacuum) -> loader -> MG06 -> robot closes / opens MG06 with the fork -> loader -> robot -> right crate.

The vacuum is an adhesion actuator on the four D15 cups (force = vacuum x cup area): it only pulls what the cups
actually touch, so a badly placed tool or a shifted PCB makes the pick fail like in reality.

usage:  python legacy/test_cell_cycle.py [--record frames.npz] [--render cell_cycle.mp4] [--progress]
"""
import sys, os, json, numpy as np, mujoco
sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))   # runs without pip install -e
from kinali_sim import model as cell, paths

m = cell.build_model(); d = mujoco.MjData(m)
P = cell.P
J = lambda n: m.jnt_qposadr[m.joint(n).id]
QR = np.array([J('aubo/' + j) for j in cell.ROBOT_JOINTS])
qa, qh, qy, qz, qjl, qjr, qpcb = (J(n) for n in ['lid_hinge', 'handle_hinge', 'y_axis', 'z_axis', 'jaw_L', 'jaw_R', 'pcb_free'])
TOOL, PCB = m.body('aubo/tool/ee_tool').id, m.body('pcb').id
CUPS = {m.body(f'aubo/tool/vgc10_cup{i}').id for i in range(cell.CUP_N)}
A_ROBOT = slice(3, 9); A_VAC = [m.actuator(f'vacuum{i}').id for i in range(cell.CUP_N)]
GNAME = [m.geom(g).name or (m.mesh(m.geom_dataid[g]).name if m.geom_dataid[g] >= 0 else str(g)) for g in range(m.ngeom)]
BNAME = [m.body(m.geom_bodyid[g]).name for g in range(m.ngeom)]
LOADER_B = {'y_carriage', 'z_carriage', 'jaw_L', 'jaw_R'}


def mj(s): s = np.clip(s, 0, 1); return s**3 * (10 - 15 * s + 6 * s * s)


def loader_target(ch, t):
    v = START[ch]
    for t0, T, x in LOADER[ch]:
        if t >= t0: v = v + (x - v) * mj((t - t0) / T)
    return v


KV_KP = m.actuator_biasprm[A_ROBOT, 2] / m.actuator_biasprm[A_ROBOT, 1]     # kv/kp of the joint servos


def load_plan(path=paths.LEGACY_PLAN):
    """load a plan file (cell_plan.npz, written by legacy/plan_cell.py) into the module state used by run()"""
    global PL, DT, QPLAN, T_END, LOADER, EVENTS, PHASES, START, VPLAN
    PL = np.load(path)
    DT, QPLAN, T_END = float(PL['dt']), PL['q'], float(PL['T'])
    LOADER, EVENTS, PHASES = json.loads(str(PL['loader'])), json.loads(str(PL['events'])), json.loads(str(PL['phases']))
    START = {'y': P['y_home'], 'z': 0.0, 'jaw': float(PL['J_OPEN'])}
    VPLAN = np.gradient(QPLAN, DT, axis=0)


load_plan()


def robot_target(t):
    """servo set-point: planned position + velocity feed-forward (kv/kp * v_ref)"""
    x = min(t / DT, len(QPLAN) - 1.001); i = int(x); w = x - i; j = min(i + 1, len(QPLAN) - 1)
    q = (1 - w) * QPLAN[i] + w * QPLAN[j]; v = (1 - w) * VPLAN[i] + w * VPLAN[j]
    return q + KV_KP * v, q


def phase(t):
    ph = [p for t0, p in PHASES if t >= t0]; return ph[-1] if ph else ''


_CLS = {}
COLLISION_CHECKS = ['robot <-> MG06 / workstation / crates', 'robot tool <-> handle (outside closing / opening)', 'robot <-> loader',
                    'loader <-> MG06 / workstation', 'PCB <-> lid / press plate']


def classify(g1, g2, ph):
    key = (g1, g2)
    if key not in _CLS:
        s = {BNAME[g1], BNAME[g2]}; rob = any(b.startswith('aubo/') for b in s)
        if rob and 'pcb' in s: k = None
        elif s == {'aubo/tool/ee_tool', 'mg06_handle'}: k = 'fork'
        elif rob: k = 'robot <-> ' + ('loader' if s & LOADER_B else 'MG06 / workstation / crates')
        elif s & LOADER_B and 'pcb' not in s: k = 'loader <-> MG06 / workstation'
        elif 'pcb' in s and s & {'mg06_lid', 'mg06_press', 'mg06_handle'}: k = 'PCB <-> lid / press plate'
        else: k = None
        _CLS[key] = k
    k = _CLS[key]
    if k == 'fork':
        k = None if (ph.startswith('robot: close MG06') or ph.startswith('robot: open MG06')) else 'robot tool <-> handle (outside closing / opening)'
    return k


def cup_contacts():
    cg = set()
    for c in d.contact[:d.ncon]:
        b = {m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]}
        if PCB in b and b & CUPS and c.dist < 0.0005: cg.add(c.geom1 if m.geom_bodyid[c.geom1] in CUPS else c.geom2)
    return len(cg)


def run(render=None, record=None, plan=None):
    """physics run of the whole cycle; prints the report and returns its values (lengths in mm, angles in deg)"""
    if plan is not None: load_plan(plan)
    mujoco.mj_resetData(m, d)
    d.qpos[qa] = -0.6974; d.qpos[qy] = START['y']; d.qpos[qjl] = d.qpos[qjr] = START['jaw']
    pp, pq = cell.pcb_in_crate('crate_pick'); d.qpos[qpcb:qpcb + 3], d.qpos[qpcb + 3:qpcb + 7] = pp + [0, 0, 0.0003], pq
    d.qpos[QR] = QPLAN[0]; d.ctrl[:3] = [START['y'], START['z'], START['jaw']]; d.ctrl[A_ROBOT] = QPLAN[0]; d.ctrl[A_VAC] = 0
    d.act[:] = 0; d.act[m.actuator_actadr[0]] = START['y']                    # filtered loader drives start at home
    mujoco.mj_forward(m, d)
    ev = list(EVENTS); worst = {}; log = []; vac = []; track_err = 0.0; slip = 0.0; rel0 = None
    pen = {}; fork_force = 0.0; loader_lag = 0.0; marks = {}; ph = ''
    if render:
        import imageio
        from PIL import Image, ImageDraw
        W, H = 432, 324; r = mujoco.Renderer(m, H, W)
        c0 = mujoco.MjvCamera(); c0.lookat[:] = [0.15, -0.40, 0.05]; c0.distance = 2.1; c0.azimuth = 150; c0.elevation = -24
        cams = [c0]
        for n in ('cam_pcb', 'cam_tool'):
            c = mujoco.MjvCamera(); c.type = mujoco.mjtCamera.mjCAMERA_FIXED; c.fixedcamid = m.camera(n).id; cams.append(c)
        opt = mujoco.MjvOption(); opt.flags[mujoco.mjtVisFlag.mjVIS_TENDON] = 0
        writer = imageio.get_writer(render, fps=24, quality=7, macro_block_size=4); nframes = 0; nxt = 0.0
    fc = np.zeros(6); rec_q, rec_txt, rnext = [], [], 0.0
    while d.time < T_END:
        t = d.time
        while ev and t >= ev[0][0]:
            te, name = ev.pop(0); on = name == 'suction_on'
            d.ctrl[A_VAC] = 1.0 if on else 0.0; rel0 = None
            ncups = cup_contacts() if on else None; vac.append({'t': t, 'on': on, 'cups': ncups})
            log.append(f'{t:6.2f}s vacuum {"ON " if on else "OFF"}' + (f' ({ncups} of 4 cups touch the PCB)' if on else ''))
        d.ctrl[:3] = [loader_target('y', t), loader_target('z', t), loader_target('jaw', t)]
        d.ctrl[A_ROBOT], q_ref = robot_target(t)
        mujoco.mj_step(m, d)
        if d.time < t: raise RuntimeError(f'simulation unstable at t={t:.3f} s ({ph}) - MuJoCo reset the state')
        if '--progress' in sys.argv and int(round(d.time / m.opt.timestep)) % 4000 == 0:
            import time as _t; print(f'  [{_t.strftime("%H:%M:%S")}] t={d.time:5.1f} {ph}', flush=True)
        track_err = max(track_err, np.max(np.abs(d.qpos[QR] - q_ref)))
        loader_lag = max(loader_lag, abs(d.qpos[qy] - d.ctrl[0]), abs(d.qpos[qz] - d.ctrl[1]))
        if d.ctrl[A_VAC[0]] > 0.5 and cup_contacts():         # PCB slip in the cups while the vacuum holds it
            R = d.xmat[TOOL].reshape(3, 3); rel = R.T @ (d.xpos[PCB] - d.xpos[TOOL])
            rel0 = rel if rel0 is None else rel0; slip = max(slip, np.linalg.norm(rel - rel0))
        ff = 0.0
        for i, c in enumerate(d.contact[:d.ncon]):
            k = classify(c.geom1, c.geom2, ph)
            if k and c.dist < worst.get(k, (0,))[0]: worst[k] = (c.dist, GNAME[c.geom1], GNAME[c.geom2], round(d.time, 2))
            b1, b2 = BNAME[c.geom1], BNAME[c.geom2]
            if 'pcb' in (b1, b2) and not (b1.startswith('aubo/') or b2.startswith('aubo/')):   # thin board: penetration
                other = b2 if b1 == 'pcb' else b1
                grp = 'crate' if other == 'workstation' else ('loader jaws' if other in LOADER_B else ('MG06 holder' if other == 'mg06_base' else other))
                if c.dist < pen.get(grp, (0,))[0]: pen[grp] = (c.dist, round(d.time, 2))
            if {b1, b2} == {'aubo/tool/ee_tool', 'mg06_handle'}:
                mujoco.mj_contactForce(m, d, i, fc); ff += fc[0]
        fork_force = max(fork_force, ff)
        if render or int(round(d.time / m.opt.timestep)) % 20 == 0: ph = phase(d.time)
        if ph.startswith('loader: up, to park') and 'placed' not in marks: marks['placed'] = (d.qpos[qpcb:qpcb + 7].copy(), d.time)
        if 'away, MG06 closed' in ph and 'closed' not in marks: marks['closed'] = (np.degrees(d.qpos[qa]), np.degrees(d.qpos[qh]), d.time)
        if record and d.time >= rnext:
            rnext += 1 / 24; rec_q.append(d.qpos.copy())
            rec_txt.append(f't = {d.time:5.1f} s   {ph}   |  loader y {1000*d.qpos[qy]:+6.1f} z {1000*d.qpos[qz]:5.1f} jaw {1000*d.qpos[qjl]:4.1f} mm'
                           f'  |  lid {np.degrees(d.qpos[qa]):5.1f}  handle {np.degrees(d.qpos[qh]):4.1f} deg  |  vacuum {"ON" if d.ctrl[A_VAC[0]] > 0.5 else "off"}')
        if render and d.time >= nxt:
            nxt += 1 / 24; ims = []
            for c in cams: r.update_scene(d, c, opt); ims.append(r.render())
            im = np.hstack(ims); pi = Image.fromarray(im); dr = ImageDraw.Draw(pi); dr.rectangle([0, 0, im.shape[1], 20], fill=(0, 0, 0))
            dr.text((6, 4), f't = {d.time:5.1f} s   {ph}   |  loader y {1000*d.qpos[qy]:+6.1f} z {1000*d.qpos[qz]:5.1f} jaw {1000*d.qpos[qjl]:4.1f} mm'
                    f'  |  lid {np.degrees(d.qpos[qa]):5.1f}  handle {np.degrees(d.qpos[qh]):4.1f} deg  |  vacuum {"ON" if d.ctrl[A_VAC[0]] > 0.5 else "off"}', fill=(255, 255, 0))
            writer.append_data(np.array(pi)); nframes += 1
    # ---- report ----
    print('cycle time %.1f s' % T_END)
    for l in log: print(l)
    pm, tm = marks['placed']; e = pm[:3] - np.array(P['place_pos'])
    qd = np.zeros(4); qi = np.zeros(4); mujoco.mju_negQuat(qi, np.array(P['quat'])); mujoco.mju_mulQuat(qd, pm[3:], qi)
    placed_tilt = np.degrees(2*np.arccos(min(1,abs(qd[0]))))
    print(f'PCB in MG06 (t={tm:.1f}s): error {1000*np.linalg.norm(e):.2f} mm (dx {1000*e[0]:.2f} dy {1000*e[1]:.2f} dz {1000*e[2]:.2f}), tilt {placed_tilt:.2f} deg')
    if 'closed' in marks: print('MG06 closed with the robot away (t=%.1fs): lid %.1f deg, handle %.1f deg' % (marks['closed'][2], *marks['closed'][:2]))
    pf = d.qpos[qpcb:qpcb + 7]; cc = cell.crate_center('crate_place')
    tilt = np.degrees(np.arccos(min(1, abs(1 - 2 * (pf[4]**2 + pf[5]**2)))))
    print(f'PCB at the end: x {1000*pf[0]:.1f} y {1000*pf[1]:.1f} z {1000*pf[2]:.1f} mm (right crate centre {1000*cc[0]:.1f} {1000*cc[1]:.1f}, floor {1000*cell.CRATE_FLOOR:.1f}); tilt {tilt:.2f} deg')
    print('robot joint tracking error max %.4f rad, PCB slip in the cups max %.2f mm, loader axis lag max %.1f mm' % (track_err, 1000 * slip, 1000 * loader_lag))
    print('max contact force fork -> handle bar: %.0f N' % fork_force)
    print('deepest PCB penetration: ' + ', '.join(f'{k} {1000*v[0]:.2f} mm (t={v[1]})' for k, v in sorted(pen.items())))
    for k in COLLISION_CHECKS:
        v = worst.get(k); print(f'{k:50s}: ' + ('no contact' if v is None else f'deepest {1000*v[0]:.2f} mm  {v[1]} / {v[2]}  t={v[3]}'))
    print('MG06 at the end: lid %.1f deg, handle %.1f deg' % (np.degrees(d.qpos[qa]), np.degrees(d.qpos[qh])))
    if render: writer.close(); print('saved video', render, round(nframes / 24, 1), 's')
    if record: np.savez_compressed(record, qpos=np.array(rec_q), text=np.array(rec_txt)); print('recorded', len(rec_q), 'frames ->', record)
    return {'cycle_time': T_END, 'vacuum': vac,
            'placed': {'t': tm, 'error_mm': 1000 * np.linalg.norm(e), 'd_mm': (1000 * e).tolist(), 'tilt_deg': placed_tilt},
            'closed': None if 'closed' not in marks else {'t': marks['closed'][2], 'lid_deg': marks['closed'][0], 'handle_deg': marks['closed'][1]},
            'final_pcb': {'pos_mm': (1000 * pf[:3]).tolist(), 'crate_centre_mm': (1000 * cc).tolist(),
                          'dist_xy_mm': 1000 * np.linalg.norm(pf[:2] - cc), 'tilt_deg': tilt},
            'track_err_rad': track_err, 'slip_mm': 1000 * slip, 'loader_lag_mm': 1000 * loader_lag, 'fork_force_n': fork_force,
            'penetration_mm': {k: 1000 * v[0] for k, v in pen.items()},
            'contacts': {k: (None if k not in worst else {'depth_mm': 1000 * worst[k][0], 'geoms': worst[k][1:3], 't': worst[k][3]})
                         for k in COLLISION_CHECKS},
            'mg06_end': {'lid_deg': np.degrees(d.qpos[qa]), 'handle_deg': np.degrees(d.qpos[qh])}}


if __name__ == '__main__':
    arg = lambda k: sys.argv[sys.argv.index(k) + 1] if k in sys.argv else None
    run(arg('--render'), arg('--record'))
