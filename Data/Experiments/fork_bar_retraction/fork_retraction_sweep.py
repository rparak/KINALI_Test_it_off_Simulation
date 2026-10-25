# Option 2 experiment (F11, 2026-10-25; not adopted, design-decisions): the closing path moved off the handle bar.
# Run from the repository root: .venv/bin/python Example/Simulation/fork_retraction_sweep.py R_MM taught|optimized DX_UM WAIT_S
# One line per run; the table in design-decisions is 0 / 0.55 / 0.60 / 0.65 / 0.70 mm x dx -5 -1 0 1 5 um (wait 0) and
# waits 0.004 / 0.1 s (dx 0), both programs.
import os, sys, math, tempfile; sys.path.append('src'); import numpy as np, mujoco
from Configuration.Core import Load_Cell_Configuration, Read_PCB_Meta, Configure_MuJoCo_Log
from Configuration.Program_Parser import Load_Constants, Load_Simulation, Load_Program, Load_Optimization, Free_Steps, CONST_PROGRAM_YAML, CONST_OPTIMIZED_PROGRAM_YAML
from Collision.Core import Build_Planning_Model, Robot_Collision_Cls
from Kinematics.Core import Robot_Kinematics_Cls
import Program.Core as pc
from Program.Core import Build_Program, Pose_Sample, Bar_Gap, Bar_Gaps
from Scene.Core import Build_Cell_Model
from Simulation.Core import Trajectory_Drive_Cls, Physics_Run_Cls, Check_Program_Acceptance
Configure_MuJoCo_Log()
r, which, dx, wait = float(sys.argv[1]) / 1000, sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
cell, meta, c = Load_Cell_Configuration(), Read_PCB_Meta(), Load_Constants(); s = Load_Simulation(c)
c.Paths['PATH_FORK_CLOSE'].Bar_Retraction = r
taught = Load_Program(c, s, cell, CONST_PROGRAM_YAML)
path = CONST_PROGRAM_YAML if which == 'taught' else CONST_OPTIMIZED_PROGRAM_YAML
if wait > 0:
    text = open(path).read()
    old = '    - Robot.MoveJ(P_MG06_FORK_START)                      # legacy: joint_move(q_f0, \'-> fork start pose\')\n' if which == 'taught' else '    - Loader.WaitDone(J_PARK)\n'
    assert text.count(old) == 1
    path = os.path.join(tempfile.mkdtemp(), 'var_%s_%g_%g_%g.yaml' % (which, r, dx, wait))
    open(path, 'w').write(text.replace(old, old + '    - Wait(%g)\n' % wait))
prog = Load_Program(c, s, cell, path)
# The end point follows the retracted end (as P_MG06_CLOSE_END re-taught there would).
snap = pc._Builder_Cls.Snap
pc._Builder_Cls.Snap = lambda self, last, target, step, text: last if target == 'P_MG06_CLOSE_END' and r > 0 else snap(self, last, target, step, text)
pm = Build_Planning_Model(cell); kin = Robot_Kinematics_Cls(pm, Robot_Collision_Cls(pm).Data)
tr, _ = Build_Program(prog, cell, meta, kin, {})
seg = tr.Segment_Command.index('Robot.MovePath(PATH_FORK_CLOSE)'); a, b = int(tr.Segment_Start[seg]), int(tr.Segment_End[seg])
overlap = 1.0
for i in range(a, b):
    Pose_Sample(kin, tr, i); overlap = min(overlap, Bar_Gap(pm, kin.Data))
free = Free_Steps(prog) if prog.Optimized is not None else Free_Steps(prog, Load_Optimization(taught))
m = Build_Cell_Model(cell); drive = Trajectory_Drive_Cls(tr, prog, free); run = Physics_Run_Cls(m, drive, meta, cell); d = run.Data
d.qpos[run.qpcb] += dx * 1e-6; mujoco.mj_forward(m, d)
t0, t1 = float(tr.T[a]), float(tr.T[b - 1]); fc = np.zeros(6); peak = (0, 0, 0, 0); hinge = 0.0; hd = m.jnt_dofadr[m.joint('handle_hinge').id]
while not run.Done:
    run.Step()
    if t0 <= d.time <= t1 + 0.5:
        f, n, fv = 0.0, 0, np.zeros(3)
        for i, ct in enumerate(d.contact[:d.ncon]):
            if {m.body(m.geom_bodyid[ct.geom1]).name, m.body(m.geom_bodyid[ct.geom2]).name} == {'aubo/tool/ee_tool', 'mg06_handle'}:
                mujoco.mj_contactForce(m, d, i, fc); f += fc[0]; n += 1; fv += ct.frame.reshape(3, 3).T @ fc[:3]
        if f > peak[0]: peak = (f, n, np.linalg.norm(fv), d.time - t0)
        hinge = max(hinge, abs(d.qfrc_constraint[hd]))
res = run.Result(); checks = Check_Program_Acceptance(res, cell)
closed = res.Closed
print('r %.2f mm %-9s dx %+3g um wait %5.3f s | min gap %+.3f mm | force %.0f N (%d pts, net-at-peak %.0f N) at +%.3f s, hinge-own %.1f N m | NET %.1f N HINGE %.2f N m SQUEEZE %.0f N | closed lid %.2f deg handle %.2f deg | end lid %.1f handle %.1f | %d/%d PASS%s | cycle %.3f'
      % (1000 * r, which, dx, wait, 1000 * overlap, peak[0], peak[1], peak[2], peak[3], hinge, res.Fork_Net_Force, res.Handle_Load, res.Fork_Squeeze, math.degrees(closed[0]) if closed else float('nan'),
         math.degrees(closed[1]) if closed else float('nan'), math.degrees(res.Lid_End), math.degrees(res.Handle_End), sum(x.Passed for x in checks), len(checks),
         '' if all(x.Passed for x in checks) else ' FAIL: ' + '; '.join(x.Name for x in checks if not x.Passed), tr.T[-1]))
