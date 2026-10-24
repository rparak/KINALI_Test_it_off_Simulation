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
# File Name: ../Simulation/Core.py
## =========================================================================== ##

# JSON (JSON encoder and decoder)
import json
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass, field
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco
# Custom Lib.: the files, the cell, the contact categories
#   ../Configuration/Core
from Configuration.Core import CONST_REFERENCE_PLAN, PCB_Meta_Str, Cell_Configuration_Str
#   ../Configuration/Program_Parser - a loader position
from Configuration.Program_Parser import Loader_Position_Str
#   ../Scene/Core
from Scene.Core import CONST_ROBOT_JOINTS, CONST_CUP_N, CONST_CRATE_FLOOR, Crate_Center, PCB_In_Crate
#   ../Collision/Core
from Collision.Core import CONST_COLLISION_CHECKS, CONST_LOADER_BODIES, Contact_Classifier_Cls, Penetration_Group, Cup_Contacts

"""
Description:
    Initialization of constants.

    The physics run of a plan (SPEC §7.6): servo set-points with velocity feed-forward, min-jerk loader
    profiles, vacuum events, the instability guard; the measured values, the report of SPEC §4.6 and the
    acceptance checks of SPEC §10.1. Ported from legacy/test_cell_cycle.py (git tag legacy-planner-reference;
    design-decisions): the report and every measured value were identical to that run's. It prints nothing.
"""
# The PCB starts 0.3 mm above the input crate's floor, so it settles onto it.
#   Unit [float] (m)
CONST_PCB_DROP = 0.0003
# The robot servos are actuators 3..8 (SPEC §4.3).
#   Unit [slice]
CONST_ROBOT_ACTUATORS = slice(3, 9)
# The progress callback is called every this many steps (2 s of simulated time).
#   Unit [int]
CONST_PROGRESS_STEPS = 4000
# SPEC §10.1: the picks and the cups that must touch at each (the limits are in Configuration/Cell.yaml).
#   Unit [int], [int]
CONST_PICKS, CONST_CUPS = 2, 4


class Simulation_Error_Cls(RuntimeError):
    """
    Description:
        The simulation cannot run or became unstable (MuJoCo reset the state). The message says when and
        what to check; a script prints it verbatim as an [ERROR] line and exits 2.
    """


@dataclass
class Legacy_Plan_Str:
    """
    Description:
        A plan in the format of the legacy IK planner (cell_plan.npz; the planner is legacy/plan_cell.py at git tag
        legacy-planner-reference). Built by Read_Legacy_Plan.
    """

    # Sample time of the robot trajectory.
    #   Unit [float] (s)
    Dt: float = 0.004
    # The robot joint trajectory, one row per sample.
    #   Unit [Matrix<float> Nx6] (rad)
    Q: np.ndarray = field(default_factory=lambda: np.zeros((0, 6)))
    # The cycle time.
    #   Unit [float] (s)
    T_End: float = 0.0
    # Loader moves per channel ('y', 'z', 'jaw'): [start time, duration, target].
    #   Unit [Dict<string, List<List<float>>>] (s, s, m)
    Loader: tp.Dict[str, tp.List[tp.List[float]]] = field(default_factory=dict)
    # Vacuum events: [time, 'suction_on' or 'suction_off'].
    #   Unit [List<List>] (s, -)
    Events: tp.List[tp.List] = field(default_factory=list)
    # Phase labels: [start time, label].
    #   Unit [List<List>] (s, -)
    Phases: tp.List[tp.List] = field(default_factory=list)
    # The jaw opening at the start.
    #   Unit [float] (m)
    Jaw_Open: float = 0.0


@dataclass
class Vacuum_Event_Str:
    """
    Description:
        One vacuum switch of a run, and how many cups touched the PCB when it went on.
    """

    # When it switched.
    #   Unit [float] (s)
    T: float = 0.0
    # On (True) or off.
    #   Unit [bool]
    On: bool = False
    # Cups touching the PCB at switch-on (None at switch-off).
    #   Unit [int]
    Cups: tp.Optional[int] = None


@dataclass
class Cycle_Result_Str:
    """
    Description:
        What a cycle run measured (SPEC §4.6), in SI units. Built by Run_Cycle; formatted by Format_Report.
    """

    # The cycle time.
    #   Unit [float] (s)
    Cycle_Time: float = 0.0
    # The vacuum switches.
    #   Unit [List<Vacuum_Event_Str>]
    Vacuum: tp.List[Vacuum_Event_Str] = field(default_factory=list)
    # When the PCB was left in MG06, its position error against the place target and its tilt.
    #   Unit [float] (s), [Vector<float> 1x3] (m), [float] (rad)
    Placed_T: float = 0.0
    Placed_Error: np.ndarray = field(default_factory=lambda: np.zeros(3))
    Placed_Tilt: float = 0.0
    # MG06 closed with the robot away: lid and handle angles and the time, or None if not reached.
    #   Unit [Tuple<float>] (rad, rad, s)
    Closed: tp.Optional[tp.Tuple[float, float, float]] = None
    # The PCB's final pose, the output crate's centre, the PCB's final tilt.
    #   Unit [Vector<float> 1x7] (m, -), [Vector<float> 1x2] (m), [float] (rad)
    Final_PCB: np.ndarray = field(default_factory=lambda: np.zeros(7))
    Crate_Centre: np.ndarray = field(default_factory=lambda: np.zeros(2))
    Final_Tilt: float = 0.0
    # Largest robot joint tracking error, PCB slip in the cups, loader axis lag.
    #   Unit [float] (rad), [float] (m), [float] (m)
    Track_Error: float = 0.0
    Slip: float = 0.0
    Loader_Lag: float = 0.0
    # Largest total contact force of the fork on the handle bar.
    #   Unit [float] (N)
    Fork_Force: float = 0.0
    # Deepest PCB penetration per group: (distance, time).
    #   Unit [Dict<string, Tuple<float>>] (m, s)
    Penetration: tp.Dict[str, tp.Tuple[float, float]] = field(default_factory=dict)
    # Deepest contact per category of CONST_COLLISION_CHECKS: (distance, geom 1, geom 2, time), or absent.
    #   Unit [Dict<string, Tuple>] (m, -, -, s)
    Contacts: tp.Dict[str, tp.Tuple[float, str, str, float]] = field(default_factory=dict)
    # The MG06 lid's open stop (the model's lid_hinge lower limit), where the run starts.
    #   Unit [float] (rad)
    Lid_Open: float = 0.0
    # MG06 lid and handle at the end.
    #   Unit [float] (rad)
    Lid_End: float = 0.0
    Handle_End: float = 0.0


@dataclass
class Acceptance_Check_Str:
    """
    Description:
        One acceptance criterion of SPEC §10.1 and its verdict on a run.
    """

    # The criterion.
    #   Unit [string]
    Name: str = ''
    # Whether the run meets it.
    #   Unit [bool]
    Passed: bool = False
    # The measured value(s), for the report.
    #   Unit [string]
    Detail: str = ''


def Read_Legacy_Plan(path: str = CONST_REFERENCE_PLAN) -> Legacy_Plan_Str:
    """
    Description:
        Read a legacy plan (cell_plan.npz), the only reader of the format in src/.

    Args:
        (1) path [string]: The file (default Data/Reference/cell_plan.npz).

    Returns:
        (1) parameter [Legacy_Plan_Str(object)]: The plan.
    """

    try:
        plan = np.load(path)
        return Legacy_Plan_Str(Dt=float(plan['dt']), Q=plan['q'], T_End=float(plan['T']), Loader=json.loads(str(plan['loader'])),
                               Events=json.loads(str(plan['events'])), Phases=json.loads(str(plan['phases'])),
                               Jaw_Open=float(plan['J_OPEN']))
    except (OSError, KeyError, ValueError) as error:
        raise Simulation_Error_Cls('The plan %s cannot be read (%s). Use Data/Reference/cell_plan.npz or a plan in its format.'
                                   % (path, error)) from error


def Min_Jerk(s: float) -> float:
    """
    Description:
        The min-jerk profile 10 s^3 - 15 s^4 + 6 s^5 of a normalised time, clipped to [0, 1].

    Args:
        (1) s [float]: Normalised time.

    Returns:
        (1) parameter [float]: The fraction of the move done.
    """

    s = np.clip(s, 0, 1)
    return s**3 * (10 - 15 * s + 6 * s * s)


def Loader_Target(plan: Legacy_Plan_Str, start: tp.Dict[str, float], channel: str, t: float) -> float:
    """
    Description:
        The loader command of one channel at time t: every move started so far, min-jerk, in order.

    Args:
        (1) plan [Legacy_Plan_Str(object)]: The plan.
        (2) start [Dict<string, float>]: The start value per channel (m).
        (3) channel [string]: 'y', 'z' or 'jaw'.
        (4) t [float]: The time (s).

    Returns:
        (1) parameter [float]: The command (m).
    """

    v = start[channel]
    for t0, T, x in plan.Loader[channel]:
        if t >= t0:
            v = v + (x - v) * Min_Jerk((t - t0) / T)
    return v


def Robot_Target(plan: Legacy_Plan_Str, v_plan: np.ndarray, kv_kp: np.ndarray, t: float) -> tp.Tuple[np.ndarray, np.ndarray]:
    """
    Description:
        The servo set-point at time t: the planned position, linearly interpolated, plus velocity
        feed-forward kv/kp * v_ref (SPEC §4.3).

    Args:
        (1) plan [Legacy_Plan_Str(object)]: The plan.
        (2) v_plan [Matrix<float> Nx6]: The planned joint velocities (rad/s).
        (3) kv_kp [Vector<float> 1x6]: kv / kp of each servo (s).
        (4) t [float]: The time (s).

    Returns:
        (1) parameter [Vector<float> 1x6]: The servo command (rad).
        (2) parameter [Vector<float> 1x6]: The planned position q_ref (rad).
    """

    n = len(plan.Q)
    x = min(t / plan.Dt, n - 1.001)
    i = int(x)
    w = x - i
    j = min(i + 1, n - 1)
    q = (1 - w) * plan.Q[i] + w * plan.Q[j]
    v = (1 - w) * v_plan[i] + w * v_plan[j]
    return q + kv_kp * v, q


def Phase_At(plan: Legacy_Plan_Str, t: float) -> str:
    """
    Description:
        The label of the last phase started at or before t ('' before the first).

    Args:
        (1) plan [Legacy_Plan_Str(object)]: The plan.
        (2) t [float]: The time (s).

    Returns:
        (1) parameter [string]: The phase label.
    """

    ph = [p for t0, p in plan.Phases if t >= t0]
    return ph[-1] if ph else ''


def Run_Cycle(model: mujoco.MjModel, plan: Legacy_Plan_Str, pcb_meta: PCB_Meta_Str, cell: Cell_Configuration_Str,
              loader_home: Loader_Position_Str, progress: tp.Optional[tp.Callable[[float, str], None]] = None) -> Cycle_Result_Str:
    """
    Description:
        The physics run of a plan from the PCB in the input crate to the end of the cycle, with the
        measurements of SPEC §4.6. MuJoCo resets the state on instability; the run detects the time going
        backwards and raises instead of continuing (SPEC §4.7).

    Args:
        (1) model [mujoco.MjModel]: The compiled cell (Scene.Core.Build_Cell_Model).
        (2) plan [Legacy_Plan_Str(object)]: The plan.
        (3) pcb_meta [PCB_Meta_Str(object)]: The PCB poses.
        (4) cell [Cell_Configuration_Str(object)]: The cell configuration (the phase period, the cup touch).
        (5) loader_home [Loader_Position_Str(object)]: The loader's home (Constants.yaml L_HOME), where it starts.
        (6) progress [function]: Called with (time [s], phase) every CONST_PROGRESS_STEPS steps, or None.

    Returns:
        (1) parameter [Cycle_Result_Str(object)]: The measured values.
    """

    m = model
    d = mujoco.MjData(m)
    J = lambda n: m.jnt_qposadr[m.joint(n).id]
    QR = np.array([J('aubo/' + j) for j in CONST_ROBOT_JOINTS])
    qa, qh, qy, qz, qjl, qjr, qpcb = (J(n) for n in ['lid_hinge', 'handle_hinge', 'y_axis', 'z_axis', 'jaw_L', 'jaw_R', 'pcb_free'])
    TOOL, PCB = m.body('aubo/tool/ee_tool').id, m.body('pcb').id
    CUPS = {m.body(f'aubo/tool/vgc10_cup{i}').id for i in range(CONST_CUP_N)}
    A_VAC = [m.actuator(f'vacuum{i}').id for i in range(CONST_CUP_N)]
    A_ROBOT = CONST_ROBOT_ACTUATORS
    classifier = Contact_Classifier_Cls(m)
    kv_kp = m.actuator_biasprm[A_ROBOT, 2] / m.actuator_biasprm[A_ROBOT, 1]
    v_plan = np.gradient(plan.Q, plan.Dt, axis=0)
    start = {'y': loader_home.Y, 'z': 0.0, 'jaw': plan.Jaw_Open}
    # 1\ Initial state: MG06 open, loader home with open jaws, the PCB just above the input crate floor,
    #    the robot at the plan's first sample; the filtered loader drives start at home.
    mujoco.mj_resetData(m, d)
    lid_open = m.jnt_range[m.joint('lid_hinge').id][0]
    d.qpos[qa] = lid_open
    d.qpos[qy] = start['y']
    d.qpos[qjl] = d.qpos[qjr] = start['jaw']
    pp, pq = PCB_In_Crate('crate_pick')
    d.qpos[qpcb:qpcb + 3], d.qpos[qpcb + 3:qpcb + 7] = pp + [0, 0, CONST_PCB_DROP], pq
    d.qpos[QR] = plan.Q[0]
    d.ctrl[:3] = [start['y'], start['z'], start['jaw']]
    d.ctrl[A_ROBOT] = plan.Q[0]
    d.ctrl[A_VAC] = 0
    d.act[:] = 0
    d.act[m.actuator_actadr[0]] = start['y']
    mujoco.mj_forward(m, d)
    ev = list(plan.Events)
    worst, vacuum, pen, marks = {}, [], {}, {}
    track_err, slip, fork_force, loader_lag = 0.0, 0.0, 0.0, 0.0
    rel0, ph = None, ''
    fc = np.zeros(6)
    # 2\ The loop: events, commands, one step, the guard, the measurements.
    while d.time < plan.T_End:
        t = d.time
        while ev and t >= ev[0][0]:
            _, name = ev.pop(0)
            on = name == 'suction_on'
            d.ctrl[A_VAC] = 1.0 if on else 0.0
            rel0 = None
            vacuum.append(Vacuum_Event_Str(T=t, On=on, Cups=Cup_Contacts(m, d, PCB, CUPS, cell.Cup_Touch) if on else None))
        d.ctrl[:3] = [Loader_Target(plan, start, 'y', t), Loader_Target(plan, start, 'z', t), Loader_Target(plan, start, 'jaw', t)]
        d.ctrl[A_ROBOT], q_ref = Robot_Target(plan, v_plan, kv_kp, t)
        mujoco.mj_step(m, d)
        if d.time < t:
            raise Simulation_Error_Cls('The simulation became unstable at t = %.3f s (phase: %s): MuJoCo reset the state. '
                                       'Check the plan around that time and the last change to the model.' % (t, ph or 'start'))
        step = int(round(d.time / m.opt.timestep))
        if progress is not None and step % CONST_PROGRESS_STEPS == 0:
            progress(d.time, ph)
        track_err = max(track_err, np.max(np.abs(d.qpos[QR] - q_ref)))
        loader_lag = max(loader_lag, abs(d.qpos[qy] - d.ctrl[0]), abs(d.qpos[qz] - d.ctrl[1]))
        # PCB slip in the cups while the vacuum holds it.
        if d.ctrl[A_VAC[0]] > 0.5 and Cup_Contacts(m, d, PCB, CUPS, cell.Cup_Touch):
            R = d.xmat[TOOL].reshape(3, 3)
            rel = R.T @ (d.xpos[PCB] - d.xpos[TOOL])
            rel0 = rel if rel0 is None else rel0
            slip = max(slip, np.linalg.norm(rel - rel0))
        ff = 0.0
        for i, c in enumerate(d.contact[:d.ncon]):
            k = classifier.Classify(c.geom1, c.geom2, ph)
            if k and c.dist < worst.get(k, (0,))[0]:
                worst[k] = (c.dist, classifier.Geom_Name(c.geom1), classifier.Geom_Name(c.geom2), round(d.time, 2))
            b1, b2 = classifier.Body_Name(c.geom1), classifier.Body_Name(c.geom2)
            # The thin board: its penetration into whatever holds it.
            if 'pcb' in (b1, b2) and not (b1.startswith('aubo/') or b2.startswith('aubo/')):
                grp = Penetration_Group(b2 if b1 == 'pcb' else b1)
                if c.dist < pen.get(grp, (0,))[0]:
                    pen[grp] = (c.dist, round(d.time, 2))
            if {b1, b2} == {'aubo/tool/ee_tool', 'mg06_handle'}:
                mujoco.mj_contactForce(m, d, i, fc)
                ff += fc[0]
        fork_force = max(fork_force, ff)
        if step % cell.Phase_Steps == 0:
            ph = Phase_At(plan, d.time)
        if ph.startswith('loader: up, to park') and 'placed' not in marks:
            marks['placed'] = (d.qpos[qpcb:qpcb + 7].copy(), d.time)
        if 'away, MG06 closed' in ph and 'closed' not in marks:
            marks['closed'] = (d.qpos[qa], d.qpos[qh], d.time)
    # 3\ The values the report and the acceptance need.
    if 'placed' not in marks:
        raise Simulation_Error_Cls("The run never reached the phase 'loader: up, to park': the plan does not place the PCB in MG06.")
    pm, tm = marks['placed']
    qd, qi = np.zeros(4), np.zeros(4)
    mujoco.mju_negQuat(qi, np.array(pcb_meta.Quat))
    mujoco.mju_mulQuat(qd, pm[3:], qi)
    pf = d.qpos[qpcb:qpcb + 7].copy()
    return Cycle_Result_Str(Cycle_Time=plan.T_End, Vacuum=vacuum, Placed_T=tm, Placed_Error=pm[:3] - np.array(pcb_meta.Place_Pos),
                            Placed_Tilt=2 * np.arccos(min(1, abs(qd[0]))), Closed=marks.get('closed'), Final_PCB=pf,
                            Crate_Centre=Crate_Center('crate_place'),
                            Final_Tilt=np.arccos(min(1, abs(1 - 2 * (pf[4]**2 + pf[5]**2)))), Track_Error=track_err, Slip=slip,
                            Loader_Lag=loader_lag, Fork_Force=fork_force, Penetration=pen, Contacts=worst,
                            Lid_Open=lid_open, Lid_End=d.qpos[qa], Handle_End=d.qpos[qh])


def Format_Report(result: Cycle_Result_Str) -> tp.List[str]:
    """
    Description:
        The report lines of SPEC §4.6 (mm and deg), identical to the legacy run's (SPEC §10.2).

    Args:
        (1) result [Cycle_Result_Str(object)]: A run's values.

    Returns:
        (1) parameter [List<string>]: The lines, without newlines.
    """

    r = result
    lines = ['cycle time %.1f s' % r.Cycle_Time]
    for v in r.Vacuum:
        lines.append(f'{v.T:6.2f}s vacuum {"ON " if v.On else "OFF"}' + (f' ({v.Cups} of 4 cups touch the PCB)' if v.On else ''))
    e = r.Placed_Error
    lines.append(f'PCB in MG06 (t={r.Placed_T:.1f}s): error {1000*np.linalg.norm(e):.2f} mm (dx {1000*e[0]:.2f} dy {1000*e[1]:.2f} '
                 f'dz {1000*e[2]:.2f}), tilt {np.degrees(r.Placed_Tilt):.2f} deg')
    if r.Closed is not None:
        lines.append('MG06 closed with the robot away (t=%.1fs): lid %.1f deg, handle %.1f deg'
                     % (r.Closed[2], np.degrees(r.Closed[0]), np.degrees(r.Closed[1])))
    pf, cc = r.Final_PCB, r.Crate_Centre
    lines.append(f'PCB at the end: x {1000*pf[0]:.1f} y {1000*pf[1]:.1f} z {1000*pf[2]:.1f} mm (right crate centre {1000*cc[0]:.1f} '
                 f'{1000*cc[1]:.1f}, floor {1000*CONST_CRATE_FLOOR:.1f}); tilt {np.degrees(r.Final_Tilt):.2f} deg')
    lines.append('robot joint tracking error max %.4f rad, PCB slip in the cups max %.2f mm, loader axis lag max %.1f mm'
                 % (r.Track_Error, 1000 * r.Slip, 1000 * r.Loader_Lag))
    lines.append('max contact force fork -> handle bar: %.0f N' % r.Fork_Force)
    lines.append('deepest PCB penetration: ' + ', '.join(f'{k} {1000*v[0]:.2f} mm (t={v[1]})' for k, v in sorted(r.Penetration.items())))
    for k in CONST_COLLISION_CHECKS:
        v = r.Contacts.get(k)
        lines.append(f'{k:50s}: ' + ('no contact' if v is None else f'deepest {1000*v[0]:.2f} mm  {v[1]} / {v[2]}  t={v[3]}'))
    lines.append('MG06 at the end: lid %.1f deg, handle %.1f deg' % (np.degrees(r.Lid_End), np.degrees(r.Handle_End)))
    return lines


def Check_Acceptance(result: Cycle_Result_Str, cell: Cell_Configuration_Str) -> tp.List[Acceptance_Check_Str]:
    """
    Description:
        The acceptance criteria of SPEC §10.1 (items 1-5 and 7; item 6, no instability, is Run_Cycle
        returning at all) on a run's values.

    Args:
        (1) result [Cycle_Result_Str(object)]: A run's values.
        (2) cell [Cell_Configuration_Str(object)]: The limits (Cell.yaml Acceptance).

    Returns:
        (1) parameter [List<Acceptance_Check_Str>]: One verdict per criterion, in SPEC order.
    """

    r, out = result, []
    for k in CONST_COLLISION_CHECKS:
        v = r.Contacts.get(k)
        out.append(Acceptance_Check_Str(k, v is None, 'no contact' if v is None else
                                        'deepest %.2f mm %s / %s t=%s' % (1000 * v[0], v[1], v[2], v[3])))
    picks = [v for v in r.Vacuum if v.On]
    out.append(Acceptance_Check_Str('vacuum ON %dx with %d/%d cups' % (CONST_PICKS, CONST_CUPS, CONST_CUPS),
                                    len(picks) == CONST_PICKS and all(v.Cups == CONST_CUPS for v in picks),
                                    ', '.join('t=%.2fs %s/%d' % (v.T, v.Cups, CONST_CUPS) for v in picks)))
    error = float(np.linalg.norm(r.Placed_Error))
    out.append(Acceptance_Check_Str('PCB in MG06 error < %.1f mm, tilt < %.1f deg' % (1000 * cell.Place_Error, np.degrees(cell.Place_Tilt)),
                                    error < cell.Place_Error and r.Placed_Tilt < cell.Place_Tilt,
                                    '%.2f mm, %.2f deg' % (1000 * error, np.degrees(r.Placed_Tilt))))
    distance = float(np.linalg.norm(r.Final_PCB[:2] - r.Crate_Centre))
    out.append(Acceptance_Check_Str('PCB in the output crate < %.1f mm from the centre' % (1000 * cell.Crate_Distance),
                                    distance < cell.Crate_Distance, '%.2f mm' % (1000 * distance)))
    out.append(Acceptance_Check_Str('MG06 ends open', abs(r.Lid_End - r.Lid_Open) < cell.Open_Tolerance
                                    and abs(r.Handle_End) < cell.Open_Tolerance,
                                    'lid %.1f deg, handle %.1f deg' % (np.degrees(r.Lid_End), np.degrees(r.Handle_End))))
    deepest = max([-v[0] for v in r.Penetration.values()] + [0.0])
    out.append(Acceptance_Check_Str('PCB penetration <= %.1f mm' % (1000 * cell.Penetration), deepest <= cell.Penetration,
                                    ', '.join('%s %.2f mm' % (k, -1000 * v[0]) for k, v in sorted(r.Penetration.items()))))
    return out
