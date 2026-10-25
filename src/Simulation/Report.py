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
# File Name: ../Simulation/Report.py
## =========================================================================== ##

# Math (Mathematical functions)
import math
# Typing (Support for type hints)
import typing as tp
# Dataclasses (Data Classes)
from dataclasses import dataclass
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Custom Lib.: the files, the cell, the contact categories
#   ../Configuration/Core
from Configuration.Core import Cell_Configuration_Str
#   ../Scene/Core
from Scene.Core import CONST_CRATE_FLOOR
#   ../Collision/Core
from Collision.Core import CONST_COLLISION_CHECKS

#   ../Simulation/Core - the run's measured values
from Simulation.Core import Cycle_Result_Str, Place_Physics_Str
#   ../Simulation/Const - the constants the run and its report share
from Simulation.Const import CONST_CUPS, CONST_LOADER_CUT, CONST_VACUUM_OK

"""
Description:
    Initialization of constants.

    A physics run's acceptance and report (SPEC §10.1, §7.6): the criteria checked from the run's returned values, the
    program warnings, the place into MG06's items, the closest free transfer to the loader, and the report's lines. It
    prints nothing.
"""
# SPEC §10.1: the picks (the limits are in Configuration/Cell.yaml).
#   Unit [int]
CONST_PICKS = 2
# A robot actuator counts as saturated (its torque at the model's force limit, the demand clipped) at this fraction of the
# limit (step 6: FAIL; the limits are the vendored model's estimates, KNOWN_ISSUES.md).
#   Unit [float] (-)
CONST_TORQUE_SATURATION = 0.999
# The jaws close no further while the PCB is placed into MG06 when their gap never shrinks by more than this (their contact
# compliance under the clamp; measured 0.0002 mm).
#   Unit [float] (m)
CONST_JAW_STILL = 1e-5


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


def Format_Report(result: Cycle_Result_Str) -> tp.List[str]:
    """
    Description:
        The report lines of SPEC §4.6 (mm and deg), as the legacy run printed them (commit 7a8560c8c16806739a02da8e989c2a2f65d1e41d).

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
    lines.append('summed normal force fork -> handle bar: %.0f N (%d contact points)' % (r.Fork_Force, r.Fork_Points))
    lines.append('deepest PCB penetration: ' + ', '.join(f'{k} {1000*v[0]:.2f} mm (t={v[1]})' for k, v in sorted(r.Penetration.items())))
    for k in CONST_COLLISION_CHECKS:
        v = r.Contacts.get(k)
        lines.append(f'{k:50s}: ' + ('no contact' if v is None else f'deepest {1000*v[0]:.2f} mm  {v[1]} / {v[2]}  t={v[3]}'))
    lines.append('MG06 at the end: lid %.1f deg, handle %.1f deg' % (np.degrees(r.Lid_End), np.degrees(r.Handle_End)))
    return lines


def Check_Acceptance(result: Cycle_Result_Str, cell: Cell_Configuration_Str) -> tp.List[Acceptance_Check_Str]:
    """
    Description:
        The acceptance criteria of SPEC §10.1 (items 1-5 and 7; item 6, no instability, is Run_Physics
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


def Check_Program_Acceptance(result: Cycle_Result_Str, cell: Cell_Configuration_Str) -> tp.List[Acceptance_Check_Str]:
    """
    Description:
        The acceptance criteria of a program's run (SPEC §10.1, F11): Check_Acceptance's, then every vacuum hold keeps 4 of 4
        cups (from the vacuum on to off: the release at the take and every carry included), the PCB is never lost, every WaitDI
        on the vacuum switch gets its input in time, the robot and the carried PCB at least Acceptance.Loader_Distance from the
        loader in every free transfer; step 6's bounds: the slip and the tracking error (Slip_Max, Tracking_Max), the fork's net
        force on the handle bar and the handle hinge load (Fork_Net_Force_Max, Handle_Load_Max), the free transfers at least
        Clearance.Loader_Free from the loader, no robot actuator saturated (CONST_TORQUE_SATURATION of the model's limits); the B
        work's place into MG06 (owner, 2026-10-25; Place_Checks): no jaw closing at L_PLACE, no press there, the release's
        transient and settled penetration, no side guide touched on the way down, the PCB seated, the handover's centring and the
        re-grip's ledge gap.

    Args:
        (1) result [Cycle_Result_Str(object)]: A program run's values.
        (2) cell [Cell_Configuration_Str(object)]: The limits (Cell.yaml Acceptance).

    Returns:
        (1) parameter [List<Acceptance_Check_Str>]: One verdict per criterion, in SPEC order.
    """

    r, out = result, Check_Acceptance(result, cell)
    holds = '; '.join('%.2f-%.2f s min %d/%d%s' % (h.T_On, h.T_Off, h.Min_Cups, CONST_CUPS, ' at %.3f s' % h.T_Min if h.Min_Cups < CONST_CUPS else '')
                      for h in r.Holds)
    out.append(Acceptance_Check_Str('every vacuum hold keeps %d/%d cups' % (CONST_CUPS, CONST_CUPS),
                                    len(r.Holds) == CONST_PICKS and all(h.Min_Cups == CONST_CUPS for h in r.Holds), holds or 'no hold'))
    out.append(Acceptance_Check_Str('the PCB is never lost', r.Fault_Kind != 'lost', r.Fault if r.Fault_Kind == 'lost' else 'never lost'))
    out.append(Acceptance_Check_Str('every WaitDI gets %s in time' % CONST_VACUUM_OK, r.Fault_Kind != 'input',
                                    r.Fault if r.Fault_Kind == 'input' else 'every one'))
    closest = Free_Loader_Distance(r)
    out.append(Acceptance_Check_Str('free transfers >= %.1f mm from the loader' % (1000 * cell.Loader_Distance),
                                    closest is None or closest[0] >= cell.Loader_Distance,
                                    'no free transfer' if closest is None else '%.2f mm at %.3f s, %s (%s)' % (1000 * closest[0], closest[1],
                                                                                                             closest[3], closest[2])))
    out.append(Acceptance_Check_Str('PCB slip <= %.2f mm' % (1000 * cell.Slip_Max), r.Slip <= cell.Slip_Max, '%.2f mm' % (1000 * r.Slip)))
    out.append(Acceptance_Check_Str('tracking error <= %.4f rad' % cell.Tracking_Max, r.Track_Error <= cell.Tracking_Max, '%.4f rad' % r.Track_Error))
    out.append(Acceptance_Check_Str('fork net force <= %.0f N (sim. regression bound)' % cell.Fork_Net_Force_Max,
                                    r.Fork_Net_Force <= cell.Fork_Net_Force_Max,
                                    '%.1f N (summed normal %.0f N, for information)' % (r.Fork_Net_Force, r.Fork_Force)))
    out.append(Acceptance_Check_Str('hinge load <= %.1f N m (sim. regression bound)' % cell.Handle_Load_Max, r.Handle_Load <= cell.Handle_Load_Max,
                                    '%.2f N m' % r.Handle_Load))
    out.append(Acceptance_Check_Str('free transfers >= Loader_Free %.1f mm' % (1000 * cell.Loader_Free),
                                    closest is None or closest[0] >= cell.Loader_Free,
                                    'no free transfer' if closest is None else '%.2f mm at %.3f s, %s (%s)' % (1000 * closest[0], closest[1],
                                                                                                             closest[3], closest[2])))
    share = r.Joint_Torque / r.Torque_Limit
    out.append(Acceptance_Check_Str('no robot actuator saturated (model limits)', bool(np.all(share < CONST_TORQUE_SATURATION)),
                                    ' '.join('%.0f%%' % (100 * x) for x in share) + ' of %s N m' % '/'.join('%g' % x for x in r.Torque_Limit)))
    return out + Place_Checks(r.Place, cell)


def Place_Checks(place: tp.Optional[Place_Physics_Str], cell: Cell_Configuration_Str) -> tp.List[Acceptance_Check_Str]:
    """
    Description:
        The place into MG06 and the gripper operations around it (B work, items 7 and 9; owner, 2026-10-25): from reaching
        L_PLACE to the open command the jaw gap never shrinks (CONST_JAW_STILL) and the PCB - holder force stays within
        Place_Holder_Force; after the release the deepest penetration within Release_Penetration while it drops and within
        Rest_Penetration + Settle_Margin settled; no side guide touched from the travel height to the release; Seat_Time after the
        release the PCB seated (at least Seat_Supports supports within Seat_Height, slower than Seat_Speed, tilted less than
        Seat_Tilt, touching no side guide and no jaw); the handover clamp's centring at most Handover.Lateral_Max; the re-grip's
        PCB-ledge gap within Handover.Ledge_Gap +- Ledge_Tolerance. A run without a place fails each, 'not measured'.

    Args:
        (1) place [Place_Physics_Str(object)]: The measurements, or None.
        (2) cell [Cell_Configuration_Str(object)]: The limits.

    Returns:
        (1) parameter [List<Acceptance_Check_Str>]: One verdict per criterion.
    """

    names = ['place: no jaw closing at L_PLACE', 'place: holder force <= %.1f N (jaws closed)' % cell.Place_Holder_Force,
             'release: penetration <= %.2f mm while dropping' % (1000 * cell.Release_Penetration),
             'release: settled penetration <= %.4f mm' % (1000 * (cell.Rest_Penetration + cell.Settle_Margin)),
             'place: no side guide touched on the way down', 'PCB seated in MG06 within %.1f s' % cell.Seat_Time,
             'handover: centring <= %.1f mm' % (1000 * cell.Lateral_Max),
             're-grip: ledge gap %.1f +- %.1f mm' % (1000 * cell.Ledge_Gap, 1000 * cell.Ledge_Tolerance)]
    p = place
    if p is None or math.isinf(p.Release):
        return [Acceptance_Check_Str(n, False, 'not measured (no release into MG06)') for n in names]
    within = [n for n, gap in sorted(p.Support_Gaps.items()) if abs(gap) <= cell.Seat_Height]
    seated = (len(p.Supports) >= cell.Seat_Supports and len(within) >= cell.Seat_Supports and p.Speed < cell.Seat_Speed and p.Tilt < cell.Seat_Tilt
              and not p.Other_Contacts)
    low, high = cell.Ledge_Gap - cell.Ledge_Tolerance, cell.Ledge_Gap + cell.Ledge_Tolerance
    return [Acceptance_Check_Str(names[0], p.Jaw_Closing <= CONST_JAW_STILL, 'jaw gap decrease %.4f mm' % (1000 * p.Jaw_Closing)),
            Acceptance_Check_Str(names[1], p.Holder_Force <= cell.Place_Holder_Force, '%.2f N' % p.Holder_Force),
            Acceptance_Check_Str(names[2], -p.Transient <= cell.Release_Penetration, '%.4f mm (drop %.3f mm from the release at %.3f s)'
                                 % (-1000 * p.Transient, 1000 * p.Drop, p.Release)),
            Acceptance_Check_Str(names[3], -p.Settled <= cell.Rest_Penetration + cell.Settle_Margin, '%.4f mm' % (-1000 * p.Settled)),
            Acceptance_Check_Str(names[4], p.Guide_Min > 0.0, 'closest %.3f mm, %s' % (1000 * p.Guide_Min, p.Guide_Pair)),
            Acceptance_Check_Str(names[5], seated, 'on %s (%d within %.2f mm), %.3f mm/s, tilt %.3f deg, settled in %.3f s%s' % (
                '/'.join(p.Supports) or 'no support', len(within), 1000 * cell.Seat_Height, 1000 * p.Speed, math.degrees(p.Tilt), p.Settle_Time,
                ', touching ' + ', '.join(p.Other_Contacts) if p.Other_Contacts else '')),
            Acceptance_Check_Str(names[6], abs(p.Centring) <= cell.Lateral_Max, '%+.3f mm along the jaw axis' % (1000 * p.Centring)),
            Acceptance_Check_Str(names[7], low <= p.Regrip_Gap <= high, '%.3f mm' % (1000 * p.Regrip_Gap))]


def Free_Loader_Distance(result: Cycle_Result_Str) -> tp.Optional[tp.Tuple[float, float, str, str]]:
    """
    Description:
        The closest robot - loader or carried PCB - loader distance over a run's free transfers (F11).

    Args:
        (1) result [Cycle_Result_Str(object)]: A run's values.

    Returns:
        (1) parameter [Tuple<float, float, string, string>]: (distance (m), time (s), pair, step), or None without a free step.
    """

    found = [(v[0], v[1], v[2], s.Name) for s in result.Steps if s.Free for v in (s.Robot_Loader, s.PCB_Loader) if v is not None]
    return min(found) if found else None


def Program_Warnings(result: Cycle_Result_Str, cell: Cell_Configuration_Str) -> tp.List[str]:
    """
    Description:
        The warnings of a program's run, which never fail it (F11, owner): a step whose PCB slip or robot tracking error exceeds
        Acceptance.Slip_Max / Tracking_Max (1.5 x the legacy run's; also an acceptance failure), a free transfer closer to the loader than
        Clearance.Loader_Free.

    Args:
        (1) result [Cycle_Result_Str(object)]: A program run's values.
        (2) cell [Cell_Configuration_Str(object)]: The thresholds.

    Returns:
        (1) parameter [List<string>]: One sentence per warning.
    """

    out = []
    for s in result.Steps:
        if s.Slip > cell.Slip_Max:
            out.append('%s: PCB slip %.2f mm at %.3f s above %.2f mm (1.5 x the legacy run)'
                       % (s.Name, 1000 * s.Slip, s.Slip_T, 1000 * cell.Slip_Max))
        if s.Track_Error > cell.Tracking_Max:
            out.append('%s: robot tracking error %.4f rad above %.4f rad (1.5 x the legacy run)' % (s.Name, s.Track_Error, cell.Tracking_Max))
        for what, v in (('robot', s.Robot_Loader), ('carried PCB', s.PCB_Loader)):
            if s.Free and v is not None and v[0] < cell.Loader_Free:
                out.append('%s: %s %.2f mm from the loader at %.3f s (%s), within Clearance.Loader_Free %.1f mm'
                           % (s.Name, what, 1000 * v[0], v[1], v[2], 1000 * cell.Loader_Free))
    return out


def Format_Program_Report(result: Cycle_Result_Str, cell: Cell_Configuration_Str, name: str) -> tp.List[str]:
    """
    Description:
        The report of a program's physics run (F11): the program and its fault, the lines of SPEC §4.6 (Format_Report), the
        vacuum holds, a row per step (FREE or LOCKED, parallel, the tracking error, the slip, the closest robot - loader and
        carried PCB - loader distances, the deepest contact of a CONST_COLLISION_CHECKS category), the slip of every carrying
        free transfer with the tool's tilt and the TCP's acceleration then and their maxima, the warnings, the acceptance of SPEC
        §10.1 (Check_Program_Acceptance) and the project's sentence on simulation results.

    Args:
        (1) result [Cycle_Result_Str(object)]: A program run's values.
        (2) cell [Cell_Configuration_Str(object)]: The limits.
        (3) name [string]: The program.

    Returns:
        (1) parameter [List<string>]: The lines, without newlines.
    """

    r = result
    mm = lambda v: '>%g' % (1000 * CONST_LOADER_CUT) if v is None or v[0] >= CONST_LOADER_CUT else '%.2f' % (1000 * v[0])
    lines = ['program %s: physics run of its trajectory, cycle %.3f s%s' % (name, r.Cycle_Time, '' if r.Fault is None else '; STOPPED: ' + r.Fault)]
    lines += Format_Report(r)
    lines.append('fork on the handle bar: net contact force max %.0f N at %.3f s, direction in the fork frame %s; squeeze max %.0f N; handle hinge '
                 'load max %.1f N m (the summed normal force above adds the squeeze, KNOWN_ISSUES.md)'
                 % (r.Fork_Net_Force, r.Fork_Net_T, '[%.2f %.2f %.2f]' % tuple(r.Fork_Net_Direction), r.Fork_Squeeze, r.Handle_Load))
    p = r.Place
    if p is not None and not math.isinf(p.Release):
        lines.append('place into MG06: the jaws release the PCB at %.3f s, it drops %.3f mm and is seated in %.3f s on %s, at z %.3f mm, tilt %.3f '
                     'deg; the closest side guide on the way down %.3f mm (%s)'
                     % (p.Release, 1000 * p.Drop, p.Settle_Time, '/'.join(p.Supports), 1000 * p.Seated_Z, math.degrees(p.Tilt), 1000 * p.Guide_Min,
                        p.Guide_Pair))
    lines.append('vacuum holds: ' + ('; '.join('%.3f-%.3f s min %d/%d cups%s' % (h.T_On, h.T_Off, h.Min_Cups, CONST_CUPS, ' at %.3f s' % h.T_Min
                                                                              if h.Min_Cups < CONST_CUPS else '') for h in r.Holds) or 'none'))
    lines.append('%-24s %15s %-6s %-3s %11s %9s %13s %13s %s' % ('step', 'span s', 'kind', 'par', 'track rad', 'slip mm', 'robot-loader',
                                                                 'PCB-loader', 'deepest contact'))
    for s in r.Steps:
        deepest = min(s.Contacts.items(), key=lambda kv: kv[1][0]) if s.Contacts else None
        contact = '-' if deepest is None else '%s %.2f mm (%s / %s, t=%s)' % (deepest[0], 1000 * deepest[1][0], *deepest[1][1:])
        lines.append('%-24s %7.3f-%7.3f %-6s %-3s %11.4f %9.2f %13s %13s %s' % (
            s.Name or '(start)', s.Start, s.End, 'FREE' if s.Free else 'LOCKED', 'yes' if s.Parallel else '', s.Track_Error, 1000 * s.Slip,
            mm(s.Robot_Loader), mm(s.PCB_Loader), contact))
    lines.append('robot-loader: to the carriages and jaws (in a FREE step also the PCB in the jaws); PCB-loader: the carried PCB to them, '
                 'which touches the jaws at the handover and the take by design (pcb_jaws)')
    lines.append('carrying free transfers (slip max and when; tool tilt from vertical and TCP acceleration then; their maxima while carried):')
    for s in r.Steps:
        if s.Free and s.Slip > 0:
            lines.append('  %-22s slip %.2f mm at %.3f s: tilt %.1f deg, acceleration %.2f m/s^2; max tilt %.1f deg, max acceleration %.2f m/s^2'
                         % (s.Name, 1000 * s.Slip, s.Slip_T, np.degrees(s.Slip_Tilt), s.Slip_Acceleration, np.degrees(s.Tilt_Max),
                            s.Acceleration_Max))
    warnings = Program_Warnings(r, cell)
    lines += ['[WARNING] %s' % w for w in warnings] or ['warnings: none']
    checks = Check_Program_Acceptance(r, cell)
    lines += ['%-50s %s  %s' % (c.Name, 'PASS' if c.Passed else 'FAIL', c.Detail) for c in checks]
    lines.append('the fork net force and hinge load bounds are regression bounds derived from the simulation (the worst mode x 1.15) and '
                 'the torque bound is the model\'s actuator estimate - not safety limits: no vendor rating is known (KNOWN_ISSUES.md)')
    lines.append('%s: the physics run %s SPEC §10.1 (%d of %d). Simulation results only: not validated for the real cell.'
                 % (name, 'meets' if all(c.Passed for c in checks) else 'FAILS', sum(c.Passed for c in checks), len(checks)))
    return lines
