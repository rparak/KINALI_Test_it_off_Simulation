"""`kinali-sim` command line.

    kinali-sim export-xml [--out FILE]      write the whole cell as one MJCF (default models/cell_scene.xml)
    kinali-sim run [--plan FILE]            headless physics run of a legacy plan (default legacy/cell_plan.npz),
                                            prints the report, writes output/report.txt, checks brief §5.6

Exit codes of `run`: 0 all checks pass, 1 a check failed, 2 the simulation became unstable.
"""
import sys, io, argparse, contextlib, importlib.util
from pathlib import Path
from kinali_sim import paths

# brief §5.6 acceptance limits
PLACE_ERR_MM, PLACE_TILT_DEG = 0.5, 0.2         # PCB in MG06
CRATE_DIST_MM = 2.0                             # PCB in the output crate, horizontal distance from the crate centre
LID_OPEN_DEG, OPEN_TOL_DEG = -39.96, 2.0        # MG06 ends open: lid at its open stop (-0.6974 rad), handle up (0 deg)
N_PICKS, N_CUPS = 2, 4                          # vacuum ON events (crate pick, loader take) and cups that must touch


def load_legacy_cycle():
    """import legacy/test_cell_cycle.py as a module (it builds the model and loads the default plan on import)"""
    spec = importlib.util.spec_from_file_location('legacy_test_cell_cycle', paths.LEGACY / 'test_cell_cycle.py')
    mod = importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
    return mod


def acceptance(res):
    """list of (check, passed, detail) for the brief §5.6 criteria, from the values returned by the legacy run"""
    out = []
    for k, v in res['contacts'].items():
        out.append((k, v is None, 'no contact' if v is None else f"deepest {v['depth_mm']:.2f} mm {v['geoms'][0]} / {v['geoms'][1]} t={v['t']}"))
    picks = [e for e in res['vacuum'] if e['on']]
    out.append((f'vacuum ON {N_PICKS}x with {N_CUPS}/{N_CUPS} cups', len(picks) == N_PICKS and all(e['cups'] == N_CUPS for e in picks),
                ', '.join(f"t={e['t']:.2f}s {e['cups']}/{N_CUPS}" for e in picks)))
    p = res['placed']
    out.append((f'PCB in MG06 error < {PLACE_ERR_MM} mm, tilt < {PLACE_TILT_DEG} deg',
                p['error_mm'] < PLACE_ERR_MM and p['tilt_deg'] < PLACE_TILT_DEG, f"{p['error_mm']:.2f} mm, {p['tilt_deg']:.2f} deg"))
    f = res['final_pcb']
    out.append((f'PCB in the output crate < {CRATE_DIST_MM} mm from the centre', f['dist_xy_mm'] < CRATE_DIST_MM, f"{f['dist_xy_mm']:.2f} mm"))
    e = res['mg06_end']
    out.append(('MG06 ends open', abs(e['lid_deg'] - LID_OPEN_DEG) < OPEN_TOL_DEG and abs(e['handle_deg']) < OPEN_TOL_DEG,
                f"lid {e['lid_deg']:.1f} deg, handle {e['handle_deg']:.1f} deg"))
    return out


class _Tee(io.TextIOBase):
    def __init__(self, *streams): self.streams = streams
    def write(self, s):
        for st in self.streams: st.write(s)
        return len(s)
    def flush(self):
        for st in self.streams: st.flush()


def cmd_export_xml(a):
    from kinali_sim import model
    out = Path(a.out) if a.out else paths.CELL_SCENE_XML
    model.export_xml(out)
    print(f'wrote {out}')
    return 0


def cmd_run(a):
    plan = Path(a.plan).resolve() if a.plan else paths.LEGACY_PLAN
    report = Path(a.report) if a.report else paths.REPORT
    cycle = load_legacy_cycle()
    buf = io.StringIO(); code = 0
    with contextlib.redirect_stdout(_Tee(sys.stdout, buf)):
        try:
            res = cycle.run(plan=plan)
        except RuntimeError as ex:                      # MuJoCo reset the state (d.time went backwards)
            print(f'[ERROR] {ex}'); res = None; code = 2
    report.parent.mkdir(parents=True, exist_ok=True); report.write_text(buf.getvalue())
    if res is not None:
        checks = acceptance(res); code = 0 if all(ok for _, ok, _ in checks) else 1
        print('\nacceptance (brief §5.6):', file=sys.stderr)
        for name, ok, detail in checks: print(f"  [{'PASS' if ok else 'FAIL'}] {name:55s} {detail}", file=sys.stderr)
    print(f'report written to {report}' + ('' if code == 0 else f' - FAILED (exit {code})'), file=sys.stderr)
    return code


def main(argv=None):
    ap = argparse.ArgumentParser(prog='kinali-sim', description='KINALI Test it off cell simulation (MuJoCo)')
    sub = ap.add_subparsers(dest='cmd', required=True)
    p = sub.add_parser('export-xml', help='write the whole cell as one MJCF for the MuJoCo viewer')
    p.add_argument('--out', help=f'output file (default {paths.CELL_SCENE_XML.relative_to(paths.ROOT)})')
    p.set_defaults(func=cmd_export_xml)
    p = sub.add_parser('run', help='headless physics run of a legacy plan + report + acceptance checks')
    p.add_argument('--plan', help=f'plan file (default {paths.LEGACY_PLAN.relative_to(paths.ROOT)})')
    p.add_argument('--report', help=f'report file (default {paths.REPORT.relative_to(paths.ROOT)})')
    p.set_defaults(func=cmd_run)
    a = ap.parse_args(argv)
    return a.func(a)


if __name__ == '__main__':
    sys.exit(main())
