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
# File Name: ../Example/cell.py
## =========================================================================== ##

# System (Default)
import sys
# OS (Operating system interfaces)
import os
# The repository root, built from this file's location, so the command runs from any working directory.
#   Unit [string]
CONST_REPO_ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
#   Add access if it is not in the system path.
if os.path.join(CONST_REPO_ROOT, 'src') not in sys.path:
    sys.path.append(os.path.join(CONST_REPO_ROOT, 'src'))
# Argparse (Command-line parsing)
import argparse
# Functools (Higher-order functions)
import functools
# Importlib (The implementation of import)
import importlib
# Typing (Support for type hints)
import typing as tp
# Custom Lib.:
#   ../Configuration/Core, ../Configuration/Const, ../Configuration/Program_Parser - the logs, the folders, the points
from Configuration.Core import Script_Log_Cls, Configuration_Error_Cls, CONST_REPORTS
from Configuration.Const import CONST_ROOT
from Configuration.Program_Parser import Load_Constants, Load_Simulation, CONST_OPTIMIZED_PROGRAM_YAML

"""
Description:
    Initialization of constants.

    The one user entry point of the cell (owner, 2026-10-28; design-decisions: one user entry point): a subcommand per user
    action, each calling the internal modules of Example/Cell/ (and the regression tools of Example/Maintenance/ for run).
    The modules print their lines; each subcommand's lines go to logger/cell_<subcommand>.log.
"""
# The programs by the name shown (cell.py's --program) and by the modules' run names.
#   Unit [Dict<string, string>]
CONST_PROGRAMS = {'default': 'taught', 'optimized': 'optimized'}
# The exit codes (as the modules: 0 success, 1 a check failed, 2 the action cannot run).
#   Unit [int]
CONST_EXIT_OK, CONST_EXIT_FAILED, CONST_EXIT_ERROR = 0, 1, 2
# The sentence every result carries (KNOWN_ISSUES.md).
#   Unit [string]
CONST_SIMULATION_ONLY = 'Simulation results only: not validated for the real cell.'


@functools.lru_cache(maxsize=None)
def _Module(name: str) -> tp.Any:
    """
    Description:
        An internal module of Example/ (Cell/<module>.py or Maintenance/<tool>.py), imported by its name from its folder (put
        on sys.path), so that the worker processes of a parallel run (physics_baseline.Capture_All) import it again by name.

    Args:
        (1) name [string]: Its path below Example/.

    Returns:
        (1) parameter [module]: The module.
    """

    folder = os.path.join(CONST_REPO_ROOT, 'Example', os.path.dirname(name))
    if folder not in sys.path:
        sys.path.append(folder)
    return importlib.import_module(os.path.splitext(os.path.basename(name))[0])


def _Say(log_name: str, line: str) -> None:
    """
    Description:
        Print one of cell.py's own lines and append it to the subcommand's log.

    Args:
        (1) log_name [string]: The log (cell_<subcommand>).
        (2) line [string]: The line.
    """

    log = Script_Log_Cls(log_name, append=True)
    print(log.Line(line))
    log.Close()


def Viewer_Launch(platform: str, environ: tp.Mapping[str, str], executable: str) -> tp.Tuple[str, tp.Optional[str]]:
    """
    Description:
        How a viewer subcommand starts (owner, 2026-10-29): MuJoCo's passive viewer runs on macOS only under mjpython, so on
        macOS outside it cell.py re-launches itself under the mjpython next to the interpreter ('exec'), or names the
        missing launcher ('missing'); anywhere else, or already under mjpython (its launcher sets MJPYTHON_BIN), it runs
        ('run').

    Args:
        (1) platform [string]: sys.platform.
        (2) environ [Mapping<string, string>]: The environment.
        (3) executable [string]: sys.executable.

    Returns:
        (1) parameter [string]: 'run', 'exec' or 'missing'.
        (2) parameter [string]: The mjpython path for 'exec' and 'missing', else None.
    """

    if platform != 'darwin' or 'MJPYTHON_BIN' in environ:
        return 'run', None
    path = os.path.join(os.path.dirname(executable), 'mjpython')
    return ('exec', path) if os.path.isfile(path) and os.access(path, os.X_OK) else ('missing', path)


def _Viewer(argv: tp.List[str], log_name: str) -> tp.Optional[int]:
    """
    Description:
        Before a viewer opens: re-launch cell.py with the same arguments under mjpython when Viewer_Launch says so (the
        process is replaced), or exit 2 with the install hint when mjpython is missing; None when the viewer can run here.

    Args:
        (1) argv [List<string>]: cell.py's arguments.
        (2) log_name [string]: The log.

    Returns:
        (1) parameter [int]: CONST_EXIT_ERROR when mjpython is missing, or None.
    """

    action, path = Viewer_Launch(sys.platform, os.environ, sys.executable)
    if action == 'exec':
        sys.stdout.flush()
        os.execv(path, [path, os.path.abspath(__file__)] + list(argv))
    if action == 'missing':
        _Say(log_name, '[ERROR] The viewer needs MuJoCo\'s mjpython on macOS and there is none at %s: run ./install.sh (it installs '
                       'mujoco with mjpython into .venv), then Example/cell.py again.' % path)
        return CONST_EXIT_ERROR
    return None


def Check(args: argparse.Namespace, log_name: str) -> int:
    """
    Description:
        cell.py check: the key-point table (all points or --name) and the configuration and programs check. Writes nothing.

    Args:
        (1) args [argparse.Namespace]: The options.
        (2) log_name [string]: The log.

    Returns:
        (1) parameter [int]: 0 every point passes and every file loads, 1 otherwise.
    """

    points = _Module('Cell/check_keypoints.py').main(['--name', args.name] if args.name else [], log_name)
    programs = _Module('Cell/check_programs.py').main([], log_name)
    if points != CONST_EXIT_OK:
        _Say(log_name, '[ERROR] A key point fails (FAIL above): correct it in Configuration/Constants.yaml; for a point with a Reference '
                       'or a PCB_Hold, Example/cell.py point <name> prints the corrected lines.')
    return max(points, programs)


def Point(args: argparse.Namespace, argv: tp.List[str], log_name: str) -> int:
    """
    Description:
        cell.py point NAME: the point's check row; for a point defined by a Reference (or a PCB_Hold) the resolved Joints and
        Pose lines to paste into Configuration/Constants.yaml; with --show the viewer (--headless: posed and printed).

    Args:
        (1) args [argparse.Namespace]: The options.
        (2) argv [List<string>]: cell.py's arguments (for the mjpython re-launch).
        (3) log_name [string]: The log.

    Returns:
        (1) parameter [int]: The worst exit code of the steps.
    """

    if args.show and not args.headless:
        refused = _Viewer(argv, log_name)
        if refused is not None:
            return refused
    code = _Module('Cell/check_keypoints.py').main(['--name', args.name], log_name)
    try:
        constants = Load_Constants()
        simulation = Load_Simulation(constants)
        target = args.name in constants.Points and (constants.Points[args.name].Reference is not None
                                                    or simulation.Points[args.name].Hold_Location is not None)
    except Configuration_Error_Cls:
        target = False
    if target:
        code = max(code, _Module('Cell/resolve_keypoint.py').main(['--name', args.name] + (['--seed', args.seed] if args.seed else []), log_name))
    if args.show or args.headless:
        options = ['--name', args.name] + (['--headless'] if args.headless else [])
        for option in ('loader', 'gripper', 'pcb', 'mg06'):
            if getattr(args, option) is not None:
                options += ['--' + option, getattr(args, option)]
        code = max(code, _Module('Cell/show_keypoint.py').main(options, log_name))
    return code


def Build(args: argparse.Namespace, log_name: str) -> int:
    """
    Description:
        cell.py build: the Default program's trajectory and its kinematic check (Data/Generated/Trajectories/Program.npz,
        Data/Generated/Reports/Program_check.txt).

    Args:
        (1) args [argparse.Namespace]: The options (none).
        (2) log_name [string]: The log.

    Returns:
        (1) parameter [int]: 0 the check passes, 1 otherwise.
    """

    code = _Module('Cell/build_program.py').main([], log_name)
    if code != CONST_EXIT_OK:
        _Say(log_name, '[ERROR] The Default program does not build or its check fails (above): correct Configuration/ and run '
                       'Example/cell.py build again.')
    return code


def Optimize(args: argparse.Namespace, log_name: str) -> int:
    """
    Description:
        cell.py optimize: the optimizer (Data/Optimized/Program_Optimized.yaml, about 60 s), then the Optimized program's
        trajectory and its kinematic check.

    Args:
        (1) args [argparse.Namespace]: The options (none).
        (2) log_name [string]: The log.

    Returns:
        (1) parameter [int]: 0 the program and its check pass, 1 otherwise.
    """

    code = _Module('Cell/optimize_program.py').main([], log_name)
    if code == CONST_EXIT_OK:
        code = _Module('Cell/build_program.py').main(['--program', CONST_OPTIMIZED_PROGRAM_YAML], log_name)
    if code != CONST_EXIT_OK:
        _Say(log_name, '[ERROR] The Optimized program fails (above): correct Configuration/ (Optimization.yaml for the optimizer) and run '
                       'Example/cell.py optimize again.')
    return code


def Run(args: argparse.Namespace, log_name: str) -> int:
    """
    Description:
        cell.py run: both programs' physics runs (in parallel) against their baselines in test/Simulation/data/ - SPEC §10.1's
        acceptance and the drift - each report written to Data/Generated/Reports/<program>_physics.txt, and the comparison
        table. With --program one program's full physics report instead (printed and written), without the baselines.

    Args:
        (1) args [argparse.Namespace]: The options.
        (2) log_name [string]: The log.

    Returns:
        (1) parameter [int]: 0 within the acceptance and the baselines, 1 a failure, 2 a run cannot run.
    """

    if args.program is not None:
        return _Module('Cell/run_program.py').main(['--program', CONST_PROGRAMS[args.program]], log_name)
    baseline, run_program = _Module('Maintenance/physics_baseline.py'), _Module('Cell/run_program.py')
    results = _Module('Cell/results.py')
    log = Script_Log_Cls(log_name, append=True)
    code, runs = baseline.Compare_Programs(list(results.CONST_NAMES), None, baseline.CONST_BASELINES, log)
    log.Close()
    if code == CONST_EXIT_ERROR:
        return code
    os.makedirs(CONST_REPORTS, exist_ok=True)
    for program, now in runs.items():
        name = os.path.splitext(os.path.basename(run_program.CONST_VERSIONS[program]))[0]
        with open(os.path.join(CONST_REPORTS, name + '_physics.txt'), 'w', encoding='utf-8') as handle:
            handle.write('\n'.join(now['report']) + '\n')
    cycles = {p: now['metrics']['cycle_time'] for p, now in runs.items()}
    passed = {p: (sum(ok for _, ok, _ in now['acceptance']), len(now['acceptance'])) for p, now in runs.items()}
    for line in results.Table_Lines(cycles, passed):
        _Say(log_name, line)
    _Say(log_name, '[INFO] The physics reports are in %s (<program>_physics.txt).' % os.path.relpath(CONST_REPORTS, CONST_ROOT))
    if code != CONST_EXIT_OK:
        _Say(log_name, '[ERROR] The physics differs from the baselines (FAIL above). An acceptance FAIL is a fault to correct; if only '
                       'a drift FAILs and the change is intended, re-capture the baselines: Example/cell.py all --recapture')
    _Say(log_name, '[INFO] ' + CONST_SIMULATION_ONLY)
    return code


def Show(args: argparse.Namespace, argv: tp.List[str], log_name: str) -> int:
    """
    Description:
        cell.py show: a program in MuJoCo's viewer - its trajectory (kinematic; Home pause, Page Down or '.' next step, Page Up
        back, End the step's end) or with --physics its physics run (forward only; --speed). --headless poses (or runs to
        --time) and prints instead of opening the viewer.

    Args:
        (1) args [argparse.Namespace]: The options.
        (2) argv [List<string>]: cell.py's arguments (for the mjpython re-launch).
        (3) log_name [string]: The log.

    Returns:
        (1) parameter [int]: 0 shown, 1 a refused request or a stale trajectory, 2 the viewer cannot start.
    """

    if args.physics and args.step is not None:
        _Say(log_name, '[ERROR] --step poses a kinematic trajectory; a physics run goes forward only: use --physics --time T.')
        return CONST_EXIT_FAILED
    if not args.physics and args.speed is not None:
        _Say(log_name, '[ERROR] --speed is for the physics run: add --physics.')
        return CONST_EXIT_FAILED
    if not args.headless:
        refused = _Viewer(argv, log_name)
        if refused is not None:
            return refused
    options = (['--time', str(args.time)] if args.time is not None else []) + (['--headless'] if args.headless else []) \
        + (['--watchdog', str(args.watchdog)] if args.watchdog is not None else [])
    if args.physics:
        options += ['--speed', str(args.speed)] if args.speed is not None else []
        return _Module('Cell/show_physics.py').main(['--program', CONST_PROGRAMS[args.program]] + options, log_name)
    options += ['--step', args.step] if args.step is not None else []
    return _Module('Cell/show_program.py').main(['--version', args.program] + options, log_name)


def Recapture(log_name: str) -> int:
    """
    Description:
        cell.py all --recapture, after check, build and optimize passed (owner, 2026-10-29): the kinematic golden and the
        physics baselines compared first (what changed is printed), each re-captured when it differs - the baselines refused
        when a perturbation run fails SPEC §10.1 -, then the README's results block regenerated from the baselines (nothing
        else of the README), the table, and the reminder to commit and to refresh the support-chat brief.

    Args:
        (1) log_name [string]: The log.

    Returns:
        (1) parameter [int]: 0 re-captured (or unchanged), 2 a run cannot run or the re-capture is refused.
    """

    golden, baseline, results = _Module('Maintenance/golden_program.py'), _Module('Maintenance/physics_baseline.py'), _Module('Cell/results.py')
    commit, dirty = _Module('Cell/build_program.py').Git_State()
    source = 'Example/cell.py all --recapture at %s%s' % (commit[:7], ' with uncommitted changes' if dirty else '')
    changed = []
    if golden.main([], log_name) != CONST_EXIT_OK:
        if golden.main(['--out', golden.CONST_GOLDEN, '--source', source], log_name) != CONST_EXIT_OK:
            return CONST_EXIT_ERROR
        changed.append(os.path.relpath(golden.CONST_GOLDEN, CONST_ROOT))
    log = Script_Log_Cls(log_name, append=True)
    code, runs = baseline.Compare_Programs(list(results.CONST_NAMES), None, baseline.CONST_BASELINES, log)
    log.Close()
    if code == CONST_EXIT_ERROR:
        return code
    differ = [p for p, now in runs.items() if any(baseline.Compare(now, baseline.Read_Baseline(p, baseline.CONST_BASELINES)))]
    if differ:
        if baseline.main(['--out', baseline.CONST_BASELINES], log_name) != CONST_EXIT_OK:
            _Say(log_name, '[ERROR] The re-capture is refused (above): an acceptance check fails, which no baseline may accept; correct '
                           'the change and run Example/cell.py all --recapture again.')
            return CONST_EXIT_ERROR
        changed.append(os.path.relpath(baseline.CONST_BASELINES, CONST_ROOT))
    cycles, passed = results.Baseline_Results(baseline.CONST_BASELINES)
    readme = os.path.join(CONST_ROOT, 'README.md')
    if results.Update_Readme(readme, results.Readme_Block(cycles, passed)):
        changed.append('README.md (the results block)')
    for line in results.Table_Lines(cycles, passed):
        _Say(log_name, line)
    if not changed:
        _Say(log_name, '[INFO] Nothing to re-capture: the golden, the physics baselines and the README results block are current.')
        return CONST_EXIT_OK
    _Say(log_name, '[INFO] Re-captured: %s. Commit them with the change that moved them (in their own commit, the reason in its '
                   'message).' % ', '.join(changed))
    _Say(log_name, '[INFO] Then refresh the support-chat brief, .claude/project-docs/SUPPORT_CHAT.md (/support-brief REFRESH).')
    _Say(log_name, '[INFO] ' + CONST_SIMULATION_ONLY)
    return CONST_EXIT_OK


def All(args: argparse.Namespace, log_name: str) -> int:
    """
    Description:
        cell.py all: check -> build -> optimize -> run, stopping at the first failure with what to do; the optimizer only when
        Data/Optimized/Program_Optimized.yaml is stale (else its trajectory is built); --recapture instead of run re-captures
        the golden and the physics baselines after an intended change and regenerates the README's results block (Recapture).

    Args:
        (1) args [argparse.Namespace]: The options (--recapture).
        (2) log_name [string]: The log.

    Returns:
        (1) parameter [int]: The first failing step's exit code, or 0.
    """

    code = Check(argparse.Namespace(name=None), log_name)
    if code != CONST_EXIT_OK:
        _Say(log_name, '[ERROR] all stopped at check: correct what FAILs above, then run Example/cell.py all again.')
        return code
    code = Build(args, log_name)
    if code != CONST_EXIT_OK:
        _Say(log_name, '[ERROR] all stopped at build (above).')
        return code
    stale = _Module('Cell/check_programs.py').Optimized_Stale()
    if stale:
        _Say(log_name, '[INFO] %s is stale (%s changed): the optimizer runs.' % (os.path.relpath(CONST_OPTIMIZED_PROGRAM_YAML, CONST_ROOT),
                                                                            ', '.join(stale)))
        code = Optimize(args, log_name)
    else:
        _Say(log_name, '[INFO] %s is current: the optimizer is skipped (Example/cell.py optimize runs it anyway); its trajectory is built.'
             % os.path.relpath(CONST_OPTIMIZED_PROGRAM_YAML, CONST_ROOT))
        code = _Module('Cell/build_program.py').main(['--program', CONST_OPTIMIZED_PROGRAM_YAML], log_name)
    if code != CONST_EXIT_OK:
        _Say(log_name, '[ERROR] all stopped at optimize (above).')
        return code
    if args.recapture:
        return Recapture(log_name)
    code = Run(argparse.Namespace(program=None), log_name)
    if code != CONST_EXIT_OK:
        _Say(log_name, '[ERROR] all stopped at run (above).')
        return code
    _Say(log_name, '[INFO] all passed: check, build, optimize, run.')
    return CONST_EXIT_OK


def Parser() -> argparse.ArgumentParser:
    """
    Description:
        cell.py's command line: a subcommand per user action, each with its --help.

    Returns:
        (1) parameter [argparse.ArgumentParser]: The parser.
    """

    parser = argparse.ArgumentParser(prog='cell.py', formatter_class=argparse.RawDescriptionHelpFormatter,
                                     description='The Test-it-off cell: check the key points, build and optimize the programs, run them '
                                                 'with physics, view them.\n' + CONST_SIMULATION_ONLY,
                                     epilog='exit codes: 0 success, 1 a check failed, 2 cannot run (a missing or stale input, no viewer)\n'
                                            'Run a subcommand with --help for its options.')
    commands = parser.add_subparsers(dest='command', metavar='{check,point,build,optimize,run,show,all}', required=True)
    command = commands.add_parser('check', help='the point table and the configuration and programs check (writes nothing)',
                                  description='The key-point table (IK, family, clearance; every point or one) and the configuration '
                                              'and programs check: everything loads, and whether the Optimized program and the built '
                                              'trajectories are current. Writes nothing; exit 1 when a point FAILs or a file does not load.')
    command.add_argument('--name', help='check this key point only')
    command = commands.add_parser('point', help='one key point: its check row and the corrected Joints/Pose lines; --show to view it',
                                  description='One key point of Configuration/Constants.yaml: its check row; for a point defined by a '
                                              'Reference and Offset (or a PCB_Hold) the resolved Joints and Pose lines to paste after an '
                                              'edit; --show opens the viewer on the cell posed at it (mjpython on macOS).')
    command.add_argument('name', help='the point, e.g. P_LOADER_WAIT')
    command.add_argument('--seed', help='the point whose joints seed the IK (default: its own)')
    command.add_argument('--show', action='store_true', help='open the viewer on the cell posed at the point')
    command.add_argument('--headless', action='store_true', help='pose the cell and print it, no window')
    command.add_argument('--loader', help='with --show: a loader position (L_...) instead of the point\'s scene')
    command.add_argument('--gripper', help='with --show: a gripper command (G_...) instead of the point\'s scene')
    command.add_argument('--pcb', help='with --show: a PCB location instead of the point\'s scene')
    command.add_argument('--mg06', choices=['Open', 'Closed'], help='with --show: an MG06 state instead of the point\'s')
    commands.add_parser('build', help='the Default program: trajectory + kinematic check report',
                        description='Build the Default program (Configuration/Program.yaml) into its 4 ms trajectory and check it '
                                    'kinematically (clearances, limits, interlocks): Data/Generated/Trajectories/Program.npz and '
                                    'Data/Generated/Reports/Program_check.txt. Exit 1 when it cannot be built or the check fails.')
    commands.add_parser('optimize', help='the Optimized program (Data/Optimized/Program_Optimized.yaml) and its trajectory',
                        description='Run the optimizer (Configuration/Optimization.yaml; about 60 s): it writes '
                                    'Data/Optimized/Program_Optimized.yaml (committed), then builds and checks its trajectory. Exit 1 '
                                    'when the program or its check fails.')
    command = commands.add_parser('run', help='physics of both programs: acceptance, drift vs the baselines, comparison table',
                                  description='The physics run of both programs (in parallel, about 20 s): the 28 acceptance checks '
                                              'and the drift against the baselines in test/Simulation/data/, the reports in '
                                              'Data/Generated/Reports/, and the comparison table (cycle time, throughput, change). '
                                              'Needs both trajectories (cell.py build, cell.py optimize). Exit 1 on a failure, 2 when a '
                                              'run cannot run.')
    command.add_argument('--program', choices=list(CONST_PROGRAMS), help='one program\'s full physics report instead (no baselines)')
    command = commands.add_parser('show', help='the viewer: a program\'s trajectory, or its physics run with --physics',
                                  description='Play a program in MuJoCo\'s viewer (mjpython on macOS; cell.py re-launches itself under '
                                              'it): its trajectory (kinematic) or, with --physics, its physics run in real time. Keys: '
                                              'Home pause / resume, Page Down or . next step (physics: . one sample), Page Up step start '
                                              '(kinematic only), End step end.')
    command.add_argument('--program', choices=list(CONST_PROGRAMS), default='default', help='the program (default: default)')
    command.add_argument('--physics', action='store_true', help='play the physics run instead of the trajectory')
    command.add_argument('--step', help='pose the end of this step (its number or name; kinematic only)')
    command.add_argument('--time', type=float, help='pose (or start paused) at this time [s]')
    command.add_argument('--speed', type=float, help='with --physics: the playback speed against real time (default 1)')
    command.add_argument('--headless', action='store_true', help='pose (physics: run to --time) and print, no window')
    command.add_argument('--watchdog', type=float, metavar='S', help='debug: exit 3 with every thread\'s traceback when the viewer stalls for S s')
    command = commands.add_parser('all', help='check -> build -> optimize -> run; --recapture after an intended change',
                                  description='check, build, optimize (the optimizer only when Data/Optimized/Program_Optimized.yaml '
                                              'is stale) and run, stopping at the first failure with what to do (about 45 s; '
                                              '105 s with the optimizer). --recapture, after a change you intend: instead of run, '
                                              're-capture the kinematic golden (test/Program/data/) where it differs and the physics '
                                              'baselines (test/Simulation/data/) when a run fails, drifts beyond its tolerances or '
                                              'its report differs, saying what changed, and regenerate the '
                                              'results block of README.md (about 2 min more).')
    command.add_argument('--recapture', action='store_true',
                         help='re-capture the golden, the baselines (only a run outside its tolerances or with a changed report) and the README '
                              'results block')
    return parser


def main(argv: tp.Optional[tp.List[str]] = None) -> int:
    """
    Description:
        Parse the command line, start the subcommand's log (logger/cell_<subcommand>.log, rewritten) and run it.

            .venv/bin/python Example/cell.py {check,point,build,optimize,run,show,all} [options]

    Args:
        (1) argv [List<string>]: The arguments (default: the command line).

    Returns:
        (1) parameter [int]: The subcommand's exit code.
    """

    argv = sys.argv[1:] if argv is None else list(argv)
    args = Parser().parse_args(argv)
    log_name = 'cell_' + args.command
    Script_Log_Cls(log_name).Close()
    if args.command == 'check':
        return Check(args, log_name)
    if args.command == 'point':
        return Point(args, argv, log_name)
    if args.command == 'build':
        return Build(args, log_name)
    if args.command == 'optimize':
        return Optimize(args, log_name)
    if args.command == 'run':
        return Run(args, log_name)
    if args.command == 'all':
        return All(args, log_name)
    return Show(args, argv, log_name)


if __name__ == '__main__':
    sys.exit(main())
