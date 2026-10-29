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
# File Name: ../test/Example/test_cell.py
## =========================================================================== ##

# System (Default)
import os
# Importlib (The implementation of import)
import importlib.util
# Shutil (High-level file operations)
import shutil
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Example/cell.py, the one user entry point (design-decisions: one user entry point): its command line and help, the
    viewer's launch decision (headless: no window opens), and its subcommands run as a person runs them (a fresh
    interpreter, an unrelated working directory); the heavy ones in a fresh copy of the repository.
"""
# cell.py and its subcommands.
#   Unit [string], [List<string>]
CONST_CELL = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..', 'Example', 'cell.py')
CONST_SUBCOMMANDS = ['check', 'point', 'build', 'optimize', 'run', 'show', 'all']


@pytest.fixture(scope='module')
def cell():
    """
    Description:
        Example/cell.py imported as a module (a script, not part of src/).
    """

    spec = importlib.util.spec_from_file_location('cell', CONST_CELL)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_every_subcommand_is_listed_and_has_its_help(tmp_path, run_script):
    """
    Description:
        cell.py --help lists the seven subcommands and the exit codes; each subcommand's --help exits 0 with its usage; no
        subcommand is a usage error (exit 2).
    """

    result = run_script('cell.py', ['--help'], tmp_path)
    assert result.returncode == 0 and '{check,point,build,optimize,run,show,all}' in result.stdout
    assert 'exit codes: 0 success, 1 a check failed, 2 cannot run' in result.stdout
    for name in CONST_SUBCOMMANDS:
        result = run_script('cell.py', [name, '--help'], tmp_path)
        assert result.returncode == 0 and result.stdout.startswith('usage: cell.py %s' % name), name
    assert run_script('cell.py', [], tmp_path).returncode == 2
    assert os.listdir(tmp_path) == []


def test_the_viewer_runs_here_re_launches_under_mjpython_or_names_it(cell, tmp_path):
    """
    Description:
        Viewer_Launch: outside macOS, or already under mjpython (MJPYTHON_BIN set by its launcher), the viewer runs here; on macOS
        under python cell.py re-launches itself under the mjpython beside the interpreter when it is there and executable,
        else it names the missing one.
    """

    python = tmp_path / 'bin' / 'python'
    python.parent.mkdir()
    assert cell.Viewer_Launch('linux', {}, str(python)) == ('run', None)
    assert cell.Viewer_Launch('darwin', {'MJPYTHON_BIN': '/x'}, str(python)) == ('run', None)
    assert cell.Viewer_Launch('darwin', {}, str(python)) == ('missing', str(tmp_path / 'bin' / 'mjpython'))
    (tmp_path / 'bin' / 'mjpython').write_text('')
    assert cell.Viewer_Launch('darwin', {}, str(python))[0] == 'missing'
    os.chmod(tmp_path / 'bin' / 'mjpython', 0o755)
    assert cell.Viewer_Launch('darwin', {}, str(python)) == ('exec', str(tmp_path / 'bin' / 'mjpython'))


def test_the_re_launch_keeps_the_arguments_and_a_missing_mjpython_exits_2(cell, monkeypatch, capsys, tmp_path):
    """
    Description:
        _Viewer: 'exec' replaces the process with mjpython running cell.py with the same arguments (os.execv, recorded here, no
        window); 'missing' prints the install hint and returns 2; 'run' returns None (the viewer opens in this process).
    """

    monkeypatch.setattr('Configuration.Core.CONST_LOGGER', str(tmp_path))
    calls = []
    monkeypatch.setattr(cell.os, 'execv', lambda path, args: calls.append((path, args)))
    monkeypatch.setattr(cell, 'Viewer_Launch', lambda *_: ('exec', '/venv/bin/mjpython'))
    assert cell._Viewer(['show', '--program', 'optimized'], 'cell_test') is None
    assert calls == [('/venv/bin/mjpython', ['/venv/bin/mjpython', os.path.abspath(cell.__file__), 'show', '--program', 'optimized'])]
    monkeypatch.setattr(cell, 'Viewer_Launch', lambda *_: ('missing', '/venv/bin/mjpython'))
    assert cell._Viewer(['show'], 'cell_test') == 2
    assert 'there is none at /venv/bin/mjpython: run ./install.sh' in capsys.readouterr().out
    monkeypatch.setattr(cell, 'Viewer_Launch', lambda *_: ('run', None))
    assert cell._Viewer(['show'], 'cell_test') is None and len(calls) == 1


def test_check_runs_the_point_table_and_the_programs_check(tmp_path, run_script, root):
    """
    Description:
        cell.py check: the 25 points (0 FAIL) and the programs check; both in logger/cell_check.log; nothing written to the
        working directory; --name checks one point.
    """

    result = run_script('cell.py', ['check'], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '[INFO] 25 keypoint(s): 0 FAIL, 1 WARN, 24 PASS.' in result.stdout
    assert '[INFO] Configuration and programs: they load;' in result.stdout and os.listdir(tmp_path) == []
    log = open(os.path.join(root, 'logger', 'cell_check.log'), encoding='utf-8').read()
    assert '25 keypoint(s)' in log and 'Configuration and programs: they load' in log
    result = run_script('cell.py', ['check', '--name', 'P_HOME'], tmp_path)
    assert result.returncode == 0 and '1 keypoint(s): 0 FAIL' in result.stdout


def test_point_prints_its_row_its_resolved_lines_and_poses_it(tmp_path, run_script):
    """
    Description:
        cell.py point: a point with a Reference prints its check row and its resolved Joints and Pose lines; a point defined by
        its joints (P_HOME) its row only; --headless poses the cell and prints its context; an unknown point exits 1.
    """

    result = run_script('cell.py', ['point', 'P_INPUT_ABOVE'], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '1 keypoint(s): 0 FAIL' in result.stdout and '[INFO] P_INPUT_ABOVE resolved; paste these lines' in result.stdout
    result = run_script('cell.py', ['point', 'P_HOME', '--headless'], tmp_path)
    assert result.returncode == 0 and 'resolved' not in result.stdout and '[INFO] loader ' in result.stdout
    result = run_script('cell.py', ['point', 'nowhere'], tmp_path)
    assert result.returncode == 1 and "[ERROR] There is no point 'nowhere'" in result.stdout
    assert os.listdir(tmp_path) == []


def test_show_refuses_a_step_in_physics_and_a_speed_without_it(tmp_path, run_script):
    """
    Description:
        cell.py show: --physics --step and --speed without --physics exit 1 naming what to use instead; no viewer opens.
    """

    result = run_script('cell.py', ['show', '--physics', '--step', '3', '--headless'], tmp_path)
    assert result.returncode == 1 and 'use --physics --time T' in result.stdout
    result = run_script('cell.py', ['show', '--speed', '0.5', '--headless'], tmp_path)
    assert result.returncode == 1 and '--speed is for the physics run: add --physics.' in result.stdout


@pytest.mark.slow
def test_run_compares_both_programs_writes_their_reports_and_the_table(tmp_path, fresh_tree, built_trajectory, script_command):
    """
    Description:
        cell.py run in a fresh tree with both trajectories built: both programs within their baselines (exit 0), their physics
        reports in Data/Generated/Reports/, the table with 70.404 s / 59.148 s, 51.1 / 60.9 PCB/h, -16.0 % and +19.0 %, 28/28.
    """

    import subprocess
    tree = fresh_tree(tmp_path / 'tree')
    folder = tree / 'Data' / 'Generated' / 'Trajectories'
    folder.mkdir(parents=True)
    for program in ('taught', 'optimized'):
        shutil.copy(built_trajectory(program), folder)
    work = tmp_path / 'work'
    work.mkdir()
    result = subprocess.run(script_command(tree, 'cell.py') + ['run'], cwd=str(work), capture_output=True, text=True, timeout=900)
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-2000:]
    assert '[INFO] taught: 0 failure(s) (bounds and drift)' in result.stdout and '[INFO] optimized: 0 failure(s)' in result.stdout
    assert 'Default program          70.404                51.1  28/28 pass' in result.stdout
    assert 'Optimized program        59.148                60.9  28/28 pass' in result.stdout
    assert 'change                  -16.0 %             +19.0 %' in result.stdout
    reports = tree / 'Data' / 'Generated' / 'Reports'
    assert sorted(os.listdir(reports)) == ['Program_Optimized_physics.txt', 'Program_physics.txt'] and os.listdir(work) == []
    assert (reports / 'Program_physics.txt').read_text() == (tree / 'test' / 'Simulation' / 'data' / 'baseline_report_taught.txt').read_text()


def _Run_Cell(script_command, tree, work, *arguments):
    """
    Description:
        cell.py <arguments> of the given tree, run as a person runs it (a fresh interpreter, from work).
    """

    import subprocess
    return subprocess.run(script_command(tree, 'cell.py') + list(arguments), cwd=str(work), capture_output=True, text=True, timeout=1800)


def _Fresh_Work(tmp_path, fresh_tree):
    """
    Description:
        A fresh tree and an empty working directory beside it: (tree, work).
    """

    tree, work = fresh_tree(tmp_path / 'tree'), tmp_path / 'work'
    work.mkdir()
    return tree, work


def _Recapturable_Paths(root):
    """
    Description:
        The files all --recapture may write, relative to the root: the README and every committed golden and baseline file.
    """

    return ['README.md'] + [os.path.join('test', folder, 'data', name) for folder in ('Program', 'Simulation')
                            for name in sorted(os.listdir(os.path.join(root, 'test', folder, 'data')))]


@pytest.mark.slow
def test_all_passes_in_a_fresh_tree(tmp_path, fresh_tree, script_command):
    """
    Description:
        cell.py all in a fresh tree (no trajectories): check, build, the optimizer skipped (the committed Optimized program is
        current), its trajectory built, run - exit 0, 'all passed', nothing written into the working directory.
    """

    tree, work = _Fresh_Work(tmp_path, fresh_tree)
    result = _Run_Cell(script_command, tree, work, 'all')
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-2000:]
    assert 'is current: the optimizer is skipped' in result.stdout and '[INFO] all passed: check, build, optimize, run.' in result.stdout
    assert os.listdir(work) == []


@pytest.mark.slow
def test_recapture_restores_what_an_intended_change_moved_and_the_compares_accept_it(tmp_path, fresh_tree, script_command, root):
    """
    Description:
        As after an intended change, the golden's check report line, a baseline's cycle time and the README's results block
        are made to differ in a fresh tree: all --recapture says what differs, re-captures the golden and both baselines and
        regenerates the block, so every re-captured file is again the committed one byte for byte (the provenance fields
        aside: the golden's source, the baselines' environment commit and date), the rest of the README untouched; it
        reminds to commit and to refresh the support-chat brief. The golden compare and the baselines compare then pass
        against the re-captured files (they accept the re-captured provenance).
    """

    import json
    import subprocess
    tree, work = _Fresh_Work(tmp_path, fresh_tree)
    golden = tree / 'test' / 'Program' / 'data' / 'golden_check_report.txt'
    golden.write_text(golden.read_text().replace('keypoints used: 24', 'keypoints used: 23'))
    report = tree / 'test' / 'Simulation' / 'data' / 'baseline_report_taught.txt'
    report.write_text(report.read_text().replace('cycle 70.404 s', 'cycle 70.400 s', 1))
    readme = tree / 'README.md'
    readme.write_text(readme.read_text(encoding='utf-8').replace('| Cycle time [s] [P] | 70.404 |', '| Cycle time [s] [P] | 70.400 |'),
                      encoding='utf-8')
    result = _Run_Cell(script_command, tree, work, 'all', '--recapture')
    assert result.returncode == 0, result.stdout[-3000:] + result.stderr[-2000:]
    assert "report line 73: 'keypoints used: 23" in result.stdout
    assert '  diff -program Program: physics run of its trajectory, cycle 70.400 s' in result.stdout
    assert '[INFO] Re-captured: test/Program/data, test/Simulation/data, README.md (the results block).' in result.stdout
    assert '/support-brief REFRESH' in result.stdout and os.listdir(work) == []
    provenance = {os.path.join('test', 'Program', 'data', 'golden_program.json'): ['source'],
                  os.path.join('test', 'Simulation', 'data', 'baseline_environment.json'): ['commit', 'date']}
    paths = _Recapturable_Paths(root)
    assert sorted(os.listdir(tree / 'test' / 'Program' / 'data')) + sorted(os.listdir(tree / 'test' / 'Simulation' / 'data')) == \
        [os.path.basename(path) for path in paths[1:]]
    for path in paths:
        now, committed = open(os.path.join(str(tree), path), 'rb').read(), open(os.path.join(root, path), 'rb').read()
        if path in provenance:
            now, committed = json.loads(now), json.loads(committed)
            for key in provenance[path]:
                now.pop(key), committed.pop(key)
        assert now == committed, path
    compares = [subprocess.Popen(script_command(tree, tool), cwd=str(work), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                for tool in ('Maintenance/golden_program.py', 'Maintenance/physics_baseline.py')]
    (golden_out, _), (baseline_out, _) = [c.communicate(timeout=900) for c in compares]
    golden_code, baseline_code = [c.returncode for c in compares]
    assert golden_code == 0 and '[INFO] 0 difference(s) against' in golden_out, golden_out[-2000:]
    assert baseline_code == 0 and '[INFO] taught: 0 failure(s) (bounds and drift)' in baseline_out, baseline_out[-2000:]
    assert '[INFO] optimized: 0 failure(s) (bounds and drift)' in baseline_out and os.listdir(work) == []


@pytest.mark.slow
def test_recapture_on_an_unchanged_tree_has_nothing_to_recapture(tmp_path, fresh_tree, script_command, root):
    """
    Description:
        all --recapture in a fresh, unchanged tree: the golden, both baselines and the README's results block are current -
        exit 0, 'Nothing to re-capture', and every file it could re-capture is still the committed one byte for byte.
    """

    tree, work = _Fresh_Work(tmp_path, fresh_tree)
    result = _Run_Cell(script_command, tree, work, 'all', '--recapture')
    assert result.returncode == 0 and 'Nothing to re-capture' in result.stdout, result.stdout[-2000:]
    paths = _Recapturable_Paths(root)
    for path in paths:
        assert open(os.path.join(str(tree), path), 'rb').read() == open(os.path.join(root, path), 'rb').read(), path
    assert os.listdir(work) == []
