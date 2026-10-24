## =========================================================================== ##
# MIT License
# Copyright (c) 2026 Roman Parak
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
# File Name: ../test/Install/test_install.py
## =========================================================================== ##

# System (Default)
import os
# Subprocess (Subprocess management)
import subprocess
# Regular expressions (Default)
import re
# Pytest (Testing framework) [pip3 install pytest]
import pytest

"""
Description:
    Initialization of constants.

    install.sh is sourced in a fresh /bin/bash with a hermetic PATH: stubs for the commands that
    describe the machine (uname, sw_vers, the versioned python, python3) ahead of the system
    directories. What ran for real: the script's own functions, date, mkdir and rm. What ran only
    against stubs: platform detection, the interpreter lookup, the venv interpreter. pip is not run
    here; ./install.sh --recreate ran for real on macOS 26.6.2, Apple M2 Pro, 2026-10-24 (SPEC §11).
"""
# The Python and MuJoCo versions install.sh requires; written here, not read from the script under test.
#   Unit [string]
CONST_PYTHON_VERSION = '3.11'
CONST_MUJOCO_VERSION = '3.16.0'
# One entry per platform block of install.sh: uname -s, uname -m, the PLATFORM it sets, the word its
# INFO line names, the name of its candidate constant.
#   Unit [List<Tuple<string>>]
CONST_PLATFORMS = [('Darwin', 'arm64', 'macos', 'macOS', 'CONST_PYTHON_MACOS'),
                   ('Linux', 'x86_64', 'linux', 'Linux', 'CONST_PYTHON_LINUX')]
# One log line: 'YYYY-MM-DD HH:MM:SS [LEVEL  ] message', the level padded to 7.
#   Unit [Pattern]
CONST_LOG_LINE = re.compile(r'^\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2} \[(INFO   |WARNING|ERROR  )\] \S.*$')


def _Stub(directory, name, body):
    path = directory / name
    path.write_text('#!/bin/sh\n' + body + '\n', encoding='utf-8')
    path.chmod(0o755)
    return path


def _Run(root, stubs, script):
    """
    Description:
        Source install.sh and run a snippet; returns (exit code, stdout lines, stderr).
    """

    command = 'source "%s"; %s' % (os.path.join(root, 'install.sh'), script)
    result = subprocess.run(['/bin/bash', '-c', command], capture_output=True, text=True, timeout=60,
                            env={'PATH': '%s:/usr/bin:/bin' % stubs, 'HOME': str(stubs)})
    lines = [line for line in result.stdout.splitlines() if line]
    return result.returncode, lines, result.stderr


def _Python_Stub(directory, name, version):
    """
    Description:
        An interpreter stub whose version query prints `version`.
    """

    return _Stub(directory, name, 'echo %s' % version)


@pytest.fixture
def stubs(tmp_path):
    """
    Description:
        A stub directory; python3 records that it was called (it must never be).
    """

    directory = tmp_path / 'bin'
    directory.mkdir()
    _Stub(directory, 'python3', 'touch "%s/python3_was_called"; exit 1' % tmp_path)
    _Stub(directory, 'sw_vers', 'echo 26.6.2')
    return directory


def test_the_script_parses(root):
    """
    Description:
        bash -n accepts the script, and it runs under set -euo pipefail.
    """

    assert subprocess.run(['/bin/bash', '-n', os.path.join(root, 'install.sh')]).returncode == 0
    with open(os.path.join(root, 'install.sh'), encoding='utf-8') as handle:
        assert '\nset -euo pipefail\n' in handle.read()


@pytest.mark.parametrize('system, arch, platform, word, constant', CONST_PLATFORMS)
def test_the_platform_is_detected(root, stubs, system, arch, platform, word, constant):
    """
    Description:
        uname decides the platform, and the INFO line naming it is printed in the log format.
    """

    _Stub(stubs, 'uname', '[ "$1" = "-s" ] && echo %s || echo %s' % (system, arch))
    code, lines, _ = _Run(root, stubs, 'Detect_Platform; echo "PLATFORM=$PLATFORM"')
    assert code == 0 and lines[-1] == 'PLATFORM=%s' % platform
    assert any('[INFO   ] Detected %s' % word in line for line in lines[:-1])
    assert all(CONST_LOG_LINE.match(line) for line in lines[:-1])


def test_an_unsupported_system_fails_with_error_lines(root, stubs):
    """
    Description:
        An operating system without a platform block is refused with ERROR lines, the last naming
        the supported platforms, and a non-zero exit.
    """

    _Stub(stubs, 'uname', '[ "$1" = "-s" ] && echo FreeBSD || echo amd64')
    code, lines, _ = _Run(root, stubs, 'Detect_Platform')
    assert code == 1 and all(CONST_LOG_LINE.match(line) for line in lines)
    assert '[ERROR  ]' in lines[-1] and all(word in lines[-1] for _, _, _, word, _ in CONST_PLATFORMS)


@pytest.mark.parametrize('system, arch, platform, word, constant', CONST_PLATFORMS)
def test_python_is_the_first_explicit_candidate_reporting_3_11_never_python3(root, stubs, tmp_path, system, arch,
                                                                           platform, word, constant):
    """
    Description:
        A missing candidate and one of another version are skipped (the second with a WARNING); the
        next candidate reporting 3.11 is used; a name is looked up on PATH; the bare python3 is never
        called.
    """

    old = _Python_Stub(stubs, 'old_python', '3.9')
    _Python_Stub(stubs, 'python' + CONST_PYTHON_VERSION, CONST_PYTHON_VERSION)
    select = 'PLATFORM=%s; %s="%s %s python%s";' % (platform, constant, stubs / 'absent', old, CONST_PYTHON_VERSION)
    code, lines, _ = _Run(root, stubs, '%s Find_Python; echo "PYTHON=$PYTHON"' % select)
    assert code == 0 and lines[-1] == 'PYTHON=%s' % (stubs / ('python' + CONST_PYTHON_VERSION))
    assert any('[WARNING] %s reports Python \'3.9\'' % old in line for line in lines)
    assert all(CONST_LOG_LINE.match(line) for line in lines[:-1])
    assert not (tmp_path / 'python3_was_called').exists()


@pytest.mark.parametrize('system, arch, platform, word, constant', CONST_PLATFORMS)
def test_no_python_3_11_is_refused_with_its_install_command(root, stubs, tmp_path, system, arch, platform, word,
                                                            constant):
    """
    Description:
        Without a 3.11 candidate the lookup fails naming what it tried and the platform's install
        command (shown, never run: a prerequisite is checked by its presence only).
    """

    old = _Python_Stub(stubs, 'old_python', '2.7')
    code, lines, _ = _Run(root, stubs, 'PLATFORM=%s; %s="%s"; Find_Python' % (platform, constant, old))
    assert code == 1 and 'tried: %s' % old in lines[-2]
    assert 'Install Python %s with: ' % CONST_PYTHON_VERSION in lines[-1]
    assert ('brew install python@3.11' if platform == 'macos' else 'apt install python3.11') in lines[-1]
    assert not (tmp_path / 'python3_was_called').exists()


def _Fake_Root(tmp_path, venv_version):
    """
    Description:
        A root holding a .venv whose interpreter reports `venv_version`.
    """

    fake = tmp_path / 'repository'
    (fake / '.venv' / 'bin').mkdir(parents=True)
    _Python_Stub(fake / '.venv' / 'bin', 'python', venv_version)
    return fake


def test_a_broken_venv_is_reported_and_kept_without_recreate(root, stubs, tmp_path):
    """
    Description:
        A .venv of another Python is refused with the --recreate hint and is not deleted.
    """

    fake = _Fake_Root(tmp_path, '3.9')
    code, lines, _ = _Run(root, stubs, 'CONST_ROOT="%s"; Ensure_Venv no' % fake)
    assert code == 1 and '--recreate' in lines[-1]
    assert (fake / '.venv' / 'bin' / 'python').exists()


def test_recreate_deletes_the_venv_and_creates_a_new_one(root, stubs, tmp_path):
    """
    Description:
        With --recreate the existing .venv is deleted (WARNING) and created again by the found Python.
    """

    fake = _Fake_Root(tmp_path, '3.9')
    (fake / '.venv' / 'marker').write_text('old')
    creator = _Stub(stubs, 'creator', 'mkdir -p "$3/bin"')
    code, lines, _ = _Run(root, stubs, 'CONST_ROOT="%s"; PYTHON="%s"; Ensure_Venv yes' % (fake, creator))
    assert code == 0 and any('[WARNING] --recreate: deleted' in line for line in lines)
    assert (fake / '.venv' / 'bin').is_dir() and not (fake / '.venv' / 'marker').exists()


def test_a_working_venv_is_reused(root, stubs, tmp_path):
    """
    Description:
        A .venv of Python 3.11 is kept and reused, nothing is deleted.
    """

    fake = _Fake_Root(tmp_path, CONST_PYTHON_VERSION)
    code, lines, _ = _Run(root, stubs, 'CONST_ROOT="%s"; Ensure_Venv no' % fake)
    assert code == 0 and 'reused' in lines[-1]


def test_another_mujoco_version_is_refused(root, stubs, tmp_path):
    """
    Description:
        The verification prints the versions and fails when MuJoCo is not 3.16.0.
    """

    fake = tmp_path / 'repository'
    (fake / '.venv' / 'bin').mkdir(parents=True)
    _Stub(fake / '.venv' / 'bin', 'python', 'case "$2" in *numpy*) echo "Python 3.11.2, mujoco 3.15.0";; *) echo 3.15.0;; esac')
    code, lines, _ = _Run(root, stubs, 'CONST_ROOT="%s"; Verify_Install' % fake)
    assert code == 1 and 'Verified: Python 3.11.2, mujoco 3.15.0' in lines[0]
    assert 'not %s' % CONST_MUJOCO_VERSION in lines[-2]
    _Stub(fake / '.venv' / 'bin', 'python', 'case "$2" in *numpy*) echo "Python 3.11.2, mujoco 3.16.0";; *) echo 3.16.0;; esac')
    code, lines, _ = _Run(root, stubs, 'CONST_ROOT="%s"; Verify_Install' % fake)
    assert code == 0


def test_the_run_folders_are_created(root, stubs, tmp_path):
    """
    Description:
        logger/ and Data/Generated/ are created under the root, and creating them again is harmless.
    """

    for _ in range(2):
        code, _, _ = _Run(root, stubs, 'CONST_ROOT="%s"; Make_Folders' % tmp_path)
        assert code == 0
    assert (tmp_path / 'logger').is_dir() and (tmp_path / 'Data' / 'Generated').is_dir()


def test_an_unknown_option_is_refused_and_detect_only_changes_nothing(root, stubs, tmp_path):
    """
    Description:
        An unknown option fails with the usage line; --detect-only reports the plan and stops before
        any change (no .venv, no folders).
    """

    _Stub(stubs, 'uname', '[ "$1" = "-s" ] && echo Darwin || echo arm64')
    code, lines, _ = _Run(root, stubs, 'Main --bogus')
    assert code == 1 and 'Usage: ./install.sh [--detect-only | --recreate]' in lines[-1]
    python = _Python_Stub(stubs, 'brew_python', CONST_PYTHON_VERSION)
    code, lines, _ = _Run(root, stubs, 'CONST_ROOT="%s"; CONST_PYTHON_MACOS="%s"; Main --detect-only' % (tmp_path, python))
    assert code == 0 and 'Detect only' in lines[-1] and any('Plan:' in line for line in lines)
    assert sorted(os.listdir(tmp_path)) == ['bin']


@pytest.mark.parametrize('system, arch, platform, word, constant', CONST_PLATFORMS)
def test_the_shipped_candidates_are_explicit_never_a_bare_python3(root, stubs, system, arch, platform, word, constant):
    """
    Description:
        Every shipped interpreter candidate is an absolute path to a python3.11 or the name python3.11;
        a bare python3 (miniforge's on the development Mac) is never among them.
    """

    code, lines, _ = _Run(root, stubs, 'echo "$%s"' % constant)
    candidates = lines[-1].split()
    assert code == 0 and candidates
    for candidate in candidates:
        assert os.path.basename(candidate) == 'python' + CONST_PYTHON_VERSION, candidate
        assert candidate.startswith('/') or candidate == 'python' + CONST_PYTHON_VERSION, candidate
