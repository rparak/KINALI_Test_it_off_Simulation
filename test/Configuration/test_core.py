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
# File Name: ../test/Configuration/test_core.py
## =========================================================================== ##

# System (Default)
import os
import sys
# Subprocess (Subprocess management)
import subprocess
# JSON (JSON encoder and decoder)
import json
# Pytest (Testing framework) [pip3 install pytest]
import pytest
# Custom Lib.: the module under test
#   ../Configuration/Core
from Configuration.Core import (Read_PCB_Meta, Configuration_Error_Cls, Script_Log_Cls, Script_Log_Path, CONST_PCB_META,
                                CONST_LOGGER)

"""
Description:
    Initialization of constants.

    Expected values come from SPEC §4.4 (the brief's key numbers), written out here.
"""
# The named files of the repository, relative to its root (SPEC §6).
#   Unit [Dict<string, string>]
CONST_FILES = {'CONST_SCENE_XML': 'models/scene.xml', 'CONST_CELL_SCENE_XML': 'models/cell_scene.xml',
               'CONST_ROBOT_XML': 'models/aubo_i5/aubo_i5.xml', 'CONST_TOOL_XML': 'models/robot_tool/vgc10_opener_free.xml',
               'CONST_PCB_META': 'Data/PCB/pcb_meta.json', 'CONST_FORK_PATH': 'Data/Paths/fork_close_open_path.npz',
               'CONST_REFERENCE_PLAN': 'Data/Reference/cell_plan.npz'}


def _Run_Python(root, code, cwd):
    """
    Description:
        Run Python code in a fresh interpreter from cwd, with src/ on its path; returns the result.
    """

    prefix = 'import sys; sys.path.insert(0, %r)\n' % os.path.join(root, 'src')
    return subprocess.run([sys.executable, '-c', prefix + code], cwd=str(cwd), capture_output=True, text=True, timeout=300)


@pytest.mark.parametrize('name, relative', sorted(CONST_FILES.items()))
def test_every_named_file_resolves_from_any_working_directory(root, tmp_path, name, relative):
    """
    Description:
        In a fresh interpreter started in an unrelated directory, each file constant is the file at its
        repository-relative path, and the file exists.
    """

    result = _Run_Python(root, 'import Configuration.Core as C; print(C.%s)' % name, tmp_path)
    assert result.returncode == 0, result.stderr
    path = result.stdout.strip()
    assert os.path.samefile(path, os.path.join(root, relative))


def test_the_pcb_meta_has_the_spec_values():
    """
    Description:
        Read_PCB_Meta returns the loader positions, jaw values and PCB poses of SPEC §4.4 (to their 5
        printed decimals) in metres.
    """

    meta = Read_PCB_Meta()
    assert meta.Y_Home == pytest.approx(-0.1000, abs=5e-6)
    assert meta.Y_Place == pytest.approx(0.19666, abs=5e-6)
    assert meta.Z_Place == pytest.approx(0.06722, abs=5e-6)
    assert meta.Jaw_Open == pytest.approx(0.035, abs=5e-6)
    assert meta.Jaw_Grip == pytest.approx(0.01775, abs=5e-6)
    assert meta.Hold_Pos == pytest.approx([0.0057, -0.30829, 0.14961], abs=5e-6)
    assert meta.Place_Pos == pytest.approx([0.0057, -0.011636, 0.216826], abs=5e-7)
    assert meta.Quat == pytest.approx([0.99696, 0.07794, 0.0, 0.0], abs=5e-6)


def test_a_missing_file_is_named(tmp_path):
    """
    Description:
        A missing file is a Configuration_Error_Cls naming it.
    """

    path = str(tmp_path / 'absent.json')
    with pytest.raises(Configuration_Error_Cls, match='absent.json does not exist'):
        Read_PCB_Meta(path)


def test_every_problem_of_a_malformed_file_is_named_at_once(tmp_path):
    """
    Description:
        Invalid JSON is refused; in valid JSON a missing key and a wrong vector length are both named in
        one error.
    """

    broken = tmp_path / 'broken.json'
    broken.write_text('{not json', encoding='utf-8')
    with pytest.raises(Configuration_Error_Cls, match='is not valid JSON'):
        Read_PCB_Meta(str(broken))
    with open(CONST_PCB_META, encoding='utf-8') as handle:
        raw = json.load(handle)
    del raw['y_home']
    raw['quat'] = [1.0, 0.0, 0.0]
    mutated = tmp_path / 'mutated.json'
    mutated.write_text(json.dumps(raw), encoding='utf-8')
    with pytest.raises(Configuration_Error_Cls) as error:
        Read_PCB_Meta(str(mutated))
    assert 'y_home is missing' in str(error.value) and 'quat must be a list of 4 numbers' in str(error.value)


def test_mujoco_logs_into_the_configured_file_never_the_working_directory(root, tmp_path):
    """
    Description:
        After Configure_MuJoCo_Log, building the cell (which makes MuJoCo log its attach warning,
        KNOWN_ISSUES.md) writes the configured log file and no MUJOCO_LOG.TXT in the working directory.
    """

    log = tmp_path / 'logs' / 'MUJOCO_LOG.TXT'
    work = tmp_path / 'work'
    work.mkdir()
    code = ('from Configuration.Core import Configure_MuJoCo_Log\nConfigure_MuJoCo_Log(%r)\n'
            'from Scene.Core import Build_Cell_Model\nBuild_Cell_Model()\n' % str(log))
    result = _Run_Python(root, code, work)
    assert result.returncode == 0, result.stderr
    assert 'Attach conflict' in log.read_text()
    assert os.listdir(work) == []


def test_a_script_log_keeps_every_line_and_returns_it(tmp_path, monkeypatch):
    """
    Description:
        Script_Log_Cls.Line writes each line to logger/<script>.log at once and returns it unchanged for
        the script to print; a new run rewrites the file.
    """

    monkeypatch.setattr('Configuration.Core.CONST_LOGGER', str(tmp_path))
    log = Script_Log_Cls('demo')
    assert log.Line('[INFO] one') == '[INFO] one'
    log.Line('[INFO] two')
    assert open(log.Path, encoding='utf-8').read() == '[INFO] one\n[INFO] two\n'
    log.Close()
    log = Script_Log_Cls('demo')
    log.Line('[INFO] three')
    log.Close()
    assert open(os.path.join(str(tmp_path), 'demo.log'), encoding='utf-8').read() == '[INFO] three\n'


def test_a_script_log_lives_in_logger():
    """
    Description:
        A script's log file is logger/<script>.log.
    """

    assert Script_Log_Path('run_cycle') == os.path.join(CONST_LOGGER, 'run_cycle.log')
    assert os.path.isdir(CONST_LOGGER)
