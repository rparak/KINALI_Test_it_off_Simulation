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
# File Name: ../test/Example/Program/test_build_program.py
## =========================================================================== ##

# System (Default)
import os
# PyYAML (YAML parser and emitter) [pip3 install pyyaml]
import yaml
# Pytest (Testing framework) [pip3 install pytest]
import pytest


@pytest.fixture(scope='module')
def built(tmp_path_factory, run_script):
    out = tmp_path_factory.mktemp('built')
    cwd = tmp_path_factory.mktemp('cwd')
    return out, cwd, run_script('Program/build_program.py', ['--out', str(out), '--compare-legacy'], cwd)


@pytest.mark.slow
def test_the_default_program_is_built_checked_and_written(built):
    """
    Description:
        From an unrelated directory the script writes the trajectory (npz, csv) and the check report into --out, prints the
        report and the legacy diagnostic, and writes nothing into the working directory; exit 0, every step passes.
    """

    out, cwd, result = built
    assert result.returncode == 0, result.stdout + result.stderr
    assert sorted(os.listdir(out)) == ['Program_Default.csv', 'Program_Default.npz', 'Program_Default_check.txt'] and os.listdir(cwd) == []
    report = open(os.path.join(out, 'Program_Default_check.txt'), encoding='utf-8').read()
    assert report.startswith('program Program_Default: 22 steps, 17214 samples (4 ms), cycle time 68.852 s;')
    assert '  cycle time: program 68.852 s, legacy 65.768 s, difference +3.084 s' in report
    assert '  vacuum_on    program    3.276 s, legacy    3.276 s, +0.000 s' in report
    assert all(line in result.stdout for line in report.splitlines())
    assert '[INFO] Program_Default: the check passed.' in result.stdout


@pytest.mark.slow
def test_a_program_that_cannot_be_built_is_an_error_line(tmp_path, run_script, root):
    """
    Description:
        A program whose test runs with MG06 open (the test step moved before the MG06 is closed) is an [ERROR] line naming
        the step and the command, exit 1, nothing written.
    """

    with open(os.path.join(root, 'Configuration', 'Program_Default.yaml'), encoding='utf-8') as handle:
        tree = yaml.safe_load(handle)
    tree['Steps'].insert(1, tree['Steps'].pop(10))
    path = tmp_path / 'Program_Broken.yaml'
    path.write_text(yaml.safe_dump(tree, sort_keys=False), encoding='utf-8')
    out = tmp_path / 'out'
    result = run_script('Program/build_program.py', ['--program', str(path), '--out', str(out)], tmp_path)
    assert result.returncode == 1 and '[ERROR] Step test, Test Process.Test_Duration (2 s): the test needs the PCB in MG06' in result.stdout
    assert not out.exists()
