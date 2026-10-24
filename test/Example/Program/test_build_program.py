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
    assert sorted(os.listdir(out)) == ['Program.csv', 'Program.npz', 'Program_check.txt'] and os.listdir(cwd) == []
    report = open(os.path.join(out, 'Program_check.txt'), encoding='utf-8').read()
    assert report.startswith('program Program: 22 steps, 17350 samples (4 ms), cycle time 69.396 s;')
    assert '  cycle time: program 69.396 s, legacy 65.768 s, difference +3.628 s' in report
    assert '  vacuum_on    program    3.276 s, legacy    3.276 s, +0.000 s' in report
    assert all(line in result.stdout for line in report.splitlines())
    assert '[INFO] Program: the check passed.' in result.stdout


@pytest.mark.slow
def test_a_program_that_cannot_be_built_is_an_error_line(tmp_path, run_script, root):
    """
    Description:
        A program that runs the tester's job before MG06 is closed (Tester.Run(J_TEST) as S1_Approach_Input's first line) is an
        [ERROR] line naming the segment and the line, exit 1, nothing written.
    """

    with open(os.path.join(root, 'Configuration', 'Program.yaml'), encoding='utf-8') as handle:
        text = handle.read()
    old = '  S1_Approach_Input:\n'
    assert old in text
    path = tmp_path / 'Program.yaml'
    path.write_text(text.replace(old, old + '    - Tester.Run(J_TEST)\n', 1), encoding='utf-8')
    out = tmp_path / 'out'
    result = run_script('Program/build_program.py', ['--program', str(path), '--out', str(out)], tmp_path)
    assert result.returncode == 1, result.stdout + result.stderr
    assert '[ERROR] Step S1_Approach_Input, Tester.Run(J_TEST) > Test(Process.Test_Duration): the test needs the PCB in MG06' in result.stdout
    assert not out.exists()


@pytest.mark.slow
def test_the_optimized_program_is_built_and_a_stale_or_edited_one_refused(tmp_path, run_script, root):
    """
    Description:
        --program Data/Optimized/Program_Optimized.yaml builds and checks the optimized program (exit 0, 16 steps, cycle
        57.652 s); a copy whose recorded input hash differs is refused naming the input and the optimizer's command; a copy
        whose recorded Schedule the build does not reproduce is refused naming the segment.
    """

    optimized = os.path.join(root, 'Data', 'Optimized', 'Program_Optimized.yaml')
    result = run_script('Program/build_program.py', ['--program', optimized, '--out', str(tmp_path)], tmp_path)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'program Program_Optimized: 16 steps, 14414 samples (4 ms), cycle time 57.652 s;' in result.stdout
    text = open(optimized, encoding='utf-8').read()
    stale = tmp_path / 'Stale.yaml'
    cell = text.split('    cell: "', 1)[1].split('"', 1)[0]
    stale.write_text(text.replace('    cell: "%s"' % cell, '    cell: "%s"' % ('0' * 64), 1), encoding='utf-8')
    result = run_script('Program/build_program.py', ['--program', str(stale), '--out', str(tmp_path)], tmp_path)
    assert result.returncode == 1 and 'was optimized from other inputs (cell changed since); make it again: .venv/bin/python ' \
                                       'Example/Program/optimize_program.py' in result.stdout
    edited = tmp_path / 'Edited.yaml'
    edited.write_text(text.replace('    S6_Close_MG06: 17.924', '    S6_Close_MG06: 18.000', 1), encoding='utf-8')
    result = run_script('Program/build_program.py', ['--program', str(edited), '--out', str(tmp_path)], tmp_path)
    assert result.returncode == 1 and 'does not match its recorded Schedule' in result.stdout and 'S6_Close_MG06: recorded 18.000 s, built 17.924 s' \
        in result.stdout
