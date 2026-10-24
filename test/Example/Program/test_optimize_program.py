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
# File Name: ../test/Example/Program/test_optimize_program.py
## =========================================================================== ##

# System (Default)
import os
# Pytest (Testing framework) [pip3 install pytest]
import pytest


@pytest.mark.slow
def test_the_optimizer_reproduces_the_committed_program_byte_for_byte(tmp_path_factory, run_script, root):
    """
    Description:
        B5: the optimizer run again (a fresh interpreter, from an unrelated directory, --out) writes exactly the committed
        Data/Optimized/Program_Optimized.yaml - the same seed, inputs, platform - and its report: every interlock holds, the
        check passes, the LOCKED comparison, exit 0; nothing in the working directory.
    """

    out, cwd = tmp_path_factory.mktemp('optimized'), tmp_path_factory.mktemp('cwd')
    result = run_script('Program/optimize_program.py', ['--out', str(out)], cwd)
    assert result.returncode == 0, result.stdout + result.stderr
    assert sorted(os.listdir(out)) == ['Program_Optimized.yaml', 'optimize_program.txt'] and os.listdir(cwd) == []
    written = open(os.path.join(out, 'Program_Optimized.yaml'), 'rb').read()
    assert written == open(os.path.join(root, 'Data', 'Optimized', 'Program_Optimized.yaml'), 'rb').read()
    report = open(os.path.join(out, 'optimize_program.txt'), encoding='utf-8').read()
    assert report.count(' holds: ') == 8 and 'FAILS' not in report.split('LOCKED parts')[0]
    assert 'cycle 57.132 s (taught 68.852 s, -11.720 s)' in report and 'Program_Optimized: the program check passed.' in report
    assert '[INFO] The optimized program passes its check.' in result.stdout
