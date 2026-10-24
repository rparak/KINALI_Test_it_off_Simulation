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
# File Name: ../test/Example/Program/test_golden_program.py
## =========================================================================== ##

# System (Default)
import os
# JSON (JSON encoder and decoder)
import json
# Pytest (Testing framework) [pip3 install pytest]
import pytest


@pytest.mark.slow
def test_the_default_compare_finds_no_difference_and_out_writes_the_same_data(root, tmp_path, run_script):
    """
    Description:
        Without options the script compares the built taught program with test/Program/data/: no difference, exit 0. --out
        writes the three files: the reports byte-identical to the committed ones, the record equal but for its source; a
        changed record compares with exit 1 naming the difference.
    """

    result = run_script('Program/golden_program.py', [], tmp_path)
    assert result.returncode == 0 and result.stdout.startswith('[INFO] 0 difference(s) against '), result.stdout
    out = tmp_path / 'golden'
    result = run_script('Program/golden_program.py', ['--out', str(out), '--source', 'a test'], tmp_path)
    assert result.returncode == 0 and '71 commands' in result.stdout, result.stdout
    data = os.path.join(root, 'test', 'Program', 'data')
    for name in ('golden_check_report.txt', 'golden_keypoint_table.txt'):
        assert (out / name).read_bytes() == open(os.path.join(data, name), 'rb').read(), name
    written, committed = json.loads((out / 'golden_program.json').read_text()), json.load(open(os.path.join(data, 'golden_program.json')))
    assert written['source'] == 'a test' and {**written, 'source': ''} == {**committed, 'source': ''}
    written['cycle_time'] += 0.004
    (out / 'golden_program.json').write_text(json.dumps(written))
    result = run_script('Program/golden_program.py', ['--compare', str(out)], tmp_path)
    assert result.returncode == 1 and '  cycle_time: ' in result.stdout, result.stdout
