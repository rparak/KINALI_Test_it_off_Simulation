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
# File Name: ../test/Example/test_model_fingerprint.py
## =========================================================================== ##

# System (Default)
import os
# JSON (JSON encoder and decoder)
import json


def test_the_default_compare_names_only_the_green_pcb(tmp_path, run_script):
    """
    Description:
        Without options the script compares with the baseline: exactly one difference, the PCB visual's
        colour, and exit 1 (a difference exists); the baseline is not written.
    """

    result = run_script('model_fingerprint.py', [], tmp_path)
    lines = result.stdout.splitlines()
    assert result.returncode == 1 and lines[0].startswith('[WARNING] 1 difference(s) against ')
    assert len(lines) == 2 and lines[1].startswith('  geoms/pcb_vis/geom_rgba: ')


def test_out_writes_a_fingerprint_that_compares_clean_with_itself(root, tmp_path, run_script, fingerprint_tool):
    """
    Description:
        --out writes the fingerprint; compared with it the model has no difference, and it differs from the
        baseline only in the green PCB.
    """

    path = tmp_path / 'fp.json'
    assert run_script('model_fingerprint.py', ['--out', str(path)], tmp_path).returncode == 0
    assert run_script('model_fingerprint.py', ['--compare', str(path)], tmp_path).returncode == 0
    with open(os.path.join(root, 'test', 'Scene', 'data', 'baseline_model_fingerprint.json'), encoding='utf-8') as handle:
        baseline = json.load(handle)
    differences = fingerprint_tool.Compare_Fingerprints(json.loads(path.read_text()), baseline)
    assert [line.split(':')[0] for line in differences] == ['geoms/pcb_vis/geom_rgba']
