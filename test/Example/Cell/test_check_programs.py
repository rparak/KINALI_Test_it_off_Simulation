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
# File Name: ../test/Example/Cell/test_check_programs.py
## =========================================================================== ##

# System (Default)
import os
# Subprocess (Subprocess management)
import subprocess

"""
Description:
    Example/Cell/check_programs.py, the configuration and programs check of cell.py check: in a fresh tree (no built
    trajectories), every file loads, the trajectories are named as not built, and an edited cell makes the Optimized
    program stale; nothing is written.
"""


def test_a_fresh_tree_loads_and_names_what_to_make(tmp_path, fresh_tree, script_command):
    """
    Description:
        A fresh tree: exit 0, the Optimized program current, both trajectories not built yet with the subcommand that builds
        each (cell.py build, cell.py optimize); after a value of Cell.yaml changes the Optimized program is stale (cell and planning_model
        changed: cell.py optimize); nothing is written to the tree or the working directory.
    """

    tree, work = fresh_tree(tmp_path / 'tree'), tmp_path / 'work'
    work.mkdir()
    result = subprocess.run(script_command(tree, 'Cell/check_programs.py'), cwd=str(work), capture_output=True, text=True, timeout=300)
    assert result.returncode == 0, result.stdout + result.stderr
    assert '[INFO] Data/Optimized/Program_Optimized.yaml is current (its inputs unchanged).' in result.stdout
    assert '[WARNING] Data/Generated/Trajectories/Program.npz is not built yet: run Example/cell.py build' in result.stdout
    assert '[WARNING] Data/Generated/Trajectories/Program_Optimized.npz is not built yet: run Example/cell.py optimize' in result.stdout
    assert 'they load; 2 warning(s).' in result.stdout and os.listdir(work) == []
    cell = tree / 'Configuration' / 'Cell.yaml'
    text = cell.read_text()
    assert text.count('Pressure: 40000.0') == 1, 'the edit below needs the shipped vacuum pressure'
    cell.write_text(text.replace('Pressure: 40000.0', 'Pressure: 41000.0'))
    result = subprocess.run(script_command(tree, 'Cell/check_programs.py'), cwd=str(work), capture_output=True, text=True, timeout=300)
    assert result.returncode == 0
    assert 'was optimized from other inputs (cell, planning_model changed since): run Example/cell.py optimize' in result.stdout
    assert not (tree / 'Data' / 'Generated').exists()
