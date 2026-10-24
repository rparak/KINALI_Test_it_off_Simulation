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
# File Name: ../test/Example/test_export_cell_scene.py
## =========================================================================== ##

# System (Default)
import os
# MuJoCo (Multi-Joint dynamics with Contact) [pip3 install mujoco]
import mujoco


def test_the_export_into_another_folder_loads(tmp_path, run_script):
    """
    Description:
        --out writes cell_scene.xml there, it loads (mesh paths relative to it), and the working directory
        stays empty.
    """

    work = tmp_path / 'work'
    work.mkdir()
    result = run_script('export_cell_scene.py', ['--out', str(tmp_path / 'out')], work)
    assert result.returncode == 0, result.stderr
    path = tmp_path / 'out' / 'cell_scene.xml'
    assert result.stdout.startswith('[INFO] Wrote %s' % path)
    model = mujoco.MjModel.from_xml_path(str(path))
    assert (model.nq, model.nu) == (24, 13)
    assert os.listdir(work) == []
