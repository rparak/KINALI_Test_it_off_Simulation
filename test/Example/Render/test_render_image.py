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
# File Name: ../test/Example/Render/test_render_image.py
## =========================================================================== ##

# Matplotlib (Plotting) [pip3 install matplotlib]
import matplotlib.image as mpimg


def test_the_pcb_image_is_written_into_out(tmp_path, run_script):
    """
    Description:
        The default camera writes pcb_green.png (960 x 720) into --out.
    """

    result = run_script('Render/render_image.py', ['--out', str(tmp_path)], tmp_path)
    assert result.returncode == 0, result.stderr
    assert mpimg.imread(str(tmp_path / 'pcb_green.png')).shape[:2] == (720, 960)


def test_an_unknown_camera_is_an_error_line(tmp_path, run_script):
    """
    Description:
        An unknown camera is an [ERROR] line naming the cameras, exit 1, no file.
    """

    result = run_script('Render/render_image.py', ['--camera', 'cam_none', '--out', str(tmp_path / 'out')], tmp_path)
    assert result.returncode == 1 and "[ERROR] The model has no camera 'cam_none'" in result.stdout
    assert not (tmp_path / 'out').exists()
